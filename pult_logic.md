# Логика работы `pult.ino` (BLE HID <-> MQTT Gateway)

## 1) Общая идея

Устройство работает как **BLE HID**-сервер (клиент: проектор), а управление получает через **MQTT** (кнопки/громкость).

Поток команд выглядит так:

1. В MQTT прилетает сообщение на топик управления.
2. Коллбек `mqttCallback()` извлекает имя кнопки из топика.
3. `handleMqttCommand()` переводит имя кнопки в HID-действие (consumer или keyboard).
4. Если проектор уже подключен — действие отправляется сразу.
5. Если проектор «спит» (не подключен) — запускается **wake advertising** или **pairing advertising**, а команда сохраняется в `pending*` и отправляется при подключении BLE.

Дополнительно поддерживается специальная команда `find_pult`, которая подключается к «беепу» по BLE и включает/выключает зуммер.

## 2) Состояния (FSM) и индикация

Устройство использует небольшую FSM `State`:

- `kIdle` — проектор не подключен, обычная пауза.
- `kWaitingPair` — идет окно **pairing** (нормальное advertising), ждем подключения проектора.
- `kWakingUp` — активен короткий режим **wake advertising**, чтобы проектор «проснулся».
- `kPaired` — проектор подключен, можно слать HID-нотификации; LED горит постоянно.

Индикация светодиодом:

- в `kWaitingPair` — быстрое мигание (через `ledTickPairing()`),
- в `kWakingUp` — медленное мигание (через `ledTickWaking()`),
- в `kPaired` — `ledOn()` постоянно,
- в `kIdle` — `ledOff()` (в обработчиках и при остановках pairing).

Тайминги:

- `PAIR_TIMEOUT_MS` — сколько держим pairing advertising (`startPairing()` -> `stopPairing()`).
- `WAKE_DURATION_MS` — сколько держим wake advertising (`startWakeUp()` -> `stopWakeUp()`).

## 3) Advertising: normal vs wake

### Normal advertising (pairing)

Функция `setNormalAdvertising()` настраивает advertising payload так, чтобы проектор мог инициировать подключение.

Ключевые моменты:

- normal advertising работает в режиме **дополнительного устройства**, а не клона штатного пульта;
- имя устройства в рекламе: `Dangbei-ESP` (чтобы в проекторе добавить его через пункт **«Добавить устройство»**);
- публикуются `complete services`: `1812` (HID) и `180F` (Battery);
- такой режим позволяет сохранить подключение оригинального пульта и добавить ESP вторым устройством.

### Wake advertising

Функция `setWakeAdvertising()` настраивает advertising для «wake» сценария:

- оставляются те же флаги и appearance,
- указываются partial services `180F` и `1812`,
- добавляются **manufacturer data** (массив `mfg[]`), чтобы проектор распознал wake-пакет,
- advertising стартует сразу внутри `setWakeAdvertising()` (и затем выключается по таймеру в FSM).

Важно: **wake-логика для `power` не менялась** и продолжает работать как раньше.

`startWakeUp()` ставит состояние `kWakingUp` и запоминает `wakeStartTime`.

`stopWakeUp()`:

- останавливает wake advertising,
- возвращает normal advertising через `setNormalAdvertising()`,
- переводит в pairing: вызывает `startPairing()` (чтобы после пробуждения проектор подключился).

## 4) BLE HID: что именно публикуется

HID-сервис создается в `bleInit()`:

- `pServer` — BLE server.
- Service `1812` (HID).
- Характеристики:
  - `2A4A` (HID Information),
  - `2A4B` (Report Map) — содержит `hidReportDescriptor`,
  - `2A4C` (HID Control Point),
  - `2A4E` (Protocol Mode),
  - `2A4D` — две report-нотификации (consumer controls и keyboard), отличающиеся настройкой report reference через дескриптор `2908`.

Также добавляются минимальные сервисы:

- Battery (`180F` / `2A19`) и Device Information (`180A` / `2A50`).

`hidReportDescriptor` определяет форматы report для consumer/keyboards.

## 5) Отправка команд в HID

Есть две функции отправки:

- `sendConsumer(uint16_t usage)`
  - формирует 3-байтовый payload, включая `report ID = 0x01`,
  - делает `notify()` в `pConsumerReport`,
  - после короткой задержки отправляет “release” (обнуление), чтобы действие считалось одиночным.

- `sendKeyboard(uint8_t keycode)`
  - формирует 8-байтовый payload, включая `report ID = 0x0A`,
  - делает `notify()` в `pKeyboardReport`,
  - затем отправляет release (сброс ключа).

Важно: обе функции сначала проверяют `projectorConnected` и наличие соответствующих characteristic (`pConsumerReport` / `pKeyboardReport`).

## 6) Очередь команд при “сне” проектора

Команда может прийти, когда проектор еще не подключился.

Для этого используется:

- `pendingType` (enum `PendingType`) — `kKeyboard`, `kConsumer` или `kNone`,
- `pendingValue` — код клавиши (для keyboard <= 0xFF) или consumer usage (<= 0xFFFF).

`queueCommandAndConnect(type, value)`:

1. Запоминает ожидаемую команду в `pending*`.
2. Если текущий процесс уже идет (`kWaitingPair` или `kWakingUp`) — ничего не стартует заново.
3. Если проектор подключен — тоже ничего не делает.
4. Иначе запускает `startPairing()`.

Когда BLE-клиент (проектор) подключается, в `ServerCallbacks::onConnect()`:

- `projectorConnected = true`, состояние переводится в `kPaired`,
- advertising выключается,
- если `pendingType != kNone`, сохраненная команда отправляется сразу (через `sendKeyboard()`/`sendConsumer()`),
- pending сбрасывается.

При `onDisconnect()`:

- `projectorConnected = false`,
- `currentState = kIdle`,
- LED выключается,
- publish делается для топика `.../connected` со значением `0` (retain=true).

## 7) MQTT: маршрутизация топиков и команды

Поддерживаемые управляющие топики имеют вид:

`/devices/<DEVICE_ID>/controls/<button>/on`

`mqttCallback()`:

- превращает `topic` в строку,
- проверяет, что топик совпадает по префиксу и суффиксу `/on`,
- извлекает `<button>` между ними,
- вызывает `handleMqttCommand(button)`.

`handleMqttCommand()` выполняет маппинг:

- `power`
  - если `projectorConnected` — отправляет keyboard `0x66`,
  - если нет — делает `startWakeUp()` (чтобы разбудить проектор и потом связать).

- кнопки навигации (`up/down/left/right/ok/back/quick_menu`)
  - если подключен — отправляет сразу через `sendKeyboard(code)`,
  - если не подключен — вызывает `queueCommandAndConnect(...)` и переводит в pairing.

- `volume_up`, `volume_down`
  - `volume_up` -> consumer `0x00E9`,
  - `volume_down` -> consumer `0x00EA`,
  - если проектор не подключен — сохраняет pending и уходит в pairing.

- `home`, `menu`
  - `home` -> consumer `0x0223`,
  - `menu` -> consumer `0x0040`,
  - аналогично: либо отправка сразу, либо pending + pairing.

- `find_pult`
  - запускает BLE-поиск/паринг с внешним “beep” (по `FIND_PULT_REMOTE_ADDRESS`),
  - если pairing успешен — включает зуммер на 5 секунд и выключает,
  - затем при необходимости отключает BLE-клиента.

- `volume_set_N`
  - если имя команды начинается с `volume_set_`, берется `N` (через `atoi`),
  - вызывается `setVolume(N)`.

### setVolume(N)

`setVolume()` только для активного BLE-соединения:

1. Ограничивает `N` в диапазоне `0..15`.
2. Делает “сброс громкости в ноль” серией consumer команд `0x00EA` (15 раз).
3. Затем поднимает до нужного уровня consumer `0x00E9` (N раз).

## 8) MQTT метаданные

`publishMeta()` публикует retained-метаданные и подписывается на управляющие топики.

Для каждой кнопки (из массива `buttons[]`) отправляются:

- `meta/type` = `pushbutton`,
- `meta/title` = человекочитаемое имя,
- `meta/order` = порядок в UI,
- сам топик `.../<button>` со значением `"0"` (retain=true),
- подписка на `.../<button>/on` для приема команд.

Отдельно публикуется `volume` как `range` (0..15) и подписка на `volume/on`.

После успешного подключения к брокеру `mqttConnect()` вызывает `publishMeta()`.

## 9) setup() и loop()

### setup()

1. `Serial.begin()`
2. Настройка GPIO:
   - `LED_PIN` как OUTPUT,
   - `BUTTON_PIN` как `INPUT_PULLUP`.
3. `wifiConnect()` -> подключение WiFi.
4. Настройка MQTT:
   - `mqttClient.setServer(...)`,
   - `mqttClient.setCallback(mqttCallback)`,
   - `mqttClient.setKeepAlive(60)`,
   - `mqttConnect()` -> ожидание подключения и публикация метаданных.
5. `bleInit()` -> поднятие BLE HID server.
6. `startPairing()` -> старт pairing advertising при старте.

### loop()

В цикле делается:

1. Если MQTT отключен — снова подключаемся (`mqttConnect()`).
2. Вызывается `mqttClient.loop()` для обработки входящих сообщений.
3. Обработка кнопки `BUTTON_PIN` (BOOT):
   - при удержании кнопки принудительно запускается `startPairing()` (если не в wake/pairing уже).
4. Прогон FSM:
   - в `kWaitingPair` мигаем LED и отслеживаем `pairStartTime`, по истечению `PAIR_TIMEOUT_MS` -> `stopPairing()`,
   - в `kWakingUp` мигаем LED и отслеживаем `wakeStartTime`, по истечению `WAKE_DURATION_MS` -> `stopWakeUp()`,
   - в `kPaired` LED горит постоянно,
   - в `kIdle` ничего особенного.

В конце цикла небольшая задержка: `delay(10)`.

## 10) Подтвержденный рабочий сценарий

Текущий подтвержденный сценарий работы:

1. ESP подключается к Wi-Fi и MQTT, принимает команды из топиков `/devices/<DEVICE_ID>/controls/<button>/on`.
2. В проекторе ESP добавляется через меню **«Добавить устройство»** как отдельный BLE-пульт (`Dangbei-ESP`).
3. Оригинальный физический пульт продолжает работать (ESP не вытесняет его).
4. Управление проектором через MQTT-команды на ESP работает.
5. Команда `power` (wake-сценарий) работает в прежнем режиме.

## 11) Настройка с нуля (быстрый чеклист)

1. Прошить `pult.ino` на ESP32 и открыть `Serial Monitor` (115200).
2. Дождаться логов `WiFi OK`, `MQTT connected`, `BLE server initialized`.
3. На проекторе открыть Bluetooth/пульты и выбрать **«Добавить устройство»**.
4. В списке устройств выбрать `Dangbei-ESP` и завершить pairing.
5. Проверить, что оригинальный пульт продолжает работать параллельно.
6. Отправить тестовую MQTT-команду в `/devices/<DEVICE_ID>/controls/up/on` (или другую кнопку) и убедиться, что проектор реагирует.
7. Отправить MQTT-команду `power` и убедиться, что wake-сценарий/включение работает.

Если после перепрошивки ESP не появляется в списке добавления:
- перезапустить pairing кнопкой `BOOT` на ESP (или перезагрузить ESP);
- удалить старую запись ESP из проектора и добавить заново;
- проверить, что проектор и ESP находятся рядом во время pairing.


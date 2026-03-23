# Сигналы между физическим пультом и проектором (что известно по коду)

Этот файл описывает только те форматы и соответствия, которые явно видны в текущей прошивке (`pult_logic.ino` и функции отправки HID в `pult.ino`).

## 1) Что ESP шлёт проектору (HID)

Проектор подключается к ESP как BLE HID-устройство. ESP отправляет HID-notify в две логические группы (consumer controls и keyboard).

### 1.1 Consumer controls (громкость/служебные consumer-коды)

В `sendConsumerRaw(usage)` формируется payload из **3 байт**:

1. `data[0]` = `usage & 0xFF`
2. `data[1]` = `(usage >> 8) & 0xFF`
3. `data[2]` = `0x00`

Дальше вызывается `pConsumerReport->notify()`.

Используемые consumer-коды в MQTT-мэппинге:

- `0x00E9` = `volume_up`
- `0x00EA` = `volume_down`
- `0x0223` = `home`
- `0x0040` = `menu`

### 1.2 Keyboard (кнопки навигации)

В `sendKeyboardRaw(keycode)` формируется payload из **7 байт**:

`{ 0x00, 0x00, keycode, 0x00, 0x00, 0x00, 0x00 }`

Дальше вызывается `pKeyboardReport->notify()`.

Используемые keyboard-коды в MQTT-мэппинге:

- `0x52` = `up`
- `0x51` = `down`
- `0x50` = `left`
- `0x4F` = `right`
- `0x28` = `ok`
- `0x29` = `back`
- `0x3F` = `quick_menu`
- `0x66` = `power` (ключ, который используется и как wake-сигнал)

> Важно: в `bleInit()` оба отчёта публикуются через одну UUID характеристику `2A4D`, но различаются report reference дескрипторами `2908` (для consumer и keyboard).

## 2) Что ESP получает от физического пульта и как пробрасывает

ESP выступает BLE-клиентом к физическому пульту. При подписке (`subscribePultHid`) ESP подписывается на notify для UUID характеристики `2A4D`.

В `PultService::pultRemotePacket(data, len)`:

### 2.1 Consumer-пакеты от пульта

Если `len == 3`, то вычисляется:

`usage = data[0] | (data[1] << 8)`

Дополнительно:

- если `usage == 0xCCCC`, пакет игнорируется (`return`);
- иначе пакет ставится в очередь как `PendingType::kConsumer` с этим `usage`.

### 2.2 Keyboard-пакеты от пульта

Если `len == 7`, то `keycode = data[2]`.

Дальше пакет ставится в очередь как `PendingType::kKeyboard` с этим `keycode`.

### 2.3 Неизвестный размер

Если длина не равна `3` и не равна `7`, пакет не конвертируется в действия, а печатается в Serial как `PULT RX unknown len=...`.

## 3) Поведение при отключённом проекторе (важно для power)

Обработка очереди в `PultService::processForward()`:

- если проектор подключён (`projectorConnected == true`) — ESP шлёт HID сразу (`sendConsumerRaw` / `sendKeyboardRaw`);
- если проектор отключён — ESP:
  - для consumer ставит в очередь `queueCommandAndConnect(...)` (запускается pairing advertising);
  - для keyboard:
    - игнорирует `key == 0x00`
    - если `key == 0x66`, вызывает `startWakeUp()` и не отправляет в HID сразу (ESP пытается разбудить проектор wake-advertising’ом);
    - иначе ставит в очередь `queueCommandAndConnect(...)`.

Таким образом, `0x66` — это единственный keyboard-код, который при отсутствии BLE-соединения используется как триггер wake (а не обычная пересылка в HID).

## 4) Сигналы “wake/pairing” со стороны ESP к проектору (контекст power)

Это не приходит от физического пульта, но напрямую используется для сценария `power`:

- `startWakeUp()` переводит FSM в `kWakingUp` и запускает `setWakeAdvertising()` с:
  - flags `BLE_HS_ADV_F_DISC_LTD`
  - partial services `180F` и `1812`
  - manufacturer data `mfg[]`
- после `WAKE_DURATION_MS` вызывается `stopWakeUp()`, затем `startPairing()` — обычное advertising с:
  - complete services `1812` и `180F`
  - name `Dangbei-ESP`


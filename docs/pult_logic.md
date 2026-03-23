# Логика скетча `pult`

## Файлы

| Файл | Назначение |
|------|------------|
| `pult.ino` | BLE HID **сервер** (проектор), FSM рекламы, HID send, pending, `setup`/`loop`, `gPult`, батарея пульта |
| `projector_logic.ino` | реализация HID сервера проектора и FSM рекламы (send/pending/advertising) |
| `pult_state.h` | `PultState`, глобальный `gPult` |
| `pending_types.h` / `*_service.h` | public API (`PendingType`, `ProjectorService`, `PultService`, `MqttService`) |
| `pult_logic.ino` | BLE **клиент** к физическому пульту, pult HID → проектор |
| `mqtt_logic.ino` | Wi‑Fi, MQTT, команды и метаданные |

## Два BLE-линка

1. **Проектор ↔ ESP** — периферия HID (`1812`), команды с MQTT и из pult.
2. **ESP ↔ пульт** — централь (`gPult.client`), адрес `FIND_PULT_REMOTE_ADDRESS`, опционально при `gPult.enabled`.

## Состояние

- **`projectorConnected`** — есть GATT-сессия с проектором; без неё `send*` не шлёт.
- **`State`** — только реклама: `kIdle` | `kWaitingPair` | `kWakingUp`. Не дублирует «подключён к проектору».
- **`gPult`** — клиент к пульту: `enabled`, `client`, `hidSubscribed`, backoff reconnect (`lastReconnectMs`, `reconnectBackoffMs`), очередь `forward*` для пересылки HID с пульта.

`onConnect` / `onDisconnect` (сервер): выставляют `projectorConnected`, `currentState = kIdle`, стоп рекламы, LED off, `publishProjectorConnected`, сброс/отправка `pending*`.

## Реклама (проектор)

- **Normal** (`setNormalAdvertising`): `DISC_GEN`, имя `Dangbei-ESP`, сервисы `1812`+`180F`, appearance `0x03C1` → `startPairing()` стартует advertising на `PAIR_TIMEOUT_MS`.
- **Wake** (`setWakeAdvertising`): `DISC_LTD`, partial `180F`/`1812`, manufacturer data `mfg[]`, сразу `start()`; по таймеру `WAKE_DURATION_MS` → `stopWakeUp()` → снова normal + `startPairing()`.
- **`startWakeUp()`** — если нет `projectorConnected`: `kWakingUp`, wake-пакет (MQTT `power` без линка).

LED: мигает в `kWaitingPair` / `kWakingUp`; при линке с проектором — погашен.

## HID на проектор

- **Consumer**: report 3 байта → `pConsumerReport` notify; `sendConsumer` = нажатие + `delay(50)` + release `0x0000`.
- **Keyboard**: report 7 байт → `pKeyboardReport` notify; `sendKeyboard` = key + `delay(50)` + release `0x00`.
- **`send*Raw`** — без автorelease (relay / внутренние нужды).

## Очередь до линка

`pendingType` / `pendingValue` — **одна** отложенная команда.  
`queueCommandAndConnect`: пишет pending; если уже `kWaitingPair`/`kWakingUp` — только ждёт; если уже `projectorConnected` — не трогает; иначе `startPairing()`.  
При connect pending исполняется через `sendKeyboard`/`sendConsumer` и сбрасывается.

## MQTT

**Топик команд:** `/devices/<DEVICE_ID>/controls/<имя>/on`  
Исключение: `volume` — значение в **payload**, внутри собирается `volume_set_<N>`.

**Обработка:** `mqttMessageCallback` → `MqttService::callback` → `handleCommand`.

**Команды (суть):**

| Имя | Действие |
|-----|----------|
| `power` | линк есть → `sendKeyboard(0x66)`; нет → `startWakeUp()` |
| `volume_up` / `volume_down` | consumer `0x00E9` / `0x00EA` — сразу или `queueCommandAndConnect` |
| `home` / `menu` | consumer `0x0223` / `0x0040` — так же |
| навигация, `ok`, `back`, `quick_menu` | keyboard — так же |
| `find_pult` | `PultService::buzzerSwitch()` (FFF0/FFF2/FFF3) |
| `remote_pult_on` / `_off` | `gPult.enabled`, сброс backoff; ON → `connectPultAndSubscribeHid()` |
| `volume_set_N` | `setVolume(N)`, только при `projectorConnected`; N ∈ 0…20: 20× Vol−, затем N× Vol+ |

**Метаданные** (`publishMeta`, retained + subscribe где нужно):

- Кнопки из массива: `meta/type` pushbutton, title, order, `controls/<name>` = `0`, subscribe `.../on`.
- `remote_pult_battery`: meta `text`, order `13`, значение в `publishRemotePultBatteryState` (`--` или %).
- `connected`: meta `text`, order `17`, `publishProjectorConnected(0/1)`.
- `volume`: range max 20, order `16`, subscribe `volume/on`.

После подключения к брокеру: `connect()` → `publishMeta()`.

## Pult (код)

- Notify с пульта (`2A4D`) → `pultRemotePacket` → `enqueueForward` → в **`loop`** `processForward`: в проектор `sendConsumerRaw` / `sendKeyboardRaw` или снова `queueCommandAndConnect` / при key `0x66` без линка — `startWakeUp()`. Пакеты 3 байта = consumer usage (игнор `0xCCCC`), 7 байт = keyboard (key в `[2]`).
- Подключение пульта: `connectAndPair` (vendor FEF6, secure, ожидание pairing); при `gPult.enabled` короткий таймаут pairing допускается с продолжением линка.
- **`connectPultAndSubscribeHid`**: connect + сброс `hidSubscribed` + `subscribePultHid` на все notify `2A4D`.
- Разрыв пульта при включённом pult: в `loop` reconnect с удвоением backoff до `PULT_RECONNECT_MAX_MS` (сейчас лимит снижен для более быстрой реакции).
- Батарея пульта: GATT `180F`/`2A19`, опрос раз в `REMOTE_BATTERY_POLL_MS` при подключённом клиенте; при обрыве — `remotePultBatteryPercent = -1`, MQTT.

Автостарт в `setup`: `gPult.enabled = true`, затем `connectPultAndSubscribeHid()`.

## `setup` / `loop`

**setup:** `gPult.reconnectBackoffMs`, GPIO, `connectWifi`, MQTT server+callback+`connect`, `bleInit`, `startPairing`, автостарт pult (выше).

**loop:** держать MQTT; `processForward`; опрос батареи пульта; при `gPult.enabled` и нет линка к пульту — reconnect по backoff; BOOT: если нет линка с проектором и не wake/pair — `startPairing()`; FSM: таймауты `PAIR_TIMEOUT_MS` / `WAKE_DURATION_MS`; `delay(10)`.

## Чеклист

1. Прошивка, Serial 115200.  
2. Wi‑Fi / MQTT в логах.  
3. Проектор: добавить пульт **`Dangbei-ESP`**.  
4. Проверка MQTT-кнопки и `power` без линка (wake → pairing).

Если ESP не виден: BOOT для pairing, удалить старую запись в проекторе, близко к проектору.

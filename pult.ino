#include <NimBLEDevice.h>
#include <NimBLEClient.h>
#include <NimBLERemoteService.h>
#include <NimBLERemoteCharacteristic.h>
#include <NimBLEServer.h>
#include <NimBLEService.h>
#include <NimBLECharacteristic.h>
#include <NimBLEDescriptor.h>
#include <NimBLEAdvertising.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Нужны ранние объявления, т.к. relay-обработчик расположен выше основной FSM-части.
enum class PendingType : uint8_t { kNone, kKeyboard, kConsumer };
extern bool projectorConnected;
void sendConsumer(uint16_t usage);
void sendKeyboard(uint8_t keycode);
void sendConsumerRaw(uint16_t usage);
void sendKeyboardRaw(uint8_t keycode);
static void queueCommandAndConnect(PendingType type, uint16_t value);

// Основная идея:
// - Устройство выступает как BLE HID (клавиатура/consumer controls) для проектора.
// - Управление приходит по MQTT (кнопки/громкость), а затем превращается в HID-нотификации.
// - Перед подключением/когда проектор "спит" используется wake advertising + окно pairing.

// ---- Настройки ----
#define WIFI_SSID        "RT-GPON-N11"
#define WIFI_PASSWORD    "limit678"
#define MQTT_HOST        "192.168.0.15"
#define MQTT_PORT        1883
#define MQTT_USER        ""
#define MQTT_PASS        ""
#define DEVICE_ID        "dangbei_remote"

#define LED_PIN           2
#define BUTTON_PIN        0

// Тайминги FSM (pairing) и wake-окна.
#define PAIR_TIMEOUT_MS   30000
#define WAKE_DURATION_MS  5000

// ---- Поиск пульта (beep) ----
#define FIND_PULT_REMOTE_ADDRESS "70:50:E7:85:2F:DF"
#define FIND_PULT_PAIR_TIMEOUT_MS 10000
#define FIND_PULT_FAST_PAIR_WAIT_MS 1200
#define RELAY_RECONNECT_BASE_MS 2000
#define RELAY_RECONNECT_MAX_MS 15000
#define RELAY_DEBUG 1

static NimBLEClient* pFindClient = nullptr;
static volatile bool findConnected = false;
static volatile bool findPairingDone = false;
static bool remoteRelayEnabled = false;
static bool remoteRelaySubscribed = false;
static uint32_t relayLastReconnectTryMs = 0;
static uint32_t relayReconnectIntervalMs = RELAY_RECONNECT_BASE_MS;
static volatile bool relayForwardPending = false;
static volatile PendingType relayForwardType = PendingType::kNone;
static volatile uint16_t relayForwardValue = 0;

static void relayRemotePacket(const uint8_t* data, size_t len);
static bool findPultSubscribeRelay();
static void enqueueRelayForward(PendingType type, uint16_t value);
static void processRelayForward();

// Callbacks для BLE-соединения с "беепом":
// используются только чтобы дождаться успешного pairing перед отправкой команд на зуммер.
class FindClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) override {
    (void)pClient;
    findConnected = true;
    relayReconnectIntervalMs = RELAY_RECONNECT_BASE_MS;
    Serial.println("FIND_PULT: connected");
  }
  void onDisconnect(NimBLEClient* pClient, int reason) override {
    (void)pClient;
    findConnected = false;
    remoteRelaySubscribed = false;
    Serial.printf("FIND_PULT: disconnected reason=%d\n", reason);
    if (remoteRelayEnabled) {
      Serial.println("RELAY: link lost, waiting for wake and reconnect...");
    } else {
      Serial.println("RELAY: OFF");
    }
  }
  void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
    findPairingDone = true;
    if (connInfo.isEncrypted()) Serial.println("FIND_PULT: pairing success");
    else Serial.println("FIND_PULT: pairing failed");
  }
};

static bool findPultConnectAndPair() {
  if (pFindClient && pFindClient->isConnected()) {
    findConnected = true;
    findPairingDone = true;
    Serial.println("FIND_PULT: already connected");
    return true;
  }

  findPairingDone = false;
  findConnected = false;

  if (pFindClient == nullptr) {
    pFindClient = NimBLEDevice::createClient();
    pFindClient->setClientCallbacks(new FindClientCallbacks());
  }

  Serial.println("FIND_PULT: connecting...");
  if (!pFindClient->connect(NimBLEAddress(FIND_PULT_REMOTE_ADDRESS, BLE_ADDR_PUBLIC))) {
    Serial.println("FIND_PULT: connection failed");
    return false;
  }

  // Vendor activation (как в beep.ino)
  NimBLERemoteService* vendorSvc = pFindClient->getService(
    NimBLEUUID("0000fef6-0000-1000-8000-00805f9b34fb"));
  if (vendorSvc) {
    NimBLERemoteCharacteristic* ctrl = vendorSvc->getCharacteristic(
      NimBLEUUID("005f0002-2ff2-4ed5-b045-4c7463617865"));
    if (ctrl && ctrl->canWrite()) {
      uint8_t cmd[1] = {0x01};
      ctrl->writeValue(cmd, 1, true);
      Serial.println("FIND_PULT: vendor activated");
    }
  }

  pFindClient->secureConnection();

  const uint32_t pairWaitMs = remoteRelayEnabled ? FIND_PULT_FAST_PAIR_WAIT_MS
                                                 : FIND_PULT_PAIR_TIMEOUT_MS;
  Serial.print("FIND_PULT: waiting pairing");
  uint32_t t = millis();
  while (!findPairingDone && (millis() - t) < pairWaitMs) {
    Serial.print(".");
    delay(200);
  }
  Serial.println();

  if (!findPairingDone) {
    if (remoteRelayEnabled) {
      Serial.println("FIND_PULT: fast-pair timeout, continue with connected link");
      return true;
    }
    Serial.println("FIND_PULT: pairing timeout");
    return false;
  }

  delay(300);
  return true;
}

static bool findPultBuzzerSwitch() {
  if (!pFindClient) return false;
  NimBLERemoteService* svc = pFindClient->getService(NimBLEUUID((uint16_t)0xFFF0));
  if (!svc) {
    Serial.println("FIND_PULT: FFF0 service not found");
    return false;
  }

  NimBLERemoteCharacteristic* writeChar = svc->getCharacteristic(NimBLEUUID((uint16_t)0xFFF2));
  NimBLERemoteCharacteristic* readChar = svc->getCharacteristic(NimBLEUUID((uint16_t)0xFFF3));
  if (!writeChar) {
    Serial.println("FIND_PULT: FFF2 char not found");
    return false;
  }

  // Читаем текущее состояние с read-характеристики (FFF3): 0x01=ON, 0x02=OFF.
  uint8_t current = 0x02; // безопасный дефолт: считаем OFF, если чтение недоступно
  if (readChar && readChar->canRead()) {
    std::string v = readChar->readValue();
    if (!v.empty()) current = (uint8_t)v[0];
  }

  uint8_t next = (current == 0x01) ? 0x02 : 0x01;
  uint8_t cmd[1] = {next};
  writeChar->writeValue(cmd, 1, false);
  Serial.printf("FIND_PULT: buzzer %s\n", (next == 0x01) ? "ON" : "OFF");
  return true;
}

static void onFindPultNotify(NimBLERemoteCharacteristic* pChr, uint8_t* data, size_t len, bool isNotify) {
  (void)isNotify;
  if (!pChr || !data || len == 0) return;
  relayRemotePacket(data, len);
}

static void relayRemotePacket(const uint8_t* data, size_t len) {
  if (!remoteRelayEnabled) return;

  // По финальной таблице пульта Dangbei:
  // - 3 байта: consumer control (handle 0x002A)
  // - 7 байт: keyboard (handle 0x0036)
  if (len == 3) {
    uint16_t usage = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
#if RELAY_DEBUG
    Serial.printf("RELAY RX consumer: %02X %02X %02X (usage=0x%04X)\n",
                  data[0], data[1], data[2], usage);
#endif
    if (usage == 0xCCCC) return; // vendor mouse mode

    enqueueRelayForward(PendingType::kConsumer, usage);
    return;
  }

  if (len == 7) {
    uint8_t keycode = data[2];
#if RELAY_DEBUG
    Serial.printf("RELAY RX keyboard: %02X %02X %02X %02X %02X %02X %02X (key=0x%02X)\n",
                  data[0], data[1], data[2], data[3], data[4], data[5], data[6], keycode);
#endif
    enqueueRelayForward(PendingType::kKeyboard, keycode);
    return;
  }

#if RELAY_DEBUG
  Serial.printf("RELAY RX unknown len=%u: ", (unsigned)len);
  for (size_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();
#endif
}

static void enqueueRelayForward(PendingType type, uint16_t value) {
  // Пишем только одно последнее событие (аналогично pending-команде для MQTT).
  relayForwardType = type;
  relayForwardValue = value;
  relayForwardPending = true;
}

static void processRelayForward() {
  if (!relayForwardPending) return;

  noInterrupts();
  PendingType type = relayForwardType;
  uint16_t value = relayForwardValue;
  relayForwardType = PendingType::kNone;
  relayForwardValue = 0;
  relayForwardPending = false;
  interrupts();

  if (type == PendingType::kNone) return;

  if (type == PendingType::kConsumer) {
    if (projectorConnected) {
#if RELAY_DEBUG
      Serial.printf("RELAY TX consumer -> projector: 0x%04X\n", value);
#endif
      // Для relay прокидываем как есть (включая release 0x0000), без авто-release.
      sendConsumerRaw(value);
    } else {
      if (value == 0x0000) return; // release не имеет смысла ставить в очередь
#if RELAY_DEBUG
      Serial.printf("RELAY QUEUE consumer: 0x%04X\n", value);
#endif
      queueCommandAndConnect(PendingType::kConsumer, value);
    }
  } else if (type == PendingType::kKeyboard) {
    uint8_t key = (uint8_t)(value & 0xFF);
    if (projectorConnected) {
#if RELAY_DEBUG
      Serial.printf("RELAY TX keyboard -> projector: 0x%02X\n", key);
#endif
      // Для relay прокидываем как есть (включая release key=0x00), без авто-release.
      sendKeyboardRaw(key);
    } else {
      if (key == 0x00) return; // release не имеет смысла ставить в очередь
      // Power-клавиша с физического пульта при выключенном проекторе
      // должна запускать wake-сценарий, как MQTT-команда "power".
      if (key == 0x66) {
#if RELAY_DEBUG
        Serial.println("RELAY POWER key while disconnected -> startWakeUp()");
#endif
        startWakeUp();
        return;
      }
#if RELAY_DEBUG
      Serial.printf("RELAY QUEUE keyboard: 0x%02X\n", key);
#endif
      queueCommandAndConnect(PendingType::kKeyboard, key);
    }
  }
}

static bool findPultSubscribeRelay() {
  if (!pFindClient || !pFindClient->isConnected()) return false;
  if (remoteRelaySubscribed) return true;

  NimBLERemoteService* hidSvc = pFindClient->getService(NimBLEUUID((uint16_t)0x1812));
  if (!hidSvc) {
    Serial.println("RELAY: HID service 1812 not found");
    return false;
  }

  // Нужны обе report-характеристики (consumer + keyboard). Ищем все notify-репорты в HID.
  auto chars = hidSvc->getCharacteristics(true);

  uint8_t subscribedCount = 0;
  for (auto* c : chars) {
    if (!c) continue;
    if (c->getUUID().equals(NimBLEUUID((uint16_t)0x2A4D)) && c->canNotify()) {
      if (c->subscribe(true, onFindPultNotify, true)) {
        subscribedCount++;
      }
    }
  }

  if (subscribedCount == 0) {
    Serial.println("RELAY: no notify report subscribed");
    return false;
  }

  remoteRelaySubscribed = true;
  Serial.printf("RELAY: subscribed reports=%u\n", subscribedCount);
  return true;
}

// ---- HID Report Descriptor ----
// Описывает структуру HID-репортов, которые отправляются в:
// - consumer controls (громкость и т.п.)
// - клавиатуру (навигация: up/down/left/right/ok/back/…)
static const uint8_t hidReportDescriptor[] = {
    0x05,0x0c,0x09,0x01,0xa1,0x01,0x85,0x01,0x19,0x00,0x2a,0x9c,
    0x02,0x15,0x00,0x26,0x9c,0x02,0x95,0x01,0x75,0x10,0x81,0x00,
    0x09,0x02,0xa1,0x02,0x05,0x09,0x19,0x01,0x29,0x0a,0x15,0x01,
    0x25,0x0a,0x95,0x01,0x75,0x08,0x81,0x40,0xc0,0xc0,0x06,0x01,
    0xff,0x09,0x01,0xa1,0x02,0x85,0x05,0x09,0x14,0x75,0x08,0x95,
    0x14,0x15,0x80,0x25,0x7f,0x81,0x22,0x85,0x04,0x09,0x04,0x75,
    0x08,0x95,0x01,0x91,0x02,0xc0,0x05,0x01,0x09,0x06,0xa1,0x01,
    0x85,0x0a,0x75,0x01,0x95,0x08,0x05,0x07,0x19,0xe0,0x29,0xe7,
    0x15,0x00,0x25,0x01,0x81,0x02,0x95,0x01,0x75,0x08,0x81,0x01,
    0x95,0x05,0x75,0x01,0x05,0x08,0x19,0x01,0x29,0x05,0x91,0x02,
    0x95,0x01,0x75,0x03,0x91,0x01,0x95,0x06,0x75,0x08,0x15,0x00,
    0x26,0xff,0x00,0x05,0x07,0x19,0x00,0x29,0xff,0x81,0x00,0xc0,
};

// ---- Состояния ----
// Простая FSM управляет поведением устройства:
// - kIdle: проектор не подключен, обычная пауза
// - kWaitingPair: окно pairing после startPairing()
// - kWakingUp: короткое wake-окно (другая рекламная "пачка")
// - kPaired: проектор подключен и устройство готово слать HID-нотификации
enum class State {
  kWaitingPair,
  kPaired,
  kIdle,
  kWakingUp,
};

State currentState = State::kIdle;
bool projectorConnected = false;
uint32_t pairStartTime = 0;
uint32_t wakeStartTime = 0;

// ---- BLE ----
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pConsumerReport = nullptr;
NimBLECharacteristic* pKeyboardReport = nullptr;

// ---- Ожидающая команда (если пришла до подключения) ----
PendingType pendingType = PendingType::kNone;
uint16_t pendingValue = 0; // keycode (<=0xFF) или consumer usage (<=0xFFFF)

// ---- WiFi + MQTT ----
// MQTT используется как "транспорт" для команд:
// - subscribe на `/devices/{DEVICE_ID}/controls/<name>/on`
// - при подключении отправляем HID-команды и публикем retain-метаданные для UI
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ---- LED ----
// Светодиод — индикация текущего состояния FSM:
// - мигание в pairing
// - медленное мигание в wake
// - в kPaired горит постоянно
void ledOn()  { digitalWrite(LED_PIN, LOW); }
void ledOff() { digitalWrite(LED_PIN, HIGH); }
void ledTickPairing() {
  uint32_t t = millis() % 400;
  if (t < 200) ledOn(); else ledOff();
}
void ledTickWaking() {
  // Медленное мигание при wake — 100мс вкл / 900мс выкл
  uint32_t t = millis() % 1000;
  if (t < 100) ledOn(); else ledOff();
}

// ---- Advertising helpers ----
// Подготавливаем advertising payload под два режима:
// - нормальный режим (pairing): чтобы проектор смог инициировать подключение
// - wake режим: короткий и "специфичный" пакет, по которому проектор просыпается
void setNormalAdvertising() {
  NimBLEAdvertising* pAdv = pServer->getAdvertising();
  pAdv->stop();
  pAdv->clearData();

  NimBLEAdvertisementData advData;
  // В normal режиме ESP должен быть отдельным устройством (не клон оригинального пульта),
  // чтобы проектор мог добавить его как дополнительный remote через "Добавить устройство".
  // Wake-совместимость для power-сценария остается в setWakeAdvertising().
  advData.setFlags(BLE_HS_ADV_F_BREDR_UNSUP | BLE_HS_ADV_F_DISC_GEN);
  advData.setAppearance(0x03C1);
  advData.setCompleteServices16(std::vector<NimBLEUUID>{
      NimBLEUUID("1812"), NimBLEUUID("180F")});
  // У оригинального пульта Name = "Dangbei", 
  // если использовать его, ESP будет подключаться проектором автоматически при старте рекламирования.
  // но оригинальный пульт уже не подключится, так как он будет считаться дубликатом.
  advData.setName("Dangbei-ESP"); 
  pAdv->setAdvertisementData(advData);
  pAdv->setScanFilter(false, false);
  pAdv->setScanResponseData(NimBLEAdvertisementData{});
}

// Для пробуждения испольуем специфичный пакет, который проектор может распознать.
void setWakeAdvertising() {
  Serial.println("Setting WAKE advertising...");
  NimBLEAdvertising* pAdv = pServer->getAdvertising();
  pAdv->stop();
  pAdv->clearData();

  NimBLEAdvertisementData wakeData;
  // Собираем через NimBLE-поля, чтобы получить нужный пакет без Local Name.
  wakeData.setFlags(BLE_HS_ADV_F_BREDR_UNSUP | BLE_HS_ADV_F_DISC_LTD);
  wakeData.setPartialServices16(std::vector<NimBLEUUID>{
      NimBLEUUID("180F"), NimBLEUUID("1812")});
  wakeData.setAppearance(0x03C1);
  const uint8_t mfg[] = {0x46, 0x00, 0x08, 0xE9, 0x20, 0xFF, 0x9D, 0x16, 0xA8, 0xFF, 0xFF, 0xFF, 0xFF};
  wakeData.setManufacturerData(mfg, sizeof(mfg));

  pAdv->setAdvertisementData(wakeData);
  pAdv->setScanFilter(false, false);
  pAdv->setScanResponseData(NimBLEAdvertisementData{});
  pAdv->start();
  Serial.println("Wake advertising started.");
}

// ---- Wake on BLE ----
// wake advertising поднимается на короткое время.
// После истечения WAKE_DURATION_MS делаем stopWakeUp(), затем автоматически стартуем pairing.
void startWakeUp() {
  if (projectorConnected) return; // уже подключён
  Serial.println("Wake on BLE started...");
  currentState = State::kWakingUp;
  wakeStartTime = millis();
  setWakeAdvertising();
}

void stopWakeUp() {
  pServer->getAdvertising()->stop();
  setNormalAdvertising();
  Serial.println("Wake on BLE stopped.");
  // После wake сразу начинаем pairing — проектор должен подключиться
  startPairing();
}

// ---- Pairing ----
// Открывает окно нормального advertising на PAIR_TIMEOUT_MS.
// Если проектор не подключился за это время — возвращаемся в kIdle.
void startPairing() {
  if (projectorConnected) return;
  if (currentState == State::kWaitingPair) return;
  Serial.println("Starting pairing (30s)...");
  currentState = State::kWaitingPair;
  pairStartTime = millis();
  setNormalAdvertising();
  pServer->getAdvertising()->start();
}

void stopPairing() {
  pServer->getAdvertising()->stop();
  currentState = State::kIdle;
  ledOff();
  Serial.println("Pairing stopped.");
}

// ---- Отправка команд ----
// Отправка HID-нотификаций в два отчета:
// - sendConsumer(): Consumer Controls (обычно для громкости и некоторых сервисных кнопок)
// - sendKeyboard(): Keyboard (стрелки/OK/Back/…)
// В обоих случаях делается "release" (сброс) после небольшой задержки, чтобы событие не
// считалось удерживаемым постоянно.
void sendConsumer(uint16_t usage) {
  if (!projectorConnected || !pConsumerReport) return;
  sendConsumerRaw(usage);
  delay(50);
  sendConsumerRaw(0x0000);
}

void sendConsumerRaw(uint16_t usage) {
  if (!projectorConnected || !pConsumerReport) return;
  uint8_t data[3] = {(uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8), 0x00};
  pConsumerReport->setValue(data, 3);
  pConsumerReport->notify();
}

void sendKeyboard(uint8_t keycode) {
  if (!projectorConnected || !pKeyboardReport) return;
  sendKeyboardRaw(keycode);
  delay(50);
  sendKeyboardRaw(0x00);
}

void sendKeyboardRaw(uint8_t keycode) {
  if (!projectorConnected || !pKeyboardReport) return;
  uint8_t data[7] = {0x00, 0x00, keycode, 0x00, 0x00, 0x00, 0x00};
  pKeyboardReport->setValue(data, 7);
  pKeyboardReport->notify();
}

// Команда, полученная по MQTT/кнопке, может прийти пока проектор "спит".
// Тогда мы сохраняем одно последнее ожидаемое действие (pendingType/pendingValue) и запускаем pairing.
static void queueCommandAndConnect(PendingType type, uint16_t value) {
  pendingType = type;
  pendingValue = value;

  // Если уже идёт wake/pairing — просто ждём подключения.
  if (currentState == State::kWaitingPair || currentState == State::kWakingUp) return;
  if (projectorConnected) return;
  startPairing();
}

// Громкость до нуля затем до нужного значения
void setVolume(int level) {
  if (!projectorConnected) return;
  level = constrain(level, 0, 20);
  Serial.printf("Setting volume to %d\n", level);
  // Сбрасываем до нуля (20 нажатий Volume-)
  for (int i = 0; i < 20; i++) {
    sendConsumer(0x00EA);
    delay(50);
    sendKeyboardRaw(0x00);
  }
  // Поднимаем до нужного уровня
  for (int i = 0; i < level; i++) {
    sendConsumer(0x00E9);
    delay(50);
    sendKeyboardRaw(0x00);
  }
}

// ---- MQTT команды ----
// map-логика: строки топиков MQTT -> HID коды/usage-значения.
// Если проектор уже подключен — шлем сразу.
// Если нет — стартуем wake (для `power`) или очередь + pairing (для остальных кнопок).
void handleMqttCommand(const char* button) {
  Serial.printf("CMD: %s\n", button);

  if (strcmp(button, "power") == 0) {
    // Если соединение активно — считаем, что проектор включен (тогглим Power).
    // Если соединения нет — шлём wake advertising, чтобы проектор проснулся и подключился.
    if (projectorConnected) sendKeyboard(0x66);
    else startWakeUp();
  }
  else if (strcmp(button, "volume_up")   == 0) {
    if (projectorConnected) sendConsumer(0x00E9);
    else queueCommandAndConnect(PendingType::kConsumer, 0x00E9);
  }
  else if (strcmp(button, "volume_down") == 0) {
    if (projectorConnected) sendConsumer(0x00EA);
    else queueCommandAndConnect(PendingType::kConsumer, 0x00EA);
  }
  else if (strcmp(button, "home")        == 0) {
    if (projectorConnected) sendConsumer(0x0223);
    else queueCommandAndConnect(PendingType::kConsumer, 0x0223);
  }
  else if (strcmp(button, "menu")        == 0) {
    if (projectorConnected) sendConsumer(0x0040);
    else queueCommandAndConnect(PendingType::kConsumer, 0x0040);
  }
  else if (strcmp(button, "up")          == 0) {
    if (projectorConnected) sendKeyboard(0x52);
    else queueCommandAndConnect(PendingType::kKeyboard, 0x52);
  }
  else if (strcmp(button, "down")        == 0) {
    if (projectorConnected) sendKeyboard(0x51);
    else queueCommandAndConnect(PendingType::kKeyboard, 0x51);
  }
  else if (strcmp(button, "left")        == 0) {
    if (projectorConnected) sendKeyboard(0x50);
    else queueCommandAndConnect(PendingType::kKeyboard, 0x50);
  }
  else if (strcmp(button, "right")       == 0) {
    if (projectorConnected) sendKeyboard(0x4F);
    else queueCommandAndConnect(PendingType::kKeyboard, 0x4F);
  }
  else if (strcmp(button, "ok")          == 0) {
    if (projectorConnected) sendKeyboard(0x28);
    else queueCommandAndConnect(PendingType::kKeyboard, 0x28);
  }
  else if (strcmp(button, "back")        == 0) {
    if (projectorConnected) sendKeyboard(0x29);
    else queueCommandAndConnect(PendingType::kKeyboard, 0x29);
  }
  else if (strcmp(button, "quick_menu")  == 0) {
    if (projectorConnected) sendKeyboard(0x3F);
    else queueCommandAndConnect(PendingType::kKeyboard, 0x3F);
  }
  else if (strcmp(button, "find_pult")   == 0) {
    Serial.println("FIND_PULT: start (5s)");
    findPultBuzzerSwitch();
    // Соединение с основным пультом оставляем открытым, чтобы держать второй BLE-линк:
    // проектор <-> ESP (HID) и ESP <-> пульт (find_pult).
    Serial.println("FIND_PULT: done (link kept)");
  }
  else if (strcmp(button, "remote_relay_on") == 0) {
    remoteRelayEnabled = true;
    relayLastReconnectTryMs = 0;
    relayReconnectIntervalMs = RELAY_RECONNECT_BASE_MS;
    if (findPultConnectAndPair()) {
      remoteRelaySubscribed = false; // на случай переподключения
      if (findPultSubscribeRelay()) {
        Serial.println("RELAY: ON");
      } else {
        Serial.println("RELAY: ON requested, subscribe failed");
      }
    } else {
      Serial.println("RELAY: ON requested, pult connect failed");
    }
  }
  else if (strcmp(button, "remote_relay_off") == 0) {
    remoteRelayEnabled = false;
    remoteRelaySubscribed = false;
    relayLastReconnectTryMs = 0;
    relayReconnectIntervalMs = RELAY_RECONNECT_BASE_MS;
    Serial.println("RELAY: OFF");
  }
  else {
    // Проверяем команду установки громкости: "volume_set_N"
    if (strncmp(button, "volume_set_", 11) == 0) {
      int level = atoi(button + 11);
      setVolume(level);
    } else {
      Serial.printf("Unknown: %s\n", button);
    }
  }
}

// Обработчик входящего MQTT:
// извлекает `<button>` из `/devices/{DEVICE_ID}/controls/<button>/on` и вызывает handleMqttCommand().
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String prefix = "/devices/" DEVICE_ID "/controls/";
  String suffix = "/on";

  if (t.startsWith(prefix) && t.endsWith(suffix)) {
    String button = t.substring(prefix.length(),
                                t.length() - suffix.length());
    // Для range-контрола volume значение приходит в payload.
    if (button == "volume") {
      char buf[12];
      unsigned int n = (length < sizeof(buf) - 1) ? length : (sizeof(buf) - 1);
      memcpy(buf, payload, n);
      buf[n] = '\0';

      char cmd[24];
      snprintf(cmd, sizeof(cmd), "volume_set_%d", atoi(buf));
      handleMqttCommand(cmd);
      return;
    }

    handleMqttCommand(button.c_str());
  }
}

// ---- MQTT метаданные ----
// Публикуем retained-метаданные для интерфейса/автоконфигурации:
// тип/заголовок/порядок для каждой кнопки и топик, куда отправлять on-события.
void publishMeta() {
  mqttClient.publish(
    "/devices/" DEVICE_ID "/meta/name",
    "Dangbei Remote", true);

  struct { const char* name; const char* title; int order; } buttons[] = {
    {"power",       "Power",       0},
    {"home",        "Home",        1},
    {"menu",        "Menu",        2},
    {"quick_menu",  "Quick Menu",  3},
    {"back",        "Back",        4},
    {"ok",          "OK",          5},
    {"up",          "Up",          6},
    {"down",        "Down",        7},
    {"left",        "Left",        8},
    {"right",       "Right",       9},
    {"volume_up",   "Volume+",    10},
    {"volume_down", "Volume-",    11},
    {"find_pult",   "Find pult",  12},
    {"remote_relay_on", "Remote relay ON", 14},
    {"remote_relay_off", "Remote relay OFF", 15},
  };

  char topic[128];
  char val[8];
  for (auto& b : buttons) {
    snprintf(topic, sizeof(topic),
             "/devices/%s/controls/%s/meta/type", DEVICE_ID, b.name);
    mqttClient.publish(topic, "pushbutton", true);

    snprintf(topic, sizeof(topic),
             "/devices/%s/controls/%s/meta/title", DEVICE_ID, b.name);
    mqttClient.publish(topic, b.title, true);

    snprintf(topic, sizeof(topic),
             "/devices/%s/controls/%s/meta/order", DEVICE_ID, b.name);
    snprintf(val, sizeof(val), "%d", b.order);
    mqttClient.publish(topic, val, true);

    snprintf(topic, sizeof(topic),
             "/devices/%s/controls/%s", DEVICE_ID, b.name);
    mqttClient.publish(topic, "0", true);

    snprintf(topic, sizeof(topic),
             "/devices/%s/controls/%s/on", DEVICE_ID, b.name);
    mqttClient.subscribe(topic);
  }

  // Громкость как range 0-20
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/volume/meta/type", "range", true);
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/volume/meta/title", "Volume", true);
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/volume/meta/max", "20", true);
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/volume/meta/order", "16", true);
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/volume", "0", true);
  mqttClient.subscribe(
    "/devices/" DEVICE_ID "/controls/volume/on");

  Serial.println("MQTT meta published.");
}

void mqttConnect() {
  // Бесконечно пытаемся подключиться к брокеру.
  // После успешного подключения публикуем метаданные (retained), чтобы UI знал кнопки.
  while (!mqttClient.connected()) {
    Serial.println("MQTT connecting...");
    if (mqttClient.connect(DEVICE_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("MQTT connected.");
      publishMeta();
    } else {
      Serial.printf("MQTT failed rc=%d, retry 3s\n", mqttClient.state());
      delay(3000);
    }
  }
}

void wifiConnect() {
  // Подключаемся к WiFi с таймаутом 15 секунд.
  Serial.printf("WiFi connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi failed!");
  }
}

// ---- BLE Server Callbacks ----
// Коллбеки сервера HID:
// - onConnect(): считаем проектор "живым", выставляем kPaired и сразу сбрасываем pending-команду (если была)
// - onDisconnect(): переводим в kIdle и выключаем индикацию
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    projectorConnected = true;
    currentState = State::kPaired;
    // По фактическому UX: при активном соединении с проектором светодиод должен быть погашен.
    ledOff();
    Serial.print("Projector connected: ");
    Serial.println(connInfo.getAddress().toString().c_str());
    pServer->getAdvertising()->stop();

    // Если команда пришла до подключения — отправим её сейчас.
    if (pendingType != PendingType::kNone) {
      PendingType t = pendingType;
      uint16_t v = pendingValue;
      pendingType = PendingType::kNone;
      pendingValue = 0;
      if (t == PendingType::kKeyboard) sendKeyboard((uint8_t)(v & 0xFF));
      else if (t == PendingType::kConsumer) sendConsumer(v);
    }

    // Публикуем статус подключения
    mqttClient.publish(
      "/devices/" DEVICE_ID "/controls/connected",
      "1", true);
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    projectorConnected = false;
    currentState = State::kIdle;
    ledOff();
    Serial.printf("Projector disconnected, reason=%d\n", reason);

    mqttClient.publish(
      "/devices/" DEVICE_ID "/controls/connected",
      "0", true);
  }
};

// ---- BLE Init ----
// Инициализация BLE HID сервиса:
// - 1812 (HID): создаем характеристики и report reference
// - 180F (Battery) + 180A (Device Information): по минимуму, чтобы клиентам было "как у HID"
// - выставляем advertising по умолчанию (нормальный режим для pairing)
void bleInit() {
  NimBLEDevice::init("dangbei_remote");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(true, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pHid = pServer->createService("1812");

  pHid->createCharacteristic("2A4A", NIMBLE_PROPERTY::READ)
      ->setValue((uint8_t[]){0x11,0x01,0x00,0x01}, 4);

  pHid->createCharacteristic("2A4B", NIMBLE_PROPERTY::READ)
      ->setValue(hidReportDescriptor, sizeof(hidReportDescriptor));

  pHid->createCharacteristic("2A4C", NIMBLE_PROPERTY::WRITE_NR);

  auto pProto = pHid->createCharacteristic(
      "2A4E", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);
  pProto->setValue((uint8_t[]){0x01}, 1);

  pConsumerReport = pHid->createCharacteristic(
      "2A4D", NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  auto pRepRef1 = new NimBLEDescriptor("2908",
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE, 2);
  pRepRef1->setValue((uint8_t[]){0x01, 0x01}, 2);
  pConsumerReport->addDescriptor(pRepRef1);

  pKeyboardReport = pHid->createCharacteristic(
      "2A4D", NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  auto pRepRef2 = new NimBLEDescriptor("2908",
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE, 2);
  pRepRef2->setValue((uint8_t[]){0x0A, 0x01}, 2);
  pKeyboardReport->addDescriptor(pRepRef2);

  pHid->start();

  NimBLEService* pBat = pServer->createService("180F");
  pBat->createCharacteristic("2A19",
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY)
      ->setValue((uint8_t[]){0x64}, 1);
  pBat->start();

  NimBLEService* pDev = pServer->createService("180A");
  pDev->createCharacteristic("2A50", NIMBLE_PROPERTY::READ)
      ->setValue((uint8_t[]){0x02,0x54,0x2B,0x00,0x16,0x00,0x00}, 7);
  pDev->start();

  setNormalAdvertising();
  Serial.println("BLE server initialized.");
}

void setup() {
  // Порядок старта:
  // 1) GPIO (LED/кнопка)
  // 2) WiFi + MQTT
  // 3) BLE server (HID) и старт pairing
  Serial.begin(115200);
  Serial.println("\n=== Dangbei BLE -> MQTT Gateway ===");

  pinMode(LED_PIN, OUTPUT);
  ledOff();
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  wifiConnect();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  mqttConnect();

  bleInit();
  startPairing();

  // Автостарт relay при загрузке ESP (эквивалент remote_relay_on).
  remoteRelayEnabled = true;
  relayLastReconnectTryMs = 0;
  relayReconnectIntervalMs = RELAY_RECONNECT_BASE_MS;
  Serial.println("RELAY: auto-start ON");
  if (findPultConnectAndPair()) {
    remoteRelaySubscribed = false;
    if (findPultSubscribeRelay()) {
      Serial.println("RELAY: ON");
    } else {
      Serial.println("RELAY: auto-start subscribe failed");
    }
  } else {
    Serial.println("RELAY: auto-start connect failed");
  }
}

void loop() {
  // Основной цикл:
  // - держим MQTT в рабочем состоянии
  // - обрабатываем кнопку BOOT (перезапуск pairing при необходимости)
  // - прогоняем FSM (wake/pairing таймеры + индикация)
  if (!mqttClient.connected()) mqttConnect();
  mqttClient.loop();
  processRelayForward();

  // В relay-режиме после idle-disconnect пульт обычно начинает рекламироваться
  // при нажатии кнопки. Периодически пробуем reconnect (без постоянного удержания линка).
  if (remoteRelayEnabled && (!pFindClient || !pFindClient->isConnected())) {
    uint32_t t = millis();
    if (relayLastReconnectTryMs == 0 || (t - relayLastReconnectTryMs) >= relayReconnectIntervalMs) {
      relayLastReconnectTryMs = t;
      Serial.printf("RELAY: reconnect attempt (interval=%lu ms)...\n",
                    (unsigned long)relayReconnectIntervalMs);
      if (findPultConnectAndPair()) {
        remoteRelaySubscribed = false;
        findPultSubscribeRelay();
        relayReconnectIntervalMs = RELAY_RECONNECT_BASE_MS;
      } else {
        uint32_t next = relayReconnectIntervalMs * 2;
        relayReconnectIntervalMs = (next > RELAY_RECONNECT_MAX_MS) ? RELAY_RECONNECT_MAX_MS : next;
      }
    }
  }

  // Кнопка BOOT
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("Button pressed — restarting pairing");
      if (!projectorConnected &&
          currentState != State::kWaitingPair &&
          currentState != State::kWakingUp) {
        startPairing();
      }
      while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    }
  }

  // Обработка топика громкости
  // (volume/on приходит как число 0-20)

  // Важно: берём millis() ПОСЛЕ обработки MQTT/кнопок, иначе при смене state в этом же проходе
  // может быть now < wakeStartTime и вылезет unsigned underflow (wake стопнётся "мгновенно").
  uint32_t now = millis();

  switch (currentState) {
    case State::kWaitingPair:
      ledTickPairing();
      if (now - pairStartTime >= PAIR_TIMEOUT_MS) {
        stopPairing();
        Serial.println("Pairing timeout.");
      }
      break;

    case State::kWakingUp:
      ledTickWaking();
      if (now - wakeStartTime >= WAKE_DURATION_MS) {
        stopWakeUp();
      }
      break;

    case State::kPaired:
      // Светодиод горит постоянно
      break;

    case State::kIdle:
      break;
  }

  delay(10);
}
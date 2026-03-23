// Обработка подключения проектора к ESP (HID server)
// ESP выступает в качестве BLE сервера, проектор подключается к ESP

#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEService.h>
#include <NimBLECharacteristic.h>
#include <NimBLEAdvertising.h>
#include <NimBLEDescriptor.h>
#include <NimBLEServer.h>

#include "services.h"

// ---- HID Report Descriptor ----
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
  0x26,0xff,0x00,0x05,0x07,0x19,0x00,0x29,0xff,0x81,0x00,0xc0
};

// ---- Состояния рекламы ----
enum class State {
  kIdle,
  kWaitingPair,
  kWakingUp,
  kReconnecting,
};

static State currentState = State::kIdle;
static uint32_t pairStartTime = 0;
static uint32_t wakeStartTime = 0;
static uint32_t reconnectStartTime = 0;

// Связь с проектором (HID-ready) — глобальная, нужна для relay/mqtt.
bool projectorConnected = false;

// ---- BLE ----
static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pConsumerReport = nullptr;
static NimBLECharacteristic* pKeyboardReport = nullptr;

// ---- Ожидающая команда при “сне” проектора ----
static PendingType pendingType = PendingType::kNone;
static uint16_t pendingValue = 0; // keycode (<=0xFF) или consumer usage (<=0xFFFF)

// ---- LED ----
void ledOn()  { digitalWrite(LED_PIN, LOW); }
void ledOff() { digitalWrite(LED_PIN, HIGH); }

static void ledTickPairing() {
  uint32_t t = millis() % 400;
  if (t < 200) ledOn(); else ledOff();
}

static void ledTickWaking() {
  // Медленное мигание при wake — 100мс вкл / 900мс выкл
  uint32_t t = millis() % 1000;
  if (t < 100) ledOn(); else ledOff();
}

// ---- Advertising helpers ----
// Рекламирование для pairing (normal advertising)
static void setNormalAdvertising() {
  NimBLEAdvertising* pAdv = pServer->getAdvertising();
  pAdv->stop();
  pAdv->clearData();

  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_BREDR_UNSUP | BLE_HS_ADV_F_DISC_GEN);
  advData.setAppearance(0x03C1);
  advData.setCompleteServices16(std::vector<NimBLEUUID>{
      NimBLEUUID("1812"), NimBLEUUID("180F")});
  // Название штатного пульта "Dangbei", если установить это имя вместо "Dangbei-ESP",
  // то проектор будет подключаться к ESP автоматически без ручного поиска устройств
  advData.setName("Dangbei-ESP");

  pAdv->setAdvertisementData(advData);
  pAdv->setScanFilter(false, false);
  pAdv->setScanResponseData(NimBLEAdvertisementData{});
}

// Рекламирование для включения проектора
// Фиксированный пакет данных, который ожидает проектор для включения
static void setWakeAdvertising() {
  NimBLEAdvertising* pAdv = pServer->getAdvertising();
  pAdv->stop();
  pAdv->clearData();

  NimBLEAdvertisementData wakeData;
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
}

// ---- Wake on BLE ----
// Запуск включения проектора, если не подключен к ESP
void ProjectorService::startWakeUp() {
  if (projectorConnected) return; // уже подключён
  currentState = State::kWakingUp;
  wakeStartTime = millis();
  setWakeAdvertising();
}

// Отложенное подключение к проектору после включения
void ProjectorService::stopWakeUp() {
  pServer->getAdvertising()->stop();
  setNormalAdvertising();
  // После wake сразу начинаем pairing — проектор должен подключиться
  startPairing();
}

// ---- Pairing ----
void ProjectorService::startPairing() {
  if (projectorConnected) return;
  if (currentState == State::kWaitingPair) return;
  currentState = State::kWaitingPair;
  pairStartTime = millis();
  setNormalAdvertising();
  pServer->getAdvertising()->start();
}

void ProjectorService::stopPairing() {
  pServer->getAdvertising()->stop();
  currentState = State::kIdle;
  ledOn();
}

bool ProjectorService::projectorIsInWakeOrPairing() {
  return currentState == State::kWaitingPair || currentState == State::kWakingUp;
}

// Обработка состояний подключения к проектору
void ProjectorService::projectorLoopTick() {
  uint32_t now = millis();
  switch (currentState) {
    // В состоянии ожидания (idle) ничего не делаем 
    case State::kIdle:
      break;
    // Мигаем светодиодом при ожидании подключения
    case State::kWaitingPair:
      ledTickPairing();
      if (now - pairStartTime >= PAIR_TIMEOUT_MS) {
        stopPairing();
        Serial.println("PROJECTOR: Pairing timeout.");
      }
      break;
    // Мигаем светодиодом при включении проектора
    case State::kWakingUp:
      ledTickWaking();
      if (now - wakeStartTime >= WAKE_DURATION_MS) {
        stopWakeUp();
      }
      break;
    // Пауза перед переподключением после разрыва соединения
    case State::kReconnecting:
      if (now - reconnectStartTime >= RECONNECT_DELAY_MS) {
        Serial.println("PROJECTOR: Reconnecting after disconnect...");
        currentState = State::kIdle;
        startPairing();
      }
      break;
  }
}

// ---- Отправка команд в HID ----
void ProjectorService::sendConsumer(uint16_t usage) {
  if (!projectorConnected || !pConsumerReport) return;
  sendConsumerRaw(usage);
  delay(50);
  sendConsumerRaw(0x0000);
}
void ProjectorService::sendConsumerRaw(uint16_t usage) {
  if (!projectorConnected || !pConsumerReport) return;
  uint8_t data[3] = {(uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8), 0x00};
  pConsumerReport->setValue(data, 3);
  pConsumerReport->notify();
}

void ProjectorService::sendKeyboard(uint8_t keycode) {
  if (!projectorConnected || !pKeyboardReport) return;
  sendKeyboardRaw(keycode);
  delay(50);
  sendKeyboardRaw(0x00);
}
void ProjectorService::sendKeyboardRaw(uint8_t keycode) {
  if (!projectorConnected || !pKeyboardReport) return;
  // Payload как в текущем HID keyboard report: 7 байт
  uint8_t data[7] = {0x00, 0x00, keycode, 0x00, 0x00, 0x00, 0x00};
  pKeyboardReport->setValue(data, 7);
  pKeyboardReport->notify();
}

// Запоминание команды для отправки после подключения к проектору
void ProjectorService::queueCommandAndConnect(PendingType type, uint16_t value) {
  pendingType = type;
  pendingValue = value;

  // Если уже идёт wake/pairing — просто ждём подключения.
  if (currentState == State::kWaitingPair || currentState == State::kWakingUp) return;
  if (projectorConnected) return;
  startPairing();
}

// Установка громкости числовым значением (0-20) для удобства управления голосом
void ProjectorService::setVolume(int level) {
  if (!projectorConnected) return;
  level = constrain(level, 0, 20);
  Serial.printf("Setting volume to %d\n", level);

  for (int i = 0; i < 20; i++) {
    sendConsumer(0x00EA);
    delay(50);
    sendKeyboardRaw(0x00);
  }
  for (int i = 0; i < level; i++) {
    sendConsumer(0x00E9);
    delay(50);
    sendKeyboardRaw(0x00);
  }
}

// ---- BLE Server Callbacks ----
class ServerCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer* pServer_, NimBLEConnInfo& connInfo) override {
    (void)pServer_;
    projectorConnected = true;
    currentState = State::kIdle;
    ledOff();

    // advertising больше не нужно
    pServer->getAdvertising()->stop();

    // Если команда пришла до подключения — отправим её сейчас.
    if (pendingType != PendingType::kNone) {
      PendingType t = pendingType;
      uint16_t v = pendingValue;
      pendingType = PendingType::kNone;
      pendingValue = 0;
      if (t == PendingType::kKeyboard) ProjectorService::sendKeyboard((uint8_t)(v & 0xFF));
      else if (t == PendingType::kConsumer) ProjectorService::sendConsumer(v);
    }

    MqttService::publishProjectorConnected(true);
  }

  void onDisconnect(NimBLEServer* pServer_, NimBLEConnInfo& connInfo, int reason) override {
    (void)pServer_;
    (void)connInfo;
    projectorConnected = false;
    ledOn();
    MqttService::publishProjectorConnected(false);
    Serial.printf("PROJECTOR: Disconnected (reason=%d), will reconnect...\n", reason);
    currentState = State::kReconnecting;
    reconnectStartTime = millis();
  }
};

// ---- BLE Init ----
void ProjectorService::bleInit() {
  // Инициализация BLE-стека NimBLE и базовые параметры устройства.
  NimBLEDevice::init("dangbei_remote");
  // Усиление передатчика (чтобы стабильнее работала связь с проектором).
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  // Включаем безопасность (концептуально: защищённые соединения/согласование).
  NimBLEDevice::setSecurityAuth(true, false, false);
  // IO-capability: без ввода/вывода (для сопоставления/спаривания по умолчанию).
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  // Создаём BLE-сервер и привязываем обработчики connect/disconnect.
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // HID-over-GATT сервис (UUID 0x1812) — проектор подключается как BLE HID устройство.
  NimBLEService* pHid = pServer->createService("1812");

  // HID Information (2A4A) — основные сведения о профиле HID.
  pHid->createCharacteristic("2A4A", NIMBLE_PROPERTY::READ)
      ->setValue((uint8_t[]){0x11, 0x01, 0x00, 0x01}, 4);
  // HID Report Descriptor (2A4B) — дескриптор отчётов (consumer + keyboard).
  pHid->createCharacteristic("2A4B", NIMBLE_PROPERTY::READ)
      ->setValue(hidReportDescriptor, sizeof(hidReportDescriptor));
  // HID Control Point (2A4C) — позволяет управляющие команды (без необходимости тут).
  pHid->createCharacteristic("2A4C", NIMBLE_PROPERTY::WRITE_NR);
  // HID Protocol Mode (2A4E) — режим протокола (обычно 0x01).
  auto pProto = pHid->createCharacteristic("2A4E", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);
  pProto->setValue((uint8_t[]){0x01}, 1);

  // Отчёт consumer controls (2A4D) — notify-канал для HID Consumer report.
  // Отличаем consumer/keyboard через Report Reference (descriptor 2908).
  pConsumerReport = pHid->createCharacteristic("2A4D", NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  auto pRepRef1 = new NimBLEDescriptor("2908", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE, 2);
  pRepRef1->setValue((uint8_t[]){0x01, 0x01}, 2);
  pConsumerReport->addDescriptor(pRepRef1);

  // Отчёт keyboard (2A4D) — второй notify-канал, также через Report Reference (2908).
  pKeyboardReport = pHid->createCharacteristic("2A4D", NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
  auto pRepRef2 = new NimBLEDescriptor("2908", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE, 2);
  pRepRef2->setValue((uint8_t[]){0x0A, 0x01}, 2);
  pKeyboardReport->addDescriptor(pRepRef2);

  // Запускаем HID сервис.
  pHid->start();

  // Сервис батареи (Battery Service 0x180F) — чтобы проектор мог читать/подписываться на заряд.
  NimBLEService* pBat = pServer->createService("180F");
  pBat->createCharacteristic("2A19", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY)
      ->setValue((uint8_t[]){0x64}, 1);
  pBat->start();

  // Device Information (0x180A) — даём «device type» (2A50) для идентификации.
  NimBLEService* pDev = pServer->createService("180A");
  pDev->createCharacteristic("2A50", NIMBLE_PROPERTY::READ)
      ->setValue((uint8_t[]){0x02, 0x54, 0x2B, 0x00, 0x16, 0x00, 0x00}, 7);
  pDev->start();

  // Готовим обычную рекламу: паринг/публичные сервисы, чтобы проектор мог найти ESP.
  setNormalAdvertising();
  Serial.println("PROJECTOR: BLE server initialized.");
}

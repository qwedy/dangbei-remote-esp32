// Попытка организовать проброс нажатий кнопок с физического пульта -> ESP -> проектор
// ESP выступает в качестве BLE клиента, поключается к пульту
//
// По большей части провальная идея, пульт дисконнектится каждые 10-50 секунд бездействия
// и чтоб переподключиться надо нажать кнопку на пульте, ждать ~1-5 секунд  коннектапрежде чем действительно управлять
// Или постоянно бомбить пульт  для поддерки связи, но это быстро посадит батарейки пульта  
// Или снифферить трафик между пультом и проектором и вычислять логику работы по дефолту

// Но заниматься такой фигнёй ради функции "поиска пульта" не хочется

#include "pult_state.h"
#include "services.h"

// ---- Настройки пульта ----
#define FIND_PULT_REMOTE_ADDRESS "70:50:E7:85:2F:DF" // MAC адрес физического пульта
#define FIND_PULT_PAIR_TIMEOUT_MS 10000
#define FIND_PULT_FAST_PAIR_WAIT_MS 1200
#define PULT_AUTOSTART_DELAY_MS 1200

// Опрос батареи пульта (GATT 180F/2A19) служит keep-alive:
// периодический GATT-read сбрасывает таймер бездействия пульта,
// не давая ему уйти в deep sleep. Интервал 5 с — с запасом меньше
// наблюдаемого минимального таймаута (~10 с). Энергозатраты ~2 мс
// радио каждые 5 с — пренебрежимо мало.
#define REMOTE_BATTERY_POLL_MS 5000u

PultState gPult;
static uint32_t pultAutoStartReadyAtMs = 0;

/** Процент батареи физического пульта (BLE), -1 = неизвестно / нет связи. */
volatile int16_t remotePultBatteryPercent = -1;
uint32_t remoteBatteryLastPollMs = 0;


// BLE client callbacks для линка ESP <-> физический пульт (find / pult).
class PultFindClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) override;
  void onDisconnect(NimBLEClient* pClient, int reason) override;
  void onAuthenticationComplete(NimBLEConnInfo& connInfo) override;
};

// Обработка события подключения к физическому пульту
void PultFindClientCallbacks::onConnect(NimBLEClient* pClient) {
  (void)pClient;
  uint32_t now = millis();
  gPult.findLinkUp = true;
  gPult.connectedAtMs = now;
  uint32_t reconnectMs = (gPult.connectStartedMs > 0 && now >= gPult.connectStartedMs)
                           ? (now - gPult.connectStartedMs)
                           : 0;
  Serial.printf("PULT: connected at=%lu ms, reconnect_ms=%lu\n",
                (unsigned long)now, (unsigned long)reconnectMs);
  remoteBatteryLastPollMs = millis();
  PultService::pollRemoteBattery();
  MqttService::publishRemotePultBatteryState();
  MqttService::publishPultConnected(true);
}

// Обработка события отключения от физического пульта
void PultFindClientCallbacks::onDisconnect(NimBLEClient* pClient, int reason) {
  (void)pClient;
  uint32_t now = millis();
  gPult.findLinkUp = false;
  gPult.hidSubscribed = false;
  gPult.disconnectedAtMs = now;
  uint32_t sessionMs = (gPult.connectedAtMs > 0 && now >= gPult.connectedAtMs)
                         ? (now - gPult.connectedAtMs)
                         : 0;
  if (sessionMs >= 5000) gPult.hadStableSession = true;
  Serial.printf("PULT: disconnected at=%lu ms reason=%d (0x%X) session_ms=%lu\n",
                (unsigned long)now, reason, (unsigned)reason, (unsigned long)sessionMs);
  if (gPult.enabled) {
    Serial.println("PULT: link lost, waiting for wake and reconnect...");
  } else {
    Serial.println("PULT: OFF");
  }
  remotePultBatteryPercent = -1;
  MqttService::publishRemotePultBatteryState();
  MqttService::publishPultConnected(false);
}

// Обработка события успешной аутентификации
void PultFindClientCallbacks::onAuthenticationComplete(NimBLEConnInfo& connInfo) {
  gPult.findPairingDone = true;
  if (connInfo.isEncrypted()) {
    Serial.println("PULT: pairing success");
  } else {
    Serial.println("PULT: pairing failed");
  }
}

// BLE scan callback: обнаружение рекламы пульта для мгновенного реконнекта.
// Вместо периодических слепых connect() ESP непрерывно сканирует эфир
// и подключается сразу, как только пульт начинает рекламироваться.
class PultScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override {
    if (device->getAddress() == NimBLEAddress(FIND_PULT_REMOTE_ADDRESS, BLE_ADDR_PUBLIC)
        && device->isConnectable()) {
      gPult.advSeen = true;
      NimBLEDevice::getScan()->stop();
    }
  }
};

static PultScanCallbacks pultScanCbs;

void PultService::startScan() {
  gPult.advSeen = false;
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&pultScanCbs, false);
  pScan->setActiveScan(false);
  pScan->setInterval(80);   // 50 ms
  pScan->setWindow(80);     // 50 ms — 100% duty cycle
  if (pScan->start(0, false)) {
    Serial.println("PULT: scan started (waiting for advertisement)");
  } else {
    Serial.println("PULT: scan start failed");
  }
}

void PultService::stopScan() {
  if (NimBLEDevice::getScan()->isScanning()) {
    NimBLEDevice::getScan()->stop();
  }
  gPult.advSeen = false;
}

// Обработка событий получения уведомлений от физического пульта
static void pultOnNotify(NimBLERemoteCharacteristic* pChr, uint8_t* data, size_t len, bool isNotify) {
  (void)isNotify;
  if (!pChr || !data || len == 0) return;
  if (!gPult.hidSubscribed) return;
  PultService::pultRemotePacket(data, len);
}

// Подключение к физическому пульту и начало pairing
bool PultService::connectAndPair() {
  if (gPult.client && gPult.client->isConnected()) {
    gPult.findLinkUp = true;
    gPult.findPairingDone = true;
    Serial.println("PULT: already connected (skip reconnect)");
    return true;
  }

  gPult.findPairingDone = false;
  gPult.findLinkUp = false;
  // Создание нового клиента если он не существует
  if (gPult.client == nullptr) {
    gPult.client = NimBLEDevice::createClient();
    gPult.client->setClientCallbacks(new PultFindClientCallbacks());
  }
  // Со scan-based reconnect мы вызываем connect() только когда точно видели
  // connectable advertisement. Реальное время установления связи ~2.6-2.9 с
  // (ожидание следующего adv ~2 с + GAP handshake ~0.7 с). Таймаут 4 с даёт
  // запас ~1 с сверх наблюдаемого максимума.
  gPult.client->setConnectTimeout(4000);

  gPult.connectStartedMs = millis();
  Serial.printf("PULT: connecting at=%lu ms...\n", (unsigned long)gPult.connectStartedMs);
  if (!gPult.client->connect(NimBLEAddress(FIND_PULT_REMOTE_ADDRESS, BLE_ADDR_PUBLIC), false)) {
    uint32_t failMs = millis();
    uint32_t spentMs = (failMs >= gPult.connectStartedMs) ? (failMs - gPult.connectStartedMs) : 0;
    Serial.printf("PULT: connection failed at=%lu ms, connect_ms=%lu\n",
                  (unsigned long)failMs, (unsigned long)spentMs);
    return false;
  }

  if (!gPult.hadStableSession) {
    NimBLERemoteService* vendorSvc = gPult.client->getService(
      NimBLEUUID("0000fef6-0000-1000-8000-00805f9b34fb"));
    if (vendorSvc) {
      NimBLERemoteCharacteristic* ctrl = vendorSvc->getCharacteristic(
        NimBLEUUID("005f0002-2ff2-4ed5-b045-4c7463617865"));
      if (ctrl && ctrl->canWrite()) {
        uint8_t cmd[1] = {0x01};
        ctrl->writeValue(cmd, 1, true);
        Serial.println("PULT: vendor activated");
      }
    }

    gPult.client->secureConnection();
    const uint32_t pairWaitMs =
        gPult.enabled ? FIND_PULT_FAST_PAIR_WAIT_MS : FIND_PULT_PAIR_TIMEOUT_MS;
    Serial.printf("PULT: waiting pairing (timeout_ms=%lu)", (unsigned long)pairWaitMs);
    uint32_t t = millis();
    while (!gPult.findPairingDone && (millis() - t) < pairWaitMs) {
      Serial.print(".");
      delay(200);
    }
    Serial.println();

    if (!gPult.findPairingDone) {
      if (gPult.enabled) {
        Serial.println("PULT: fast-pair timeout, continue with connected link");
        return true;
      }
      Serial.println("PULT: pairing timeout");
      return false;
    }
    delay(300);
  } else {
    Serial.println("PULT: trusted reconnect (skip vendor/secure wait)");
  }
  return true;
}

// Подключение к физическому пульту и подписка на уведомления
bool PultService::connectPultAndSubscribeHid() {
  if (gPult.client && gPult.client->isConnected() && gPult.hidSubscribed) {
    Serial.println("PULT: already connected+subscribed (skip resubscribe)");
    return true;
  }
  if (!connectAndPair()) return false;
  gPult.hidSubscribed = false;
  return subscribePultHid();
}

// Переключение зуммера физического пульта
bool PultService::buzzerSwitch() {
  if (!gPult.client) return false;
  NimBLERemoteService* svc = gPult.client->getService(NimBLEUUID((uint16_t)0xFFF0));
  if (!svc) {
    Serial.println("PULT: FFF0 service not found");
    return false;
  }
  NimBLERemoteCharacteristic* writeChar = svc->getCharacteristic(NimBLEUUID((uint16_t)0xFFF2));
  NimBLERemoteCharacteristic* readChar = svc->getCharacteristic(NimBLEUUID((uint16_t)0xFFF3));
  if (!writeChar) {
    Serial.println("PULT: FFF2 char not found");
    return false;
  }
  uint8_t current = 0x02;
  if (readChar && readChar->canRead()) {
    std::string v = readChar->readValue();
    if (!v.empty()) current = (uint8_t)v[0];
  }
  uint8_t next = (current == 0x01) ? 0x02 : 0x01;
  uint8_t cmd[1] = {next};
  writeChar->writeValue(cmd, 1, false);
  Serial.printf("PULT: buzzer %s\n", (next == 0x01) ? "ON" : "OFF");
  return true;
}

// Обработка полученных пакетов от физического пульта
void PultService::pultRemotePacket(const uint8_t* data, size_t len) {
  if (!gPult.enabled) return;
  if (len == 3) {
    uint16_t usage = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    Serial.printf("PULT: RX consumer: %02X %02X %02X (usage=0x%04X)\n", data[0], data[1], data[2], usage);
    if (usage == 0xCCCC) return;
    enqueueForward(PendingType::kConsumer, usage);
    return;
  }
  if (len == 7) {
    uint8_t keycode = data[2];
    Serial.printf("PULT: RX keyboard: %02X %02X %02X %02X %02X %02X %02X (key=0x%02X)\n",
                  data[0], data[1], data[2], data[3], data[4], data[5], data[6], keycode);
    enqueueForward(PendingType::kKeyboard, keycode);
    return;
  }
  Serial.printf("PULT: RX unknown len=%u: ", (unsigned)len);
  for (size_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();
}

// Очередь forwarding packets
void PultService::enqueueForward(PendingType type, uint16_t value) {
  gPult.forwardType = type;
  gPult.forwardValue = value;
  gPult.forwardPending = true;
}

// Обработка очереди forwarding packets
void PultService::processForward() {
  if (!gPult.forwardPending) return;
  noInterrupts();
  PendingType type = gPult.forwardType;
  uint16_t value = gPult.forwardValue;
  gPult.forwardType = PendingType::kNone;
  gPult.forwardValue = 0;
  gPult.forwardPending = false;
  interrupts();
  if (type == PendingType::kNone) return;

  if (type == PendingType::kConsumer) {
    if (projectorConnected) {
      Serial.printf("PULT: TX consumer -> projector: 0x%04X\n", value);
      ProjectorService::sendConsumerRaw(value);
    } else {
      if (value == 0x0000) return;
      Serial.printf("PULT: QUEUE consumer: 0x%04X\n", value);
      ProjectorService::queueCommandAndConnect(PendingType::kConsumer, value);
    }
  } else if (type == PendingType::kKeyboard) {
    uint8_t key = (uint8_t)(value & 0xFF);
    if (projectorConnected) {
      Serial.printf("PULT: TX keyboard -> projector: 0x%02X\n", key);
      ProjectorService::sendKeyboardRaw(key);
    } else {
      if (key == 0x00) return;
      if (key == 0x66) {
        Serial.println("PULT: POWER key while disconnected -> startWakeUp()");
        ProjectorService::startWakeUp();
        return;
      }
      Serial.printf("PULT: QUEUE keyboard: 0x%02X\n", key);
      ProjectorService::queueCommandAndConnect(PendingType::kKeyboard, key);
    }
  }
}

// Подписка на уведомления от физического пульта
bool PultService::subscribePultHid() {
  if (!gPult.client || !gPult.client->isConnected()) return false;
  if (gPult.hidSubscribed) return true;
  NimBLERemoteService* hidSvc = gPult.client->getService(NimBLEUUID((uint16_t)0x1812));
  if (!hidSvc) {
    Serial.println("PULT: HID service 1812 not found");
    return false;
  }
  auto chars = hidSvc->getCharacteristics(false);
  if (chars.empty()) {
    chars = hidSvc->getCharacteristics(true);
  }
  Serial.printf("PULT: HID chars count=%u (cached=%s) at=%lu ms\n",
                (unsigned)chars.size(),
                chars.empty() ? "miss" : "hit",
                (unsigned long)millis());
  uint8_t subscribedCount = 0;
  for (auto* c : chars) {
    if (!c) continue;
    if (c->getUUID().equals(NimBLEUUID((uint16_t)0x2A4D)) && c->canNotify()) {
      if (c->subscribe(true, pultOnNotify, true)) subscribedCount++;
    }
  }
  if (subscribedCount == 0) {
    Serial.println("PULT: no notify report subscribed");
    return false;
  }
  gPult.hidSubscribed = true;
  Serial.printf("PULT: subscribed reports=%u at=%lu ms\n",
                subscribedCount, (unsigned long)millis());
  return true;
}

// Опрос батареи физического пульта
void PultService::pollRemoteBattery() {
  if (!gPult.client || !gPult.client->isConnected()) {
    remotePultBatteryPercent = -1;
    return;
  }
  NimBLERemoteService* batSvc = gPult.client->getService(NimBLEUUID((uint16_t)0x180F));
  if (!batSvc) return;
  NimBLERemoteCharacteristic* batChr = batSvc->getCharacteristic(NimBLEUUID((uint16_t)0x2A19));
  if (!batChr || !batChr->canRead()) return;
  std::string v = batChr->readValue();
  if (v.empty()) return;
  int pct = (uint8_t)v[0];
  if (pct > 100) pct = 100;
  remotePultBatteryPercent = (int16_t)pct;
}

// Инициализация пульта (автостарт при загрузке ESP)
void PultService::init() {
  gPult.enabled = true;
  pultAutoStartReadyAtMs = millis() + PULT_AUTOSTART_DELAY_MS;
  Serial.println("PULT: auto-start ON");
  Serial.printf("PULT: auto-start delayed by %lu ms\n", (unsigned long)PULT_AUTOSTART_DELAY_MS);
}

// Основной цикл пульта: forward, опрос батареи, BLE scan/reconnect
void PultService::loopTick() {
  processForward();

  // Периодический опрос батареи пульта и публикация в MQTT при изменении
  if (gPult.client && gPult.client->isConnected()) {
    uint32_t tb = millis();
    if (remoteBatteryLastPollMs == 0 || (tb - remoteBatteryLastPollMs) >= REMOTE_BATTERY_POLL_MS) {
      remoteBatteryLastPollMs = tb;
      int16_t prevBat = remotePultBatteryPercent;
      pollRemoteBattery();
      if (remotePultBatteryPercent != prevBat) {
        MqttService::publishRemotePultBatteryState();
      }
    }
  } else {
    remoteBatteryLastPollMs = 0;
  }

  // Подключение к физическому пульту через BLE scan.
  // Пульт отключается после бездействия ~10 с, при нажатии кнопки снова
  // начинает рекламироваться. Пассивный scan ловит рекламу мгновенно,
  // после чего сразу инициируется connect — задержка ~100-200 мс вместо 1-3 с.
  uint32_t t = millis();
  if (gPult.enabled && t >= pultAutoStartReadyAtMs &&
      (!gPult.client || !gPult.client->isConnected())) {
    if (gPult.advSeen) {
      gPult.advSeen = false;
      Serial.printf("PULT: advertisement detected at=%lu ms, connecting...\n", (unsigned long)t);
      if (connectPultAndSubscribeHid()) {
        Serial.println("PULT: connected via scan");
      } else {
        Serial.println("PULT: connect after scan failed, restarting scan");
        startScan();
      }
    } else if (!NimBLEDevice::getScan()->isScanning()) {
      startScan();
    }
  }
}

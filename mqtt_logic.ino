#include "services.h"
#include "pult_state.h"

static void hidConsumerNowOrQueue(uint16_t usage) {
  if (projectorConnected) {
    ProjectorService::sendConsumer(usage);
  } else {
    ProjectorService::queueCommandAndConnect(PendingType::kConsumer, usage);
  }
}

static void hidKeyboardNowOrQueue(uint8_t keycode) {
  if (projectorConnected) {
    ProjectorService::sendKeyboard(keycode);
  } else {
    ProjectorService::queueCommandAndConnect(PendingType::kKeyboard, keycode);
  }
}

void MqttService::handleCommand(const char* button) {
  Serial.printf("CMD: %s\n", button);

  // Если соединение активно — считаем, что проектор включен (тогглим Power).
  // Если соединения нет — шлём wake advertising, чтобы проектор проснулся и подключился.
  if (strcmp(button, "power") == 0) {
    if (projectorConnected) {
      ProjectorService::sendKeyboard(0x66);
    } else {
      ProjectorService::startWakeUp();
    }
  }
  else if (strcmp(button, "volume_up")   == 0) hidConsumerNowOrQueue(0x00E9);
  else if (strcmp(button, "volume_down") == 0) hidConsumerNowOrQueue(0x00EA);
  else if (strcmp(button, "home")        == 0) hidConsumerNowOrQueue(0x0223);
  else if (strcmp(button, "menu")        == 0) hidConsumerNowOrQueue(0x0040);
  else if (strcmp(button, "up")          == 0) hidKeyboardNowOrQueue(0x52);
  else if (strcmp(button, "down")        == 0) hidKeyboardNowOrQueue(0x51);
  else if (strcmp(button, "left")        == 0) hidKeyboardNowOrQueue(0x50);
  else if (strcmp(button, "right")       == 0) hidKeyboardNowOrQueue(0x4F);
  else if (strcmp(button, "ok")          == 0) hidKeyboardNowOrQueue(0x28);
  else if (strcmp(button, "back")        == 0) hidKeyboardNowOrQueue(0x29);
  else if (strcmp(button, "quick_menu")  == 0) hidKeyboardNowOrQueue(0x3F);
  else if (strcmp(button, "find_pult")   == 0) {
    Serial.println("MQTT: Buzzer toggle");
    PultService::buzzerSwitch();
  }
  else if (strcmp(button, "remote_pult_on") == 0) {
    gPult.enabled = true;
    if (PultService::connectPultAndSubscribeHid()) {
      Serial.println("PULT: ON");
    } else {
      Serial.println("PULT: ON requested, scan will start");
    }
  }
  else {
    // Проверяем команду установки громкости: "volume_set_N"
    if (strncmp(button, "volume_set_", 11) == 0) {
      int level = atoi(button + 11);
      ProjectorService::setVolume(level);
    } else {
      Serial.printf("Unknown: %s\n", button);
    }
  }
}

// Обработчик входящего MQTT:
// извлекает `<button>` из `/devices/{DEVICE_ID}/controls/<button>/on` и вызывает handleCommand().
void MqttService::callback(char* topic, uint8_t* payload, unsigned int length) {
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
      handleCommand(cmd);
      return;
    }
    // Для бинарного контрола volume_change:
    // payload "1" -> громче, payload "0" -> тише.
    if (button == "volume_change") {
      char buf[8];
      unsigned int n = (length < sizeof(buf) - 1) ? length : (sizeof(buf) - 1);
      memcpy(buf, payload, n);
      buf[n] = '\0';
      int dir = atoi(buf);
      if (dir == 1) {
        hidConsumerNowOrQueue(0x00E9);
      } else if (dir == 0) {
        hidConsumerNowOrQueue(0x00EA);
      } else {
        Serial.printf("Unknown volume_change payload: %s\n", buf);
      }
      return;
    }

    handleCommand(button.c_str());
  }
}

// ---- MQTT метаданные ----
// Публикуем retained-метаданные для интерфейса/автоконфигурации:
// тип/заголовок/порядок для каждой кнопки и топик, куда отправлять on-события.
void MqttService::publishMeta() {
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
    {"volume_change", "Volume +/-", 10},
    {"find_pult",   "Find pult",  12},
    {"remote_pult_on", "Remote pult ON", 13},
  };

  char topic[128];
  char val[8];
  char buf[12];
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


  // Батарея физического пульта (0–100 % по GATT Battery Level 2A19), только отображение
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/remote_pult_battery/meta/type", "text", true);
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/remote_pult_battery/meta/title", "Батарея пульта, %", true);
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/remote_pult_battery/meta/order", "13", true);
  MqttService::publishRemotePultBatteryState();

  // BLE-связь с проектором (0/1), только отображение — как у remote_pult_battery
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/connected/meta/type", "text", true);
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/connected/meta/title", "Проектор подключён", true);
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/connected/meta/order", "17", true);
    MqttService::publishProjectorConnected(projectorConnected);

  // BLE-связь с физическим пультом (0/1), только отображение — как у remote_pult_battery
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/pult_connected/meta/type", "text", true);
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/pult_connected/meta/title", "Пульт подключён", true);
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/pult_connected/meta/order", "18", true);
  bool pultConnectedNow = (gPult.client && gPult.client->isConnected());
  MqttService::publishPultConnected(pultConnectedNow);


  Serial.println("MQTT: meta published.");
}


void MqttService::publishPultConnected(bool connected) {
  if (!mqttClient.connected()) return;
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/pult_connected",
    connected ? "1" : "0", true);
}

void MqttService::publishProjectorConnected(bool connected) {
  if (!mqttClient.connected()) return;
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/connected",
    connected ? "1" : "0", true);
    // обратная связь для power кнопки (чтоб УД видел состояние проектора)
    mqttClient.publish(
      "/devices/" DEVICE_ID "/controls/power",
      connected ? "1" : "0", true);
}

void MqttService::publishRemotePultBatteryState() {
  if (!mqttClient.connected()) return;
  char buf[12];
  if (remotePultBatteryPercent < 0) {
    snprintf(buf, sizeof(buf), "--");
  } else {
    snprintf(buf, sizeof(buf), "%d", (int)remotePultBatteryPercent);
  }
  mqttClient.publish(
    "/devices/" DEVICE_ID "/controls/remote_pult_battery",
    buf, true);
}

void MqttService::connect() {
  // Бесконечно пытаемся подключиться к брокеру.
  // После успешного подключения публикуем метаданные (retained), чтобы UI знал кнопки.
  while (!mqttClient.connected()) {
    Serial.println("MQTT: connecting...");
    if (mqttClient.connect(DEVICE_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println("MQTT: connected.");
      publishMeta();
    } else {
      Serial.printf("MQTT: failed rc=%d, retry 3s\n", mqttClient.state());
      delay(3000);
    }
  }
}

void MqttService::connectWifi() {
  // Подключаемся к WiFi с таймаутом 15 секунд.
  Serial.printf("MQTT: WiFi connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nMQTT: WiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nMQTT: WiFi failed!");
  }
}


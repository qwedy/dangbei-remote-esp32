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
#include "services.h"
//#include "pult_state.h"

// ---- Настройки ----
#define WIFI_SSID        "RT-GPON-N11"
#define WIFI_PASSWORD    "limit678"
#define MQTT_HOST        "192.168.0.15"
#define MQTT_PORT        1883
#define MQTT_USER        ""
#define MQTT_PASS        ""
#define DEVICE_ID        "dangbei_remote"

#define LED_PIN           2 // Пин лампочки для индикации состояния подключения к проектору
#define BUTTON_PIN        0 // Кнопка ручного перезапуска подключения к проектору

// Тайминги FSM (pairing) и wake-окна.
#define PAIR_TIMEOUT_MS       30000 // Таймаут pairing (30 секунд) для подключение к проектору
#define WAKE_DURATION_MS      5000  // Таймаут wake (5 секунд) для подключение к проектору
#define RECONNECT_DELAY_MS    15000 // Интервал повторных попыток переподключения (15 секунд)

// MQTT клиент (глобальная переменная должна быть видна в mqtt_logic.ino).
WiFiClient espClient;
PubSubClient mqttClient(espClient);

static void mqttMessageCallback(char* topic, uint8_t* payload, unsigned int length) {
  MqttService::callback(topic, payload, length);
}


void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Dangbei BLE -> MQTT Gateway ===");

  // 1) GPIO (LED/кнопка)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // 2) WiFi + MQTT
  MqttService::connectWifi();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttMessageCallback);
  mqttClient.setKeepAlive(60);
  MqttService::connect();

  // 3) BLE server (HID) и старт pairing (коннект к проектору)
  ProjectorService::bleInit();
  ProjectorService::startPairing();

  // 4) Автостарт pult (проброс оригинального пульта через ESP)
  //PultService::init();
}


void loop() {
  // держим MQTT в рабочем состоянии
  if (!mqttClient.connected()) MqttService::connect();
  mqttClient.loop();

  // Логика физического пульта (BLE client): forward, батарея, scan/reconnect
  //PultService::loopTick();

  // Кнопка BOOT (реконнект к проектору)
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("Button pressed — restarting pairing");
      if (!projectorConnected && !ProjectorService::projectorIsInWakeOrPairing()) {
        ProjectorService::startPairing();
      }
      while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    }
  }


  ProjectorService::projectorLoopTick();

  delay(10);
}
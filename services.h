#pragma once

#include <WiFi.h>
#include <PubSubClient.h>
#include <cstddef>
#include <cstdint>

// ---- Common types ----
enum class PendingType : uint8_t { kNone, kKeyboard, kConsumer };

// Связь с проектором (HID-ready) — используется для запрета отправки, когда линка ещё нет.
extern bool projectorConnected;

// MQTT client (объявление; фактическая инициализация в `pult.ino`).
extern PubSubClient mqttClient;

// ---- Projector (ESP <-> проектор, HID server) ----
class ProjectorService {
 public:
  static void bleInit();
  static void startPairing();
  static void stopPairing();
  static void startWakeUp();
  static void stopWakeUp();
  static void projectorLoopTick();
  static bool projectorIsInWakeOrPairing();

  static void sendConsumer(uint16_t usage);
  static void sendConsumerRaw(uint16_t usage);
  static void sendKeyboard(uint8_t keycode);
  static void sendKeyboardRaw(uint8_t keycode);

  static void setVolume(int level);

  static void queueCommandAndConnect(PendingType type, uint16_t value);
};

// ---- Pult (ESP <-> физический пульт, HID client) ----
class PultService {
 public:
  static void init();
  static void loopTick();
  static bool connectAndPair();
  static bool connectPultAndSubscribeHid();
  static bool buzzerSwitch();
  static void processForward();
  static bool subscribePultHid();
  static void pultRemotePacket(const uint8_t* data, size_t len);
  static void enqueueForward(PendingType type, uint16_t value);
  static void pollRemoteBattery();
  static void startScan();
  static void stopScan();
};

// ---- MQTT ----
class MqttService {
 public:
  static void handleCommand(const char* button);
  static void callback(char* topic, uint8_t* payload, unsigned int length);
  static void publishMeta();
  static void publishRemotePultBatteryState();
  static void publishProjectorConnected(bool connected);
  static void publishPultConnected(bool connected);
  static void connect();
  static void connectWifi();
};
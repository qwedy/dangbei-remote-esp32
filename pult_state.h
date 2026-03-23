#pragma once

#include <cstdint>
#include "services.h"

class NimBLEClient;

/** Состояние BLE client (ESP ↔ физический пульт) и очереди проброса HID. */
struct PultState {
  NimBLEClient* client = nullptr;
  volatile bool findLinkUp = false;
  volatile bool findPairingDone = false;
  bool enabled = false;
  bool hidSubscribed = false;
  bool hidDiscoveryCached = false;
  bool hadStableSession = false;
  uint32_t connectStartedMs = 0;
  uint32_t connectedAtMs = 0;
  uint32_t disconnectedAtMs = 0;
  volatile bool advSeen = false;
  volatile bool forwardPending = false;
  volatile PendingType forwardType = PendingType::kNone;
  volatile uint16_t forwardValue = 0;
};

extern PultState gPult;
extern volatile int16_t remotePultBatteryPercent;
extern uint32_t remoteBatteryLastPollMs;

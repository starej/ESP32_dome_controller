// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Jurij Stare

#include <Wire.h>
#include <ArduinoJson.h>
#include <Preferences.h> // Native ESP32 NVS: only used for WiFi credentials
#include <WiFi.h>        // Native ESP32 WiFi library (station mode, DHCP)
#include <WiFiUdp.h>
#include <ESPmDNS.h>     // Advertises the controller as http://<hostname>.local on the LAN
#include <BLEDevice.h>   // BLE GATT server used as an alternative way to set network config
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "driver/pulse_cnt.h"
// OTA update helper
#include <Update.h>

// firmware versioning
#define FIRMWARE_VERSION "v1.0.11 (" __DATE__ " " __TIME__ ")"

// FRAM record layouts (defined at the top so the Arduino IDE auto-generated function
// prototypes can reference them). See the FRAM STORAGE section below for the memory map.
struct __attribute__((packed)) FramConfig {
  uint32_t magic;
  int32_t  ticksPerRot;
  int32_t  ticksLimitLow;
  int32_t  ticksLimitHigh;
  int32_t  inertiaTicks;
  double   parkAzimuth;   // degrees
  uint32_t crc;
};

// Extra tuning settings in their own record (so adding settings never invalidates the main config record)
struct __attribute__((packed)) FramExtra {
  uint32_t magic;
  int32_t  watchdogMinutes;    // close shutter when no client has talked to the controller for this long (0 = disabled)
  int32_t  shutterTimeoutSec;  // max shutter travel time before the motor is stopped and ERROR is raised
  uint32_t crc;
};

struct __attribute__((packed)) FramState {
  uint32_t magic;
  uint32_t seq;           // increments on every save; highest valid seq wins at boot
  int32_t  azTicks;
  int32_t  shutterState;
  uint32_t crc;
};


// ===== DIAGNOSTIC MODE =====
// When set to 1, all persistent *writes* (FRAM and NVS) are skipped (reads at boot still
// happen so the controller still loads its saved config).
#define DIAGNOSTIC_DISABLE_NVS_WRITES 0


// VERIFIED WAVESHARE HARDWARE PIN DEFINITIONS
// Ethernet (W5500) support has been removed — WiFi only. This board has no dedicated
// power/reset GPIO for the onboard W5500, so these pins are only used once at boot
// (see setup()) to put the chip into its lowest-power idle state: CS held deselected
// and the SPI lines left undriven so nothing ever clocks or talks to it.
#define ETH_SCLK 15
#define ETH_MISO 14
#define ETH_MOSI 13
#define ETH_INT 12
#define ETH_CS 16
#define TCA9554_ADDRESS 0x20
#define MAX_LOG_LINES 15

// Omron E6B2-CWZ6C rotary encoder wiring (2-channel quadrature)
#define DI4_ENCODER_A_PIN 7
#define DI5_ENCODER_B_PIN 8

String webLogs[MAX_LOG_LINES];
int logCount = 0;

// Network Configuration (WiFi only)
String wifiSSID = "";
String wifiPassword = "";
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000; // 20s to get a DHCP lease before giving up

// If no WiFi credentials are saved yet (first boot) or the saved credentials fail to
// connect, the controller falls back to broadcasting its own open setup Access Point
// so the user can browse to the dashboard and enter real WiFi credentials.
bool wifiApFallback = false;
const char* AP_SSID = "DomeController-Setup"; // Open network, no password

// mDNS hostname: the controller will be reachable at http://<this>.local/ on the LAN
// (works alongside WiFi and the setup AP — no need to hunt for the DHCP IP).
const char* MDNS_HOSTNAME = "domecontroller";

// --- BLE Network Configuration (alternative to the setup AP / web form) ---
// A phone connects over Bluetooth LE (no need to leave your home WiFi network to do it,
// unlike the setup AP) and writes a small JSON payload to set the WiFi config.
// Any generic BLE explorer app (nRF Connect, LightBlue, etc.) can talk to this — see the
// usage notes above setupBLE() below.
#define BLE_DEVICE_NAME       "DomeController"
#define BLE_SERVICE_UUID      "8e6a0001-9d34-4f77-8f2e-2d9c9a6d1a10"
#define BLE_CONFIG_CHAR_UUID  "8e6a0002-9d34-4f77-8f2e-2d9c9a6d1a10" // Write: JSON network config
#define BLE_STATUS_CHAR_UUID  "8e6a0003-9d34-4f77-8f2e-2d9c9a6d1a10" // Read/Notify: current status JSON
#define BLE_CONTROL_CHAR_UUID "8e6a0004-9d34-4f77-8f2e-2d9c9a6d1a10" // Write: emergency dome control commands
BLECharacteristic* bleStatusChar = nullptr;

// Alpaca listens standardly on Port 11111
WiFiServer alpacaServerWifi(11111);

// standard web server for status pages, configuration and basic controls
WiFiServer webServerWifi(80);

// Alpaca Discovery UDP Port
const unsigned int ALPACA_DISCOVERY_PORT = 32227;
WiFiUDP udpListenerWifi;

// Holder object used so we can hand back a Client* to the generic request-handling code
WiFiClient wifiWebClientHolder;
WiFiClient wifiAlpacaClientHolder;

// Buffer to hold incoming discovery packets
char packetBuffer[64];

// Returns the local IP address of whichever WiFi interface is currently active
// (station IP when connected to a router, or the softAP IP while in setup-AP fallback).
IPAddress getLocalIP() {
  return wifiApFallback ? WiFi.softAPIP() : WiFi.localIP();
}

// Returns a UDP* pointing at the discovery listener
UDP* getDiscoveryUDP() {
  return (UDP*)&udpListenerWifi;
}

// Accepts a new client on the web dashboard (port 80) server, returning nullptr if none waiting
Client* acceptWebClient() {
  wifiWebClientHolder = webServerWifi.available();
  return wifiWebClientHolder ? (Client*)&wifiWebClientHolder : nullptr;
}

// Accepts a new client on the Alpaca (port 11111) server, returning nullptr if none waiting
Client* acceptAlpacaClient() {
  wifiAlpacaClientHolder = alpacaServerWifi.available();
  return wifiAlpacaClientHolder ? (Client*)&wifiAlpacaClientHolder : nullptr;
}

// Preferences (NVS) instance - used only for WiFi credentials
Preferences preferences;

// Applies WiFi credentials to persistent storage. Shared by both the HTTP POST
// /api/v1/network/config handler and the BLE config characteristic, so both entry points
// behave identically. Does NOT reboot — the caller decides when to do that.
bool applyNetworkConfig(const String& ssid, const String& pass) {
  bool valid = (ssid.length() > 0);
  if (!valid) return false;

#if DIAGNOSTIC_DISABLE_NVS_WRITES
  logMessage("DIAG: NVS write skipped (disabled) for network-config");
  return true;
#else
  preferences.begin("network-config", false);
  preferences.putString("wifi_ssid", ssid);
  // Only overwrite the stored password if a new one was actually supplied, so an update
  // that omits the password field (e.g. re-saving the SSID) doesn't wipe it out.
  if (pass.length() > 0) {
    preferences.putString("wifi_pass", pass);
  }
  preferences.end();
  return true;
#endif
}

// Builds a small JSON status blob reporting the controller's current network state.
// Used by the BLE status characteristic so a phone can confirm what happened after a write.
String buildStatusJson() {
  JsonDocument doc;
  doc["mode"] = "wifi";
  doc["currentIp"] = getLocalIP().toString();
  doc["apFallback"] = wifiApFallback;
  doc["wifiSsid"] = wifiSSID;
  String out;
  serializeJson(doc, out);
  return out;
}

// Handles writes to the BLE config characteristic. Expects a JSON payload shaped like:
//   {"ssid":"MyNetwork","password":"secret"}
// On success, reboots the controller (same as the HTTP network config route) so the new
// setting takes effect.
class BLEConfigWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = String(characteristic->getValue().c_str());
    if (value.length() == 0) return;

    logMessage("BLE: Received network config write.");

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, value);
    if (err) {
      logMessage("BLE ERROR: Malformed JSON payload.");
      if (bleStatusChar) {
        bleStatusChar->setValue("{\"status\":\"error\",\"msg\":\"Malformed JSON.\"}");
        bleStatusChar->notify();
      }
      return;
    }

    String ssid = doc["ssid"] | "";
    String pass = doc["password"] | "";

    bool ok = applyNetworkConfig(ssid, pass);

    if (bleStatusChar) {
      bleStatusChar->setValue(ok
        ? "{\"status\":\"ok\",\"msg\":\"Network profile updated. Rebooting...\"}"
        : "{\"status\":\"error\",\"msg\":\"Missing required fields.\"}");
      bleStatusChar->notify();
    }

    if (ok) {
      logMessage("BLE: Switching to WiFi (DHCP) -> SSID: " + ssid);
      delay(500); // give the notification a moment to actually go out before rebooting
      // Persist current azimuth position before reboot so startup can restore it
      saveAzimuthToFlash();
      ESP.restart();
    } else {
      logMessage("BLE ERROR: Malformed network profile parameters.");
    }
  }
};

// Refreshes the status characteristic's value on demand whenever a client reads it.
class BLEStatusReadCallback : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *characteristic) override {
    characteristic->setValue(buildStatusJson().c_str());
  }
};

// BLE server callbacks: ensure advertising is restarted after a client disconnects
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    logMessage("BLE: Client connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    logMessage("BLE: Client disconnected — restarting advertising");
    // Restart advertising so other centrals can find the device again
    BLEDevice::startAdvertising();
  }
};

// Emergency/local dome control over BLE — a backup control path for when WiFi/the router
// is down but you're standing right at the dome. Expects a JSON payload shaped like:
//   {"cmd":"stop"}          -- abort any slew/park/home in progress immediately
//   {"cmd":"park"}          -- start the park sequence
//   {"cmd":"findhome"}      -- start the home-finding sequence
//   {"cmd":"openshutter"}   -- open the shutter
//   {"cmd":"closeshutter"}  -- close the shutter
// Note: intentionally unauthenticated for now (no PIN/pairing check) — anyone in BLE range
// can issue these commands. Reconsider this if the dome isn't in a physically private spot.
class BLEControlWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = String(characteristic->getValue().c_str());
    if (value.length() == 0) return;

    logMessage("BLE: Received dome control command.");

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, value);
    if (err) {
      logMessage("BLE ERROR: Malformed control JSON payload.");
      if (bleStatusChar) {
        bleStatusChar->setValue("{\"status\":\"error\",\"msg\":\"Malformed JSON.\"}");
        bleStatusChar->notify();
      }
      return;
    }

    String cmd = doc["cmd"] | "";
    cmd.toLowerCase();

    bool recognized = true;
    if (cmd == "stop" || cmd == "halt" || cmd == "abortslew") {
      executeAbortSlew();
    } else if (cmd == "park") {
      executePark();
    } else if (cmd == "findhome") {
      executeFindHome();
    } else if (cmd == "openshutter") {
      executeOpenShutter();
    } else if (cmd == "closeshutter") {
      executeCloseShutter();
    } else {
      recognized = false;
    }

    if (bleStatusChar) {
      bleStatusChar->setValue(recognized
        ? ("{\"status\":\"ok\",\"msg\":\"Command '" + cmd + "' executed.\"}").c_str()
        : "{\"status\":\"error\",\"msg\":\"Unknown command.\"}");
      bleStatusChar->notify();
    }

    if (!recognized) {
      logMessage("BLE ERROR: Unknown dome control command: " + cmd);
    }
  }
};

// Starts the BLE GATT server. This runs alongside WiFi, so the
// device can always be configured over Bluetooth without needing to join the setup AP.
// To use it: connect with any BLE explorer app (e.g. nRF Connect on Android/iOS, or
// LightBlue), find the device named "DomeController", open the custom service, and
// write a JSON payload (see BLEConfigWriteCallback above) to the config characteristic.
// Read/subscribe to the status characteristic to see the result.
void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(247); // allow longer JSON payloads (e.g. long WiFi passwords) in one write

  BLEServer* bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());
  BLEService* bleService = bleServer->createService(BLE_SERVICE_UUID);

  BLECharacteristic* configChar = bleService->createCharacteristic(
    BLE_CONFIG_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  configChar->setCallbacks(new BLEConfigWriteCallback());

  BLECharacteristic* controlChar = bleService->createCharacteristic(
    BLE_CONTROL_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  controlChar->setCallbacks(new BLEControlWriteCallback());

  bleStatusChar = bleService->createCharacteristic(
    BLE_STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  bleStatusChar->addDescriptor(new BLE2902());
  bleStatusChar->setCallbacks(new BLEStatusReadCallback());
  bleStatusChar->setValue(buildStatusJson().c_str());

  bleService->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  logMessage("BLE: Advertising as '" + String(BLE_DEVICE_NAME) + "' for network config and emergency dome control.");
}

// Server transaction counter for Alpaca responses (starts at 0, first returned ID will be 1)
unsigned long serverTransactionCounter = 0;

int nextServerTransactionID() {
  serverTransactionCounter++;
  if (serverTransactionCounter == 0) serverTransactionCounter = 1; // wrap guard
  return (int)serverTransactionCounter;
}

// Normalizes an azimuth value in degrees to the [0, 360) range for client-facing
// reporting. The raw encoder tick count (domeAzimuthTicks) is left untouched by
// this function -- it only affects the degrees value handed back to callers
// (Alpaca clients, the web dashboard, etc.), never the internal tick math used
// for slewing/homing/park logic.
double normalizeAzimuthDegrees(double degrees) {
  double normalized = fmod(degrees, 360.0);
  if (normalized < 0.0) normalized += 360.0;
  // Guard against floating point rounding landing exactly on 360.0
  if (normalized >= 360.0) normalized -= 360.0;
  return normalized;
}

// Idle save logic: when dome is idle for >10 minutes, save current state once
unsigned long lastActiveMillis = 0;
bool idleStateSaved = false;
const unsigned long IDLE_SAVE_MS = 10UL * 60UL * 1000UL; // 10 minutes

void markActive() {
  lastActiveMillis = millis();
  idleStateSaved = false;
}

// Dome State Variables
volatile long domeAzimuthTicks = 0;
// Encoder offset used to translate raw hardware counts into a logical zeroed value
volatile long encoderTickOffset = 0;
// Track previous raw home-sensor state so we can detect edges
volatile bool prevHomingSensorActive = false;
// Motor movement monitor state
volatile long motorMonitorPrevTicks = 0;
unsigned long motorMonitorLastMovementMillis = 0;
bool motorMonitorActive = false;
const unsigned long MOTOR_STALL_TIMEOUT_MS = 5000UL; // 5 seconds

// Periodic persistent save of azimuth ticks to FRAM (every second).
// Only writes when the tick value (or shutter state) actually changed since the last save.
unsigned long lastAzSaveMillis = 0;
long lastSavedAzTicks = 0;
int lastSavedShutter = -1;
const unsigned long AZ_SAVE_INTERVAL_MS = 1000UL; // 1 second
// (Legacy shutter integer constants removed — using `ShutterState` enum below)
long ticksPerRot = 1; // Starts at 1 to indicate uncalibrated state
bool needsCalibration = true;
long inertiaTicks = 0; // How many ticks the dome continues to move after motor stops (used for more precise slew/park control)

// --- Inertia measurement / compensation helpers ---
long inertiaMeasurementLastTick = 0;
unsigned long inertiaMeasurementSettledMillis = 0;
const unsigned long INERTIA_SETTLE_MS = 100UL;        // milliseconds of no-change to consider settled
const unsigned long INERTIA_MEASURE_TIMEOUT_MS = 2000UL; // abort measurement after this

// Inertia-based slew control state
bool useInertiaCompensation = false;
bool inertiaCutApplied = false;
long inertiaCutTargetTicks = 0;
long inertiaFinalTargetTicks = 0;
bool inertiaTargetIsCW = false;
unsigned long inertiaCoastStartMillis = 0;
const unsigned long INERTIA_COAST_TIMEOUT_MS = 10000UL; // timeout waiting for coast to finish

// Homing correction after overshoot
bool homingCorrectionActive = false;
long homingCorrectionTargetTicks = 0;
unsigned long homingCorrectionStartMillis = 0;
const unsigned long HOMING_CORRECTION_TIMEOUT_MS = 5000UL;

// Runtime-configurable ticks limits (loaded from FRAM)
long cfgTicksLimitLow = 0; // encoder ticks
long cfgTicksLimitHigh = 0; // encoder ticks
bool ticksLimitsEnabled = false;

bool isHomingSensorActive = false; // Flag to track the state of the home sensor (DI3)
bool isHomingActive = false; // Flag to indicate if the dome is actively searching for home
unsigned long homingStartMillis = 0;
const unsigned long HOMING_TIMEOUT_MS = 120000; // two minutes max for homing before timeout

// --- Calibration routine state (Action=calibrate) ---
// Measures ticksPerRot precisely: seeks home going CCW (establishes a zero reference), then
// seeks home again going CW for one full revolution, and saves the resulting tick count.
enum CalibrationPhase {
  CAL_IDLE,
  CAL_SEEK_HOME_CCW,   // Phase 1: rotate CCW until the home sensor first activates
  CAL_MEASURE_INERTIA, // Phase 1.5: measure encoder ticks caused by inertia after stop
  CAL_DEPART_HOME_CW,  // Phase 2: rotate CW until the home sensor goes inactive (leave the zone)
  CAL_SEEK_HOME_CW     // Phase 3: continue rotating CW until the home sensor activates again
};
CalibrationPhase calibrationPhase = CAL_IDLE;
unsigned long calibrationPhaseStartMillis = 0;


// Slew tracking variables
bool isSlewingToAzimuth = false;
double targetAzimuthDegrees = 0.0;
unsigned long slewStartMillis = 0;
const unsigned long SLEW_TIMEOUT_MS = 120000;

// State Bitmasks for Relays
byte currentRelayState = 0x00;

// WiFi signal-quality logging (diagnostic: correlate RSSI dips with connection stalls)
unsigned long lastRssiCheck = 0;
const unsigned long RSSI_CHECK_INTERVAL_MS = 15000; // every 15s -- frequent enough to catch a
                                                     // dip near a stall without flooding the log
int lastLoggedRssi = 0; // 0 is not a valid RSSI value, used as "not yet logged" sentinel

// Connection state tracking
bool isDriverConnected = false;



// Park tracking variables
bool isParking = false;
long defaultParkAzimuth = 180; // Default fallback to South (180°) if nothing is saved in FRAM
bool atPark = false;               // Tracks if we are physically resting at the park coordinates
unsigned long parkStartMillis = 0;
const unsigned long PARK_TIMEOUT_MS = 120000; // 2 minutes max safety cutoff

// Additional Alpaca Dome state defaults (stubs)
long domeAltitude = 0;
bool atHome = false;
bool canFindHome = true;
bool canPark = true;
bool canSetAltitude = false;
bool canSetAzimuth = true;
bool canSetPark = true;
bool canSetShutter = true;
bool canSlave = false;
bool canSyncAzimuth = true;
bool connecting = false;
String deviceName = "Dome controller";
String deviceDescription = deviceName + " based on Waveshare ESP32-S3-ETH-8DI-8RO-C";
String driverInfo = deviceName + " " + String(FIRMWARE_VERSION);
String driverVersion = String(FIRMWARE_VERSION).substring(1, String(FIRMWARE_VERSION).indexOf(' ')); // Use the firmware version defined in the project (only the major and minor numbers ie. 1.1)
int interfaceVersion = 3;


// Tracks if the dome is slaved to the telescope mount
bool isSlaved = false;

bool slewing = false;

// Digital input pin assignments (DI1..DI5 -> physical GPIO numbers)
// Change these to match your wiring if different.
const int DI1_SHUTTER_OPEN_PIN = 4; // DI1: shutter-open switch (pressed => HIGH)
const int DI2_SHUTTER_CLOSED_PIN = 5; // DI2: shutter-closed switch (pressed => HIGH)
const int DI3_HOME_PIN = 6; // DI3: home sensor (ON => HIGH)

// Forward declarations
void updateShutterSensors();
void updateHomeSensor();
unsigned long shutterTimeoutMs = 60000UL; // max shutter runtime before timeout (configurable on the dashboard, stored in FRAM)

// Timestamp when a shutter action (open/close) was started
unsigned long shutterActionStart = 0;

// Add SHUTTER_OPENING / CLOSING states if not already present
enum ShutterState { SHUTTER_OPEN, SHUTTER_CLOSED, SHUTTER_OPENING, SHUTTER_CLOSING, SHUTTER_ERROR};
ShutterState currentShutterState = SHUTTER_CLOSED;


void writeRelays(byte mask)
{
  currentRelayState = mask;
  Wire.beginTransmission(TCA9554_ADDRESS);
  Wire.write(0x01);
  Wire.write(mask);
  Wire.endTransmission();
}

void logMessage(String message) {
  // 1. Print to hardware serial ONLY if a monitor is actively handshaking/open.
  // On native ESP32-S3 USB, we also make sure the buffer is cleared quickly or skipped.
  if (Serial && Serial.availableForWrite() > 0) { 
    Serial.println(message);
  }

  // 2. Push into the web log buffer
  if (logCount < MAX_LOG_LINES) {
    unsigned long timestamp = millis();
    unsigned long hours = (timestamp / 3600000) % 24;
    unsigned long minutes = (timestamp / 60000) % 60;
    unsigned long seconds = (timestamp / 1000) % 60;

    String timeString = (hours < 10 ? "0" : "") + String(hours) + ":" +
                        (minutes < 10 ? "0" : "") + String(minutes) + ":" +
                        (seconds < 10 ? "0" : "") + String(seconds);
    webLogs[logCount] = "[" + timeString + "] " + message;
    logCount++;
  } else {
    for (int i = 1; i < MAX_LOG_LINES; i++) {
      webLogs[i - 1] = webLogs[i];
    }
    unsigned long timestamp = millis();
    unsigned long hours = (timestamp / 3600000) % 24;
    unsigned long minutes = (timestamp / 60000) % 60;
    unsigned long seconds = (timestamp / 1000) % 60;
    String timeString = (hours < 10 ? "0" : "") + String(hours) + ":" +
                        (minutes < 10 ? "0" : "") + String(minutes) + ":" +
                        (seconds < 10 ? "0" : "") + String(seconds);
    webLogs[MAX_LOG_LINES - 1] = "[" + timeString + "] " + message;
  }
}

// ===================== FRAM STORAGE (I2C, MB85RC-style, 16-bit addressing) =====================
// Stores all dome settings and the azimuth ticks. FRAM has practically
// unlimited write endurance, so the position can be saved every second.
//
// Memory map:
//   0x0000  FramConfig  (settings, written only when a setting changes)
//   0x0080  FramExtra   (watchdog + shutter timeout, written only when changed)
//   0x0100  FramState   slot A  \  ping-pong pair: every save goes to the slot that does NOT hold
//   0x0120  FramState   slot B  /  the latest valid copy, so a power cut mid-write never loses the position
// Each record has a magic number and a CRC32; corrupted/blank records are detected at boot.
#define FRAM_I2C_ADDRESS   0x50
#define FRAM_CONFIG_ADDR   0x0000
#define FRAM_EXTRA_ADDR    0x0080
#define FRAM_STATE_ADDR_A  0x0100
#define FRAM_STATE_ADDR_B  0x0120
#define FRAM_MAGIC_CONFIG  0x444F4D43UL // "DOMC"
#define FRAM_MAGIC_STATE   0x444F4D53UL // "DOMS"
#define FRAM_MAGIC_EXTRA   0x444F4D58UL // "DOMX"

FramConfig framCfg = { FRAM_MAGIC_CONFIG, 0, 0, 0, 0, 180.0, 0 };
FramExtra framExtra = { FRAM_MAGIC_EXTRA, 10, 60, 0 }; // defaults: 10 min watchdog, 60 s shutter timeout
unsigned long lastClientActivityMillis = 0;  // last time a (non-dashboard-polling) client talked to the dome API
bool watchdogTripped = false;                // latch: the watchdog acts once per silence period
bool framOk = false;            // true once the FRAM was detected at FRAM_I2C_ADDRESS
uint32_t framStateSeq = 0;      // sequence number of the most recently written state record
uint8_t framNextSlot = 0;       // 0 = slot A, 1 = slot B (the slot the next save goes to)
unsigned long framLastErrLogMillis = 0;

static uint32_t framCrc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
  }
  return ~crc;
}

bool framDetect() {
  Wire.beginTransmission(FRAM_I2C_ADDRESS);
  return Wire.endTransmission() == 0;
}

bool framWriteBytes(uint16_t addr, const void* data, size_t len) {
  Wire.beginTransmission(FRAM_I2C_ADDRESS);
  Wire.write((uint8_t)(addr >> 8));
  Wire.write((uint8_t)(addr & 0xFF));
  Wire.write((const uint8_t*)data, len);
  return Wire.endTransmission() == 0;
}

bool framReadBytes(uint16_t addr, void* data, size_t len) {
  Wire.beginTransmission(FRAM_I2C_ADDRESS);
  Wire.write((uint8_t)(addr >> 8));
  Wire.write((uint8_t)(addr & 0xFF));
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)FRAM_I2C_ADDRESS, (uint8_t)len) != len) return false;
  uint8_t* p = (uint8_t*)data;
  for (size_t i = 0; i < len; i++) p[i] = Wire.read();
  return true;
}

// Writes the settings record (with retry + read-back verify). Call after changing any framCfg field.
bool saveConfigToFram() {
#if DIAGNOSTIC_DISABLE_NVS_WRITES
  logMessage("DIAG: FRAM write skipped (disabled) for config");
  return true;
#else
  if (!framOk) { logMessage("ERROR: FRAM not available - settings NOT saved"); return false; }
  framCfg.magic = FRAM_MAGIC_CONFIG;
  framCfg.crc = framCrc32((const uint8_t*)&framCfg, sizeof(FramConfig) - sizeof(uint32_t));
  for (int attempt = 0; attempt < 3; attempt++) {
    FramConfig check;
    if (framWriteBytes(FRAM_CONFIG_ADDR, &framCfg, sizeof(framCfg)) &&
        framReadBytes(FRAM_CONFIG_ADDR, &check, sizeof(check)) &&
        memcmp(&check, &framCfg, sizeof(framCfg)) == 0) {
      return true;
    }
    delay(5);
  }
  logMessage("ERROR: Failed to save settings to FRAM after 3 attempts");
  return false;
#endif
}

void sanitizeFramExtra() {
  if (framExtra.watchdogMinutes < 0) framExtra.watchdogMinutes = 0;
  if (framExtra.watchdogMinutes > 1440) framExtra.watchdogMinutes = 1440;
  if (framExtra.shutterTimeoutSec < 5) framExtra.shutterTimeoutSec = 5;
  if (framExtra.shutterTimeoutSec > 600) framExtra.shutterTimeoutSec = 600;
}

bool saveExtraToFram() {
#if DIAGNOSTIC_DISABLE_NVS_WRITES
  return true;
#else
  if (!framOk) { logMessage("ERROR: FRAM not available - tuning settings NOT saved"); return false; }
  sanitizeFramExtra();
  framExtra.magic = FRAM_MAGIC_EXTRA;
  framExtra.crc = framCrc32((const uint8_t*)&framExtra, sizeof(FramExtra) - sizeof(uint32_t));
  for (int attempt = 0; attempt < 3; attempt++) {
    FramExtra check;
    if (framWriteBytes(FRAM_EXTRA_ADDR, &framExtra, sizeof(framExtra)) &&
        framReadBytes(FRAM_EXTRA_ADDR, &check, sizeof(check)) &&
        memcmp(&check, &framExtra, sizeof(framExtra)) == 0) {
      return true;
    }
    delay(5);
  }
  logMessage("ERROR: Failed to save tuning settings to FRAM after 3 attempts");
  return false;
#endif
}

bool framLoadExtra() {
  FramExtra e;
  if (!framReadBytes(FRAM_EXTRA_ADDR, &e, sizeof(e))) return false;
  if (e.magic != FRAM_MAGIC_EXTRA) return false;
  if (e.crc != framCrc32((const uint8_t*)&e, sizeof(e) - sizeof(uint32_t))) return false;
  framExtra = e;
  sanitizeFramExtra();
  return true;
}

bool framLoadConfig() {
  FramConfig c;
  if (!framReadBytes(FRAM_CONFIG_ADDR, &c, sizeof(c))) return false;
  if (c.magic != FRAM_MAGIC_CONFIG) return false;
  if (c.crc != framCrc32((const uint8_t*)&c, sizeof(c) - sizeof(uint32_t))) return false;
  framCfg = c;
  return true;
}

// Saves azimuth ticks + shutter state (ping-pong between two slots). Cheap: one ~20-byte I2C write.
bool saveStateToFram() {
#if DIAGNOSTIC_DISABLE_NVS_WRITES
  return true;
#else
  if (!framOk) return false;
  FramState st;
  st.magic = FRAM_MAGIC_STATE;
  st.seq = framStateSeq + 1;
  st.azTicks = (int32_t)domeAzimuthTicks;
  st.shutterState = (int32_t)currentShutterState;
  st.crc = framCrc32((const uint8_t*)&st, sizeof(st) - sizeof(uint32_t));
  uint16_t addr = framNextSlot ? FRAM_STATE_ADDR_B : FRAM_STATE_ADDR_A;
  if (!framWriteBytes(addr, &st, sizeof(st))) {
    // Throttle error logging so a dead bus can't flood the log at 1 Hz
    if (millis() - framLastErrLogMillis > 30000UL) {
      framLastErrLogMillis = millis();
      logMessage("ERROR: FRAM state write failed (I2C)");
    }
    return false;
  }
  framStateSeq = st.seq;
  framNextSlot ^= 1;
  lastSavedAzTicks = st.azTicks;
  lastSavedShutter = st.shutterState;
  return true;
#endif
}

// Reads both state slots and returns the newest valid one. Also sets up slot/sequence bookkeeping.
bool framLoadState(FramState& out) {
  FramState a, b;
  bool okA = framReadBytes(FRAM_STATE_ADDR_A, &a, sizeof(a)) && a.magic == FRAM_MAGIC_STATE &&
             a.crc == framCrc32((const uint8_t*)&a, sizeof(a) - sizeof(uint32_t));
  bool okB = framReadBytes(FRAM_STATE_ADDR_B, &b, sizeof(b)) && b.magic == FRAM_MAGIC_STATE &&
             b.crc == framCrc32((const uint8_t*)&b, sizeof(b) - sizeof(uint32_t));
  if (!okA && !okB) return false;
  bool useA = okA && (!okB || (int32_t)(a.seq - b.seq) >= 0);
  out = useA ? a : b;
  framStateSeq = out.seq;
  framNextSlot = useA ? 1 : 0; // write the next save to the other slot
  return true;
}

// Detects the FRAM and loads settings + position. A blank or corrupt record is replaced with
// defaults (uncalibrated, position 0, shutter closed) and written back.
// Returns the recovered azimuth ticks via outTicks.
void framInit(long& outTicks) {
  outTicks = 0;
  framOk = framDetect();
  if (!framOk) {
    logMessage("ERROR: FRAM not found at 0x50 - settings and position will NOT be persisted!");
    return;
  }

  if (!framLoadConfig()) {
    logMessage("STORAGE: No valid settings in FRAM - initializing defaults.");
    framCfg = { FRAM_MAGIC_CONFIG, 0, 0, 0, 0, (double)defaultParkAzimuth, 0 };
    saveConfigToFram();
  }

  if (!framLoadExtra()) {
    logMessage("STORAGE: No valid tuning settings in FRAM - initializing defaults (watchdog 10 min, shutter timeout 60 s).");
    framExtra = { FRAM_MAGIC_EXTRA, 10, 60, 0 };
    saveExtraToFram();
  }

  FramState st;
  if (!framLoadState(st)) {
    logMessage("STORAGE: No valid position in FRAM - initializing to 0.");
    domeAzimuthTicks = 0;
    currentShutterState = SHUTTER_CLOSED;
    saveStateToFram();
    st.azTicks = 0;
  }

  outTicks = st.azTicks;
  lastSavedAzTicks = st.azTicks;
  lastSavedShutter = (int)currentShutterState;
  logMessage("STORAGE: FRAM ready. ticksPerRot=" + String(framCfg.ticksPerRot) + " azTicks=" + String(st.azTicks));
}

// ---- Dome state helpers shared by the individual Alpaca properties and DeviceState ----
bool domeAtHome() {
  double currentAz = normalizeAzimuthDegrees((double)domeAzimuthTicks / ticksPerRot * 360.0);
  double homeAzimuth = 0.0;
  atHome = (abs(currentAz - homeAzimuth) < 1.0);
  return atHome;
}

bool domeAtPark() {
  double currentAz = normalizeAzimuthDegrees((double)domeAzimuthTicks / ticksPerRot * 360.0);
  atPark = (abs(currentAz - framCfg.parkAzimuth) < 1.0);
  return atPark;
}

// ASCOM ShutterStatus: 0 = Open, 1 = Closed, 2 = Opening, 3 = Closing, 4 = Error
int domeShutterStatusValue() {
  switch (currentShutterState) {
    case SHUTTER_OPEN:    return 0;
    case SHUTTER_CLOSED:  return 1;
    case SHUTTER_OPENING: return 2;
    case SHUTTER_CLOSING: return 3;
    default:              return 4;
  }
}

// IDomeV3 DeviceState: array of {Name, Value} for the operational properties
void sendAlpacaDeviceState(Client &client, int clientTransactionID, int serverTransactionID) {
  JsonDocument doc;
  doc["ClientTransactionID"] = clientTransactionID;
  doc["ServerTransactionID"] = serverTransactionID;
  doc["ErrorNumber"] = 0;
  doc["ErrorMessage"] = "";
  JsonArray arr = doc["Value"].to<JsonArray>();
  JsonObject o;
  o = arr.add<JsonObject>(); o["Name"] = "AtHome";        o["Value"] = domeAtHome();
  o = arr.add<JsonObject>(); o["Name"] = "AtPark";        o["Value"] = domeAtPark();
  o = arr.add<JsonObject>(); o["Name"] = "Azimuth";       o["Value"] = normalizeAzimuthDegrees((double)domeAzimuthTicks / ticksPerRot * 360.0);
  o = arr.add<JsonObject>(); o["Name"] = "ShutterStatus"; o["Value"] = domeShutterStatusValue();
  o = arr.add<JsonObject>(); o["Name"] = "Slewing";       o["Value"] = slewing;
  String body;
  serializeJson(doc, body);
  String response = "HTTP/1.1 200 OK\r\n"
                    "Content-Type: application/json\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "Content-Length: " + String(body.length()) + "\r\n"
                    "Connection: close\r\n"
                    "\r\n" + body;
  client.print(response);
}

// Routes that may be called while no client has connected (identity / connection management)
bool isConnectionExemptRoute(const String& requestLine) {
  static const char* const exempt[] = {
    "/api/v1/dome/0/connect",          // connect, connected, connecting
    "/api/v1/dome/0/disconnect",
    "/api/v1/dome/0/description",
    "/api/v1/dome/0/driverinfo",
    "/api/v1/dome/0/driverversion",
    "/api/v1/dome/0/interfaceversion",
    "/api/v1/dome/0/name",
    "/api/v1/dome/0/supportedactions"
  };
  for (size_t i = 0; i < sizeof(exempt) / sizeof(exempt[0]); i++) {
    if (requestLine.indexOf(exempt[i]) >= 0) return true;
  }
  return false;
}

bool isValidNumber(const String& s) {
  if (s.length() == 0) return false;
  bool seenDigit = false;
  bool seenDot = false;
  int start = 0;
  if (s[0] == '-' || s[0] == '+') start = 1;
  if (start >= (int)s.length()) return false; // just a sign, nothing else

  for (int i = start; i < (int)s.length(); i++) {
    char c = s[i];
    if (c == '.') {
      if (seenDot) return false; // two dots
      seenDot = true;
    } else if (isDigit(c)) {
      seenDigit = true;
    } else {
      return false; // any other character = invalid
    }
  }
  return seenDigit;
}

pcnt_unit_handle_t pcnt_unit = NULL;

void initHardwareEncoder() {
    // 1. Set internal pull-ups on encoder pins
    pinMode(DI4_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(DI5_ENCODER_B_PIN, INPUT_PULLUP);

    // 2. Configure PCNT Unit
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
    };
    pcnt_new_unit(&unit_config, &pcnt_unit);

    // 3. Lower Glitch Filter to 500 ns (allows high-frequency edge detection)
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 500, // 0.5 microseconds
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    // 4. Configure single channel (Pin A = Signal Edge, Pin B = Direction Level)
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = DI4_ENCODER_A_PIN,
        .level_gpio_num = DI5_ENCODER_B_PIN,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);

    // 5. Set 1x Edge Actions (Counts ONLY on Rising Edge of Pin A)
    pcnt_channel_set_edge_action(
        pcnt_chan_a, 
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, // Rising edge action
        PCNT_CHANNEL_EDGE_ACTION_HOLD      // Falling edge ignored
    );

    // 6. Set Level Actions (Pin B state determines count direction)
    pcnt_channel_set_level_action(
        pcnt_chan_a, 
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,   // High level = normal count
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE // Low level = inverted count direction
    );

    // 7. Enable, clear, and start PCNT unit
    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
}

// Read raw hardware encoder counter (PCNT)
int getRawEncoderCount() {
  int count = 0;
  if (pcnt_unit != NULL) {
    pcnt_unit_get_count(pcnt_unit, &count);
  }
  return count;
}

// Return logical encoder ticks, adjusted by the current offset so we can zero at home
long getEncoderTicks() {
  return (long)getRawEncoderCount() - encoderTickOffset;
}

// Set the logical encoder ticks to a desired value by adjusting the offset.
void setLogicalEncoderTicks(long desiredTicks) {
  int raw = getRawEncoderCount();
  encoderTickOffset = (long)raw - desiredTicks;
  domeAzimuthTicks = desiredTicks;
}

// Align the logical encoder ticks to either 0 or the nearest multiple of ticksPerRot,
// whichever is closer. Called when the home sensor is triggered so the logical
// position stays consistent across rotations.
void alignEncoderToHomeSensor() {
  if (ticksPerRot <= 1) {
    // Unknown calibration; default to zero
    setLogicalEncoderTicks(0);
    logMessage("HOME-SYNC: ticksPerRot unknown, zeroed encoder.");
    return;
  }

  long current = getEncoderTicks(); // logical ticks prior to alignment

  // Integer rounding to nearest multiple without using floating math
  long k;
  if (current >= 0) k = (current + ticksPerRot / 2) / ticksPerRot;
  else k = (current - ticksPerRot / 2) / ticksPerRot;

  long candidate = k * ticksPerRot;

  long distToCandidate = current >= candidate ? current - candidate : candidate - current;
  long distToZero = current >= 0 ? current : -current;

  long chosen = (distToZero <= distToCandidate) ? 0 : candidate;

  setLogicalEncoderTicks(chosen);
  logMessage("HOME-SYNC: Sensor triggered; synced encoder to " + String(chosen) + " ticks (k=" + String(k) + ").");
}

// ----- Limit checking helpers (ticks-only) -----
bool ticksWithinLimits(long ticks) {
  if (!ticksLimitsEnabled) return true;
  if (cfgTicksLimitLow <= cfgTicksLimitHigh) return (ticks >= cfgTicksLimitLow && ticks <= cfgTicksLimitHigh);
  // if misconfigured (low > high) treat as disabled
  return true;
}

bool canMoveCW() {
  long cur = getEncoderTicks();
  long nextTicks = cur + 1;
  if (!ticksWithinLimits(nextTicks)) return false;
  return true;
}

bool canMoveCCW() {
  long cur = getEncoderTicks();
  long nextTicks = cur - 1;
  if (!ticksWithinLimits(nextTicks)) return false;
  return true;
}

// Monitor azimuth motor activity: when motor relays are active the encoder must
// show movement. If the encoder does not change for a sustained period, assume
// a stall and stop the motor for safety.
void monitorMotorMovement() {
  bool motorActive = (currentRelayState & 0x03) != 0; // Relay bits 0x01/0x02
  long currentTicks = getEncoderTicks();

  if (motorActive) {
    // Enforce configured ticks limits immediately
    if (ticksLimitsEnabled && cfgTicksLimitLow <= cfgTicksLimitHigh) {
      if (currentTicks <= cfgTicksLimitLow) {
        logMessage("LIMIT: TicksLimitLow reached; stopping dome.");
        stopDome();
        isSlewingToAzimuth = false; slewing = false; isParking = false; isHomingActive = false; motorMonitorActive = false;
        return;
      }
      if (currentTicks >= cfgTicksLimitHigh) {
        logMessage("LIMIT: TicksLimitHigh reached; stopping dome.");
        stopDome();
        isSlewingToAzimuth = false; slewing = false; isParking = false; isHomingActive = false; motorMonitorActive = false;
        return;
      }
    }

    // (Degrees-based azimuth limits removed) rely solely on tick limits above.
    if (!motorMonitorActive) {
      // Motor just started
      motorMonitorActive = true;
      motorMonitorPrevTicks = currentTicks;
      motorMonitorLastMovementMillis = millis();
      return;
    }

    if (currentTicks != motorMonitorPrevTicks) {
      // Encoder moved — update marker
      motorMonitorPrevTicks = currentTicks;
      motorMonitorLastMovementMillis = millis();
      return;
    }

    // No movement observed while motor commanded
    if (millis() - motorMonitorLastMovementMillis > MOTOR_STALL_TIMEOUT_MS) {
      logMessage("MOTOR-SAFETY: No encoder movement detected while motor commanded. Stopping dome.");
      stopDome();
      // Clear active movement flags to keep state consistent
      isSlewingToAzimuth = false;
      slewing = false;
      isParking = false;
      isHomingActive = false;
      motorMonitorActive = false;
    }
  } else {
    // Motor not active -> reset monitor
    motorMonitorActive = false;
    motorMonitorPrevTicks = currentTicks;
    motorMonitorLastMovementMillis = millis();
  }
}

void setup()
{
  Serial.begin(115200);

  //fixes the issue (slow performance) on ESP32-S3 when using USB-C and not having the serial monitor opened
  Serial.setTxTimeoutMs(0);

  logMessage("\n--- INITIALIZING ALPACA DOME CONTROLLER ---");

  // Fire up I2C Relay expander
  Wire.begin(42, 41);
  Wire.beginTransmission(TCA9554_ADDRESS);
  Wire.write(0x03);
  Wire.write(0x00);
  Wire.endTransmission();
  writeRelays(0x00);

  // Initialize modern PCNT hardware driver
  initHardwareEncoder();

  // --- LOAD FROM FRAM (I2C is already up: Wire.begin() above) ---
  long recoveredTicks = 0;
  framInit(recoveredTicks);

  ticksPerRot = framCfg.ticksPerRot;
  if (ticksPerRot > 1)
  {
    needsCalibration = false;
    logMessage("STORAGE: Loaded existing calibration profile: " + String(ticksPerRot) + " ticks per turn.");
  }
  else
  {
    needsCalibration = true;
    logMessage("WARNING: No calibration found. Dome requires an initial baseline run.");
  }

  // Load configured ticks limits into runtime variables
  cfgTicksLimitLow = framCfg.ticksLimitLow;
  cfgTicksLimitHigh = framCfg.ticksLimitHigh;
  ticksLimitsEnabled = !(cfgTicksLimitLow == 0 && cfgTicksLimitHigh == 0);
  logMessage("CONFIG: Loaded ticks limits: " + String(cfgTicksLimitLow) + "," + String(cfgTicksLimitHigh));

  // Load inertia compensation parameter (measured during calibration)
  inertiaTicks = framCfg.inertiaTicks;
  if (inertiaTicks > 0) logMessage("STORAGE: Loaded inertiaTicks: " + String(inertiaTicks) + " ticks.");

  // Load park azimuth
  defaultParkAzimuth = (long)framCfg.parkAzimuth;

  // Shutter travel timeout + communication watchdog (silence is measured from boot)
  shutterTimeoutMs = (unsigned long)framExtra.shutterTimeoutSec * 1000UL;
  lastClientActivityMillis = millis();
  watchdogTripped = false;
  logMessage("CONFIG: Shutter timeout " + String(framExtra.shutterTimeoutSec) + "s, communication watchdog " +
             (framExtra.watchdogMinutes > 0 ? String(framExtra.watchdogMinutes) + " min" : String("disabled")));

  // Periodic azimuth-tick persistence tracking
  lastAzSaveMillis = millis();

  // If ticks limits are enabled and the recovered value lies outside them, log a
  // warning but preserve the recovered value. We avoid clamping because physical
  // inertia or prior motion can legitimately leave the dome outside soft limits.
  if (ticksLimitsEnabled && cfgTicksLimitLow <= cfgTicksLimitHigh) {
    if (recoveredTicks < cfgTicksLimitLow || recoveredTicks > cfgTicksLimitHigh) {
      logMessage("STORAGE: Recovered saved az ticks " + String(recoveredTicks) + " outside configured ticks limits [" + String(cfgTicksLimitLow) + "," + String(cfgTicksLimitHigh) + "]; preserving value (warning only).");
    }
  }

  // Apply the logical tick alignment so the PCNT/raw counter offset matches the saved logical ticks.
  setLogicalEncoderTicks(recoveredTicks);
  domeAzimuthTicks = recoveredTicks;

  logMessage("STORAGE: Recovered azimuth ticks=" + String(recoveredTicks) + " applied.");

  // Configure digital inputs with internal pull-ups to fight the strapping defaults
  pinMode(DI1_SHUTTER_OPEN_PIN, INPUT_PULLUP);
  pinMode(DI2_SHUTTER_CLOSED_PIN, INPUT_PULLUP);
  pinMode(DI3_HOME_PIN, INPUT_PULLUP);


  // --- LOAD NETWORK CONFIG FROM FLASH (WiFi only — Ethernet support removed) ---
  preferences.begin("network-config", true); // Read-only
  wifiSSID = preferences.getString("wifi_ssid", "");
  wifiPassword = preferences.getString("wifi_pass", "");
  preferences.end();

  // Put the onboard W5500 Ethernet PHY into its lowest-power idle state. This board has
  // no dedicated power/reset GPIO for the W5500, so the next best thing available in
  // software is to never clock or select it at all: hold CS permanently deasserted
  // (HIGH) and leave the SPI lines as inputs so nothing ever drives them and no SPI
  // transaction is ever attempted.
  pinMode(ETH_CS, OUTPUT);
  digitalWrite(ETH_CS, HIGH);     // deselect — the chip never sees a transaction
  pinMode(ETH_SCLK, INPUT);
  pinMode(ETH_MOSI, INPUT);
  pinMode(ETH_MISO, INPUT);
  pinMode(ETH_INT, INPUT_PULLUP); // avoid a floating input on the (now unused) INT line

  String dashboardHost = "";

  // --- WIFI MODE (dynamic IP via DHCP) ---
  bool staConnected = false;

  if (wifiSSID.length() > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    logMessage("NET: Connecting to WiFi SSID '" + wifiSSID + "'...");

    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
    }
    staConnected = (WiFi.status() == WL_CONNECTED);
  } else {
    logMessage("NET: No WiFi SSID configured yet.");
  }

  if (staConnected) {
    wifiApFallback = false;
    // Disable WiFi modem-sleep / power-save. By default the ESP32 WiFi radio
    // periodically powers down between DTIM beacons to save power, and yields
    // radio time to the concurrently-running BLE stack (see BLEDevice::init
    // above) under WiFi/BLE coexistence arbitration. An incoming TCP packet
    // (e.g. NINA's Alpaca poll) that arrives during one of those windows can
    // sit un-acknowledged for a couple of seconds until the radio wakes back
    // up -- invisible to our own loop()/request-handling code (which never
    // sees the connection until it's actually accepted), but very visible to
    // the client as a multi-second stall. This is the standard fix for that
    // class of intermittent ESP32 WiFi latency, especially when BLE is active
    // alongside WiFi as it is here.
    WiFi.setSleep(false);
    dashboardHost = WiFi.localIP().toString();
    logMessage("NET: WiFi connected. IP address: " + dashboardHost);
  } else {
    // --- FIRST-BOOT / CONNECT-FAILURE FALLBACK: open setup Access Point ---
    wifiApFallback = true;
    logMessage(wifiSSID.length() > 0
      ? "ERROR: WiFi failed to connect within timeout. Starting setup Access Point instead."
      : "NET: Starting setup Access Point for first-time WiFi configuration.");

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID); // open network, no password
    dashboardHost = WiFi.softAPIP().toString();
    logMessage("NET: Setup AP '" + String(AP_SSID) + "' is active. Connect to it, then browse to http://" + dashboardHost + "/ and enter your WiFi credentials on the Network Configuration panel.");
  }

  bool udpListenerSuccess = udpListenerWifi.begin(ALPACA_DISCOVERY_PORT);
  if (udpListenerSuccess) {
    logMessage("Alpaca UDP Discovery listening at: " + dashboardHost + ":" + String(ALPACA_DISCOVERY_PORT));
  } else {
    logMessage("ERROR: Failed to start Alpaca UDP Discovery listener.");
  }

  alpacaServerWifi.begin();
  webServerWifi.begin();

  // --- mDNS: advertise the controller as http://<hostname>.local/ regardless of mode ---
  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("alpacadiscovery", "udp", ALPACA_DISCOVERY_PORT);
    logMessage("NET: mDNS responder started: http://" + String(MDNS_HOSTNAME) + ".local/");
  } else {
    logMessage("ERROR: Failed to start mDNS responder.");
  }

  // --- BLE: always-available alternative way to set network config, alongside WiFi ---
  setupBLE();

  logMessage("Alpaca Discovery Endpoint Listening at: " + dashboardHost + ":" + String(ALPACA_DISCOVERY_PORT));
  logMessage("Human Web Dashboard Listening at: http://" + dashboardHost + "/");
}

// FIXED FOR ARDUINOJSON V7: Uses a template to automatically wrap any variable type safely
template <typename T>
void sendAlpacaResponse(Client &client, int clientTransactionID, int serverTransactionID, T value)
{
  JsonDocument doc; // V7 uses unified JsonDocument instead of StaticJsonDocument
  doc["ClientTransactionID"] = clientTransactionID;
  doc["ServerTransactionID"] = serverTransactionID;
  doc["ErrorNumber"] = 0;
  doc["ErrorMessage"] = "";
  doc["Value"] = value;

  String body;
  serializeJson(doc, body);
  // Build the whole response (headers + body) as one String and send it in a single
  // client.print() call. Each separate client.println()/print() call used to become
  // its own individual WiFi transmission (full 802.11 CSMA/CA contention + ACK per
  // frame) unless TCP's Nagle algorithm happened to coalesce them -- so many small
  // writes could cost far more than one bigger one. Sending it all at once sidesteps
  // that entirely, regardless of Nagle/TCP_NODELAY settings.
  String response = "HTTP/1.1 200 OK\r\n"
                     "Content-Type: application/json\r\n"
                     "Access-Control-Allow-Origin: *\r\n"
                     "Content-Length: " + String(body.length()) + "\r\n"
                     "Connection: close\r\n"
                     "\r\n" + body;
  client.print(response);
}

void sendAlpacaArrayResponse(Client &client, int clientTransactionID, int serverTransactionID, const char* const values[], size_t count)
{
  JsonDocument doc;
  doc["ClientTransactionID"] = clientTransactionID;
  doc["ServerTransactionID"] = serverTransactionID;
  doc["ErrorNumber"] = 0;
  doc["ErrorMessage"] = "";

  JsonArray arr = doc["Value"].to<JsonArray>();
  for (size_t i = 0; i < count; i++) {
    arr.add(values[i]);
  }

  String body;
  serializeJson(doc, body);
  String response = "HTTP/1.1 200 OK\r\n"
                     "Content-Type: application/json\r\n"
                     "Access-Control-Allow-Origin: *\r\n"
                     "Content-Length: " + String(body.length()) + "\r\n"
                     "Connection: close\r\n"
                     "\r\n" + body;
  client.print(response);
}

void sendAlpacaErrorResponse(Client &client, int clientTransactionID, int serverTransactionID, int errorNumber, const char *errorMessage)
{
  JsonDocument doc;
  doc["ClientTransactionID"] = clientTransactionID;
  doc["ServerTransactionID"] = serverTransactionID;
  doc["ErrorNumber"] = errorNumber;
  doc["ErrorMessage"] = errorMessage;

  String body;
  serializeJson(doc, body);
  String response = "HTTP/1.1 400 Bad Request\r\n"
                     "Content-Type: application/json\r\n"
                     "Access-Control-Allow-Origin: *\r\n"
                     "Content-Length: " + String(body.length()) + "\r\n"
                     "Connection: close\r\n"
                     "\r\n" + body;
  client.print(response);
}

// Send an Alpaca-style error response (non-zero ErrorNumber)
void sendAlpacaError(Client &client, int clientTransactionID, int serverTransactionID, int errorNumber, const char *errorMessage)
{
  DynamicJsonDocument doc(256);
  doc["ClientTransactionID"] = clientTransactionID;
  doc["ServerTransactionID"] = serverTransactionID;
  doc["ErrorNumber"] = errorNumber;
  doc["ErrorMessage"] = errorMessage;

  String body;
  serializeJson(doc, body);
  String response = "HTTP/1.1 200 OK\r\n"
                     "Content-Type: application/json\r\n"
                     "Access-Control-Allow-Origin: *\r\n"
                     "Content-Length: " + String(body.length()) + "\r\n"
                     "Connection: close\r\n"
                     "\r\n" + body;
  client.print(response);
}

// Overload helper for actions that do not return a specific value (like PUT commands)
void sendAlpacaEmptyResponse(Client &client, int clientTransactionID, int serverTransactionID)
{
  JsonDocument doc;
  doc["ClientTransactionID"] = clientTransactionID;
  doc["ServerTransactionID"] = serverTransactionID;
  doc["ErrorNumber"] = 0;
  doc["ErrorMessage"] = "";

  String body;
  serializeJson(doc, body);
  String response = "HTTP/1.1 200 OK\r\n"
                     "Content-Type: application/json\r\n"
                     "Access-Control-Allow-Origin: *\r\n"
                     "Content-Length: " + String(body.length()) + "\r\n"
                     "Connection: close\r\n"
                     "\r\n" + body;
  client.print(response);
}

// Simple CORS helpers and generic JSON responder
void sendCORSPreflight(Client &client)
{
  String response = "HTTP/1.1 204 No Content\r\n"
                     "Access-Control-Allow-Origin: *\r\n"
                     "Access-Control-Allow-Methods: GET, POST, PUT, OPTIONS\r\n"
                     "Access-Control-Allow-Headers: Content-Type\r\n"
                     "Content-Length: 0\r\n"
                     "Connection: close\r\n"
                     "\r\n";
  client.print(response);
}

// Motor Action Definitions
void stopDome()
{
  // Turn off relays 1 to 4 immediately
  writeRelays(0x00);
  logMessage("DOME: Motors Stopped.");
  // Mark activity (stop is an activity) so idle timer resets
  markActive();
  slewing = false;
}

void openRelayInterlock()
{
  // Safety step: Drop both drive lines low and wait for mechanical settling
  writeRelays(0x00);
  delay(100);
}

bool slewClockwise()
{
  if (!canMoveCW()) {
    logMessage("LIMIT: CW movement blocked by configured limits.");
    stopDome();
    return false;
  }
  openRelayInterlock();
  writeRelays(0x01); // Engage ONLY Relay 1
  logMessage("DOME: Slewing Clockwise...");
  markActive();
  slewing = true;
  return true;
}

bool slewCounterClockwise()
{
  if (!canMoveCCW()) {
    logMessage("LIMIT: CCW movement blocked by configured limits.");
    stopDome();
    return false;
  }
  openRelayInterlock();
  writeRelays(0x02); // Engage ONLY Relay 2
  logMessage("DOME: Slewing Counter-Clockwise...");
  markActive();
  slewing = true;
  return true;
}

void initiateSlewToAzimuth(double targetDegrees) {
  // 1. Constrain target between 0 and 360
  if (targetDegrees < 0.0) targetDegrees += 360.0;
  if (targetDegrees >= 360.0) targetDegrees = fmod(targetDegrees, 360.0);
  
  targetAzimuthDegrees = targetDegrees;
  
  // 2. Convert current position (ticks) back to degrees
  double currentDegrees = ((double)domeAzimuthTicks / ticksPerRot) * 360.0;
  
  // 3. Calculate shortest angular distance
  double delta = targetAzimuthDegrees - currentDegrees;
  while (delta > 180.0) delta -= 360.0;
  while (delta < -180.0) delta += 360.0;

  // If we are already close enough (e.g., within 1.5 degrees), don't move
  if (abs(delta) < 1.5) {
    logMessage("SLEW: Already at target destination. No movement needed.");
    return;
  }

  stopDome(); // Clear existing relay commands safely
  delay(100); 

  // 4. Determine direction and compute a ticks-based target so we can apply inertia
  // compensation (pre-cut) when the motion is long enough to reach full speed.
  bool started = false;
  long startTicks = getEncoderTicks();
  double targetTicksD = (targetAzimuthDegrees / 360.0) * (double)ticksPerRot;
  long baseTargetTicks = (long)round(targetTicksD);

  // Compute CW and CCW equivalents nearest to current position
  long targetTicksCW = baseTargetTicks;
  while (targetTicksCW < startTicks) targetTicksCW += ticksPerRot;
  long targetTicksCCW = baseTargetTicks;
  while (targetTicksCCW > startTicks) targetTicksCCW -= ticksPerRot;

  long selectedTargetTicks = baseTargetTicks;
  bool selectedCW = false;

  if (ticksLimitsEnabled && cfgTicksLimitLow <= cfgTicksLimitHigh && ticksPerRot > 1) {
    bool cwAllowed = (startTicks >= cfgTicksLimitLow && targetTicksCW <= cfgTicksLimitHigh);
    bool ccwAllowed = (startTicks <= cfgTicksLimitHigh && targetTicksCCW >= cfgTicksLimitLow);

    if (cwAllowed && ccwAllowed) {
      if (delta > 0) { selectedCW = true; selectedTargetTicks = targetTicksCW; started = slewClockwise(); }
      else           { selectedCW = false; selectedTargetTicks = targetTicksCCW; started = slewCounterClockwise(); }
    } else if (cwAllowed) { selectedCW = true; selectedTargetTicks = targetTicksCW; started = slewClockwise(); }
    else if (ccwAllowed)  { selectedCW = false; selectedTargetTicks = targetTicksCCW; started = slewCounterClockwise(); }
    else { logMessage("SLEW: Movement blocked — neither CW nor CCW path stays inside configured tick limits."); started = false; }
  }
  else {
    if (delta > 0) { selectedCW = true; selectedTargetTicks = targetTicksCW; logMessage("SLEW: Target " + String(targetAzimuthDegrees) + "° is Clockwise. Moving CW..."); started = slewClockwise(); }
    else           { selectedCW = false; selectedTargetTicks = targetTicksCCW; logMessage("SLEW: Target " + String(targetAzimuthDegrees) + "° is Counter-Clockwise. Moving CCW..."); started = slewCounterClockwise(); }
  }

  if (started) {
    isSlewingToAzimuth = true;
    slewing = true;
    slewStartMillis = millis();

    // Determine whether to apply inertia pre-cut: only when the tick distance is large enough
    long distanceTicks = selectedTargetTicks - startTicks;
    if (distanceTicks < 0) distanceTicks = -distanceTicks;
    if (distanceTicks > 40 && inertiaTicks > 0) {
      useInertiaCompensation = true;
      inertiaCutApplied = false;
      inertiaFinalTargetTicks = selectedTargetTicks;
      inertiaTargetIsCW = selectedCW;
      if (selectedCW) inertiaCutTargetTicks = inertiaFinalTargetTicks - inertiaTicks;
      else            inertiaCutTargetTicks = inertiaFinalTargetTicks + inertiaTicks;
      inertiaCoastStartMillis = 0;
      logMessage("SLEW: Inertia compensation enabled. Pre-cut at " + String(inertiaCutTargetTicks) + " ticks (final " + String(inertiaFinalTargetTicks) + ").");
    } else {
      useInertiaCompensation = false;
      inertiaCutApplied = false;
    }
  } else {
    isSlewingToAzimuth = false;
    slewing = false;
    logMessage("SLEW: Movement blocked by limits. Not starting slew.");
  }
}

void updateHomeSensor()
{
  bool newState = !digitalRead(DI3_HOME_PIN);

  // Rising edge: sensor just became active -> perform alignment
  if (newState && !prevHomingSensorActive) {
    alignEncoderToHomeSensor();
  }

  isHomingSensorActive = newState;
  prevHomingSensorActive = newState;

}

void updateShutterSensors()
{
  // Read shutter limit switches: pressed => HIGH
  bool openLimitHit = !digitalRead(DI1_SHUTTER_OPEN_PIN);
  bool closedLimitHit = !digitalRead(DI2_SHUTTER_CLOSED_PIN);

  //logMessage("SHUTTER: Open limit switch state (active=1): " + String(openLimitHit));
  //logMessage("SHUTTER: Closed limit switch state (active=1): " + String(closedLimitHit));
  //logMessage("SHUTTER: Current shutter state enum: " + String(currentShutterState));

  // MOVING OPEN
  if (currentShutterState == SHUTTER_OPENING) {
    if (openLimitHit) {
      // Turn off Open Relay (0x04) but preserve other bits
      writeRelays(currentRelayState & ~0x04);
      currentShutterState = SHUTTER_OPEN;
      logMessage("SHUTTER: Fully OPEN limit switch hit. Inverter disabled cleanly.");
    }
    else if (millis() - shutterActionStart > shutterTimeoutMs) {
      // Timeout reached - stop motor for safety
      writeRelays(currentRelayState & ~0x04);
      currentShutterState = SHUTTER_ERROR;
      logMessage("CRITICAL ERROR: Shutter open command TIMED OUT after " + String(shutterTimeoutMs / 1000) + "s! Motor stopped for safety.");
    }
  }

  // MOVING CLOSED
  if (currentShutterState == SHUTTER_CLOSING) {
    if (closedLimitHit) {
      // Turn off Close Relay (0x08)
      writeRelays(currentRelayState & ~0x08);
      currentShutterState = SHUTTER_CLOSED;
      logMessage("SHUTTER: Fully CLOSED limit switch hit. Inverter disabled cleanly.");
    }
    else if (millis() - shutterActionStart > shutterTimeoutMs) {
      // Timeout reached - stop motor for safety
      writeRelays(currentRelayState & ~0x08);
      currentShutterState = SHUTTER_ERROR;
      logMessage("CRITICAL ERROR: Shutter close command TIMED OUT after " + String(shutterTimeoutMs / 1000) + "s! Motor stopped for safety.");
    }
  }

  // If the shutter state is ERROR but a limit switch is now active, update state
  if (currentShutterState == SHUTTER_ERROR) {
    if (openLimitHit) {
      currentShutterState = SHUTTER_OPEN;
      logMessage("SHUTTER: Open limit detected while state ERROR — set to OPEN.");
    } else if (closedLimitHit) {
      currentShutterState = SHUTTER_CLOSED;
      logMessage("SHUTTER: Closed limit detected while state ERROR — set to CLOSED.");
    }
  }

  // If neither limit switch is active and the shutter motor is not commanded,
  // the physical position is unknown — but we mark it as ERROR, even though UNKNOWN would be a better fit
  bool shutterMotorActive = (currentRelayState & 0x0C) != 0; // 0x04 = Open, 0x08 = Close
  if (!openLimitHit && !closedLimitHit && !shutterMotorActive) {
    if (currentShutterState != SHUTTER_ERROR) {
      currentShutterState = SHUTTER_ERROR;
      logMessage("SHUTTER: Limit switches inactive and motor idle — state set to ERROR.");
    }
  }
}

void saveAzimuthToFlash() {
  // Immediately save the current azimuth ticks (and shutter state) to FRAM
  saveStateToFram();
  logMessage("STORAGE: Current azimuth position saved to FRAM: " + String(domeAzimuthTicks) + " ticks");
}

void manageHomingLogic() {
  if (!isHomingActive) return; // Exit quickly if we aren't currently searching for home

  // Read the inductive home sensor. Active/detected = HIGH at the GPIO — the board's
  // opto-isolation stage inverts the field-side signal (field terminal reads HIGH when idle,
  // LOW when triggered; the MCU-side GPIO reads the opposite).
  bool homeSensorHit = isHomingSensorActive;

  // If a correction run was started (we intentionally moved a short distance to recenter
  // the dome after an initial home hit), monitor its completion here.
  if (homingCorrectionActive) {
    long currentTicks = getEncoderTicks();
    // If we've reached the correction target (or the sensor re-activated), stop and finalize
    if ( (inertiaTicks > 0 && currentTicks <= homingCorrectionTargetTicks) || homeSensorHit ) {
      stopDome();
      homingCorrectionActive = false;
      isHomingActive = false;
      slewing = false;
      alignEncoderToHomeSensor();
      saveAzimuthToFlash();
      logMessage("HOME: Correction complete; aligned to sensor center.");
    }
    else if (millis() - homingCorrectionStartMillis > HOMING_CORRECTION_TIMEOUT_MS) {
      stopDome();
      homingCorrectionActive = false;
      isHomingActive = false;
      slewing = false;
      logMessage("HOME: Correction timed out; aborting homing correction.");
    }
    return; // While correcting, skip the normal homing logic
  }

  if (homeSensorHit) {
    // 1. Instantly stop the dome azimuth motor
    stopDome(); // Using your file's existing stopDome() method

    // 2. Align the logical encoder to the nearest full-rotation boundary (or 0)
    alignEncoderToHomeSensor();

    // 3. If inertia compensation is configured, attempt a small reverse correction to
    //    bring the dome back to the center of the sensor (undoing the inertia overshoot).
    if (inertiaTicks > 0) {
      long currentTicks = getEncoderTicks();
      long correctionTarget = currentTicks - inertiaTicks; // move CCW this many ticks
      // Clamp against configured tick limits
      if (ticksLimitsEnabled) {
        if (correctionTarget < cfgTicksLimitLow) correctionTarget = cfgTicksLimitLow;
        if (correctionTarget > cfgTicksLimitHigh) correctionTarget = cfgTicksLimitHigh;
      }

      // Start a small CCW move to recenter
      bool started = slewCounterClockwise();
      if (started) {
        homingCorrectionActive = true;
        homingCorrectionTargetTicks = correctionTarget;
        homingCorrectionStartMillis = millis();
        markActive();
        logMessage("HOME: Overshoot detected; applying inertia correction CCW to " + String(homingCorrectionTargetTicks) + " ticks.");
        return; // wait for correction to finish in subsequent loop iterations
      }
    }

    // 4. Clear states (no correction or couldn't start correction)
    isHomingActive = false;
    slewing = false; // Sync with your file's global tracking variables

    logMessage("HOME: Target detected! Dome alignment calibrated to 0 ticks.");

    saveAzimuthToFlash();
  }
  else if (millis() - homingStartMillis > HOMING_TIMEOUT_MS) {
    // Safety Catch: Stop the motor if it spins for too long without hitting the marker
    stopDome();
    isHomingActive = false;
    slewing = false;
    logMessage("CRITICAL ERROR: Homing sequence TIMED OUT! Motor stopped for safety.");
  }
}

// Starts the calibration routine (Action=calibrate). Actual progress is driven by
// manageCalibrationLogic(), called every loop() iteration, since this spans a full dome
// rotation and can't block inside the HTTP handler.
void initiateCalibration() {
  if (calibrationPhase != CAL_IDLE) {
    logMessage("CALIBRATE: Calibration already in progress. Request ignored.");
    return;
  }
  if (isHomingActive || isParking || isSlewingToAzimuth) {
    logMessage("CALIBRATE: Dome busy with another operation. Request ignored.");
    return;
  }

  logMessage("CALIBRATE: Starting calibration routine — Phase 1: seeking home (CCW)...");
  markActive();
  stopDome();
  delay(150); // Interlock settle delay, consistent with executeFindHome()
  calibrationPhase = CAL_SEEK_HOME_CCW;
  calibrationPhaseStartMillis = millis();
  slewCounterClockwise();
  slewing = true;
}

// Drives the calibration routine forward. Called every loop() iteration; does nothing
// unless a calibration is actually in progress.
void manageCalibrationLogic() {
  if (calibrationPhase == CAL_IDLE) return;

  bool homeSensorHit = isHomingSensorActive;

  // Per-phase watchdog: abort the whole routine (no changes saved) if any single phase
  // takes too long — e.g. the home sensor never trips, or the dome physically can't move.
  if (millis() - calibrationPhaseStartMillis > HOMING_TIMEOUT_MS) {
    stopDome();
    slewing = false;
    logMessage("CALIBRATE ERROR: Timed out during calibration (phase " + String((int)calibrationPhase) + "). Aborted — no changes saved.");
    calibrationPhase = CAL_IDLE;
    return;
  }

  switch (calibrationPhase) {
    case CAL_SEEK_HOME_CCW:
      if (homeSensorHit) {
        // Stop and zero the logical encoder now so we can measure inertia as the dome coasts
        stopDome();
        setLogicalEncoderTicks(0); // Establish the zero reference for the measurement lap
        logMessage("CALIBRATE: Home found (Phase 1 complete). Measuring inertia after stop...");

        // Start a non-blocking inertia measurement phase. We wait until the encoder
        // reading settles (no change for INERTIA_SETTLE_MS) or until a timeout occurs.
        inertiaMeasurementLastTick = domeAzimuthTicks; // should be 0 immediately after setLogicalEncoderTicks
        inertiaMeasurementSettledMillis = millis();
        calibrationPhase = CAL_MEASURE_INERTIA;
        calibrationPhaseStartMillis = millis();
      }
      break;

    case CAL_MEASURE_INERTIA:
      {
        long currentTicks = domeAzimuthTicks;
        if (currentTicks != inertiaMeasurementLastTick) {
          // Encoder still changing — update the last-seen value and reset settle timer
          inertiaMeasurementLastTick = currentTicks;
          inertiaMeasurementSettledMillis = millis();
        } else {
          // Encoder has been steady for the settle period => measurement complete
          if (millis() - inertiaMeasurementSettledMillis >= INERTIA_SETTLE_MS) {
            // Save the measured inertia (absolute ticks moved after stop)
            long measuredInertia = inertiaMeasurementLastTick >= 0 ? inertiaMeasurementLastTick : -inertiaMeasurementLastTick;
            inertiaTicks = measuredInertia;
            framCfg.inertiaTicks = inertiaTicks;
            saveConfigToFram();
            logMessage("CALIBRATE: Measured inertiaTicks=" + String(inertiaTicks) + " ticks. Saved to FRAM.");

            // Continue with departing home (CW)
            delay(150); // Interlock settle delay before reversing direction
            calibrationPhase = CAL_DEPART_HOME_CW;
            calibrationPhaseStartMillis = millis();
            slewClockwise();
          }
        }

        // Timeout fallback: if measurement takes too long, persist what we have and continue
        if (millis() - calibrationPhaseStartMillis > INERTIA_MEASURE_TIMEOUT_MS) {
          long measuredInertia = inertiaMeasurementLastTick >= 0 ? inertiaMeasurementLastTick : -inertiaMeasurementLastTick;
          inertiaTicks = measuredInertia;
          framCfg.inertiaTicks = inertiaTicks;
          saveConfigToFram();
          logMessage("CALIBRATE: Inertia measurement timed out; recorded inertiaTicks=" + String(inertiaTicks));
          delay(150);
          calibrationPhase = CAL_DEPART_HOME_CW;
          calibrationPhaseStartMillis = millis();
          slewClockwise();
        }
      }
      break;

    case CAL_DEPART_HOME_CW:
      // The home sensor has physical width, so we must actually leave its active zone
      // before arming detection again — otherwise Phase 3 could trigger instantly and
      // produce a bogus near-zero ticksPerRot.
      if (!homeSensorHit) {
        logMessage("CALIBRATE: Departed home zone. Phase 3: seeking home again (CW) for one full revolution...");
        calibrationPhase = CAL_SEEK_HOME_CW;
        calibrationPhaseStartMillis = millis();
      }
      break;

    case CAL_SEEK_HOME_CW:
      if (homeSensorHit) {
        stopDome();
        slewing = false;

        long measuredTicks = domeAzimuthTicks;
        if (measuredTicks <= 0) {
          logMessage("CALIBRATE ERROR: Measured tick count was invalid (" + String(measuredTicks) + "). Calibration aborted — no changes saved.");
        } else {
          ticksPerRot = measuredTicks;
          framCfg.ticksPerRot = ticksPerRot;
          saveConfigToFram();
          needsCalibration = false;
          setLogicalEncoderTicks(0); // Back at home; re-zero for normal operation
          logMessage("CALIBRATE: Complete! Measured " + String(ticksPerRot) + " ticks per full rotation. Saved to FRAM.");
        }

        calibrationPhase = CAL_IDLE;
      }
      break;

    default:
      calibrationPhase = CAL_IDLE;
      break;
  }
}

void manageSlewLogic() {
  if (!isSlewingToAzimuth) return;

  // Convert current ticks to degrees and capture tick position
  double currentDegrees = ((double)domeAzimuthTicks / ticksPerRot) * 360.0;
  if (currentDegrees < 0.0) currentDegrees += 360.0;
  long currentTicks = getEncoderTicks();

  // If an inertia pre-cut is configured and hasn't been applied yet, watch for reaching the
  // pre-cut tick target and cut the motor early so the dome can coast to the final target.
  if (useInertiaCompensation && !inertiaCutApplied) {
    if (inertiaTargetIsCW) {
      if (currentTicks >= inertiaCutTargetTicks) {
        stopDome();
        slewing = false; // motor relays are off
        inertiaCutApplied = true;
        inertiaCoastStartMillis = millis();
        logMessage("SLEW: Motor CUT for inertia compensation at " + String(inertiaCutTargetTicks) + " ticks.");
      }
    } else {
      if (currentTicks <= inertiaCutTargetTicks) {
        stopDome();
        slewing = false;
        inertiaCutApplied = true;
        inertiaCoastStartMillis = millis();
        logMessage("SLEW: Motor CUT for inertia compensation at " + String(inertiaCutTargetTicks) + " ticks.");
      }
    }
  }

  // Check how close we are to the final target destination (degrees)
  double remainingDistance = abs(targetAzimuthDegrees - currentDegrees);
  if (remainingDistance > 180.0) remainingDistance = 360.0 - remainingDistance;

  // If we've arrived (within deadband), finalize and save
  if (remainingDistance <= 1.0) {
    stopDome();
    isSlewingToAzimuth = false;
    slewing = false;
    useInertiaCompensation = false;
    inertiaCutApplied = false;
    logMessage("SLEW: Target reached successfully! Arrived at: " + String(currentDegrees) + "°");
    saveAzimuthToFlash();
  }
  // If we cut the motor early and are waiting for coast-to-target, enforce a timeout
  else if (inertiaCutApplied && (millis() - inertiaCoastStartMillis > INERTIA_COAST_TIMEOUT_MS)) {
    logMessage("SLEW: Inertia coast timeout — giving up and finalizing sweep.");
    stopDome();
    isSlewingToAzimuth = false;
    slewing = false;
    useInertiaCompensation = false;
    inertiaCutApplied = false;
  }
  // Watchdog timeout safety catch for normal slews
  else if (millis() - slewStartMillis > SLEW_TIMEOUT_MS) {
    stopDome();
    isSlewingToAzimuth = false;
    slewing = false;
    useInertiaCompensation = false;
    inertiaCutApplied = false;
    logMessage("CRITICAL ERROR: Azimuth slew execution TIMED OUT! Motor stopped.");
  }
}

void manageParkLogic() {
  if (!isParking) return;

  // 1. Fetch target coordinate from the stored settings
  long targetParkAz = (long)framCfg.parkAzimuth;

  // 2. Convert current ticks to degrees
  long currentDegrees = ((long)domeAzimuthTicks * 360L / ticksPerRot);
  if (currentDegrees < 0) currentDegrees += 360;
  long currentTicks = getEncoderTicks();

  // If an inertia pre-cut was configured for this park move and not yet applied,
  // watch for the pre-cut tick target and cut the motor early.
  if (useInertiaCompensation && !inertiaCutApplied) {
    if (inertiaTargetIsCW) {
      if (currentTicks >= inertiaCutTargetTicks) {
        stopDome();
        slewing = false;
        inertiaCutApplied = true;
        inertiaCoastStartMillis = millis();
        logMessage("PARK: Motor CUT for inertia compensation at " + String(inertiaCutTargetTicks) + " ticks.");
      }
    } else {
      if (currentTicks <= inertiaCutTargetTicks) {
        stopDome();
        slewing = false;
        inertiaCutApplied = true;
        inertiaCoastStartMillis = millis();
        logMessage("PARK: Motor CUT for inertia compensation at " + String(inertiaCutTargetTicks) + " ticks.");
      }
    }
  }

  // 3. Check remaining angular distance
  double remainingDistance = abs(targetParkAz - currentDegrees);
  if (remainingDistance > 180.0) remainingDistance = 360.0 - remainingDistance;

  // 4. Check for deadband target threshold (within 1.0 degree)
  if (remainingDistance <= 1.0) {
    stopDome();
    isParking = false;
    slewing = false;
    atPark = true; // Mark as parked successfully
    logMessage("PARK: Dome successfully nestled at park position: " + String(currentDegrees) + "°");

    saveAzimuthToFlash();
  }
  // If we cut the motor early and are waiting for coast-to-target, enforce a timeout
  else if (inertiaCutApplied && (millis() - inertiaCoastStartMillis > INERTIA_COAST_TIMEOUT_MS)) {
    logMessage("PARK: Inertia coast timeout — finalizing park sequence.");
    stopDome();
    isParking = false;
    slewing = false;
    atPark = false;
    useInertiaCompensation = false;
    inertiaCutApplied = false;
  }
  // Watchdog timeout safety catch
  else if (millis() - parkStartMillis > PARK_TIMEOUT_MS) {
    stopDome();
    isParking = false;
    slewing = false;
    atPark = false;
    logMessage("CRITICAL ERROR: Parking execution TIMED OUT! Motor stopped.");
  }
}

void initiateParkSequence() {
  logMessage("PARK: Reading saved park coordinates...");
  
  // 1. Fetch the saved park location (defaults to defaultParkAzimuth if never set)
  long targetParkAz = (long)framCfg.parkAzimuth;

  logMessage("PARK: Target angle found at " + String(targetParkAz) + "°");

  // 2. Convert current position (ticks) back to degrees
  double currentDegrees = ((double)domeAzimuthTicks / ticksPerRot) * 360.0;
  
  // 3. Calculate shortest angular path
  double delta = targetParkAz - currentDegrees;
  while (delta > 180.0) delta -= 360.0;
  while (delta < -180.0) delta += 360.0;

  // If we are already within a small deadband, we are done
  if (abs(delta) < 1.5) {
    logMessage("PARK: Already at park position at " + String(currentDegrees) + "°.");
    atPark = true;
    return;
  }

  stopDome(); // Clear existing relay commands safely
  delay(100); 

  // 4. Determine direction and compute a ticks-based target for potential inertia pre-cut
  bool started = false;
  long startTicks = getEncoderTicks();
  double targetTicksD = ((double)targetParkAz / 360.0) * (double)ticksPerRot;
  long baseTargetTicks = (long)round(targetTicksD);

  long targetTicksCW = baseTargetTicks;
  while (targetTicksCW < startTicks) targetTicksCW += ticksPerRot;
  long targetTicksCCW = baseTargetTicks;
  while (targetTicksCCW > startTicks) targetTicksCCW -= ticksPerRot;

  long selectedTargetTicks = baseTargetTicks;
  bool selectedCW = false;

  if (ticksLimitsEnabled && cfgTicksLimitLow <= cfgTicksLimitHigh && ticksPerRot > 1) {
    bool cwAllowed = (startTicks >= cfgTicksLimitLow && targetTicksCW <= cfgTicksLimitHigh);
    bool ccwAllowed = (startTicks <= cfgTicksLimitHigh && targetTicksCCW >= cfgTicksLimitLow);

    if (cwAllowed && ccwAllowed) {
      if (delta > 0) { selectedCW = true; selectedTargetTicks = targetTicksCW; started = slewClockwise(); }
      else           { selectedCW = false; selectedTargetTicks = targetTicksCCW; started = slewCounterClockwise(); }
    } else if (cwAllowed) { selectedCW = true; selectedTargetTicks = targetTicksCW; started = slewClockwise(); }
    else if (ccwAllowed)  { selectedCW = false; selectedTargetTicks = targetTicksCCW; started = slewCounterClockwise(); }
    else { logMessage("PARK: Movement blocked — neither CW nor CCW path stays inside configured tick limits."); started = false; }
  }
  else {
    if (delta > 0) { selectedCW = true; selectedTargetTicks = targetTicksCW; logMessage("PARK: Moving Clockwise to park position..."); started = slewClockwise(); }
    else           { selectedCW = false; selectedTargetTicks = targetTicksCCW; logMessage("PARK: Moving Counter-Clockwise to park position..."); started = slewCounterClockwise(); }
  }

  // Record states only if movement actually started
  if (started) {
    isParking = true;
    slewing = true;
    atPark = false;
    parkStartMillis = millis();

    // Enable inertia pre-cut when the movement is large enough
    long distanceTicks = selectedTargetTicks - startTicks;
    if (distanceTicks < 0) distanceTicks = -distanceTicks;
    if (distanceTicks > 40 && inertiaTicks > 0) {
      useInertiaCompensation = true;
      inertiaCutApplied = false;
      inertiaFinalTargetTicks = selectedTargetTicks;
      inertiaTargetIsCW = selectedCW;
      if (selectedCW) inertiaCutTargetTicks = inertiaFinalTargetTicks - inertiaTicks;
      else            inertiaCutTargetTicks = inertiaFinalTargetTicks + inertiaTicks;
      inertiaCoastStartMillis = 0;
      logMessage("PARK: Inertia compensation enabled. Pre-cut at " + String(inertiaCutTargetTicks) + " ticks (final " + String(inertiaFinalTargetTicks) + ").");
    } else {
      useInertiaCompensation = false;
      inertiaCutApplied = false;
    }
  } else {
    isParking = false;
    slewing = false;
    logMessage("PARK: Movement blocked by limits. Park sequence not started.");
  }
}

// --- Shared dome command functions -----------------------------------------------------
// These wrap the exact same logic used by the HTTP Alpaca API handlers, so the BLE
// emergency control characteristic (and anything else, in the future) stays perfectly in
// sync with the "real" API instead of re-implementing dome-safety logic a second time.

void executeAbortSlew() {
  logMessage("CMD: Abort slew.");
  stopDome();
  slewing = false;
  isSlewingToAzimuth = false;
  isParking = false;
  isHomingActive = false;
}

void executeFindHome() {
  logMessage("CMD: Find home.");
  if (isHomingSensorActive) {
    logMessage("HOME: Already at home position. No action needed.");
    return;
  }
  if (isHomingActive) {
    logMessage("HOME: Already homing. Request ignored.");
    return;
  }
  stopDome();
  delay(150); // Interlock settle delay
  {
    bool started = slewClockwise(); // Start moving clockwise to find home
    if (started) {
      isHomingActive = true;
      slewing = true;
      homingStartMillis = millis();
      markActive();
      logMessage("HOME: Sequence initiated. Searching for home sensor...");
    } else {
      isHomingActive = false;
      logMessage("HOME: Movement blocked by limits. FindHome aborted.");
    }
  }
}

void executeOpenShutter() {
  logMessage("CMD: Open shutter.");
  if (currentShutterState == SHUTTER_OPEN) {
    logMessage("SHUTTER: Already open. Ignoring request.");
    return;
  }
  byte cleanedState = currentRelayState & ~0x08; // Force Close relay off just in case
  writeRelays(cleanedState);
  delay(100); // Interlock cooling delay for physical contacts
  writeRelays(cleanedState | 0x04); // Engage Open relay
  currentShutterState = SHUTTER_OPENING;
  shutterActionStart = millis();
  logMessage("SHUTTER: Initiating movement. Relay 3 (Open) ENGAGED.");
}

void executeCloseShutter() {
  logMessage("CMD: Close shutter.");
  if (currentShutterState == SHUTTER_CLOSED) {
    logMessage("SHUTTER: Already closed. Ignoring request.");
    return;
  }
  byte cleanedState = currentRelayState & ~0x04; // Drop Open relay
  writeRelays(cleanedState);
  delay(100);
  writeRelays(cleanedState | 0x08); // Engage Close relay
  currentShutterState = SHUTTER_CLOSING;
  shutterActionStart = millis();
  logMessage("SHUTTER: Initiating movement. Relay 4 (Close) ENGAGED.");
}

// Communication watchdog: if no client has talked to the dome API for framExtra.watchdogMinutes,
// close the shutter (once per silence period). Dashboard status/log/config polling does not count.
void checkCommunicationWatchdog() {
  if (framExtra.watchdogMinutes <= 0 || watchdogTripped) return;
  unsigned long limitMs = (unsigned long)framExtra.watchdogMinutes * 60000UL;
  if (millis() - lastClientActivityMillis < limitMs) return;

  watchdogTripped = true;
  if (currentShutterState == SHUTTER_CLOSED || currentShutterState == SHUTTER_CLOSING) {
    logMessage("WATCHDOG: No client activity for " + String(framExtra.watchdogMinutes) + " min; shutter already closed/closing.");
    return;
  }
  //do not attempt to close the shutter if it's already in an error state
  if (currentShutterState == ERROR) {
    logMessage("WATCHDOG: No client activity for " + String(framExtra.watchdogMinutes) + " min; shutter in error state.");
    return;
  }
  logMessage("WATCHDOG: No client activity for " + String(framExtra.watchdogMinutes) + " min - closing shutter for safety.");
  executeCloseShutter();
}

void executePark() {
  logMessage("CMD: Park.");
  if (isParking) {
    logMessage("PARK: Already running park process. Request ignored.");
    return;
  }
  markActive();
  initiateParkSequence();
}

void manageAlpacaDiscovery() {
  UDP* udp = getDiscoveryUDP();
  int packetSize = udp->parsePacket();
  if (packetSize > 0) {
    int len = udp->read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = 0;
    }

    String requestString = String(packetBuffer);
    requestString.trim();

    if (requestString.equalsIgnoreCase("alpacadiscovery1")) {
      logMessage("DISCOVERY: Received broadcast request from " + udp->remoteIP().toString());

      // Official ASCOM compliant string formatting (strictly no spaces, cased properly)
      String discoveryResponse = "{\"AlpacaPort\":11111}";

      // Send the response back exactly to the client's discovery port
      udp->beginPacket(udp->remoteIP(), udp->remotePort());
      udp->print(discoveryResponse);
      udp->endPacket();

      logMessage("DISCOVERY: Sent compliant port configuration reply.");
    }
  }
}

void manageWebDashboard() {
  Client* clientPtr = acceptWebClient();
  if (!clientPtr) return;
  Client &client = *clientPtr;
  if (client) {
    // The TCP handshake completes before the HTTP request bytes arrive, so a client can be
    // accepted a moment before it has sent anything. Wait (bounded) for the full header block
    // instead of giving up the instant no bytes are buffered -- otherwise the connection is
    // closed with no response and the browser shows ERR_EMPTY_RESPONSE.
    String request = "";
    unsigned long start = millis();
    while (client.connected() && millis() - start < 1000) {
      if (client.available()) {
        char c = client.read();
        request += c;
        if (request.endsWith("\r\n\r\n")) break; // Stop reading when headers finish
      }
    }

    // Nothing received (idle/speculative connection): just close it, nothing to answer.
    if (request.length() == 0) {
      client.stop();
      return;
    }

    // Extract request line
    int firstNewline = request.indexOf('\n');
    String requestLine = firstNewline >= 0 ? request.substring(0, firstNewline) : request;

    if (requestLine.indexOf("GET / ") >= 0 || requestLine.indexOf("GET /index.html") >= 0) {
      // Use the address the browser used (Host header, minus any port); fall back to our own IP
      String hostHeader = "";
      int hostIdx = request.indexOf("Host: ");
      if (hostIdx >= 0) {
        int hostEnd = request.indexOf("\r\n", hostIdx);
        if (hostEnd > hostIdx) hostHeader = request.substring(hostIdx + 6, hostEnd);
      }
      int colonIdx = hostHeader.indexOf(':');
      if (colonIdx >= 0) hostHeader = hostHeader.substring(0, colonIdx);
      hostHeader.trim();
      if (hostHeader.length() == 0) hostHeader = getLocalIP().toString();

      // redirect to :[alpaca port]/setup/v1/dome/0/setup
      client.println("HTTP/1.1 302 Found");
      client.println("Location: http://" + hostHeader + ":11111/setup/v1/dome/0/setup");
      client.println("Content-Length: 0");
      client.println("Connection: close");
      client.println();

      logMessage("WEB DASHBOARD: Redirected to setup page.");
    }
    client.flush();
    delay(1);
    client.stop();
  }
}

// Helper function to extract integer parameter values from the raw transaction text
long getParameterValue(const String& params, const String& paramName) {
  // Convert everything to lowercase to stay case-insensitive
  String lowerParams = params;
  lowerParams.toLowerCase();
  String lowerName = paramName;
  lowerName.toLowerCase();
  
  // Look for either "?paramname=" or "&paramname=" or just "paramname=" in the string
  int idx = lowerParams.indexOf(lowerName + "=");
  
  if (idx >= 0) {
    // Jump past the parameter key and "=" character
    int startIdx = idx + lowerName.length() + 1;

    int endIdx = lowerParams.indexOf("&", startIdx);
    if (endIdx < 0) {
      // It might be bounded by a space if it's at the end of the HTTP line header
      endIdx = lowerParams.indexOf(" ", startIdx); 
    }
    if (endIdx < 0) {
      endIdx = lowerParams.length();
    }
    
    return lowerParams.substring(startIdx, endIdx).toInt();
  }
  return -55555; // Return default -55555 if the parameter is completely absent
}

// Helper function to extract a raw (case-preserved) string parameter value, e.g. for
// SSID/password/IP fields where case and exact characters matter.
String getStringParam(const String& params, const String& paramName) {
  String key = paramName + "=";
  int idx = params.indexOf(key);
  if (idx < 0) return "";
  int startIdx = idx + key.length();
  int endIdx = params.indexOf('&', startIdx);
  if (endIdx < 0) endIdx = params.indexOf(' ', startIdx);
  if (endIdx < 0) endIdx = params.length();
  return params.substring(startIdx, endIdx);
}

void checkSerialCommands() {
  if (Serial.available() > 0) {
    // Read the incoming line until a newline character is received
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim(); // Clean up any trailing spaces or hidden carriage returns (\r)

    // Check if the user typed "SETTICKS="
    if (inputStr.startsWith("SETTICKS=")) {
      // Extract everything after the '=' sign
      String valueStr = inputStr.substring(9); 
      long newTicks = valueStr.toInt(); // Convert the text to an integer
      
      // Assign it by adjusting the encoder offset so the logical value sticks
      setLogicalEncoderTicks(newTicks);
      
      // Calculate degrees based on your ticksPerRot constant to verify the math
      double calculatedDegrees = ((double)domeAzimuthTicks / ticksPerRot) * 360.0;
      
      logMessage("🛠️ BENCH CONFIG: Manually forced domeAzimuthTicks to: " + String(domeAzimuthTicks) + " ( Approx: " + String(calculatedDegrees) + "° )");
    }
    else if (inputStr.equalsIgnoreCase("STATUS")) {
      // Quick macro shortcut to dump current status details anytime you type STATUS
      logMessage("Current Ticks: " + String(domeAzimuthTicks) + " | Slewing state: " + String(slewing) + " | Shutter State: " + String(currentShutterState));
    }
  }
}

void loop()
{
  // Handle automatic network discovery
  manageAlpacaDiscovery();

  // Handle the port 80 dashboard tracking
  manageWebDashboard();

  //test for serial commands from the user to set domeAzimuthTicks manually
  checkSerialCommands();

  // do calibration if needed
  //processCalibration();

  // Periodically log WiFi signal strength (RSSI), shown as the signal badge on the web
  // dashboard. WiFi.RSSI() only means anything while actually associated as a station
  // (not in the setup-AP fallback).
  if (!wifiApFallback && millis() - lastRssiCheck > RSSI_CHECK_INTERVAL_MS) {
    lastRssiCheck = millis();
    if (WiFi.status() == WL_CONNECTED) {
      lastLoggedRssi = WiFi.RSSI();
    } else {
      logMessage("WIFI: not connected (status=" + String(WiFi.status()) + ")");
    }
  }

  domeAzimuthTicks = getEncoderTicks();

  // Persist azimuth ticks to FRAM every AZ_SAVE_INTERVAL_MS (1 s) when the position or shutter state changed
  {
    unsigned long _now = millis();
    if (_now - lastAzSaveMillis >= AZ_SAVE_INTERVAL_MS) {
      lastAzSaveMillis = _now;
      if (domeAzimuthTicks != lastSavedAzTicks || (int)currentShutterState != lastSavedShutter) {
        saveStateToFram();
      }
    }
  }

  // Communication watchdog (closes the shutter if no client is talking to the controller)
  checkCommunicationWatchdog();

  // Update discrete sensors each loop iteration
  updateShutterSensors();
  updateHomeSensor();
  // Safety monitor: ensure encoder moves while azimuth motor relays are active
  monitorMotorMovement();

  // Used when slewing the dome to the target azimuth.
  manageSlewLogic();

  //used when homing the dome to find the home sensor.
  manageHomingLogic();

  //used when parking the dome to find the park position.
  manageParkLogic();

  //used when running the Action=calibrate routine to measure ticksPerRot.
  manageCalibrationLogic();

  Client* alpacaClientPtr = acceptAlpacaClient();
  if (alpacaClientPtr)
  {
    Client &client = *alpacaClientPtr;
    // Read headers until blank line (end of headers)
    String request = "";
    unsigned long start = millis();
    while (client.connected() && millis() - start < 1000)
    {
      if (client.available())
      {
        char c = client.read();
        request += c;
        if (request.indexOf("\r\n\r\n") != -1)
          break;
      }
    }

    // No bytes received (idle/speculative connection, or the request arrived too late):
    // close silently. Answering an empty connection with an error can be mistaken by the
    // browser for the response to its real request.
    if (request.length() == 0) {
      client.stop();
      return;
    }

    // Extract request line (first line)
    int firstNewline = request.indexOf('\n');
    String requestLine = firstNewline >= 0 ? request.substring(0, firstNewline) : request;

    // --- ADD THIS SECTION: Read the trailing payload body for PUT and POST updates ---
    String requestBody = "";

    // Special-case: firmware upload - stream binary directly into Update
    if (requestLine.indexOf("POST /api/v1/firmware/upload") >= 0) {
      // Parse Content-Length header (case-insensitive)
      String lowerReq = request;
      lowerReq.toLowerCase();
      int clIdx = lowerReq.indexOf("content-length:");
      int contentLength = 0;
      if (clIdx >= 0) {
        int startIdx = clIdx + 15; // len("content-length:")
        int endIdx = lowerReq.indexOf("\r\n", startIdx);
        if (endIdx < 0) endIdx = lowerReq.indexOf("\n", startIdx);
        if (endIdx < 0) endIdx = lowerReq.length();
        String lenStr = lowerReq.substring(startIdx, endIdx);
        lenStr.trim();
        contentLength = lenStr.toInt();
      }

      logMessage("SYS: Incoming firmware upload request. Content-Length=" + String(contentLength));

      if (contentLength <= 0) {
        client.println("HTTP/1.1 400 Bad Request");
        client.println("Connection: close");
        client.println();
        client.println("ERROR: Missing Content-Length header");
        client.stop();
        return; // exit loop() early
      }

      // Begin the update
      if (!Update.begin((uint32_t)contentLength)) {
        logMessage("SYS: OTA begin failed - insufficient space");
        client.println("HTTP/1.1 507 Insufficient Storage");
        client.println("Connection: close");
        client.println();
        client.println("ERROR: Not enough space for firmware");
        client.stop();
        return;
      }

      // Stream the firmware directly from the client into flash
      size_t written = Update.writeStream(client);
      logMessage("SYS: OTA writeStream returned: " + String(written));

      if (Update.end(true)) { // true to set boot partition
        if (Update.isFinished()) {
          logMessage("SYS: Firmware upload complete. Rebooting...");
          const char* okBody = "OK";
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/plain");
          client.print("Content-Length: "); client.println(strlen(okBody));
          client.println("Access-Control-Allow-Origin: *");
          client.println("Connection: close");
          client.println();
          client.print(okBody);
          client.flush();
          // Give the TCP stack time to send the response fully before rebooting
          delay(2000);
          client.stop();
          delay(100);
          ESP.restart();
        } else {
          logMessage("SYS: OTA end() succeeded but not finished.");
          client.println("HTTP/1.1 500 Internal Server Error");
          client.println("Access-Control-Allow-Origin: *");
          client.println("Connection: close");
          client.println();
          client.println("ERROR: Update incomplete");
          client.stop();
          return;
        }
      } else {
        logMessage("SYS: OTA Update failed: " + String(Update.getError()));
        client.println("HTTP/1.1 500 Internal Server Error");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Connection: close");
        client.println();
        client.println("ERROR: Update failed: " + String(Update.getError()));
        client.stop();
        return;
      }
    }

    if (requestLine.indexOf("PUT ") >= 0 || requestLine.indexOf("POST ") >= 0) {
      // Determine exactly how many body bytes the client told us to expect. A fixed
      // delay + "read whatever is available" approach is a race: under CPU/SPI load
      // (motor control, WiFi traffic, etc.) the body can legitimately arrive more
      // than 15ms after the headers, in a separate TCP segment, so that approach
      // intermittently captured an empty/truncated body (losing Azimuth= and/or
      // ClientTransactionID=). Instead, wait for the declared number of bytes.
      String lowerReq = request;
      lowerReq.toLowerCase();
      int clIdx = lowerReq.indexOf("content-length:");
      int contentLength = 0;
      if (clIdx >= 0) {
        int startIdx = clIdx + 15; // len("content-length:")
        int endIdx = lowerReq.indexOf("\r\n", startIdx);
        if (endIdx < 0) endIdx = lowerReq.length();
        String lenStr = lowerReq.substring(startIdx, endIdx);
        lenStr.trim();
        contentLength = lenStr.toInt();
      }

      unsigned long bodyStart = millis();
      while ((int)requestBody.length() < contentLength && client.connected() && millis() - bodyStart < 1000) {
        if (client.available()) {
          requestBody += (char)client.read();
        }
      }
    }

    // Combine parameters and force lowercase to eliminate casing/validation issues
    String fullParams = requestLine + " " + requestBody;
    //fullParams.toLowerCase();

    // Extract the ClientTransactionID sent by the astronomy software
    long clientTxID = getParameterValue(fullParams, "clienttransactionid");

    // ClientTransactionID is optional per the Alpaca spec: a missing (or invalid) value is reported back as 0
    if (clientTxID < 0) clientTxID = 0;

    // Handle OPTIONS preflight
    if (requestLine.indexOf("OPTIONS ") >= 0)
    {
      sendCORSPreflight(client);
      client.flush();
      delay(5);
      client.stop();
      return;
    }

    // On any incoming request treat as activity (reset idle timer)
    markActive();

    // Dashboard requests carry Dashboard=1: they bypass the NotConnected check, and its background polling
    // (status/log/config) must not count as "a client is talking" for the communication watchdog.
    bool isDomeApi = (requestLine.indexOf("/api/v1/dome/0/") >= 0);
    bool fromDashboard = (getParameterValue(fullParams, "dashboard") == 1);
    if (isDomeApi) {
      bool dashboardPoll = fromDashboard && (fullParams.indexOf("Action=status") >= 0 ||
                                             fullParams.indexOf("Action=getlog") >= 0 ||
                                             fullParams.indexOf("Action=getconfig") >= 0);
      if (!dashboardPoll) {
        lastClientActivityMillis = millis();
        watchdogTripped = false;
      }
      // Alpaca: everything except identity/connection routes must fail with NotConnected (0x407) until connected
      if (!isDriverConnected && !fromDashboard && !isConnectionExemptRoute(requestLine)) {
        sendAlpacaErrorResponse(client, clientTxID, nextServerTransactionID(), 0x407, "The device is not connected.");
        client.flush();
        delay(5);
        client.stop();
        return;
      }
    }

    // --- GET CONNECTED STATUS ---
    if (requestLine.indexOf("GET /api/v1/dome/0/connected") >= 0)
    {
      // Returns true if a driver session is open, false otherwise
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), isDriverConnected);
    }
    // --- PUT CONNECTED STATE (CONNECT / DISCONNECT) ---
    else if (requestLine.indexOf("PUT /api/v1/dome/0/connected") >= 0) {
   
      bool targetConnectState = false;

      String fullParamsLower = requestLine + " " + requestBody;
      fullParamsLower.toLowerCase();
      
      // Look for the connection flag anywhere in the stream parameters
      if (fullParamsLower.indexOf("connected=true") >= 0) {
        targetConnectState = true;
      } else if (fullParamsLower.indexOf("connected=false") >= 0) {
        targetConnectState = false;
      } else {
        // return alpacaerror response if the parameter is missing or invalid
        logMessage("REQUEST URL: " + fullParams);
        sendAlpacaErrorResponse(client, clientTxID, nextServerTransactionID(), 0x401, "Missing or invalid 'connected' parameter. Use 'true' or 'false'.");
        logMessage("DATA returned: sendAlpacaErrorResponse");
        client.flush();
        delay(5);
        client.stop();
        return;
      }

      // Execute the state transition
      markActive();
      if (targetConnectState == true) {
        isDriverConnected = true;
        logMessage("SYS: ASCOM Client established a live session link. Dome Connected.");
      } else {
        isDriverConnected = false;
        stopDome(); // Safety feature: immediately halt everything if driver disconnects
        logMessage("SYS: ASCOM Client severed the session link. Dome Disconnected.");
      }
      sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
      logMessage("DATA returned: sendAlpacaEmptyResponse ("+ String(clientTxID) +", "+ String(nextServerTransactionID()) +")");
    }
    else if (requestLine.indexOf("PUT /api/v1/dome/0/connect") >= 0)
    {
      isDriverConnected = true;
      logMessage("SYS: ASCOM Client established a live session link (connect). Dome Connected.");
      sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
    }   
    else if (requestLine.indexOf("PUT /api/v1/dome/0/disconnect") >= 0)
    {
      isDriverConnected = false;
      stopDome(); // Safety feature: immediately halt everything if driver disconnects
      logMessage("SYS: ASCOM Client severed the session link (disconnect). Dome Disconnected.");
      sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
    }
    else if (requestLine.indexOf("PUT /api/v1/system/reboot") >= 0)
    {
      // Dashboard-initiated reboot: persist current state and restart safely
      logMessage("SYS: Reboot requested via web dashboard.");

      // Persist canonical azimuth degrees and some diagnostic state
      saveAzimuthToFlash();

      // Ensure motors are stopped before restarting
      stopDome();

      // Send a JSON empty response (Alpaca style) and give TCP stack time to flush
      sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
      client.flush();
      delay(2000);
      client.stop();
      delay(100);
      ESP.restart();
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/azimuth") >= 0)
    {
      double currentAz = (double)domeAzimuthTicks / ticksPerRot * 360.0;
      
      logMessage("GET azimuth: " + String(normalizeAzimuthDegrees(currentAz)) + "°");

      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), normalizeAzimuthDegrees(currentAz));
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/shutterstatus") >= 0)
    {
      // ASCOM Shutter Status mapping: 0 = Open, 1 = Closed, 2 = Opening, 3 = Closing, 4 = Error
      int statusValue = domeShutterStatusValue();

      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), statusValue);
    }
    // else if (requestLine.indexOf("PUT /api/v1/dome/0/abortslew") >= 0) {
    else if (requestLine.indexOf("PUT /api/v1/dome/0/abortslew") >= 0)
    {
      executeAbortSlew();
      sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
    }
    // Additional Alpaca dome properties and methods (stubs)
    else if (requestLine.indexOf("GET /api/v1/dome/0/altitude") >= 0)
    {
      if (canSetAltitude) sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), domeAltitude);
      else sendAlpacaError(client, clientTxID, nextServerTransactionID(), 0x400, "NotImplementedException: Dome does not support altitude.");
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/athome") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), domeAtHome());
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/atpark") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), domeAtPark());
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/canfindhome") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), canFindHome);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/canpark") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), canPark);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/cansetaltitude") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), canSetAltitude);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/cansetazimuth") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), canSetAzimuth);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/cansetpark") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), canSetPark);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/cansetshutter") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), canSetShutter);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/canslave") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), canSlave);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/cansyncazimuth") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), canSyncAzimuth);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/connecting") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), connecting);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/description") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), deviceDescription);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/driverinfo") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), driverInfo);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/driverversion") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), driverVersion);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/interfaceversion") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), interfaceVersion);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/name") >= 0)
    {    
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), deviceName);
    }
    // --- GET SLAVED STATUS ---
    else if (requestLine.indexOf("GET /api/v1/dome/0/slaved") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), isSlaved);
    }
    // --- PUT SLAVED STATE ---
    else if (requestLine.indexOf("PUT /api/v1/dome/0/slaved") >= 0)
    {
      sendAlpacaError(client, clientTxID, nextServerTransactionID(), 0x400, "Not supported.");
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/slewing") >= 0)
    {
      sendAlpacaResponse(client, clientTxID, nextServerTransactionID(), slewing);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/supportedactions") >= 0)
    {
      // return Array of Strings with supported actions
      const char* supportedActions[] = {"cw", "ccw", "stop", "close", "open", "status", "calibrate"};
      sendAlpacaArrayResponse(client, clientTxID, nextServerTransactionID(), supportedActions, 7);
    }
    else if (requestLine.indexOf("GET /api/v1/dome/0/devicestate") >= 0)
    {
      sendAlpacaDeviceState(client, clientTxID, nextServerTransactionID());
    }
    else if (requestLine.indexOf("PUT /api/v1/dome/0/commandblind") >= 0 ||
             requestLine.indexOf("PUT /api/v1/dome/0/commandbool") >= 0 ||
             requestLine.indexOf("PUT /api/v1/dome/0/commandstring") >= 0)
    {
      sendAlpacaError(client, clientTxID, nextServerTransactionID(), 0x400, "NotImplementedException: Raw device commands are not supported.");
    }
    else if (requestLine.indexOf("PUT /api/v1/dome/0/findhome") >= 0)
    {
      logMessage("API Command: Received FIND HOME request.");
      executeFindHome();
      sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
    }
    // --- OPEN SHUTTER COMMAND ---
    else if (requestLine.indexOf("PUT /api/v1/dome/0/openshutter") >= 0) {
      logMessage("API Command: Received OPEN SHUTTER request.");
      executeOpenShutter();
      sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
    }

    // --- CLOSE SHUTTER COMMAND ---
    else if (requestLine.indexOf("PUT /api/v1/dome/0/closeshutter") >= 0) {
      logMessage("API Command: Received CLOSE SHUTTER request.");
      executeCloseShutter();
      sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
    }
    else if (requestLine.indexOf("PUT /api/v1/dome/0/setpark") >= 0)
    {
      //get current azimuth and save it as the park position in flash memory
      logMessage("API Command: Received SET PARK request.");
      double currentAz = (double)domeAzimuthTicks / ticksPerRot * 360.0;  
      framCfg.parkAzimuth = normalizeAzimuthDegrees(currentAz);
      saveConfigToFram();
      defaultParkAzimuth = (long)framCfg.parkAzimuth;

      // Mark the dome as parked since we are now at the park position
      atPark = true;

      sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
    }
    else if (requestLine.indexOf("PUT /api/v1/dome/0/park") >= 0)
    {
      logMessage("API Command: Received PARK request.");
      executePark();
      sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
    }
    else if (requestLine.indexOf("PUT /api/v1/dome/0/slewtoaltitude") >= 0)
    {
      if (canSetAltitude)
      {
        // Implement altitude control logic here if supported
        sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
      }
      else
      {
        sendAlpacaError(client, clientTxID, nextServerTransactionID(), 0x400, "NotImplementedException: Dome does not support altitude control.");
      }
    }
    else if (requestLine.indexOf("PUT /api/v1/dome/0/slewtoazimuth") >= 0)
    {
      logMessage("API Command: Received SLEW TO AZIMUTH request.");
      logMessage("DEBUGG API Command: Full Params: " + fullParams + ".");

        // Search the entire request stream (including the read HTTP body)
        int azIndex = fullParams.indexOf("Azimuth=");

        double extractedAzimuth = -1.0;
        bool azimuthValid = false;

        if (azIndex >= 0) {
          String azValueStr = "";
          for (int i = azIndex + 8; i < fullParams.length(); i++) {
            char c = fullParams.charAt(i);
            if (c == '&' || c == ' ' || c == '\r' || c == '\n') break;
            azValueStr += c;
          }
          if (isValidNumber(azValueStr)) {
            extractedAzimuth = azValueStr.toDouble();
            azimuthValid = true;
          }


          // Validate and execute if data parsing was successful
          if (azimuthValid && extractedAzimuth >= 0.0 && extractedAzimuth <= 360.0) {
            markActive();
            initiateSlewToAzimuth(extractedAzimuth);
            sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
          } else {
            logMessage("SLEW ERROR: Invalid or missing Azimuth value in API payload.");
            // Return standard ASCOM error code (e.g., Invalid value error = 0x401)
            //sendAlpacaError(client, clientTxID, nextServerTransactionID(), 0x401, "Invalid value for Azimuth.");
            client.println("HTTP/1.1 400 Bad Request");
            client.println("Connection: close");
            client.println();
          }

        } else {
          //if azIndex is negative, it means the Azimuth parameter was not found in the request. This usually indicates a malformed request or missing parameter.
          //logMessage("DEBUGG API Command: azIndex=" + String(azIndex));
          //client.println("HTTP/1.1 400 Bad Request");
          //client.println("Connection: close");
          //client.println();
          logMessage("ERROR: azIndex=" + String(azIndex) + " (sending Alpaca error response)");
          sendAlpacaError(client, clientTxID, nextServerTransactionID(), 0x401, "Missing Azimuth parameter.");
        }
    }
    else if (requestLine.indexOf("PUT /api/v1/dome/0/synctoazimuth") >= 0)
    {

      // Search the entire request stream (including the read HTTP body)
      int azIndex = fullParams.indexOf("Azimuth=");

      double extractedAzimuth = -1.0;
      bool azimuthValid = false;

      if (azIndex >= 0) {
        String azValueStr = "";
        for (int i = azIndex + 8; i < fullParams.length(); i++) {
          char c = fullParams.charAt(i);
          if (c == '&' || c == ' ' || c == '\r' || c == '\n') break;
          azValueStr += c;
        }
        if (isValidNumber(azValueStr)) {
          extractedAzimuth = azValueStr.toDouble();
          azimuthValid = true;
        }
      }

      //check if azimuth is valid. It has to be between 0 and 360 degrees.
      if (azimuthValid && extractedAzimuth >= 0.0 && extractedAzimuth <= 360.0)
      {
        setLogicalEncoderTicks((long)(extractedAzimuth / 360.0 * ticksPerRot));
        markActive();

        saveAzimuthToFlash();
        sendAlpacaEmptyResponse(client, clientTxID, nextServerTransactionID());
      } else {
        logMessage("SYNC ERROR: Invalid or missing Azimuth value in API payload.");
        //sendAlpacaError(client, clientTxID, nextServerTransactionID(), 0x401, "Invalid value for Azimuth.");
        client.println("HTTP/1.1 400 Bad Request");
        client.println("Connection: close");
        client.println();
      }
      
    }
    // --- CENTRALIZED ASCOM ACTION ENDPOINT ---
    else if (requestLine.indexOf("POST /api/v1/dome/0/action") >= 0 || requestLine.indexOf("PUT /api/v1/dome/0/action") >= 0) {
      //logMessage("Alpaca API: Received Action POST invocation.");
      
      StaticJsonDocument<2048> doc;
      doc["ClientTransactionID"] = clientTxID; // Extract from payload headers if desired
      doc["ServerTransactionID"] = nextServerTransactionID(); // Include server transaction ID
      bool handled = true;

      // 1. ROTATION ACTIONS (MOMENTARY HOOKS)
      if (fullParams.indexOf("Action=cw") >= 0) {
        logMessage("SYS: Manual Rotation CW initiated.");
        if (!canMoveCW()) {
          logMessage("SYS: Manual CW blocked by limits.");
          doc["Value"] = "Blocked by limits";
        } else {
          slewClockwise(); // Start moving clockwise
          doc["Value"] = "Moving Clockwise";
        }
      } 
      else if (fullParams.indexOf("Action=ccw") >= 0) {
        logMessage("SYS: Manual Rotation CCW initiated.");
        if (!canMoveCCW()) {
          logMessage("SYS: Manual CCW blocked by limits.");
          doc["Value"] = "Blocked by limits";
        } else {
          slewCounterClockwise(); // Start moving counter-clockwise
          doc["Value"] = "Moving Counter-Clockwise";
        }
      } 
      else if (fullParams.indexOf("Action=stop") >= 0) {
        logMessage("SYS: Rotation Halt executed.");
        // Insert your code to drop power to all rotation relays instantly
        stopDome(); // Stop all rotation
        doc["Value"] = "Stopped";
      }
      
      // 2. SHUTTER MECHANISM ACTIONS
      else if (fullParams.indexOf("Action=open") >= 0) {
        logMessage("SYS: Shutter OPEN sequence triggered.");

        byte cleanedState = currentRelayState & ~0x08; 
        writeRelays(cleanedState);
        delay(100); // Interlock cooling delay for physical contacts

        writeRelays(cleanedState | 0x04);
        currentShutterState = SHUTTER_OPENING;
        shutterActionStart = millis();
        
        doc["Value"] = "Opening Shutter";
      } 
      else if (fullParams.indexOf("Action=close") >= 0) {
        logMessage("SYS: Shutter CLOSE sequence triggered.");
        
        byte cleanedState = currentRelayState & ~0x04; // Drop Open Relay 3
        writeRelays(cleanedState);
        delay(100);
        
        writeRelays(cleanedState | 0x08); // Turn ON Close Relay 4
        currentShutterState = SHUTTER_CLOSING;
        shutterActionStart = millis();

        doc["Value"] = "Closing Shutter";
      }
      
      // 3. CALIBRATION ACTION
      else if (fullParams.indexOf("Action=calibrate") >= 0) {
        logMessage("SYS: Calibration Routine initiated.");
        initiateCalibration();
        doc["Value"] = "Calibration Active";
      } 
      else if (fullParams.indexOf("Action=status") >= 0) {
        //logMessage("SYS: Status Request received.");
        // Insert your code to gather current status
        // Map shutter status to human-readable state
        if (currentShutterState == SHUTTER_OPEN) doc["dome"] = "open";
        else if (currentShutterState == SHUTTER_CLOSED) doc["dome"] = "closed";
        else if (currentShutterState == SHUTTER_OPENING) doc["dome"] = "opening";
        else if (currentShutterState == SHUTTER_CLOSING) doc["dome"] = "closing";
        else if (currentShutterState == SHUTTER_ERROR) doc["dome"] = "error";
        else doc["dome"] = "unknown";

        // Map slewing state
        doc["slewing"] = slewing;
        doc["calibrating"] = (calibrationPhase != CAL_IDLE);

        // map ticks and azimuth
        doc["ticks"] = domeAzimuthTicks;
        double currentAz = (double)domeAzimuthTicks / ticksPerRot * 360.0;
        doc["azimuth"] = normalizeAzimuthDegrees(currentAz);
        doc["timestamp"] = millis();
        doc["lastLoggedRssi"] = lastLoggedRssi;        
      } 
      else if (fullParams.indexOf("Action=getconfig") >= 0) {
        // get configuration values that do not change often and are stored in flash memory
        doc["parkAzimuth"] = (long)framCfg.parkAzimuth;
        doc["ticksPerRot"] = framCfg.ticksPerRot;
        doc["inertiaTicks"] = framCfg.inertiaTicks;
        doc["ticksLimitLow"] = framCfg.ticksLimitLow;
        doc["ticksLimitHigh"] = framCfg.ticksLimitHigh;
        doc["watchdogMinutes"] = framExtra.watchdogMinutes;
        doc["shutterTimeoutSec"] = framExtra.shutterTimeoutSec;
        // Update runtime copies of limits so enforcement takes effect immediately
        cfgTicksLimitLow = framCfg.ticksLimitLow;
        cfgTicksLimitHigh = framCfg.ticksLimitHigh;
        ticksLimitsEnabled = !(cfgTicksLimitLow == 0 && cfgTicksLimitHigh == 0);
        logMessage("CONFIG: Limits updated ticks:" + String(cfgTicksLimitLow) + "," + String(cfgTicksLimitHigh));

        //get network configuration
        preferences.begin("network-config", true);
        doc["netMode"] = "wifi";
        doc["wifiSsid"] = preferences.getString("wifi_ssid", "");
        // WiFi password intentionally NOT returned to the client for security.
        preferences.end();

        // report the currently active address too (useful when in WiFi/DHCP mode)
        doc["currentIp"] = getLocalIP().toString();
        doc["apFallback"] = wifiApFallback;

      }
      else if (fullParams.indexOf("Action=getlog") >= 0) {
        // get log lines from the circular buffer       
        String logLines = "";
        for (int i = 0; i < logCount; i++) {
          logLines += webLogs[i] + "\n";
        }
        doc["log"] = logLines;
      }
      else if (fullParams.indexOf("Action=setconfig") >= 0) {
        // store configuration values that do not change often (persisted in FRAM)
        framCfg.parkAzimuth    = (double)getParameterValue(fullParams, "parkAzimuth");
        framCfg.ticksPerRot    = (int32_t)getParameterValue(fullParams, "ticksPerRot");
        framCfg.inertiaTicks   = (int32_t)getParameterValue(fullParams, "inertiaTicks");
        framCfg.ticksLimitLow  = (int32_t)getParameterValue(fullParams, "ticksLimitLow");
        framCfg.ticksLimitHigh = (int32_t)getParameterValue(fullParams, "ticksLimitHigh");
        saveConfigToFram();

        // Tuning settings (only changed when the parameter was actually sent)
        long newWatchdog = getParameterValue(fullParams, "watchdogMinutes");
        long newShutterTimeout = getParameterValue(fullParams, "shutterTimeoutSec");
        if (newWatchdog != -55555) framExtra.watchdogMinutes = (int32_t)newWatchdog;
        if (newShutterTimeout != -55555) framExtra.shutterTimeoutSec = (int32_t)newShutterTimeout;
        sanitizeFramExtra();
        saveExtraToFram();
        shutterTimeoutMs = (unsigned long)framExtra.shutterTimeoutSec * 1000UL;
        lastClientActivityMillis = millis();
        watchdogTripped = false;
        logMessage("CONFIG: Shutter timeout " + String(framExtra.shutterTimeoutSec) + "s, watchdog " + String(framExtra.watchdogMinutes) + " min");

        // Update runtime copies of limits/park/tickperrot so enforcement takes effect immediately
        cfgTicksLimitLow = framCfg.ticksLimitLow;
        cfgTicksLimitHigh = framCfg.ticksLimitHigh;
        defaultParkAzimuth = (long)framCfg.parkAzimuth;
        ticksPerRot = framCfg.ticksPerRot;
        inertiaTicks = framCfg.inertiaTicks;
 
      }
      else {
        handled = false;
      }

      if (handled) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Access-Control-Allow-Origin: *");
        //calculate the content length of the JSON payload
        String jsonPayload;
        serializeJson(doc, jsonPayload);
        client.print("Content-Length: ");
        client.println(jsonPayload.length());
        client.println("Connection: close");
        client.println();
        client.print(jsonPayload);
      } else {
        sendAlpacaErrorResponse(client, clientTxID, nextServerTransactionID(), 0x400, "Unknown custom action requested.");
      }
    }
    // --- HARDWARE UTILITY ROUTE: WIFI CREDENTIALS ---
    if (requestLine.indexOf("POST /api/v1/network/config") >= 0) {
      logMessage("SYS API: Parsing Network Profile Modification.");

      String ssidStr = getStringParam(fullParams, "ssid");
      String passStr = getStringParam(fullParams, "password");

      bool valid = applyNetworkConfig(ssidStr, passStr);

      if (valid) {
        logMessage("NET: Switching to WiFi (DHCP) -> SSID: " + ssidStr);

        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Access-Control-Allow-Origin: *");
        client.println();
        client.println("{\"status\":\"ok\",\"msg\":\"Network profile updated successfully. Resetting ESP32 controller...\"}");
        delay(500);
        client.stop();

        // Persist current azimuth position before reboot so startup can restore it
        saveAzimuthToFlash();
        ESP.restart();
      } else {
        logMessage("SYS API: Malformed network profile parameters.");
        client.println("HTTP/1.1 400 Bad Request");
        client.println();
        client.println("{\"status\":\"error\",\"msg\":\"Malformed parameters provided.\"}");
      }
    }
    else if (requestLine.indexOf("GET /management/apiversions") >= 0)
    {
      // Returns an array of supported API versions (currently just [1])
      StaticJsonDocument<128> doc;
      JsonArray values = doc.createNestedArray("Value");
      values.add(1);
      doc["ClientTransactionID"] = clientTxID; // Or parse from query string if available
      doc["ServerTransactionID"] = nextServerTransactionID();
      
      // Send JSON response headers and payload
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Connection: close");
      client.println();
      serializeJson(doc, client);
    }
    else if (requestLine.indexOf("GET /management/v1/description") >= 0)
    {
      StaticJsonDocument<384> doc;
      JsonObject value = doc.createNestedObject("Value");
      value["ServerName"] = "Waveshare ESP32 Dome Server";
      value["Manufacturer"] = "Custom DIY";
      value["ManufacturerVersion"] = "1.0.0";
      value["Location"] = "Observatory";
      
      doc["ClientTransactionID"] = clientTxID;
      doc["ServerTransactionID"] = nextServerTransactionID();

      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Connection: close");
      client.println();
      serializeJson(doc, client);
    }
    else if (requestLine.indexOf("GET /management/v1/configureddevices") >= 0)
    {
      StaticJsonDocument<384> doc;
      JsonArray valueArray = doc.createNestedArray("Value");
      
      JsonObject device = valueArray.createNestedObject();
      device["DeviceName"] = "Waveshare Observatory Dome";
      device["DeviceType"] = "Dome";
      device["DeviceNumber"] = 0;
      // A unique string identifier for this hardware instance
      device["UniqueID"] = "DOME-WAVESHARE-ESP32-001"; 

      doc["ClientTransactionID"] = clientTxID;
      doc["ServerTransactionID"] = nextServerTransactionID();

      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Connection: close");
      client.println();
      serializeJson(doc, client);
    }
    else if (requestLine.indexOf("GET /setup/v1/dome/0/setup") >= 0)
    {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html");
      client.println("Connection: close");
      client.println();
      
      const char* htmlPage = R"OBSCTRLHTML(
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <title>ScopeDome Controller Dashboard</title>
            <style>
                :root {
                    --bg-color: #121214;
                    --card-bg: #1e1e24;
                    --text-color: #e4e4e7;
                    --accent-blue: #3b82f6;
                    --accent-red: #ef4444;
                    --accent-green: #10b981;
                    --accent-orange: #f59e0b;
                    --border-color: #3f3f46;
                }
                body {
                    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
                    background-color: var(--bg-color); color: var(--text-color);
                    margin: 0; padding: 20px; display: flex; flex-direction: column; align-items: center;
                }
                .container { width: 100%; max-width: 500px; display: flex; flex-direction: column; gap: 20px; }
                h1 { text-align: center; font-size: 1.5rem; margin: 10px 0; text-transform: uppercase; letter-spacing: 1px; }
                .card { background-color: var(--card-bg); border: 1px solid var(--border-color); border-radius: 12px; padding: 20px; box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.5); }
                .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
                .grid-3 { display: grid; grid-template-columns: 1fr 0.5fr 1fr; gap: 15px; }
                .telemetry-item { display: flex; flex-direction: column; align-items: center; justify-content: center; padding: 10px; background: rgba(0,0,0,0.2); border-radius: 8px; border: 1px solid rgba(255,255,255,0.05); }
                .telemetry-label { font-size: 0.75rem; color: #a1a1aa; text-transform: uppercase; margin-bottom: 5px; }
                .telemetry-value { font-size: 1.25rem; font-weight: bold; font-family: monospace; }
                .control-group { display: flex; flex-direction: column; gap: 10px; align-items: center; }
                .control-title { font-size: 0.9rem; font-weight: 600; color: #a1a1aa; align-self: flex-start; margin-bottom: 5px; padding-right: 20px;}
                .btn { background-color: var(--accent-blue); color: white; border: none; border-radius: 8px; padding: 12px; font-size: 1rem; font-weight: bold; cursor: pointer; user-select: none; -webkit-user-select: none; touch-action: manipulation; display: flex; align-items: center; justify-content: center; gap: 8px; width: 100%; }
                .btn:active { filter: brightness(0.8); }
                .btn-red { background-color: var(--accent-red); }
                .btn-green { background-color: var(--accent-green); }
                .btn-orange { background-color: var(--accent-orange); }
                .shutter-buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; width: 100%; }
                .status-badge { padding: 4px 12px; border-radius: 4px; font-size: 0.85rem; font-weight: bold; background: #3f3f46; text-align: center; width: fit-content; margin: 0 auto; }
                .status-open { background: rgba(16, 185, 129, 0.2); color: #10b981; }
                .status-closed { background: rgba(239, 68, 68, 0.2); color: #ef4444; }
                .status-moving { background: rgba(245, 158, 11, 0.2); color: #f59e0b; }
                
                .terminal { background: #09090b; border: 1px solid var(--border-color); font-family: 'Courier New', Courier, monospace; font-size: 0.85rem; color: #38bdf8; padding: 12px; border-radius: 8px; height: 130px; overflow-y: auto; width: calc(100% - 24px); box-shadow: inset 0 2px 4px rgba(0,0,0,0.8); }
                .terminal-line { margin-bottom: 4px; line-height: 1.2; word-break: break-all; }
                
                .form-field { display: flex; flex-direction: column; gap: 5px; width: 100%; margin-bottom: 12px; }
                .form-field label { font-size: 0.8rem; color: #a1a1aa; text-transform: uppercase; }
                .form-field input { background: #121214; border: 1px solid var(--border-color); border-radius: 6px; padding: 10px; color: white; font-family: monospace; font-size: 1rem; }
                .spinner { width: 16px; height: 16px; border: 2px solid rgba(255,255,255,0.12); border-top-color: white; border-radius: 50%; animation: spin 1s linear infinite; }
                @keyframes spin { to { transform: rotate(360deg); } }
                .fw-status { display:none; align-items:center; gap:8px; padding:8px 10px; border-radius:6px; background: rgba(255,255,255,0.02); color: #e4e4e7; font-weight:700; }
                /* RSSI color classes */
                .rssi-good-dark { color: #0b6b16 !important; }
                .rssi-good { color: #15803d !important; }
                .rssi-fair { color: #facc15 !important; }
                .rssi-weak { color: #f97316 !important; }
                .rssi-bad { color: #dc2626 !important; }
            </style>
        </head>
        <body>
            <div class="container">
                <h1>Dome Dashboard</h1>
                <div id="fw-version" style="position:absolute; top:20px; right:20px; background: rgba(255,255,255,0.02); color:#a1a1aa; padding:6px 8px; border-radius:6px; font-size:0.80rem;">__FIRMWARE_VERSION__</div>
                <div id="rssi-badge" style="position:absolute; top:46px; right:20px; background: rgba(255,255,255,0.02); color:#a1a1aa; padding:6px 8px; border-radius:6px; font-size:0.80rem;">WiFi quality: <span id="rssi-text">-</span></div>
                <div id="response-badge" style="position:absolute; top:62px; right:20px; background: rgba(255,255,255,0.02); color:#a1a1aa; padding:6px 8px; border-radius:6px; font-size:0.80rem;">Response time: <span id="response-text">-</span></div>
                <div id="conn-warning" style="display:none; background: rgba(239,68,68,0.95); color: white; padding: 10px 12px; border-radius: 8px; text-align:center; font-weight:700;">Controller not responding</div>
                
                <div class="card grid-2">
                    <div class="telemetry-item"><span class="telemetry-label">Azimuth Position</span><span id="val-azimuth" class="telemetry-value">0.0°</span></div>
                    <div class="telemetry-item"><span class="telemetry-label">Encoder Ticks</span><span id="val-ticks" class="telemetry-value">0</span></div>
                    <div class="telemetry-item"><span class="telemetry-label">Shutter Status</span><div id="val-shutter" class="status-badge">UNKNOWN</div></div>
                    <div class="telemetry-item"><span class="telemetry-label">Rotation Status</span><div id="val-slew" class="status-badge">UNKNOWN</div></div>
                </div>
                
                <div class="card control-group">
                  <span class="control-title">Dome Rotation</span>
                  <div class="grid-3" style="width: 100%;">
                    <button class="btn" id="btn-ccw">◀ Rotation CCW</button>
                    <button class="btn btn-red" id="btn-stop">■ Stop Dome</button>
                    <button class="btn" id="btn-cw">Rotation CW ▶</button>
                  </div>
                </div>
         
                <div class="card control-group">
                    <span class="control-title">Shutter Mechanism</span>
                    <div class="shutter-buttons">
                        <button class="btn btn-green" id="btn-open">▲ Open Shutter</button>
                        <button class="btn btn-red" id="btn-close">▼ Close Shutter</button>
                    </div>
                </div>

                

                <div class="card control-group">
                  <span class="control-title">Additional methods</span>
                  <div style="width:100%; display:flex; flex-direction:column; gap:8px;">
                    <div style="display:flex; gap:8px; width:100%;">
                      <button class="btn" id="btn-setpark">Set Park</button>
                      <button class="btn" id="btn-findhome">Find Home</button>
                      <button class="btn" id="btn-park">Park</button>
                    </div>
                    <div style="display:flex; gap:8px; width:100%;">
                      <select id="slew-select" style="flex:1; padding:10px; border-radius:6px; background:#121214; color:white; border:1px solid var(--border-color); font-family:monospace;">
                        <!-- options 0,30,...330 -->
                        <option value="0">0°</option>
                        <option value="30">30°</option>
                        <option value="60">60°</option>
                        <option value="90">90°</option>
                        <option value="120">120°</option>
                        <option value="150">150°</option>
                        <option value="180">180°</option>
                        <option value="210">210°</option>
                        <option value="240">240°</option>
                        <option value="270">270°</option>
                        <option value="300">300°</option>
                        <option value="330">330°</option>
                      </select>
                      <button class="btn" id="btn-slewto">Slew To</button>
                    </div>
                    <div style="display:flex; gap:8px; width:100%;">
                      <button class="btn" id="btn-syncto">Sync To Azimuth</button>
                    </div>
                  </div>
                </div>


                <details id="system-procedures-panel" style="width:100%;">
                  <summary style="font-weight:600; font-size:1rem; margin-bottom:10px; cursor:pointer;">System Procedures</summary>

                  <div class="card control-group">
                    <span class="control-title">System Procedures</span>
                    <button class="btn btn-orange" id="btn-calibrate">⚙ Run Full Azimuth Calibration</button>
                    <button class="btn btn-red" id="btn-reboot" style="margin-top:8px;">Reboot Device</button>
                    <div id="reboot-success" style="display:none; background: #16a34a; color: white; padding:10px 12px; border-radius:6px; font-weight:700; text-align:center; margin-top:8px;">Rebooting device...</div>
                  </div>
                  <div class="card control-group">
                    <span class="control-title">Firmware Update</span>
                    <div style="display:flex; flex-direction:column; gap:8px; width:100%;">
                      <input type="file" id="fw-file" accept=".bin,.img" />
                      <div style="display:flex; gap:8px; align-items:center;">
                        <button class="btn btn-orange" id="btn-upload-fw">Upload Firmware</button>
                        <progress id="fw-progress" value="0" max="100" style="flex:1; display:none; height:14px; border-radius:6px;"></progress>
                      </div>
                      <div style="font-size:0.85rem; color:#a1a1aa;">Select a compiled ESP32 firmware binary and upload. The device will reboot after a successful update.</div>
                      <div id="fw-status" class="fw-status" style="display:none; margin-top:8px;"></div>
                      <div id="fw-success" style="display:none; background: #16a34a; color: white; padding:10px 12px; border-radius:6px; font-weight:700; text-align:center; margin-top:8px;">Upload succeeded — rebooting device...</div>
                    </div>
                  </div>
                  <div class="card control-group">
                    <div style="display:flex; align-items:center; justify-content:space-between; gap:8px;">
                      <div>
                        <span class="control-title">Live System Console</span>
                        <button id="btn-logrefresh">refresh</button>
                        <input type="checkbox" id="chk-autoUpdate" unchecked> 
                        <label style="font-size: 0.7rem;" for="chk-autoUpdate">Auto-update</label>
                      </div>
                    </div>
                    <div class="terminal" id="web-terminal"></div>
                  </div>
                </details>

                <details id="dome-limits-panel" style="width:100%;">
                  <summary style="font-weight:600; font-size:1rem; margin-bottom:10px; cursor:pointer;">Dome Limits & Tuning</summary>
                  <div class="card">
                    <span class="control-title" style="display:block; margin-bottom:12px;">Dome Limits & Tuning</span>
                    <!-- Azimuth degree limits removed: use ticks limits only -->
                    <div class="form-field">
                        <label>Encoder ticks Limit Low</label>
                        <input type="number" id="cfg-ticks-low" placeholder="e.g. -1000">
                    </div>
                    <div class="form-field">
                        <label>Encoder ticks Limit High</label>
                        <input type="number" id="cfg-ticks-high" placeholder="e.g. 3000">
                    </div>
                    <div class="form-field">
                        <label>Park Azimuth (°)</label>
                        <input type="number" step="0.1" id="cfg-park-az" placeholder="e.g. 0.0">
                    </div>
                    <div class="form-field">
                        <label>Encoder ticks Per Rotation</label>
                        <input type="number" id="cfg-ticks-per-rot" placeholder="e.g. 4096">
                    </div>
                    <div class="form-field">
                        <label>Dome inertia encoder ticks</label>
                        <input type="number" id="cfg-inertia-ticks" placeholder="e.g. 5">
                    </div>
                    <div class="form-field">
                        <label>Shutter travel timeout (seconds, 5-600)</label>
                        <input type="number" min="5" max="600" id="cfg-shutter-timeout" placeholder="e.g. 60">
                    </div>
                    <div class="form-field">
                        <label>Communication watchdog (minutes, 0 = off): close shutter if no client talks to the controller</label>
                        <input type="number" min="0" max="1440" id="cfg-watchdog-min" placeholder="e.g. 10">
                    </div>
                    <div style="display:flex; gap:8px; align-items:center;">
                      <button class="btn" id="btn-save-config" style="padding:8px 10px; font-size:0.9rem;">Save Config</button>
                    </div>
                  </div>

                </details>

                <details id="network-config-panel" style="width:100%;">
                <summary style="font-weight:600; font-size:1rem; margin-bottom:10px; cursor:pointer;">Network Configuration</summary>
                  <div class="card">
                    <div id="net-ap-banner" style="display:none; background:#fff3cd; color:#664d03; border:1px solid #ffe69c; border-radius:6px; padding:10px 12px; margin-bottom:14px; font-size:0.9rem;">
                      You're connected to the device's temporary setup WiFi network. Enter your home/observatory WiFi SSID and password below and save to connect the device to it.
                    </div>
                    <span class="control-title" style="display:block; margin-bottom:12px;">WiFi Credentials (requires reboot)</span>
                    <div class="form-field">
                        <label>WiFi SSID</label>
                        <input type="text" id="net-ssid" placeholder="e.g. MyObservatoryWiFi">
                    </div>
                    <div class="form-field">
                        <label>WiFi Password</label>
                        <input type="password" id="net-password" placeholder="leave blank to keep current password">
                    </div>

                    <div style="display:flex; gap:8px; align-items:center;">
                      <button class="btn" id="btn-save-network" style="padding:8px 10px; font-size:0.9rem;">Save network configuration</button>
                    </div>
                    <!-- network save now performed together with 'Save Config' -->
                  </div>
                </details>
            </div>
            <script>
                let ipaddress = window.location.hostname; // Dynamically use the address this page was loaded from
                const baseUrl = `http://${ipaddress}:11111`;

                // Gathers the WiFi fields and POSTs them to /api/v1/network/config, with the
                // shared confirm/reboot flow used by both the "Save Config" and "Save network
                // configuration" buttons.
                const saveNetworkConfig = () => {
                  const ssid = document.getElementById('net-ssid').value;
                  const password = document.getElementById('net-password').value;
                  if (!ssid) {
                    alert('Please enter a WiFi SSID.');
                    return;
                  }
                  if (!confirm(`Switch to WiFi (DHCP) using SSID "${ssid}" and reboot device?`)) {
                    alert('Configuration saved (network not applied).');
                    return;
                  }

                  const formNet = new URLSearchParams();
                  formNet.append('ssid', ssid);
                  formNet.append('password', password);

                  fetch(`${baseUrl}/api/v1/network/config`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: formNet.toString()
                  }).then(res => res.json()).then(data => {
                    const newHost = window.location.hostname;
                    setTimeout(() => { window.location.href = `http://${newHost}`; }, 10000);
                    alert('Configuration saved. ' + (data.msg || '') + '\nThis page will attempt to reload from the new address after 10 seconds.');
                  }).catch(err => {
                    console.error(err);
                    alert('Failed to apply network settings.');
                  });
                }

                // Connection helpers: timeout wrapper + visible warning
                function fetchWithTimeout(resource, options = {}, timeout = 2000) {
                  return new Promise((resolve, reject) => {
                    const timer = setTimeout(() => reject(new Error('timeout')), timeout);
                    fetch(resource, options).then(response => {
                      clearTimeout(timer);
                      resolve(response);
                    }).catch(err => {
                      clearTimeout(timer);
                      reject(err);
                    });
                  });
                }

                function showConnectionWarning(msg) {
                  const el = document.getElementById('conn-warning');
                  if (!el) return;
                  el.innerText = msg || 'Controller not responding';
                  el.style.display = 'block';
                }

                function hideConnectionWarning() {
                  const el = document.getElementById('conn-warning');
                  if (!el) return;
                  el.style.display = 'none';
                }

                // Universal ASCOM-compliant x-www-form-urlencoded POST Sender with timeout
                const postAction = (actionName, endpoint = "/api/v1/dome/0/action", extraParams = {}) => {
                  const formData = new URLSearchParams();
                  formData.append('Action', actionName);
                  formData.append('Parameters', '');
                  formData.append('ClientTransactionID', '999');
                  formData.append('Dashboard', '1'); // marks dashboard traffic (exempt from NotConnected, polling ignored by the watchdog)
                  for (const k in extraParams) {
                    if (Object.prototype.hasOwnProperty.call(extraParams, k) && extraParams[k] !== undefined) {
                      formData.append(k, extraParams[k]);
                    }
                  }

                  const start = performance.now();
                  return fetchWithTimeout(`${baseUrl}${endpoint}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: formData.toString()
                  }, 2000)
                  .then(res => res.json())
                  .then(data => { hideConnectionWarning(); try { data.responseTimeMs = Math.max(0, Math.round(performance.now() - start)); } catch(e){} return data; })
                  .catch(err => { console.error("POST Network Error:", err); showConnectionWarning('Command not acknowledged'); throw err; });
                }

                // Rotation buttons simplified to click behavior (like Shutter buttons)
                const btnCcw = document.getElementById('btn-ccw');
                const btnCw = document.getElementById('btn-cw');
                const btnStop = document.getElementById('btn-stop');

                btnCcw.addEventListener('click', () => postAction('ccw'));
                btnCw.addEventListener('click', () => postAction('cw'));
                btnStop.addEventListener('click', () => postAction('stop'));

                // Shutter Bindings
                document.getElementById('btn-open').addEventListener('click', () => postAction('open'));
                document.getElementById('btn-close').addEventListener('click', () => postAction('close'));
                
                // Calibration Binding
                document.getElementById('btn-calibrate').addEventListener('click', () => {
                  if(confirm("Verify full azimuth homing and tick count recalibration?")) {
                    postAction('calibrate');
                  }
                });

                // Firmware Upload Binding (uses XHR to provide upload progress and visible status)
                document.getElementById('btn-upload-fw').addEventListener('click', () => {
                  const fileEl = document.getElementById('fw-file');
                  if (!fileEl || !fileEl.files || fileEl.files.length === 0) { alert('Please select a firmware file to upload.'); return; }
                  const f = fileEl.files[0];
                  if (!confirm(`Upload ${f.name} (${f.size} bytes) and reboot device after upload?`)) return;

                  const btn = document.getElementById('btn-upload-fw');
                  const progress = document.getElementById('fw-progress');
                  const fwStatus = document.getElementById('fw-status');
                  const fwSuccess = document.getElementById('fw-success');
                  const fileInput = document.getElementById('fw-file');

                  // Immediate UI feedback
                  btn.disabled = true;
                  fileInput.disabled = true;
                  progress.style.display = '';
                  progress.value = 0;
                  if (fwSuccess) fwSuccess.style.display = 'none';
                  if (fwStatus) { fwStatus.style.display = 'flex'; fwStatus.innerHTML = '<div class="spinner"></div><div>Uploading…</div>'; }

                  const xhr = new XMLHttpRequest();
                  xhr.open('POST', `${baseUrl}/api/v1/firmware/upload`, true);
                  xhr.timeout = 5 * 60 * 1000; // 5 minutes
                  xhr.upload.onprogress = (e) => {
                    if (e.lengthComputable) {
                      progress.max = e.total;
                      progress.value = e.loaded;
                      if (fwStatus) fwStatus.innerHTML = '<div class="spinner"></div><div>Uploading… ' + Math.round((e.loaded / e.total) * 100) + '%</div>';
                    }
                  };
                  xhr.onload = () => {
                    progress.style.display = 'none';
                    if (xhr.status === 200) {
                      if (fwStatus) fwStatus.style.display = 'none';
                      if (fwSuccess) { fwSuccess.innerText = 'Upload successful. Rebooting device...'; fwSuccess.style.display = 'block'; }
                      // Keep UI disabled while device reboots
                      btn.disabled = true;
                      if (fileInput) fileInput.disabled = true;
                      // Refresh the page after a short delay to allow reboot
                      setTimeout(() => { try { window.location.reload(); } catch (e) { window.location.href = window.location.href; } }, 8000);
                    } else {
                      if (fwStatus) { fwStatus.style.display = 'flex'; fwStatus.innerHTML = '<div style="color:#fecaca;">Upload failed: ' + xhr.status + '</div>'; }
                      btn.disabled = false;
                      if (fileInput) fileInput.disabled = false;
                    }
                  };
                  xhr.onerror = () => {
                    progress.style.display = 'none';
                    if (fwStatus) { fwStatus.style.display = 'flex'; fwStatus.innerHTML = '<div style="color:#fecaca;">Upload failed (network error).</div>'; }
                    btn.disabled = false;
                    if (fileInput) fileInput.disabled = false;
                  };
                  xhr.ontimeout = () => {
                    progress.style.display = 'none';
                    if (fwStatus) { fwStatus.style.display = 'flex'; fwStatus.innerHTML = '<div style="color:#fecaca;">Upload timed out.</div>'; }
                    btn.disabled = false;
                    if (fileInput) fileInput.disabled = false;
                  };
                  xhr.send(f);
                });

                // Additional Alpaca PUT helpers
                const putEndpoint = (endpoint, params = {}) => {
                  const form = new URLSearchParams();
                  form.append('Dashboard', '1'); // marks dashboard traffic (exempt from NotConnected, polling ignored by the watchdog)
                  for (const k in params) form.append(k, params[k]);
                  return fetchWithTimeout(`${baseUrl}${endpoint}`, {
                    method: 'PUT',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: form.toString()
                  }, 2000)
                  .then(res => { hideConnectionWarning(); try { return res.json(); } catch(e) { return null; } })
                  .catch(err => { console.error('PUT Network Error:', err); showConnectionWarning('Command not acknowledged'); throw err; });
                }

                // Reboot button binding
                const btnReboot = document.getElementById('btn-reboot');
                if (btnReboot) {
                  btnReboot.addEventListener('click', () => {
                    if (!confirm('Save settings and reboot device now?')) return;
                    btnReboot.disabled = true;
                    putEndpoint('/api/v1/system/reboot', { ClientTransactionID: 999 })
                      .then(() => {
                        const el = document.getElementById('reboot-success');
                        if (el) { el.innerText = 'Rebooting device...'; el.style.display = 'block'; }
                        // disable all interactive controls while rebooting
                        document.querySelectorAll('button, input, select').forEach(e => e.disabled = true);
                        setTimeout(() => { try { window.location.reload(); } catch(e) { window.location.href = window.location.href; } }, 8000);
                      })
                      .catch(err => { btnReboot.disabled = false; alert('Reboot request failed: ' + (err.message || err)); });
                  });
                }

                // Set Park
                document.getElementById('btn-setpark').addEventListener('click', () => {
                  if (!confirm('Save current azimuth as park position?')) return;
                  putEndpoint('/api/v1/dome/0/setpark', { ClientTransactionID: 999 }).then(() => alert('Park position saved.'));
                });

                // Find Home
                document.getElementById('btn-findhome').addEventListener('click', () => {
                  if (!confirm('Begin homing procedure?')) return;
                  putEndpoint('/api/v1/dome/0/findhome', { ClientTransactionID: 999 }).then(() => {});
                });

                // Park
                document.getElementById('btn-park').addEventListener('click', () => {
                  if (!confirm('Start park sequence now?')) return;
                  putEndpoint('/api/v1/dome/0/park', { ClientTransactionID: 999 }).then(() => {});
                });

                // Slew To (selected angle)
                document.getElementById('btn-slewto').addEventListener('click', () => {
                  const val = document.getElementById('slew-select').value;
                  if (!confirm(`Slew dome to ${val}°?`)) return;
                  putEndpoint('/api/v1/dome/0/slewtoazimuth', { Azimuth: val, ClientTransactionID: 999 }).then(() => {});
                });

                // Sync To Azimuth
                document.getElementById('btn-syncto').addEventListener('click', () => {
                  const val = document.getElementById('slew-select').value;
                  if (!confirm(`Sync dome position to ${val}°?`)) return;
                  putEndpoint('/api/v1/dome/0/synctoazimuth', { Azimuth: val, ClientTransactionID: 999 }).then(() => {});
                });

                // Configuration: load/save handlers (uses POST /api/v1/dome/0/action Action=getconfig/setconfig)
                const loadConfigToUI = () => {
                  postAction('getconfig')
                    .then(data => {
                      if (!data) return;
                      const cfg = data.Value || data;
                      // Dome limits
                      // Azimuth degree limits are no longer configurable from the dashboard
                      // Ticks limits (server uses ticksLimitLow/High)
                      if (cfg.ticksLimitLow !== undefined) document.getElementById('cfg-ticks-low').value = cfg.ticksLimitLow;
                      if (cfg.ticksLimitHigh !== undefined) document.getElementById('cfg-ticks-high').value = cfg.ticksLimitHigh;
                      // Park position (server uses parkAzimuth)
                      if (cfg.parkAzimuth !== undefined) document.getElementById('cfg-park-az').value = cfg.parkAzimuth;
                      // Ticks per rotation
                      if (cfg.ticksPerRot !== undefined) document.getElementById('cfg-ticks-per-rot').value = cfg.ticksPerRot;
                      if (cfg.inertiaTicks !== undefined) document.getElementById('cfg-inertia-ticks').value = cfg.inertiaTicks;
                      if (cfg.shutterTimeoutSec !== undefined) document.getElementById('cfg-shutter-timeout').value = cfg.shutterTimeoutSec;
                      if (cfg.watchdogMinutes !== undefined) document.getElementById('cfg-watchdog-min').value = cfg.watchdogMinutes;
                      // Network fields returned by getconfig.
                      if (cfg.wifiSsid !== undefined) document.getElementById('net-ssid').value = cfg.wifiSsid;
                      // WiFi password is never returned by the server; leave the field blank.
                      document.getElementById('net-ap-banner').style.display = cfg.apFallback ? '' : 'none';
                    })
                    .catch(err => console.error('Load config failed', err));
                }

                // Load config automatically when the configuration panel is opened
                document.getElementById('dome-limits-panel').addEventListener('toggle', (e) => {
                  if (e.target.open) loadConfigToUI();
                });
                // Network panel has its own dedicated getconfig load too, since it can be opened independently
                document.getElementById('network-config-panel').addEventListener('toggle', (e) => {
                  if (e.target.open) loadConfigToUI();
                });

                document.getElementById('btn-save-config').addEventListener('click', () => {
                  const params = {
                    ticksLimitLow: document.getElementById('cfg-ticks-low').value,
                    ticksLimitHigh: document.getElementById('cfg-ticks-high').value,
                    parkAzimuth: document.getElementById('cfg-park-az').value,
                    ticksPerRot: document.getElementById('cfg-ticks-per-rot').value,
                    inertiaTicks: document.getElementById('cfg-inertia-ticks').value,
                    shutterTimeoutSec: document.getElementById('cfg-shutter-timeout').value,
                    watchdogMinutes: document.getElementById('cfg-watchdog-min').value
                  };

                  // Dome config and network config are saved independently; network changes
                  // are applied via the "Save network configuration" button below since they
                  // require a reboot.
                  postAction('setconfig', '/api/v1/dome/0/action', params)
                    .then(() => alert('Configuration saved.'))
                    .catch(err => { console.error(err); alert('Failed to save configuration.'); });
                });

                document.getElementById('btn-save-network').addEventListener('click', () => {
                  saveNetworkConfig();

                });

                let lastLogHash = "";

                // Updated Background Loop: Uses POST with action=status payload instead of GET query
                let statusRequestSeq = 0;
                let latestAppliedStatusSeq = 0;
                let statusRequestInFlight = false;

                const updateStatus = () => {
                    //if (!document.getElementById('chk-autoUpdate').checked) return;

                    // Skip this tick if a previous poll hasn't resolved yet — the firmware can
                    // briefly block (e.g. relay interlock delays during findhome/calibrate),
                    // and firing overlapping requests risks an older response arriving after a
                    // newer one and stomping the freshest data (e.g. a just-reset tick count).
                    if (statusRequestInFlight) return;
                    statusRequestInFlight = true;
                    const seq = ++statusRequestSeq;

                    postAction('status')
                    .then(data => {
                        statusRequestInFlight = false;
                        if (!data) return;
                        // Drop this response if a newer one has already been applied — it's stale.
                        if (seq <= latestAppliedStatusSeq) return;
                        latestAppliedStatusSeq = seq;

                        // figure out if dome is rotating left/right and always display azimuth in [0,360)
                        const prevAzEl = document.getElementById('val-azimuth');
                        const prevAzimuth = parseFloat(prevAzEl.innerText) || 0;
                        const azRaw = parseFloat(data.azimuth);
                        const normalizeDeg = v => ((v % 360) + 360) % 360;
                        const azimuth = isNaN(azRaw) ? prevAzimuth : normalizeDeg(azRaw);
                        // smallest signed difference in range (-180,180]
                        const azimuthChange = ((azimuth - prevAzimuth + 540) % 360) - 180;

                        document.getElementById('val-ticks').innerText = data.ticks ?? 0;
                        if (data.azimuth !== undefined) {
                          prevAzEl.innerText = azimuth.toFixed(1) + "°";
                        }
                        
                        const sEl = document.getElementById('val-shutter');
                        sEl.innerText = data.dome ? data.dome.toUpperCase() : "UNKNOWN"; 
                        sEl.className = "status-badge";
                        
                        if (data.dome === "open") sEl.classList.add('status-open');
                        else if (data.dome === "closed") sEl.classList.add('status-closed');
                        else if (data.dome) sEl.classList.add('status-moving');

                        // Rotation / Slew status (data.slew expected boolean or truthy when moving)
                        const slewEl = document.getElementById('val-slew');
                        const isSlewing = !!data.slewing;
                        slewEl.innerText = isSlewing ? 'SLEWING' : 'IDLE';
                        if (azimuthChange > 0.1) slewEl.innerText += ' ▶'; // moving CW
                        else if (azimuthChange < -0.1) slewEl.innerText += ' ◀'; // moving CCW
                        else if (isSlewing) slewEl.innerText += ' ⬌'; // moving but no significant change
                        else slewEl.innerText += ' ⬤'; // idle
                        slewEl.className = 'status-badge';
                        if (isSlewing) slewEl.classList.add('status-moving'); else slewEl.classList.add('status-open');
                        // Update RSSI badge if available
                        try {
                          const rBadge = document.getElementById('rssi-text');
                          if (data && typeof data.lastLoggedRssi !== 'undefined' && data.lastLoggedRssi !== null) {
                            const rssi = Number(data.lastLoggedRssi);
                            rBadge.textContent = rssi + ' dBm';
                            rBadge.className = '';
                            if (rssi >= -50 && rssi <= -30) rBadge.classList.add('rssi-good-dark');
                            else if (rssi > -60 && rssi < -50) rBadge.classList.add('rssi-good');
                            else if (rssi > -70 && rssi <= -60) rBadge.classList.add('rssi-fair');
                            else if (rssi > -80 && rssi <= -70) rBadge.classList.add('rssi-weak');
                            else rBadge.classList.add('rssi-bad');
                          } else {
                            rBadge.textContent = '-';
                            rBadge.className = '';
                          }
                        } catch(e) {}
                        // Update Response badge if available
                        try {
                          const rBadge = document.getElementById('response-text');
                          if (data && typeof data.responseTimeMs === 'number') {
                            // add leading zero if response time is less than 10 ms
                            const responseTime = data.responseTimeMs < 100 ? '0' + data.responseTimeMs : data.responseTimeMs; 
                            rBadge.textContent = responseTime + ' ms';
                          } else {
                            rBadge.textContent = '-';
                          }
                        } catch(e) {} 
                    })
                    .then(() => hideConnectionWarning())
                    .catch(() => {
                      statusRequestInFlight = false;
                      showConnectionWarning('Controller offline');
                      document.getElementById('val-shutter').innerText = "OFFLINE";
                      document.getElementById('val-shutter').className = "status-badge";
                      document.getElementById('val-slew').innerText = "OFFLINE";
                      document.getElementById('val-slew').className = "status-badge";
                    });
                }
                
                setInterval(updateStatus, 1000);
                updateStatus(); // Execute immediately on dashboard startup

                // Updated Background Loop: Uses POST with action=getlog payload instead of GET query
                const updateLog = (e) => {
                    //return if autoupdate is not check or if refresh button was not pressed. 
                    if (!document.getElementById('chk-autoUpdate').checked && e !== 'click') return;
                    postAction('getlog')
                    .then(data => {
                        if (!data) return;
                        
                        // Process Web Logs
                        if (data.log && data.log.length > 0) {
                            const logHash = data.log;
                            if (logHash !== lastLogHash) {
                                lastLogHash = logHash;
                                const term = document.getElementById('web-terminal');
                                term.innerHTML = "";
                                let logLines = data.log.split('\n');
                                logLines.forEach(line => {
                                    const row = document.createElement('div');
                                    row.className = "terminal-line";
                                    row.innerText = "> " + line;
                                    term.appendChild(row);
                                });
                                term.scrollTop = term.scrollHeight;
                            }
                        }
                        
                    });
                }

                document.getElementById('btn-logrefresh').addEventListener('click', (e) => {
                  updateLog('click');
                });

                setInterval(() => updateLog('interval'), 10000);
                updateLog('interval'); // Execute immediately on dashboard startup

            </script>
        </body>
        </html>
      )OBSCTRLHTML";

      // Replace firmware placeholder and stream the HTML in chunks to avoid the
      // 2048-byte hardware block cutoff on Wiznet chips.
      String html = String(htmlPage);
      html.replace("__FIRMWARE_VERSION__", FIRMWARE_VERSION);

      size_t totalLength = html.length();
      size_t bytesSent = 0;
      const size_t chunkSize = 1024;

      while (bytesSent < totalLength) {
        size_t remaining = totalLength - bytesSent;
        size_t sizeToSend = (remaining > chunkSize) ? chunkSize : remaining;
        client.write((const uint8_t*)(html.c_str() + bytesSent), sizeToSend);
        bytesSent += sizeToSend;
        delayMicroseconds(10);
      }
    }
    // Redirect legacy `/setup` root to the new full setup path
    else if (requestLine.indexOf("GET /setup ") >= 0 || requestLine.indexOf("GET /setup/") >= 0 || requestLine.indexOf("GET /setup?") >= 0) {
      client.println("HTTP/1.1 302 Found");
      client.println("Location: /setup/v1/dome/0/setup");
      client.println("Connection: close");
      client.println();
    }
    else {
      sendAlpacaErrorResponse(client, clientTxID, nextServerTransactionID(), 0x400, "Bad request.");
    }

    // Flush and give the network stack (WiFi) time to actually push the final
    // TCP segment out before we tear down the socket. Closing right after write with
    // only a 1ms delay is a race: under load the response can still be sitting in the
    // send buffer, and the abrupt close then shows up on the client (NINA) side as
    // "The underlying connection was closed: An unexpected error occurred on a receive."
    client.flush();
    delay(5);
    // Before closing client, check idle-save condition
    if (!idleStateSaved && !slewing && currentShutterState != SHUTTER_OPENING && currentShutterState != SHUTTER_CLOSING) {
      unsigned long now = millis();
      if (now - lastActiveMillis > IDLE_SAVE_MS) {
        // Save once into FRAM
        logMessage("IDLE: Saving dome state to FRAM (idle >10min)");
        saveAzimuthToFlash();
        idleStateSaved = true;
      }
    }
    client.stop();
  }
}

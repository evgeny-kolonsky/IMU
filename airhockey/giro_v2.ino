/*
 * XIAO Ball v2.1 — Autonomous IMU Ball
 * Board: Seeed XIAO nRF52840 Sense
 *
 * States: IDLE → RECORDING → HAS_DATA → TRANSFERRING → IDLE
 * Commands: S(status) R:X(record) D(download) Z(sleep)
 */

#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"

// ═══════════════════════════════════════════════════
//  CONFIG
// ═══════════════════════════════════════════════════

#define DEVICE_NAME         "BALL_01"
#define SAMPLE_RATE_HZ      50
#define MAX_DURATION_SEC    30
#define MAX_SAMPLES         (MAX_DURATION_SEC * SAMPLE_RATE_HZ)  // 1000

#define SLEEP_TIMEOUT_MS    300000UL  // 5 min
#define TRANSFER_DELAY_MS   12        // Between BLE blocks (proven reliable)

// ═══════════════════════════════════════════════════
//  STATE MACHINE
// ═══════════════════════════════════════════════════

enum State { ST_IDLE, ST_RECORDING, ST_HAS_DATA, ST_TRANSFERRING };
State state = ST_IDLE;
const char* stateNames[] = {"IDLE", "RECORDING", "READY", "TRANSFER"};

// ═══════════════════════════════════════════════════
//  DATA
// ═══════════════════════════════════════════════════

struct DataPoint { int16_t ax, ay, az, gx, gy, gz; };

DataPoint logBuffer[MAX_SAMPLES];
int samplesRecorded = 0;
int targetSamples   = 0;

// ═══════════════════════════════════════════════════
//  HARDWARE
// ═══════════════════════════════════════════════════

LSM6DS3 myIMU(I2C_MODE, 0x6A);
BLEUart bleuart;

unsigned long lastActivityTime = 0;
unsigned long lastSampleTime   = 0;
unsigned long lastLedToggle    = 0;
unsigned long recordStartTime  = 0;
unsigned long lastBatRead      = 0;

float batteryVoltage = 0.0;
int   batteryPercent = 0;
bool  ledState       = false;
uint16_t connHandle  = BLE_CONN_HANDLE_INVALID;

// ═══════════════════════════════════════════════════
//  BATTERY
// ═══════════════════════════════════════════════════

void readBattery() {
  // Enable VBAT voltage divider (XIAO nRF52840 requires this)
  #ifdef VBAT_ENABLE
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  delay(5);
  #endif

  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);

  analogRead(PIN_VBAT);  // Discard first
  delay(1);

  // Average 8 readings
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(PIN_VBAT);
    delay(1);
  }
  float raw = sum / 8.0;

  #ifdef VBAT_ENABLE
  digitalWrite(VBAT_ENABLE, HIGH);
  #endif

  // Voltage divider: 1MOhm + 510KOhm
  batteryVoltage = raw * 3.0 / 4095.0 * (1000.0 + 510.0) / 510.0;

  // Clamp to sane LiPo range
  if (batteryVoltage > 4.30) batteryVoltage = 4.20;
  if (batteryVoltage < 2.50) batteryVoltage = 0.0;

  // LiPo discharge curve
  float v = batteryVoltage;
  if      (v >= 4.10) batteryPercent = 100;
  else if (v >= 3.95) batteryPercent = 80  + (int)((v - 3.95) * 133);
  else if (v >= 3.80) batteryPercent = 60  + (int)((v - 3.80) * 133);
  else if (v >= 3.70) batteryPercent = 40  + (int)((v - 3.70) * 200);
  else if (v >= 3.60) batteryPercent = 20  + (int)((v - 3.60) * 200);
  else if (v >= 3.50) batteryPercent = 5   + (int)((v - 3.50) * 150);
  else                 batteryPercent = 0;
  batteryPercent = constrain(batteryPercent, 0, 100);

  Serial.print("[BAT] raw="); Serial.print(raw, 0);
  Serial.print(" V="); Serial.print(batteryVoltage, 2);
  Serial.print(" %="); Serial.println(batteryPercent);
}

// ═══════════════════════════════════════════════════
//  DEEP SLEEP
// ═══════════════════════════════════════════════════

void enterDeepSleep() {
  Serial.println("[SLEEP] Entering deep sleep...");

  // Turn off ALL LEDs first (HIGH = OFF on XIAO nRF52840)
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  // Green charge LED (if defined)
  #ifdef LED_GREEN
  digitalWrite(LED_GREEN, HIGH);
  #endif
  // Also try pin 22 which is sometimes the charge indicator
  #ifdef PIN_LED2
  pinMode(PIN_LED2, OUTPUT);
  digitalWrite(PIN_LED2, HIGH);
  #endif

  // Visual: 3 slow blinks
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_RED, LOW);  delay(150);
    digitalWrite(LED_RED, HIGH); delay(250);
  }

  // Stop BLE
  Bluefruit.Advertising.stop();
  delay(100);

  // Configure IMU for tap-wakeup
  configureTapWakeup();

  // Deconfigure all LED pins to prevent current leak in System OFF
  // Writing HIGH first (LED off), then disconnecting the pin
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  nrf_gpio_cfg_default(g_ADigitalPinMap[LED_RED]);
  nrf_gpio_cfg_default(g_ADigitalPinMap[LED_BLUE]);
  #ifdef LED_GREEN
  digitalWrite(LED_GREEN, HIGH);
  nrf_gpio_cfg_default(g_ADigitalPinMap[LED_GREEN]);
  #endif

  // Configure INT1 pin for System OFF wake
  #ifdef PIN_LSM6DS3TR_C_INT1
  uint32_t nrfPin = g_ADigitalPinMap[PIN_LSM6DS3TR_C_INT1];
  #else
  uint32_t nrfPin = g_ADigitalPinMap[11];  // Fallback
  #endif

  nrf_gpio_cfg_sense_input(nrfPin,
                           NRF_GPIO_PIN_PULLDOWN,
                           NRF_GPIO_PIN_SENSE_HIGH);

  delay(50);
  NRF_POWER->SYSTEMOFF = 1;
  while (1) delay(1000);
}

void configureTapWakeup() {
  myIMU.writeRegister(0x10, 0x68);  // CTRL1_XL: 416Hz, +/-4g
  myIMU.writeRegister(0x11, 0x00);  // CTRL2_G: power-down gyro
  myIMU.writeRegister(0x58, 0x8E);  // TAP_CFG: XYZ + interrupts
  myIMU.writeRegister(0x59, 0x0C);  // TAP_THS: ~1.5g
  myIMU.writeRegister(0x5A, 0x7F);  // INT_DUR2: timing
  myIMU.writeRegister(0x5B, 0x80);  // WAKE_UP_THS: double-tap
  myIMU.writeRegister(0x5E, 0x08);  // MD1_CFG: route to INT1
  Serial.println("[SLEEP] Tap config done");
}

// ═══════════════════════════════════════════════════
//  LED
// ═══════════════════════════════════════════════════

void updateLED(unsigned long now) {
  switch (state) {
    case ST_IDLE:
      if (now - lastLedToggle >= 1000) {
        lastLedToggle = now; ledState = !ledState;
        digitalWrite(LED_RED, ledState ? LOW : HIGH);
      }
      break;
    case ST_RECORDING:
      if (now - lastLedToggle >= 100) {
        lastLedToggle = now; ledState = !ledState;
        digitalWrite(LED_RED, ledState ? LOW : HIGH);
      }
      break;
    case ST_HAS_DATA: {
      unsigned long phase = (now / 100) % 20;
      digitalWrite(LED_RED, (phase == 0 || phase == 2) ? LOW : HIGH);
    } break;
    case ST_TRANSFERRING:
      if (now - lastLedToggle >= 50) {
        lastLedToggle = now; ledState = !ledState;
        digitalWrite(LED_RED, ledState ? LOW : HIGH);
      }
      break;
  }
  digitalWrite(LED_BLUE, Bluefruit.connected() ? LOW : HIGH);
}

// ═══════════════════════════════════════════════════
//  BLE
// ═══════════════════════════════════════════════════

void bleConnectCb(uint16_t handle) {
  connHandle = handle;
  lastActivityTime = millis();
  Serial.println("[BLE] Connected");
}

void bleDisconnectCb(uint16_t handle, uint8_t reason) {
  connHandle = BLE_CONN_HANDLE_INVALID;
  if (state == ST_TRANSFERRING) state = ST_HAS_DATA;
  Serial.print("[BLE] Disconnected reason="); Serial.println(reason);
}

void setupBLE() {
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  // Request 247 MTU
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(bleConnectCb);
  Bluefruit.Periph.setDisconnectCallback(bleDisconnectCb);
  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
  Serial.println("[BLE] Advertising started");
}

// Send text line as single BLE write (prevents MTU splitting \n from content)
void sendLine(const char* str) {
  int len = strlen(str);
  uint8_t buf[80];
  memcpy(buf, str, len);
  buf[len] = '\n';
  bleuart.write(buf, len + 1);
}

// ═══════════════════════════════════════════════════
//  COMMAND PARSING
// ═══════════════════════════════════════════════════

void handleBLECommands() {
  if (!Bluefruit.connected() || !bleuart.available()) return;

  // Read entire BLE packet at once
  char cmd[32];
  int n = 0;
  while (bleuart.available() && n < 30) {
    cmd[n++] = (char)bleuart.read();
  }
  cmd[n] = '\0';

  // Strip \r \n
  while (n > 0 && (cmd[n-1] == '\r' || cmd[n-1] == '\n')) {
    cmd[--n] = '\0';
  }
  if (n == 0) return;

  Serial.print("[CMD] '"); Serial.print(cmd);
  Serial.print("' state="); Serial.println(stateNames[state]);

  lastActivityTime = millis();

  // ── S: Status ──
  if (cmd[0] == 'S' && n == 1) {
    if (state == ST_RECORDING) {
      // Don't process during recording — it disrupts sample timing
      sendLine("ERR:BUSY");
      return;
    }
    sendStatus();
    return;
  }

  // ── R:XX: Record ──
  if (cmd[0] == 'R' && n >= 3 && cmd[1] == ':') {
    int dur = atoi(&cmd[2]);
    dur = constrain(dur, 1, MAX_DURATION_SEC);
    if (state == ST_IDLE || state == ST_HAS_DATA) {
      startRecording(dur);
    } else {
      sendLine("ERR:BUSY");
    }
    return;
  }

  // ── D: Download ──
  if (cmd[0] == 'D' && n == 1) {
    if (samplesRecorded > 0 && (state == ST_HAS_DATA || state == ST_IDLE)) {
      state = ST_TRANSFERRING;
      sendDataOverBLE();
      state = ST_HAS_DATA;
    } else {
      sendLine("EMPTY");
    }
    return;
  }

  // ── G: Legacy download ──
  if (cmd[0] == 'G') {
    if (samplesRecorded > 0) {
      state = ST_TRANSFERRING;
      sendDataOverBLE();
      state = ST_HAS_DATA;
    } else {
      sendLine("EMPTY");
    }
    return;
  }

  // ── Z: Sleep ──
  if (cmd[0] == 'Z' && n == 1) {
    Serial.println("[CMD] Sleep requested");
    sendLine("SLEEPING");
    delay(300);
    if (Bluefruit.connected()) {
      Bluefruit.disconnect(connHandle);
      delay(500);
    }
    enterDeepSleep();
    return;
  }
}

// ═══════════════════════════════════════════════════
//  STATUS
// ═══════════════════════════════════════════════════

void sendStatus() {
  readBattery();
  char buf[64];
  int vInt = (int)batteryVoltage;
  int vFrac = ((int)(batteryVoltage * 100)) % 100;
  snprintf(buf, sizeof(buf), "ST:%s,%d,%d.%02d,%d,%d",
           stateNames[state], batteryPercent,
           vInt, vFrac, samplesRecorded, MAX_DURATION_SEC);
  sendLine(buf);
  Serial.print("[STATUS] "); Serial.println(buf);
}

// ═══════════════════════════════════════════════════
//  RECORDING
// ═══════════════════════════════════════════════════

void startRecording(int durationSec) {
  targetSamples = durationSec * SAMPLE_RATE_HZ;
  if (targetSamples > MAX_SAMPLES) targetSamples = MAX_SAMPLES;
  samplesRecorded = 0;
  state = ST_RECORDING;
  lastSampleTime = millis();
  recordStartTime = millis();

  char buf[32];
  snprintf(buf, sizeof(buf), "REC:START,%d,%d", durationSec, SAMPLE_RATE_HZ);
  sendLine(buf);
  Serial.print("[REC] Started "); Serial.print(durationSec); Serial.println("s");
}

void handleRecording() {
  unsigned long now = millis();
  unsigned long interval = 1000UL / SAMPLE_RATE_HZ;

  if (now - lastSampleTime >= interval) {
    lastSampleTime += interval;
    if (samplesRecorded < targetSamples) {
      logBuffer[samplesRecorded].ax = myIMU.readRawAccelX();
      logBuffer[samplesRecorded].ay = myIMU.readRawAccelY();
      logBuffer[samplesRecorded].az = myIMU.readRawAccelZ();
      logBuffer[samplesRecorded].gx = myIMU.readRawGyroX();
      logBuffer[samplesRecorded].gy = myIMU.readRawGyroY();
      logBuffer[samplesRecorded].gz = myIMU.readRawGyroZ();
      samplesRecorded++;
    }
    if (samplesRecorded >= targetSamples) {
      finishRecording();
    }
  }
}

void finishRecording() {
  state = ST_HAS_DATA;
  lastActivityTime = millis();
  char buf[32];
  snprintf(buf, sizeof(buf), "REC:DONE,%d", samplesRecorded);
  sendLine(buf);
  Serial.print("[REC] Done: "); Serial.println(samplesRecorded);

  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_RED, LOW); delay(40);
    digitalWrite(LED_RED, HIGH); delay(40);
  }
}

// ═══════════════════════════════════════════════════
//  DATA TRANSFER — PROVEN PROTOCOL, DO NOT CHANGE
// ═══════════════════════════════════════════════════

void sendDataOverBLE() {
  if (samplesRecorded == 0) { sendLine("EMPTY"); return; }

  Serial.print("[XFER] Start: "); Serial.println(samplesRecorded);
  digitalWrite(LED_RED, LOW);

  // 1. Header
  char header[32];
  snprintf(header, sizeof(header), "HDR:%d,%d", samplesRecorded, samplesRecorded * 12);
  sendLine(header);
  delay(500);

  // 2. Wait for RDY (ignore any stale commands like 'S')
  unsigned long start = millis();
  bool ready = false;
  while (millis() - start < 5000) {
    if (bleuart.available()) {
      char c = bleuart.read();
      if (c == 'R') {
        ready = true;
        while (bleuart.available()) bleuart.read();  // Flush rest
        break;
      }
      // Ignore anything else (stale status polls etc.)
      Serial.print("[XFER] Ignoring char: "); Serial.println(c);
    }
    delay(10);
  }
  if (!ready) {
    sendLine("TIMEOUT");
    Serial.println("[XFER] Timeout");
    digitalWrite(LED_RED, HIGH);
    return;
  }

  Serial.println("[XFER] RDY received");
  delay(200);

  // 3. Send blocks
  uint8_t block[14];
  uint16_t blockNum = 0;
  for (int i = 0; i < samplesRecorded; i++) {
    if (!Bluefruit.connected()) break;
    block[0]  = blockNum & 0xFF;          block[1]  = (blockNum >> 8) & 0xFF;
    block[2]  = logBuffer[i].ax & 0xFF;   block[3]  = (logBuffer[i].ax >> 8) & 0xFF;
    block[4]  = logBuffer[i].ay & 0xFF;   block[5]  = (logBuffer[i].ay >> 8) & 0xFF;
    block[6]  = logBuffer[i].az & 0xFF;   block[7]  = (logBuffer[i].az >> 8) & 0xFF;
    block[8]  = logBuffer[i].gx & 0xFF;   block[9]  = (logBuffer[i].gx >> 8) & 0xFF;
    block[10] = logBuffer[i].gy & 0xFF;   block[11] = (logBuffer[i].gy >> 8) & 0xFF;
    block[12] = logBuffer[i].gz & 0xFF;   block[13] = (logBuffer[i].gz >> 8) & 0xFF;

    bool sent = false;
    while (!sent) {
      if (!Bluefruit.connected()) break;
      if (bleuart.write(block, 14) == 14) sent = true;
      else delay(2);
    }
    blockNum++;
    delay(TRANSFER_DELAY_MS);
  }

  delay(500);
  sendLine("END");
  digitalWrite(LED_RED, HIGH);
  Serial.print("[XFER] Done: "); Serial.println(blockNum);
}

// ═══════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH); digitalWrite(LED_BLUE, HIGH);

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_RED, LOW); delay(60);
    digitalWrite(LED_RED, HIGH); delay(60);
  }

  Serial.println("=== XIAO Ball v2.1 ===");

  if (myIMU.begin() != 0) {
    Serial.println("[IMU] FAILED!");
    while (1) { digitalWrite(LED_RED, !digitalRead(LED_RED)); delay(100); }
  }
  Serial.println("[IMU] OK");

  readBattery();
  setupBLE();

  lastActivityTime = millis();
  state = ST_IDLE;
  samplesRecorded = 0;
  Serial.println("[READY]");
}

// ═══════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // USB serial passthrough (only when idle + no BLE)
  if (Serial && state == ST_IDLE && !Bluefruit.connected()) {
    Serial.print(myIMU.readFloatAccelX()); Serial.print(",");
    Serial.print(myIMU.readFloatAccelY()); Serial.print(",");
    Serial.print(myIMU.readFloatAccelZ()); Serial.print(",");
    Serial.print(myIMU.readFloatGyroX()); Serial.print(",");
    Serial.print(myIMU.readFloatGyroY()); Serial.print(",");
    Serial.println(myIMU.readFloatGyroZ());
    delay(20);
    lastActivityTime = now;
    return;
  }

  handleBLECommands();

  switch (state) {
    case ST_IDLE: case ST_HAS_DATA: delay(50); break;
    case ST_RECORDING: handleRecording(); break;
    case ST_TRANSFERRING: break;
  }

  updateLED(now);

  if (now - lastBatRead > 30000) { lastBatRead = now; readBattery(); }

  // Auto-sleep: ONLY when disconnected
  if (!Bluefruit.connected()) {
    if ((state == ST_IDLE || state == ST_HAS_DATA) && (now - lastActivityTime > SLEEP_TIMEOUT_MS)) {
      Serial.println("[SLEEP] Auto-sleep timeout");
      enterDeepSleep();
    }
  } else {
    lastActivityTime = now;
  }
}

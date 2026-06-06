/*
========================================================
 ESP32-C3 SUPER MINI TRANSMITTER + BMI160
 DVS / Phase DIY - ESP-NOW low latency motion packet
========================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <math.h>

#define DECK_ID 1

// ESP32-C3 Super Mini default I2C pins used by this project.
// Change if your board wiring is different.
#define SDA_PIN 6
#define SCL_PIN 7

#define BMI160_ADDR_PRIMARY 0x68
#define BMI160_ADDR_SECONDARY 0x69
#define BMI160_CHIP_ID 0x00
#define BMI160_GYRO_X_L 0x0C
#define BMI160_GYRO_Z_L 0x10
#define BMI160_CMD 0x7E
#define BMI160_GYR_CONF 0x42
#define BMI160_GYR_RANGE 0x43

#define ESPNOW_CHANNEL 11
#define SEND_RATE_HZ 500
#define SEND_INTERVAL_US (1000000UL / SEND_RATE_HZ)
#define HANDSHAKE_INTERVAL_MS 250
#define HANDSHAKE_TIMEOUT_MS 2000

#define PROTOCOL_VERSION 1
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4

#define LED_R 2
#define LED_G 3
#define LED_B 4
#define LED_COMMON_ANODE 0
#define LED_BLINK_MS 150

uint8_t receiverMAC[] = { 0xA0, 0xF2, 0x62, 0xF0, 0x89, 0xE8 };
uint8_t bmi160Address = BMI160_ADDR_PRIMARY;

float SMOOTHING = 0.16f;
float DEADZONE_RPM = 0.20f;
float RPM_MULTIPLIER = 1.00f;

// Auto-calibracao: o transmissor espera uma janela de gyro
// estavel com o toca-discos parado. Se houver movimento, a
// janela reinicia para nao gravar offset durante o giro.
#define AUTO_CALIBRATION_STABLE_SAMPLES 400
#define AUTO_CALIBRATION_SAMPLE_DELAY_MS 2
#define AUTO_CALIBRATION_STABLE_RAW_DELTA 120
#define AUTO_CALIBRATION_MIN_COMPARE_SAMPLES 20

typedef struct __attribute__((packed)) {
  uint8_t msgType;
  uint8_t version;
  uint8_t deckId;
  int16_t rpmCenti;
  int16_t gyroRaw;
  uint32_t seq;
  uint32_t timestampMicros;
} dvs_packet;

dvs_packet packet;

volatile bool receiverReady = false;
volatile uint32_t lastReceiverReplyMillis = 0;

float gyroOffsetZ = 0.0f;
float filteredRPM = 0.0f;
uint32_t sequenceNumber = 0;
uint32_t nextSendMicros = 0;

static inline uint8_t ledOnLevel() {
  return LED_COMMON_ANODE ? LOW : HIGH;
}

static inline uint8_t ledOffLevel() {
  return LED_COMMON_ANODE ? HIGH : LOW;
}

void setLED(bool red, bool green, bool blue) {
  digitalWrite(LED_R, red ? ledOnLevel() : ledOffLevel());
  digitalWrite(LED_G, green ? ledOnLevel() : ledOffLevel());
  digitalWrite(LED_B, blue ? ledOnLevel() : ledOffLevel());
}

void setupLED() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLED(false, false, false);
}

void blinkLED(bool red, bool green, bool blue, uint16_t times) {
  for (uint16_t i = 0; i < times; i++) {
    setLED(red, green, blue);
    delay(LED_BLINK_MS);
    setLED(false, false, false);
    delay(LED_BLINK_MS);
  }
}

static bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(bmi160Address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

static bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(bmi160Address);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(bmi160Address, len, true) != len) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

static bool readGyroZRaw(int16_t *gyroZ) {
  uint8_t bytes[2] = {};

  if (!readRegisters(BMI160_GYRO_Z_L, bytes, 2)) {
    return false;
  }

  *gyroZ = (int16_t)((bytes[1] << 8) | bytes[0]);
  return true;
}

bool probeBMI160() {
  uint8_t chipId = 0;

  bmi160Address = BMI160_ADDR_PRIMARY;
  if (readRegisters(BMI160_CHIP_ID, &chipId, 1) && chipId == 0xD1) {
    return true;
  }

  bmi160Address = BMI160_ADDR_SECONDARY;
  if (readRegisters(BMI160_CHIP_ID, &chipId, 1) && chipId == 0xD1) {
    return true;
  }

  return false;
}

void setupBMI160() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!probeBMI160()) {
    while (true) blinkLED(true, false, false, 2);
  }

  writeRegister(BMI160_CMD, 0x15); // gyro normal mode
  delay(80);

  // ODR 800Hz, normal filter mode. Good starting point for 500Hz transmit.
  writeRegister(BMI160_GYR_CONF, 0x2B);

  // +/-500 dps. Sensitivity is 65.6 LSB/dps.
  writeRegister(BMI160_GYR_RANGE, 0x02);
  delay(50);
}

void autoCalibrateGyroZ() {
  int stableSamples = 0;
  int32_t sum = 0;

  while (stableSamples < AUTO_CALIBRATION_STABLE_SAMPLES) {
    int16_t raw = 0;
    if (!readGyroZRaw(&raw)) {
      blinkLED(true, false, false, 1);
      continue;
    }

    if (stableSamples >= AUTO_CALIBRATION_MIN_COMPARE_SAMPLES) {
      int32_t mean = sum / stableSamples;
      if (abs((int32_t)raw - mean) > AUTO_CALIBRATION_STABLE_RAW_DELTA) {
        stableSamples = 0;
        sum = 0;
        setLED(false, false, false);
        delay(LED_BLINK_MS);
        continue;
      }
    }

    sum += raw;
    stableSamples++;

    if ((stableSamples % 25) == 0) {
      setLED(false, true, false);
    } else if ((stableSamples % 25) == 12) {
      setLED(false, false, false);
    }

    delay(AUTO_CALIBRATION_SAMPLE_DELAY_MS);
  }

  gyroOffsetZ = (float)sum / (float)stableSamples;
}

static inline float rawGyroToRPM(int16_t gyroRaw) {
  float correctedRaw = (float)gyroRaw - gyroOffsetZ;
  float dps = correctedRaw / 65.6f;
  float rpm = -(dps / 6.0f) * RPM_MULTIPLIER;

  if (fabsf(rpm) < DEADZONE_RPM) {
    rpm = 0.0f;
  }

  return rpm;
}

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len) {
  if (len != sizeof(dvs_packet)) {
    return;
  }

  dvs_packet incoming;
  memcpy(&incoming, dataPtr, sizeof(incoming));

  if (incoming.version != PROTOCOL_VERSION || incoming.deckId != DECK_ID) {
    return;
  }

  if (incoming.msgType == MSG_WELCOME || incoming.msgType == MSG_PING) {
    receiverReady = true;
    lastReceiverReplyMillis = millis();
  }
}

void setupEspNow() {
  blinkLED(false, false, true, 3);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    while (true) blinkLED(true, false, false, 2);
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    while (true) blinkLED(true, false, false, 2);
  }

  setLED(false, false, true);
}

void sendControlMessage(uint8_t msgType) {
  dvs_packet control = {};
  control.msgType = msgType;
  control.version = PROTOCOL_VERSION;
  control.deckId = DECK_ID;
  control.seq = sequenceNumber;
  control.timestampMicros = micros();

  esp_now_send(receiverMAC, (uint8_t *)&control, sizeof(control));
}

void waitForReceiver() {
  uint32_t lastHello = 0;

  while (!receiverReady) {
    uint32_t now = millis();
    if (now - lastHello >= HANDSHAKE_INTERVAL_MS) {
      sendControlMessage(MSG_HELLO);
      lastHello = now;
    }

    blinkLED(false, false, true, 1);
  }

  setLED(false, false, true);
}

void setup() {
  Serial.begin(115200);
  setupLED();
  setupBMI160();
  setupEspNow();
  waitForReceiver();
  autoCalibrateGyroZ();
  setLED(false, true, false);
  nextSendMicros = micros();
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextSendMicros) < 0) {
    return;
  }

  nextSendMicros += SEND_INTERVAL_US;
  if ((int32_t)(now - nextSendMicros) > (int32_t)SEND_INTERVAL_US) {
    nextSendMicros = now + SEND_INTERVAL_US;
  }

  int16_t gyroRaw = 0;
  if (!readGyroZRaw(&gyroRaw)) {
    return;
  }

  float rpm = rawGyroToRPM(gyroRaw);
  filteredRPM += (rpm - filteredRPM) * SMOOTHING;

  packet.msgType = MSG_DATA;
  packet.version = PROTOCOL_VERSION;
  packet.deckId = DECK_ID;
  packet.rpmCenti = (int16_t)constrain(lroundf(filteredRPM * 100.0f), -32768, 32767);
  packet.gyroRaw = gyroRaw;
  packet.seq = sequenceNumber++;
  packet.timestampMicros = now;

  esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

  if (millis() - lastReceiverReplyMillis > HANDSHAKE_TIMEOUT_MS) {
    receiverReady = false;
    setLED(false, false, false);
    waitForReceiver();
    setLED(false, true, false);
    nextSendMicros = micros();
  }
}

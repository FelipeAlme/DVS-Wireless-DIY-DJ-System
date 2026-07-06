/*
========================================================
 ESP32-C3 SUPER MINI TRANSMITTER + BMI270
 DVS / Phase DIY - ESP-NOW low latency motion packet

 Codigo: Felipe Alme
 Revisao: Doc GNA
========================================================
*/

#include <stdint.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <math.h>
#include "bmi270.h"

// ========================================================
// Protocolo DVS - constantes e structs
// ========================================================
#define PROTOCOL_VERSION 1
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4
#define MSG_BATTERY 5
#define ESPNOW_CHANNEL 1
#define SEND_RATE_HZ 250
#define SEND_INTERVAL_US (1000000UL / SEND_RATE_HZ)
#define HANDSHAKE_INTERVAL_MS 250
#define HANDSHAKE_TIMEOUT_MS 2000
#define PING_INTERVAL_MS 500
#define DECK_TIMEOUT_MS 2000
#define DEADZONE_RPM 0.20f
#define RPM_MULTIPLIER 1.00f
#define SMOOTHING 1.00f
#define CV02_RESOLUTION 1000
#define CV02_BITS 20
#define CV02_SEED 0x59017UL
#define CV02_TAPS 0x361e4UL
#define CV02_LENGTH 712000UL
#define CV02_PACKED_BYTES ((CV02_LENGTH + 7) / 8)
#define BASE_RPM 33.333f
#define MAX_RPM_RATIO 3.0f
#define RPM_SMOOTHING 0.22f
#define OUTPUT_GAIN 0.70f
#define SAMPLE_RATE 44100
#define DMA_BUF_LEN 64
#define DMA_BUF_COUNT 4

typedef struct __attribute__((packed)) {
  uint8_t msgType;
  uint8_t version;
  uint8_t deckId;
  int16_t rpmCenti;
  int16_t gyroRaw;
  uint32_t seq;
  uint32_t timestampMicros;
} dvs_packet;

typedef struct __attribute__((packed)) {
  uint8_t msgType;
  uint8_t version;
  uint8_t deckId;
  uint16_t batteryMv;
  uint8_t batteryPercent;
  uint32_t timestampMicros;
} dvs_battery_packet;

typedef struct {
  bool seen;
  int16_t rpmCenti;
  uint32_t lastSeq;
  uint32_t lastSeenMillis;
  uint32_t lastPingMillis;
  uint8_t mac[6];
} deck_state;

// ========================================================
// Interface I2C usada pelo driver oficial Bosch BMI270
// ========================================================
static BMI2_INTF_RETURN_TYPE bmi270I2CRead(
    uint8_t regAddr,
    uint8_t *regData,
    uint32_t length,
    void *interfacePtr) {
  const uint8_t deviceAddress = *(const uint8_t *)interfacePtr;

  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddr);
  if (Wire.endTransmission(false) != 0) {
    return BMI2_E_COM_FAIL;
  }

  if (Wire.requestFrom(deviceAddress, (size_t)length, true) != length) {
    return BMI2_E_COM_FAIL;
  }

  for (uint32_t i = 0; i < length; i++) {
    regData[i] = Wire.read();
  }

  return BMI2_OK;
}

static BMI2_INTF_RETURN_TYPE bmi270I2CWrite(
    uint8_t regAddr,
    const uint8_t *regData,
    uint32_t length,
    void *interfacePtr) {
  const uint8_t deviceAddress = *(const uint8_t *)interfacePtr;

  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddr);
  for (uint32_t i = 0; i < length; i++) {
    Wire.write(regData[i]);
  }

  return Wire.endTransmission(true) == 0 ? BMI2_OK : BMI2_E_COM_FAIL;
}

static void bmi270DelayUs(uint32_t period, void *interfacePtr) {
  (void)interfacePtr;
  delayMicroseconds(period);
}

// ========================================================
// LED RGB
// ========================================================
static inline void ledSet(uint8_t r, uint8_t g, uint8_t b, bool red, bool green, bool blue, bool anode) {
  digitalWrite(r, red ? (anode ? LOW : HIGH) : (anode ? HIGH : LOW));
  digitalWrite(g, green ? (anode ? LOW : HIGH) : (anode ? HIGH : LOW));
  digitalWrite(b, blue ? (anode ? LOW : HIGH) : (anode ? HIGH : LOW));
}

static inline void ledSetup(uint8_t r, uint8_t g, uint8_t b, bool anode) {
  pinMode(r, OUTPUT); pinMode(g, OUTPUT); pinMode(b, OUTPUT);
  ledSet(r, g, b, false, false, false, anode);
}

static inline void ledBlink(uint8_t r, uint8_t g, uint8_t b, bool red, bool green, bool blue, uint16_t n, bool anode, uint16_t ms) {
  for (uint16_t i = 0; i < n; i++) { ledSet(r, g, b, red, green, blue, anode); delay(ms); ledSet(r, g, b, false, false, false, anode); delay(ms); }
}

// ========================================================
// ESP-NOW transmitter helpers
// ========================================================
static inline bool espnowSetupTransmitter(const uint8_t *rxMac, esp_now_send_cb_t sCb, esp_now_recv_cb_t rCb) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(sCb);
  esp_now_register_recv_cb(rCb);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, rxMac, 6);
  p.channel = ESPNOW_CHANNEL;
  p.encrypt = false;
  return esp_now_add_peer(&p) == ESP_OK;
}

static inline void transmitterSendControl(const uint8_t *rxMac, uint8_t deckId, uint8_t msgType, uint32_t seq) {
  dvs_packet ctrl = {};
  ctrl.msgType = msgType;
  ctrl.version = PROTOCOL_VERSION;
  ctrl.deckId = deckId;
  ctrl.seq = seq;
  ctrl.timestampMicros = micros();
  esp_now_send(rxMac, (uint8_t *)&ctrl, sizeof(ctrl));
}

// ========================================================
// Fim do bloco compartilhado
// ========================================================

#define DECK_ID 2

// ESP32-C3 Super Mini default I2C pins used by this project.
#define SDA_PIN 9
#define SCL_PIN 10

#define BMI270_ADDR_PRIMARY BMI2_I2C_PRIM_ADDR
#define BMI270_ADDR_SECONDARY BMI2_I2C_SEC_ADDR
#define BMI270_I2C_TRANSFER_LENGTH 32
#define BMI270_GYRO_SENSITIVITY_500_DPS 65.536f

#define LED_R 2
#define LED_G 3
#define LED_B 4
#define LED_COMMON_ANODE 0
#define LED_BLINK_MS 150

// Bateria Li-ion medida no GPIO0 por divisor 220k/220k.
// BAT+ -> 220k -> GPIO0 -> 220k -> GND.
#define BATTERY_ADC_PIN 0
#define BATTERY_READ_INTERVAL_MS 300000UL
#define BATTERY_SAMPLE_INTERVAL_MS 10UL
#define BATTERY_ADC_SAMPLES 24
#define BATTERY_MIN_VALID_MV 2500
#define BATTERY_MAX_VALID_MV 4350
#define BATTERY_PERCENT_UNKNOWN 255
const float BATTERY_DIVIDER_FACTOR = 2.0f;
const float BATTERY_CALIBRATION_FACTOR = 1.0f;

uint8_t receiverMAC[] = {   0xE0,0x72,0xA1,0xD6,0x4E,0xF4 };
uint8_t bmi270Address = BMI270_ADDR_PRIMARY;
struct bmi2_dev bmi270Device = {};

// Auto-calibracao: o transmissor espera uma janela de gyro
// estavel com o toca-discos parado. Se houver movimento, a
// janela reinicia para nao gravar offset durante o giro.
#define AUTO_CALIBRATION_STABLE_SAMPLES 400
#define AUTO_CALIBRATION_SAMPLE_DELAY_MS 2
#define AUTO_CALIBRATION_STABLE_RAW_DELTA 200
#define AUTO_CALIBRATION_MIN_COMPARE_SAMPLES 20
#define AUTO_CALIBRATION_SMOOTH_WINDOW 4
#define AUTO_CALIBRATION_MAX_RESETS 15
#define AUTO_CALIBRATION_MAX_OFFSET_RAW 500

#define I2C_MAX_CONSECUTIVE_FAILURES 20
#define ESPNOW_SEND_WATCHDOG_US 20000UL

dvs_packet packet;
dvs_battery_packet batteryPacket;

volatile bool receiverReady = false;
volatile uint32_t lastReceiverReplyMillis = 0;
volatile bool sendInFlight = false;
volatile uint32_t sendStartedMicros = 0;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE txMux = portMUX_INITIALIZER_UNLOCKED;

float gyroOffsetZ = 0.0f;
float filteredRPM = 0.0f;
uint32_t sequenceNumber = 0;
uint32_t nextSendMicros = 0;
uint16_t i2cConsecutiveFailures = 0;
uint16_t batteryMillivolts = 0;
uint8_t batteryPercent = BATTERY_PERCENT_UNKNOWN;
uint32_t batteryAdcTotalMv = 0;
uint32_t nextBatteryReadMillis = 0;
uint32_t lastBatterySampleMillis = 0;
uint8_t batterySampleCount = 0;
bool batterySampling = false;
bool batteryReportPending = false;

// --------------------------------------------------------
// Converte tensao Li-ion em porcentagem aproximada.
// --------------------------------------------------------
static uint8_t batteryPercentFromMillivolts(uint16_t batteryMv) {
  if (batteryMv < BATTERY_MIN_VALID_MV || batteryMv > BATTERY_MAX_VALID_MV) {
    return BATTERY_PERCENT_UNKNOWN;
  }

  struct BatteryPoint {
    uint16_t mv;
    uint8_t percent;
  };

  static const BatteryPoint curve[] = {
    { 4200, 100 }, { 4110, 90 }, { 4020, 80 }, { 3950, 70 },
    { 3880, 60 }, { 3820, 50 }, { 3770, 40 }, { 3730, 30 },
    { 3680, 20 }, { 3600, 10 }, { 3400, 5 }, { 3000, 0 }
  };

  if (batteryMv >= curve[0].mv) {
    return curve[0].percent;
  }

  const size_t curveSize = sizeof(curve) / sizeof(curve[0]);
  for (size_t i = 1; i < curveSize; i++) {
    if (batteryMv >= curve[i].mv) {
      const BatteryPoint high = curve[i - 1];
      const BatteryPoint low = curve[i];
      const uint16_t rangeMv = high.mv - low.mv;
      const uint16_t deltaMv = batteryMv - low.mv;
      const uint8_t rangePercent = high.percent - low.percent;
      return low.percent + (uint8_t)((deltaMv * rangePercent) / rangeMv);
    }
  }

  return 0;
}

// --------------------------------------------------------
// Faz a media em pequenas etapas para nao bloquear RPM.
// Mede no boot e depois a cada cinco minutos.
// --------------------------------------------------------
static void setupBatteryMonitor() {
  pinMode(BATTERY_ADC_PIN, INPUT);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
  analogReadMilliVolts(BATTERY_ADC_PIN);
  nextBatteryReadMillis = millis();
}

static void serviceBatteryMonitor() {
  uint32_t now = millis();

  if (!batterySampling && (int32_t)(now - nextBatteryReadMillis) >= 0) {
    batterySampling = true;
    batteryAdcTotalMv = 0;
    batterySampleCount = 0;
    lastBatterySampleMillis = 0;
  }

  if (!batterySampling
      || (batterySampleCount > 0
          && now - lastBatterySampleMillis < BATTERY_SAMPLE_INTERVAL_MS)) {
    return;
  }

  batteryAdcTotalMv += analogReadMilliVolts(BATTERY_ADC_PIN);
  batterySampleCount++;
  lastBatterySampleMillis = now;

  if (batterySampleCount < BATTERY_ADC_SAMPLES) {
    return;
  }

  float adcMv = batteryAdcTotalMv / (float)BATTERY_ADC_SAMPLES;
  float measuredMv = adcMv * BATTERY_DIVIDER_FACTOR * BATTERY_CALIBRATION_FACTOR;
  batteryMillivolts = measuredMv > 0.0f ? (uint16_t)(measuredMv + 0.5f) : 0;
  batteryPercent = batteryPercentFromMillivolts(batteryMillivolts);
  batteryReportPending = true;
  batterySampling = false;
  nextBatteryReadMillis = now + BATTERY_READ_INTERVAL_MS;
}

// --------------------------------------------------------
// Leitura do eixo Z pelo driver oficial Bosch
// --------------------------------------------------------
static bool readGyroZRaw(int16_t *gyroZ) {
  struct bmi2_sens_data sensorData = {};
  int8_t result = bmi2_get_sensor_data(&sensorData, &bmi270Device);

  if (result != BMI2_OK) {
    return false;
  }

  *gyroZ = sensorData.gyr.z;
  return true;
}

// --------------------------------------------------------
// Inicializa o driver e carrega o arquivo de configuracao.
// O blob interno do BMI270 e perdido sem energia, portanto
// bmi270_init() precisa rodar em todo boot e recuperacao I2C.
// --------------------------------------------------------
static bool configureBMI270AtAddress(uint8_t address) {
  bmi270Address = address;
  bmi270Device = {};
  bmi270Device.intf = BMI2_I2C_INTF;
  bmi270Device.intf_ptr = &bmi270Address;
  bmi270Device.read = bmi270I2CRead;
  bmi270Device.write = bmi270I2CWrite;
  bmi270Device.delay_us = bmi270DelayUs;
  bmi270Device.read_write_len = BMI270_I2C_TRANSFER_LENGTH;
  bmi270Device.config_file_ptr = NULL;

  if (bmi270_init(&bmi270Device) != BMI2_OK) {
    return false;
  }

  struct bmi2_sens_config gyroConfig = {};
  gyroConfig.type = BMI2_GYRO;
  if (bmi2_get_sensor_config(&gyroConfig, 1, &bmi270Device) != BMI2_OK) {
    return false;
  }

  // 800 Hz oferece quatro amostras do sensor por pacote ESP-NOW de 250 Hz.
  gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_800HZ;
  gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_500;
  gyroConfig.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
  gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

  if (bmi2_set_sensor_config(&gyroConfig, 1, &bmi270Device) != BMI2_OK) {
    return false;
  }

  uint8_t sensor = BMI2_GYRO;
  if (bmi2_sensor_enable(&sensor, 1, &bmi270Device) != BMI2_OK) {
    return false;
  }

  delay(50);
  return true;
}

static bool configureBMI270() {
  return configureBMI270AtAddress(BMI270_ADDR_PRIMARY)
      || configureBMI270AtAddress(BMI270_ADDR_SECONDARY);
}

void setupBMI270() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!configureBMI270()) {
    while (true) {
      ledBlink(
          LED_R,
          LED_G,
          LED_B,
          true,
          false,
          false,
          2,
          LED_COMMON_ANODE,
          LED_BLINK_MS);
    }
  }
}

// --------------------------------------------------------
// Auto-calibracao por janela estavel com media movel
// --------------------------------------------------------
static float calibrateGyroZAttempt(bool *timedOut) {
  int32_t stableSamples = 0;
  int32_t sum = 0;
  int32_t resetCount = 0;
  int16_t smoothRing[AUTO_CALIBRATION_SMOOTH_WINDOW] = {};
  uint8_t smoothIdx = 0;
  uint8_t smoothCount = 0;

  *timedOut = false;

  while (stableSamples < AUTO_CALIBRATION_STABLE_SAMPLES) {
    int16_t raw = 0;
    if (!readGyroZRaw(&raw)) {
      ledBlink(LED_R, LED_G, LED_B, true, false, false, 1, LED_COMMON_ANODE, LED_BLINK_MS);
      continue;
    }

    // Media movel simples para reduzir ruido antes da comparacao
    smoothRing[smoothIdx] = raw;
    smoothIdx = (smoothIdx + 1) % AUTO_CALIBRATION_SMOOTH_WINDOW;
    if (smoothCount < AUTO_CALIBRATION_SMOOTH_WINDOW) {
      smoothCount++;
    }

    int32_t smoothed = 0;
    for (uint8_t j = 0; j < smoothCount; j++) {
      smoothed += smoothRing[j];
    }
    smoothed /= smoothCount;

    if (stableSamples >= AUTO_CALIBRATION_MIN_COMPARE_SAMPLES) {
      int32_t mean = sum / stableSamples;
      if (abs(smoothed - mean) > AUTO_CALIBRATION_STABLE_RAW_DELTA) {
        stableSamples = 0;
        sum = 0;
        resetCount++;
        ledSet(LED_R, LED_G, LED_B, false, false, false, LED_COMMON_ANODE);
        delay(LED_BLINK_MS);

        if (resetCount >= AUTO_CALIBRATION_MAX_RESETS) {
          *timedOut = true;
          return 0.0f;
        }
        continue;
      }
    }

    sum += smoothed;
    stableSamples++;

    if ((stableSamples % 25) == 0) {
      ledSet(LED_R, LED_G, LED_B, false, true, false, LED_COMMON_ANODE);
    } else if ((stableSamples % 25) == 12) {
      ledSet(LED_R, LED_G, LED_B, false, false, false, LED_COMMON_ANODE);
    }

    delay(AUTO_CALIBRATION_SAMPLE_DELAY_MS);
  }

  return (float)sum / (float)stableSamples;
}

// --------------------------------------------------------
// Auto-calibracao com validacao e retentativa
// --------------------------------------------------------
void autoCalibrateGyroZ() {
  bool timedOut = false;

  while (true) {
    float offset = calibrateGyroZAttempt(&timedOut);

    if (timedOut) {
      // Muitos resets: provavelmente ha movimento constante.
      // Pisca vermelho rapido e reinicia a tentativa.
      ledBlink(LED_R, LED_G, LED_B, true, false, false, 4, LED_COMMON_ANODE, LED_BLINK_MS);
      delay(500);
      continue;
    }

    if (fabsf(offset) <= (float)AUTO_CALIBRATION_MAX_OFFSET_RAW) {
      gyroOffsetZ = offset;
      return;
    }

    // Offset fora do esperado: sensor pode estar girando.
    // Pisca vermelho+azul e reinicia.
    ledBlink(LED_R, LED_G, LED_B, true, false, true, 2, LED_COMMON_ANODE, LED_BLINK_MS);
    delay(500);
  }
}

// --------------------------------------------------------
// Conversao gyro raw -> RPM
// --------------------------------------------------------
static inline float rawGyroToRPM(int16_t gyroRaw) {
  float correctedRaw = (float)gyroRaw - gyroOffsetZ;
  float dps = correctedRaw / BMI270_GYRO_SENSITIVITY_500_DPS;
  float rpm = -(dps / 6.0f) * RPM_MULTIPLIER;

  if (fabsf(rpm) < DEADZONE_RPM) {
    rpm = 0.0f;
  }

  return rpm;
}

// --------------------------------------------------------
// Recuperacao I2C: reinicia o barramento apos N falhas
// --------------------------------------------------------
static void recoverI2C() {
  Wire.end();
  delay(10);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  configureBMI270();
  i2cConsecutiveFailures = 0;
}

// --------------------------------------------------------
// Callbacks ESP-NOW
// --------------------------------------------------------
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  portENTER_CRITICAL(&txMux);
  sendInFlight = false;
  sendStartedMicros = 0;
  portEXIT_CRITICAL(&txMux);
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
    portENTER_CRITICAL(&rxMux);
    receiverReady = true;
    lastReceiverReplyMillis = millis();
    portEXIT_CRITICAL(&rxMux);
  }
}

// --------------------------------------------------------
// Handshake com o receiver
// --------------------------------------------------------
void waitForReceiver() {
  uint32_t lastHello = 0;

  while (true) {
    bool ready = false;
    portENTER_CRITICAL(&rxMux);
    ready = receiverReady;
    portEXIT_CRITICAL(&rxMux);

    if (ready) break;

    uint32_t now = millis();
    if (now - lastHello >= HANDSHAKE_INTERVAL_MS) {
      transmitterSendControl(receiverMAC, DECK_ID, MSG_HELLO, sequenceNumber++);
      lastHello = now;
    }

    ledBlink(LED_R, LED_G, LED_B, false, false, true, 1, LED_COMMON_ANODE, LED_BLINK_MS);
  }

  ledSet(LED_R, LED_G, LED_B, false, false, true, LED_COMMON_ANODE);
}

// --------------------------------------------------------
// Verifica timeout de handshake (chamado no loop)
// --------------------------------------------------------
static void checkHandshakeTimeout() {
  uint32_t lastReply;
  portENTER_CRITICAL(&rxMux);
  lastReply = lastReceiverReplyMillis;
  portEXIT_CRITICAL(&rxMux);

  if (millis() - lastReply > HANDSHAKE_TIMEOUT_MS) {
    portENTER_CRITICAL(&rxMux);
    receiverReady = false;
    portEXIT_CRITICAL(&rxMux);

    ledSet(LED_R, LED_G, LED_B, false, false, false, LED_COMMON_ANODE);
    waitForReceiver();
    ledSet(LED_R, LED_G, LED_B, false, true, false, LED_COMMON_ANODE);
    nextSendMicros = micros();

    portENTER_CRITICAL(&rxMux);
    lastReceiverReplyMillis = millis();
    portEXIT_CRITICAL(&rxMux);
  }
}

// --------------------------------------------------------
// Envia somente quando o callback do pacote anterior voltou.
// Isso evita congestionamento da fila interna do ESP-NOW.
// --------------------------------------------------------
static bool trySendPacket(const uint8_t *data, size_t length) {
  uint32_t now = micros();

  portENTER_CRITICAL(&txMux);
  if (sendInFlight && (uint32_t)(now - sendStartedMicros) > ESPNOW_SEND_WATCHDOG_US) {
    sendInFlight = false;
  }
  if (sendInFlight) {
    portEXIT_CRITICAL(&txMux);
    return false;
  }
  sendInFlight = true;
  sendStartedMicros = now;
  portEXIT_CRITICAL(&txMux);

  esp_err_t result = esp_now_send(receiverMAC, data, length);
  if (result != ESP_OK) {
    portENTER_CRITICAL(&txMux);
    sendInFlight = false;
    sendStartedMicros = 0;
    portEXIT_CRITICAL(&txMux);
    return false;
  }

  return true;
}

static bool trySendBatteryReport() {
  if (!batteryReportPending) {
    return false;
  }

  batteryPacket.msgType = MSG_BATTERY;
  batteryPacket.version = PROTOCOL_VERSION;
  batteryPacket.deckId = DECK_ID;
  batteryPacket.batteryMv = batteryMillivolts;
  batteryPacket.batteryPercent = batteryPercent;
  batteryPacket.timestampMicros = micros();

  if (!trySendPacket((const uint8_t *)&batteryPacket, sizeof(batteryPacket))) {
    return false;
  }

  batteryReportPending = false;
  return true;
}

// --------------------------------------------------------
// Setup
// --------------------------------------------------------
void setup() {
  ledSetup(LED_R, LED_G, LED_B, LED_COMMON_ANODE);
  setupBatteryMonitor();
  setupBMI270();

  if (!espnowSetupTransmitter(receiverMAC, OnDataSent, OnDataRecv)) {
    while (true) ledBlink(LED_R, LED_G, LED_B, true, false, false, 2, LED_COMMON_ANODE, LED_BLINK_MS);
  }

  ledSet(LED_R, LED_G, LED_B, false, false, true, LED_COMMON_ANODE);
  waitForReceiver();
  autoCalibrateGyroZ();

  portENTER_CRITICAL(&rxMux);
  lastReceiverReplyMillis = millis();
  portEXIT_CRITICAL(&rxMux);

  ledSet(LED_R, LED_G, LED_B, false, true, false, LED_COMMON_ANODE);
  nextSendMicros = micros();
}

// --------------------------------------------------------
// Loop principal - leitura e envio controlado em ate 250 Hz
// --------------------------------------------------------
void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextSendMicros) < 0) {
    return;
  }

  nextSendMicros += SEND_INTERVAL_US;
  if ((int32_t)(now - nextSendMicros) > (int32_t)SEND_INTERVAL_US) {
    nextSendMicros = now + SEND_INTERVAL_US;
  }

  serviceBatteryMonitor();
  if (trySendBatteryReport()) {
    checkHandshakeTimeout();
    return;
  }

  int16_t gyroRaw = 0;
  if (!readGyroZRaw(&gyroRaw)) {
    i2cConsecutiveFailures++;
    if (i2cConsecutiveFailures >= I2C_MAX_CONSECUTIVE_FAILURES) {
      recoverI2C();
    }
    checkHandshakeTimeout();
    return;
  }
  i2cConsecutiveFailures = 0;

  float rpm = rawGyroToRPM(gyroRaw);
  filteredRPM += (rpm - filteredRPM) * SMOOTHING;

  packet.msgType = MSG_DATA;
  packet.version = PROTOCOL_VERSION;
  packet.deckId = DECK_ID;
  packet.rpmCenti = (int16_t)constrain(lroundf(filteredRPM * 100.0f), -32768, 32767);
  packet.gyroRaw = gyroRaw;
  packet.seq = sequenceNumber;
  packet.timestampMicros = now;

  if (trySendPacket((const uint8_t *)&packet, sizeof(packet))) {
    sequenceNumber++;
  }

  checkHandshakeTimeout();
}

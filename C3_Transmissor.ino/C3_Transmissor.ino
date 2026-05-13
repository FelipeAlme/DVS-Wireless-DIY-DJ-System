/*
========================================================
 ESP32-C3 TRANSMISSOR DVS / PHASE DIY
 ULTRA LOW LATENCY VERSION
========================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

// =====================================================
// MPU6050
// =====================================================

#define SDA_PIN 8
#define SCL_PIN 9

#define MPU6050_ADDR 0x68

// =====================================================
// MAC ESP32-S3
// =====================================================

uint8_t receiverMAC[] = {

  0xE0,
  0x72,
  0xA1,
  0xD6,
  0x4E,
  0xF4
};

// =====================================================
// AJUSTES
// =====================================================

// smoothing ultra rápido
float SMOOTHING = 0.08f;

// deadzone
float DEADZONE = 0.8f;

// ajuste fino RPM
float RPM_MULTIPLIER = 1.00f;

// =====================================================
// DADOS
// =====================================================

typedef struct {

  float rpm;
  float gyro;

} gyro_packet;

gyro_packet data;

// =====================================================
// FILTRO
// =====================================================

volatile float filteredRPM = 0.0f;

// =====================================================
// CALLBACK
// =====================================================

void OnDataSent(

  const wifi_tx_info_t *info,

  esp_now_send_status_t status

) {

  // sem Serial aqui
  // reduz MUITO a latência
}

// =====================================================
// MPU SETUP
// =====================================================

void setupMPU() {

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  // I2C FAST
  // reduz latência

  Wire.setClock(400000);

  // wakeup MPU

  Wire.beginTransmission(
    MPU6050_ADDR
  );

  Wire.write(0x6B);
  Wire.write(0x00);

  Wire.endTransmission(true);

  // gyro ±500dps

  Wire.beginTransmission(
    MPU6050_ADDR
  );

  Wire.write(0x1B);

  // ±500°/s

  Wire.write(0x08);

  Wire.endTransmission(true);

  delay(50);
}

// =====================================================
// READ Z
// =====================================================

static inline float IRAM_ATTR readRPM_Z() {

  int16_t gyroZ;

  // gyro z register

  Wire.beginTransmission(
    MPU6050_ADDR
  );

  Wire.write(0x47);

  Wire.endTransmission(false);

  Wire.requestFrom(
    MPU6050_ADDR,
    2,
    true
  );

  gyroZ =
    (Wire.read() << 8)
    | Wire.read();

  // convert

  float dps =
    gyroZ / 65.5f;

  // REVERSE CORRETO

  float rpm =
    -(dps / 6.0f);

  rpm *= RPM_MULTIPLIER;

  // deadzone

  if(
    fabs(rpm)
    < DEADZONE
  ) {

    rpm = 0.0f;
  }

  return rpm;
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  // WIFI

  WiFi.mode(WIFI_STA);

  // IMPORTANTÍSSIMO
  // reduz delay ESP-NOW

  WiFi.setSleep(false);

  // MPU

  setupMPU();

  // ESP NOW

  if(
    esp_now_init()
    != ESP_OK
  ) {

    return;
  }

  esp_now_register_send_cb(
    OnDataSent
  );

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    receiverMAC,
    6
  );

  peerInfo.channel = 0;

  peerInfo.encrypt = false;

  if(
    esp_now_add_peer(
      &peerInfo
    ) != ESP_OK
  ) {

    return;
  }
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // =========================================
  // LEITURA
  // =========================================

  float rpm =
    readRPM_Z();

  // =========================================
  // FILTRO
  // =========================================

  filteredRPM +=

    (
      rpm
      - filteredRPM
    )

    * SMOOTHING;

  // =========================================
  // ENVIO
  // =========================================

  data.rpm =
    filteredRPM;

  data.gyro =
    filteredRPM;

  esp_now_send(

    receiverMAC,

    (uint8_t *)&data,

    sizeof(data)
  );

  // sem delay
  // sem serial
}
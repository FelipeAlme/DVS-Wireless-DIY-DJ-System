/*
========================================================
 DIY DVS / PHASE STYLE - ESP32-S3 RECEIVER
 FINAL VERSION
========================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

#include "timecode.h"

// =====================================================
// I2S
// =====================================================

#define BCK   2
#define LRCK  1
#define DATA  42

// =====================================================
// RGB LED
// =====================================================


#define LED_PIN 48
#define NUM_LEDS 1

Adafruit_NeoPixel led(
  NUM_LEDS,
  LED_PIN,
  NEO_GRB + NEO_KHZ800
);

// =====================================================
// AUDIO CONFIG
// =====================================================

#define SAMPLE_RATE      44100
#define DMA_BUF_LEN      32
#define DMA_BUF_COUNT    3

// =====================================================
// AJUSTES
// =====================================================

float DEADZONE    = 0.05f;
float OUTPUT_GAIN = 0.90f;
float MAX_RATIO   = 1.20f;
float SMOOTHING   = 0.08f;

// =====================================================
// ESP-NOW DATA
// =====================================================

typedef struct {
  float rpm;
  float gyro;
} gyro_packet;

gyro_packet incomingData;

// =====================================================
// WAV STATE
// =====================================================

#define TOTAL_FRAMES (NUM_ELEMENTS / 2)

volatile int32_t wavPosition = 0;

float filteredRPM = 0.0f;

bool connected = false;
unsigned long lastPacket = 0;

// =====================================================
// CALIBRATION
// =====================================================

bool calibrated = false;
float baseRPM = 33.0f;
unsigned long calibrationStart = 0;
float calibrationSum = 0.0f;
int calibrationCount = 0;

// =====================================================
// LED CONTROL
// =====================================================

void setLED(bool state) {

  if(state) {

    // VERDE
    led.setPixelColor(
      0,
      led.Color(0, 255, 0)
    );

  } else {

    // VERMELHO
    led.setPixelColor(
      0,
      led.Color(255, 0, 0)
    );
  }

  led.show();
}

// =====================================================
// ESP-NOW CALLBACK
// =====================================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *dataPtr,
  int len
) {
  memcpy(&incomingData, dataPtr, sizeof(incomingData));

  lastPacket = millis();
}

// =====================================================
// WRAP POSITION
// =====================================================

static inline int32_t wrapPos(int32_t p) {

  int32_t max = TOTAL_FRAMES << 8;

  while(p >= max) p -= max;
  while(p < 0)    p += max;

  return p;
}

// =====================================================
// STEREO INTERPOLATION
// =====================================================

static inline void interpolateStereo(
  int32_t pos,
  int16_t *l,
  int16_t *r
) {

  int frame = pos >> 8;
  int next  = frame + 1;

  if(next >= TOTAL_FRAMES)
    next = 0;

  float frac = (pos & 0xFF) / 256.0f;

  int i0 = frame * 2;
  int i1 = next  * 2;

  float left =
    data[i0] + (data[i1] - data[i0]) * frac;

  float right =
    data[i0 + 1] + (data[i1 + 1] - data[i0 + 1]) * frac;

  *l = (int16_t)left;
  *r = (int16_t)right;
}

// =====================================================
// I2S SETUP
// =====================================================

void setupI2S() {

  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    .bck_io_num = BCK,
    .ws_io_num = LRCK,
    .data_out_num = DATA,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// =====================================================
// AUDIO TASK
// =====================================================

void audioTask(void *param) {

  int16_t buffer[DMA_BUF_LEN * 2];

  while(true) {

    // =========================================
    // CALIBRAÇÃO
    // =========================================

    if(!calibrated) {

      if(millis() - calibrationStart < 2000) {
        calibrationSum += fabs(filteredRPM);
        calibrationCount++;
      } else {
        if(calibrationCount > 0)
          baseRPM = calibrationSum / calibrationCount;

        calibrated = true;
      }
    }

    // =========================================
    // RATIO
    // =========================================

    float ratio = 0.0f;

    if(calibrated && baseRPM > 0.01f)
      ratio = filteredRPM / baseRPM;

    if(fabs(ratio) < DEADZONE)
      ratio = 0.0f;

    if(ratio > MAX_RATIO) ratio = MAX_RATIO;
    if(ratio < -MAX_RATIO) ratio = -MAX_RATIO;

    int32_t step = (int32_t)(ratio * 256.0f);

    // =========================================
    // AUDIO
    // =========================================

    for(int i = 0; i < DMA_BUF_LEN; i++) {

      wavPosition += step;
      wavPosition = wrapPos(wavPosition);

      int16_t l, r;

      interpolateStereo(wavPosition, &l, &r);

      l = (int16_t)(l * OUTPUT_GAIN);
      r = (int16_t)(r * OUTPUT_GAIN);

      buffer[i * 2]     = l;
      buffer[i * 2 + 1] = r;
    }

    size_t written;

    i2s_write(
      I2S_NUM_0,
      buffer,
      sizeof(buffer),
      &written,
      portMAX_DELAY
    );
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

led.begin();

led.setBrightness(50);

setLED(false);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if(esp_now_init() != ESP_OK)
    return;

  esp_now_register_recv_cb(OnDataRecv);

  setupI2S();

  calibrationStart = millis();

  xTaskCreatePinnedToCore(
    audioTask,
    "audioTask",
    8192,
    NULL,
    3,
    NULL,
    1
  );
}

// =====================================================
// LOOP (LED + CONEXÃO)
// =====================================================

void loop() {

  // conexão real por timeout
  if(millis() - lastPacket > 300) {
    connected = false;
  } else {
    connected = true;
  }

  setLED(connected);

  // filtro suave do RPM
  filteredRPM +=
    (incomingData.rpm - filteredRPM)
    * SMOOTHING;
}

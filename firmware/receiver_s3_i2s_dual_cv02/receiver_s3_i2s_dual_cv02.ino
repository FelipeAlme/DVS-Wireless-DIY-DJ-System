/*
========================================================
 ESP32-S3 DVS RECEIVER - DUAL CV02 I2S DAC

 Hardware:
 - ESP32-S3 / ESP32-S3 Zero
 - 2x DAC I2S stereo externos, ex: PCM5102A
 - Sem LED
 - Sem display

 Codigo feito por Felipe Alme.
 Revisao por Doc GNA.

 Saidas:
 - I2S 0: Deck A / transmissor DECK_ID 1
 - I2S 1: Deck B / transmissor DECK_ID 2
========================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/i2s.h>
#include <math.h>

// ------------------------------------------------------
// Pinos I2S para os dois DACs.
// Ajuste conforme sua placa ESP32-S3 Zero e seus modulos.
// DAC PCM5102A comum:
// BCLK -> BCK, LCLK/LRCK -> LRCK, DIN -> DATA.
// ------------------------------------------------------
#define DAC_A_BCK_PIN   2
#define DAC_A_LRCK_PIN  1
#define DAC_A_DATA_PIN  42

#define DAC_B_BCK_PIN   14
#define DAC_B_LRCK_PIN  13
#define DAC_B_DATA_PIN  12

// ------------------------------------------------------
// Configuracao de radio e protocolo ESP-NOW.
// O receptor gerencia a conexao: recebe HELLO, responde
// WELCOME e envia PING periodico para manter cada transmissor
// conectado.
// ------------------------------------------------------
#define ESPNOW_CHANNEL 1
#define PROTOCOL_VERSION 1
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4
#define PING_INTERVAL_MS 500
#define DECK_TIMEOUT_MS 1000

// ------------------------------------------------------
// Debug serial ativo.
// Use o Monitor Serial do Arduino IDE em 115200 baud.
// Em ESP32-S3 USB nativo, habilite "USB CDC On Boot".
// ------------------------------------------------------
#define DEBUG_SERIAL 1
#define DEBUG_BAUD 115200
#define DEBUG_PRINT_INTERVAL_MS 500

// ------------------------------------------------------
// Configuracao do audio I2S.
// Cada DAC stereo recebe um barramento I2S independente.
// ------------------------------------------------------
#define SAMPLE_RATE 44100
#define DMA_BUF_LEN 64
#define DMA_BUF_COUNT 4

// ------------------------------------------------------
// Ajustes do timecode.
// BASE_RPM e a velocidade nominal do vinil. OUTPUT_GAIN
// fica abaixo de 1.0 para evitar clip em DACs sem headroom.
// ------------------------------------------------------
#define BASE_RPM 33.333f
#define DEADZONE_RPM 0.08f
#define MAX_RPM_RATIO 3.0f
#define RPM_SMOOTHING 0.22f
#define OUTPUT_GAIN 0.70f

// ------------------------------------------------------
// Parametros CV02.
// Esses valores seguem o gerador CV02 usado no app Bridge.
// Os bits sao guardados compactados para economizar RAM.
// ------------------------------------------------------
#define CV02_RESOLUTION 1000
#define CV02_BITS 20
#define CV02_SEED 0x59017UL
#define CV02_TAPS 0x361e4UL
#define CV02_LENGTH 712000UL
#define CV02_PACKED_BYTES ((CV02_LENGTH + 7) / 8)

// ------------------------------------------------------
// Pacote ESP-NOW recebido dos transmissores.
// O layout precisa continuar identico aos firmwares de
// transmitter e ao Bridge desktop.
// ------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint8_t msgType;
  uint8_t version;
  uint8_t deckId;
  int16_t rpmCenti;
  int16_t gyroRaw;
  uint32_t seq;
  uint32_t timestampMicros;
} dvs_packet;

// ------------------------------------------------------
// Estado de radio por deck.
// O callback ESP-NOW atualiza RPM, seq e tempo de chegada.
// As tasks de audio leem snapshots curtos desse estado.
// ------------------------------------------------------
typedef struct {
  bool seen;
  int16_t rpmCenti;
  uint32_t lastSeq;
  uint32_t lastSeenMillis;
  uint32_t lastPingMillis;
  uint32_t packetCount;
  uint16_t lostPackets;
  uint8_t mac[6];
} deck_state;

// ------------------------------------------------------
// Estado de audio por DAC/deck.
// Cada deck tem sua propria fase CV02 e filtro de RPM.
// ------------------------------------------------------
typedef struct {
  uint8_t deckId;
  i2s_port_t port;
  int bckPin;
  int lrckPin;
  int dataPin;
  const char *taskName;
  float filteredRpm;
  float cv02Cycle;
} audio_deck_state;

deck_state deckStates[2];
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool hasPendingWelcome = false;
uint8_t pendingWelcomeMac[6];
uint8_t pendingWelcomeDeck = 0;

uint8_t cv02PackedBits[CV02_PACKED_BYTES];
uint32_t lastDebugPrintMillis = 0;

audio_deck_state audioDecks[2] = {
  { 1, I2S_NUM_0, DAC_A_BCK_PIN, DAC_A_LRCK_PIN, DAC_A_DATA_PIN, "audioDeckA", 0.0f, 0.0f },
  { 2, I2S_NUM_1, DAC_B_BCK_PIN, DAC_B_LRCK_PIN, DAC_B_DATA_PIN, "audioDeckB", 0.0f, 0.0f }
};

// ------------------------------------------------------
// Inicializa a serial de debug.
// Nao e usada para transportar timecode, apenas para ver
// se o firmware iniciou e se os pacotes estao chegando.
// ------------------------------------------------------
void setupDebugSerial() {
#if DEBUG_SERIAL
  Serial.begin(DEBUG_BAUD);
  delay(1000);
  Serial.println();
  Serial.println("DVS dual I2S CV02 receiver boot");
  Serial.printf("Debug baud: %lu\n", (unsigned long)DEBUG_BAUD);
  Serial.printf("ESP-NOW channel: %u\n", ESPNOW_CHANNEL);
#endif
}

// ------------------------------------------------------
// Log simples de boot.
// Fica protegido por macro para poder desligar facil depois.
// ------------------------------------------------------
void debugBootLog(const char *message) {
#if DEBUG_SERIAL
  Serial.println(message);
#else
  (void)message;
#endif
}

// ------------------------------------------------------
// Calcula paridade dos taps do LFSR.
// Usado na geracao da sequencia CV02 durante o boot.
// ------------------------------------------------------
static uint8_t lfsrBit(uint32_t code, uint32_t taps) {
  uint32_t taken = code & taps;
  uint8_t parity = 0;

  while (taken != 0) {
    parity ^= (uint8_t)(taken & 0x1U);
    taken >>= 1;
  }

  return parity;
}

// ------------------------------------------------------
// Avanca um passo no LFSR CV02.
// A sequencia resultante modula a amplitude do par seno/cosseno.
// ------------------------------------------------------
static uint32_t lfsrForward(uint32_t current) {
  uint8_t bit = lfsrBit(current, CV02_TAPS | 0x1U);
  return (current >> 1) | ((uint32_t)bit << (CV02_BITS - 1));
}

// ------------------------------------------------------
// Escreve um bit no vetor compactado.
// 712000 bits ocupam cerca de 89 KB, bem menor que uma
// tabela WAV stereo 16-bit.
// ------------------------------------------------------
static void setPackedBit(uint32_t index, uint8_t value) {
  uint32_t byteIndex = index >> 3;
  uint8_t bitMask = (uint8_t)(1U << (index & 7));

  if (value != 0) {
    cv02PackedBits[byteIndex] |= bitMask;
  } else {
    cv02PackedBits[byteIndex] &= (uint8_t)~bitMask;
  }
}

// ------------------------------------------------------
// Le um bit CV02 compactado.
// Chamado pelas tasks de audio para decidir a modulacao
// do sample atual.
// ------------------------------------------------------
static inline uint8_t getPackedBit(uint32_t index) {
  uint32_t byteIndex = index >> 3;
  uint8_t bitMask = (uint8_t)(1U << (index & 7));
  return (cv02PackedBits[byteIndex] & bitMask) != 0;
}

// ------------------------------------------------------
// Gera a sequencia CV02 uma vez no boot.
// Depois disso as tasks de audio apenas consultam os bits.
// ------------------------------------------------------
void buildCv02Bits() {
  memset(cv02PackedBits, 0, sizeof(cv02PackedBits));

  uint32_t code = CV02_SEED;
  for (uint32_t i = 0; i < CV02_LENGTH; i++) {
    setPackedBit(i, (uint8_t)(code & 0x1U));
    code = lfsrForward(code);
  }
}

// ------------------------------------------------------
// Garante que um transmissor esta registrado como peer.
// Como o receptor e o coordenador, ele adiciona peers
// dinamicamente quando recebe HELLO/DATA.
// ------------------------------------------------------
bool ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  return esp_now_add_peer(&peerInfo) == ESP_OK;
}

// ------------------------------------------------------
// Envia WELCOME ou PING para um transmissor.
// Isso mantem compatibilidade com os transmitters atuais.
// ------------------------------------------------------
void sendControlMessage(const uint8_t *mac, uint8_t deckId, uint8_t msgType) {
  if (!ensurePeer(mac)) {
    return;
  }

  dvs_packet response = {};
  response.msgType = msgType;
  response.version = PROTOCOL_VERSION;
  response.deckId = deckId;
  response.timestampMicros = micros();

  esp_now_send(mac, (uint8_t *)&response, sizeof(response));
}

// ------------------------------------------------------
// Callback ESP-NOW.
// Mantem o trabalho pequeno: valida pacote, atualiza estado
// do deck e agenda WELCOME quando necessario.
// ------------------------------------------------------
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len) {
  if (len != sizeof(dvs_packet)) {
    return;
  }

  dvs_packet packet;
  memcpy(&packet, dataPtr, sizeof(packet));

  if (packet.version != PROTOCOL_VERSION || packet.deckId < 1 || packet.deckId > 2) {
    return;
  }

  const uint8_t *sourceMac = info->src_addr;
  uint8_t deckIndex = packet.deckId - 1;
  deck_state *state = &deckStates[deckIndex];

  portENTER_CRITICAL_ISR(&stateMux);
  memcpy(state->mac, sourceMac, 6);
  state->lastSeenMillis = millis();
  portEXIT_CRITICAL_ISR(&stateMux);

  if (packet.msgType == MSG_HELLO) {
    portENTER_CRITICAL_ISR(&stateMux);
    memcpy(pendingWelcomeMac, sourceMac, 6);
    pendingWelcomeDeck = packet.deckId;
    hasPendingWelcome = true;
    portEXIT_CRITICAL_ISR(&stateMux);
    return;
  }

  if (packet.msgType != MSG_DATA) {
    return;
  }

  uint16_t lost = 0;

  portENTER_CRITICAL_ISR(&stateMux);
  if (state->seen) {
    uint32_t seqDelta = packet.seq - state->lastSeq;
    if (seqDelta > 0) {
      lost = (uint16_t)min(seqDelta - 1, 65535UL);
    }
  }

  state->seen = true;
  state->rpmCenti = packet.rpmCenti;
  state->lastSeq = packet.seq;
  state->packetCount++;
  state->lostPackets = lost;
  portEXIT_CRITICAL_ISR(&stateMux);
}

// ------------------------------------------------------
// Inicializa ESP-NOW.
// Sleep desligado e canal fixo reduzem latencia e evitam
// troca de canal durante o uso.
// ------------------------------------------------------
void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    while (true) {
      delay(1000);
    }
  }

  esp_now_register_recv_cb(OnDataRecv);
}

// ------------------------------------------------------
// Inicializa um periferico I2S em modo master TX.
// Cada DAC recebe seu proprio BCK, LRCK e DATA.
// ------------------------------------------------------
void setupI2S(audio_deck_state *deck) {
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
    .bck_io_num = deck->bckPin,
    .ws_io_num = deck->lrckPin,
    .data_out_num = deck->dataPin,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(deck->port, &config, 0, NULL);
  i2s_set_pin(deck->port, &pins);
  i2s_zero_dma_buffer(deck->port);

#if DEBUG_SERIAL
  Serial.printf("I2S deck %u OK: port=%d BCK=%d LRCK=%d DATA=%d\n",
                deck->deckId,
                (int)deck->port,
                deck->bckPin,
                deck->lrckPin,
                deck->dataPin);
#endif
}

// ------------------------------------------------------
// Mantem a fase CV02 dentro do tamanho da sequencia.
// Suporta rotacao reversa quando o RPM fica negativo.
// ------------------------------------------------------
static inline void wrapCv02Cycle(float *cycle) {
  while (*cycle >= (float)CV02_LENGTH) {
    *cycle -= (float)CV02_LENGTH;
  }

  while (*cycle < 0.0f) {
    *cycle += (float)CV02_LENGTH;
  }
}

// ------------------------------------------------------
// Converte RPM atual em um par stereo CV02.
// Canal esquerdo usa -cos, canal direito usa seno, seguindo
// o mesmo formato gerado no app Bridge.
// ------------------------------------------------------
static inline void renderCv02Sample(audio_deck_state *deck, float rpm, int16_t *leftOut, int16_t *rightOut) {
  float rpmRatio = rpm / BASE_RPM;

  if (fabsf(rpm) < DEADZONE_RPM) {
    rpmRatio = 0.0f;
  }

  if (rpmRatio > MAX_RPM_RATIO) {
    rpmRatio = MAX_RPM_RATIO;
  }

  if (rpmRatio < -MAX_RPM_RATIO) {
    rpmRatio = -MAX_RPM_RATIO;
  }

  float step = ((float)CV02_RESOLUTION / (float)SAMPLE_RATE) * rpmRatio;
  deck->cv02Cycle += step;
  wrapCv02Cycle(&deck->cv02Cycle);

  uint32_t cycleIndex = (uint32_t)deck->cv02Cycle;
  float frac = deck->cv02Cycle - (float)cycleIndex;
  float angle = frac * 6.28318530718f;
  float sine = sinf(angle);
  float cosine = cosf(angle);
  uint8_t bit = getPackedBit(cycleIndex);
  float modulation = bit != 0 ? 1.0f : 1.0f - ((-cosine + 1.0f) * 0.25f);

  float left = -cosine * modulation * OUTPUT_GAIN;
  float right = sine * modulation * OUTPUT_GAIN;

  *leftOut = (int16_t)constrain(lroundf(left * 32767.0f), -32768, 32767);
  *rightOut = (int16_t)constrain(lroundf(right * 32767.0f), -32768, 32767);
}

// ------------------------------------------------------
// Le o RPM de um deck.
// Se o deck estiver sem pacote recente, retorna 0 para
// parar o timecode suavemente.
// ------------------------------------------------------
float readTargetRpm(uint8_t deckId) {
  uint8_t deckIndex = deckId <= 1 ? 0 : 1;
  int16_t rpmCenti = 0;
  uint32_t lastSeenMillis = 0;

  portENTER_CRITICAL(&stateMux);
  rpmCenti = deckStates[deckIndex].rpmCenti;
  lastSeenMillis = deckStates[deckIndex].lastSeenMillis;
  portEXIT_CRITICAL(&stateMux);

  if (lastSeenMillis == 0 || millis() - lastSeenMillis > DECK_TIMEOUT_MS) {
    return 0.0f;
  }

  return (float)rpmCenti / 100.0f;
}

// ------------------------------------------------------
// Task de audio por deck.
// Cada task gera pequenos blocos stereo e bloqueia no
// i2s_write do seu proprio DAC.
// ------------------------------------------------------
void audioTask(void *param) {
  audio_deck_state *deck = (audio_deck_state *)param;
  int16_t buffer[DMA_BUF_LEN * 2];

  while (true) {
    float targetRpm = readTargetRpm(deck->deckId);
    deck->filteredRpm += (targetRpm - deck->filteredRpm) * RPM_SMOOTHING;

    for (int i = 0; i < DMA_BUF_LEN; i++) {
      int16_t left = 0;
      int16_t right = 0;
      renderCv02Sample(deck, deck->filteredRpm, &left, &right);

      buffer[i * 2] = left;
      buffer[i * 2 + 1] = right;
    }

    size_t written = 0;
    i2s_write(deck->port, buffer, sizeof(buffer), &written, portMAX_DELAY);
  }
}

// ------------------------------------------------------
// Envia WELCOME pendente e PING periodico.
// Essa parte roda fora do callback para evitar trabalho
// pesado dentro do recebimento ESP-NOW.
// ------------------------------------------------------
void serviceEspNowControl() {
  uint8_t welcomeMacCopy[6];
  uint8_t welcomeDeckCopy = 0;
  bool shouldSendWelcome = false;

  portENTER_CRITICAL(&stateMux);
  if (hasPendingWelcome) {
    memcpy(welcomeMacCopy, pendingWelcomeMac, 6);
    welcomeDeckCopy = pendingWelcomeDeck;
    hasPendingWelcome = false;
    shouldSendWelcome = true;
  }
  portEXIT_CRITICAL(&stateMux);

  if (shouldSendWelcome) {
    sendControlMessage(welcomeMacCopy, welcomeDeckCopy, MSG_WELCOME);
#if DEBUG_SERIAL
    Serial.printf("WELCOME deck %u -> %02X:%02X:%02X:%02X:%02X:%02X\n",
                  welcomeDeckCopy,
                  welcomeMacCopy[0], welcomeMacCopy[1], welcomeMacCopy[2],
                  welcomeMacCopy[3], welcomeMacCopy[4], welcomeMacCopy[5]);
#endif
  }

  uint32_t nowMillis = millis();
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t macCopy[6];
    bool shouldPing = false;

    portENTER_CRITICAL(&stateMux);
    deck_state *state = &deckStates[i];
    if (state->lastSeenMillis != 0
        && nowMillis - state->lastSeenMillis <= DECK_TIMEOUT_MS
        && nowMillis - state->lastPingMillis >= PING_INTERVAL_MS) {
      memcpy(macCopy, state->mac, 6);
      state->lastPingMillis = nowMillis;
      shouldPing = true;
    }
    portEXIT_CRITICAL(&stateMux);

    if (shouldPing) {
      sendControlMessage(macCopy, i + 1, MSG_PING);
    }
  }
}

// ------------------------------------------------------
// Imprime status periodico dos dois decks.
// Isso fica no loop, fora do callback ESP-NOW, para nao
// atrapalhar a recepcao de radio nem o audio.
// ------------------------------------------------------
void debugPrintStatus() {
#if DEBUG_SERIAL
  uint32_t nowMillis = millis();
  if (nowMillis - lastDebugPrintMillis < DEBUG_PRINT_INTERVAL_MS) {
    return;
  }

  lastDebugPrintMillis = nowMillis;

  deck_state copy[2];
  portENTER_CRITICAL(&stateMux);
  memcpy(copy, deckStates, sizeof(copy));
  portEXIT_CRITICAL(&stateMux);

  for (uint8_t i = 0; i < 2; i++) {
    bool online = copy[i].lastSeenMillis != 0 && nowMillis - copy[i].lastSeenMillis <= DECK_TIMEOUT_MS;
    float rpm = (float)copy[i].rpmCenti / 100.0f;

    Serial.printf("D%u %s rpm=%7.2f seq=%lu rx=%lu lost=%u age=%lums  ",
                  i + 1,
                  online ? "ON " : "OFF",
                  rpm,
                  (unsigned long)copy[i].lastSeq,
                  (unsigned long)copy[i].packetCount,
                  copy[i].lostPackets,
                  copy[i].lastSeenMillis == 0 ? 0UL : (unsigned long)(nowMillis - copy[i].lastSeenMillis));
  }

  Serial.printf("heap=%lu\n", (unsigned long)ESP.getFreeHeap());
#endif
}

// ------------------------------------------------------
// Boot do receptor dual DAC.
// Ordem: gera tabela CV02 compactada, liga os dois I2S,
// liga ESP-NOW e inicia uma task de audio por deck.
// ------------------------------------------------------
void setup() {
  setupDebugSerial();
  debugBootLog("Building CV02 bit table...");
  buildCv02Bits();
  debugBootLog("CV02 bit table OK");

  debugBootLog("Starting I2S outputs...");
  setupI2S(&audioDecks[0]);
  setupI2S(&audioDecks[1]);
  debugBootLog("I2S outputs OK");

  debugBootLog("Starting ESP-NOW...");
  setupEspNow();
  debugBootLog("ESP-NOW OK");

  xTaskCreatePinnedToCore(audioTask, audioDecks[0].taskName, 8192, &audioDecks[0], 3, NULL, 1);
  xTaskCreatePinnedToCore(audioTask, audioDecks[1].taskName, 8192, &audioDecks[1], 3, NULL, 1);
  debugBootLog("Audio tasks OK");
}

// ------------------------------------------------------
// Loop principal.
// Apenas cuida do controle ESP-NOW. O audio roda isolado
// nas tasks dedicadas.
// ------------------------------------------------------
void loop() {
  serviceEspNowControl();
  debugPrintStatus();
  delay(1);
}

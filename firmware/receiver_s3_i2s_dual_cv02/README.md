# Receiver ESP32-S3 Dual I2S CV02 sem LED/display

Firmware standalone para `ESP32-S3 / ESP32-S3 Zero` que recebe dois transmissores por ESP-NOW e gera dois decks `CV02` em dois DACs I2S stereo.

Codigo feito por Felipe Alme. Revisao por Doc GNA.

## Hardware

- ESP32-S3 ou ESP32-S3 Zero
- 2x DAC I2S stereo, por exemplo `PCM5102A`
- 2x transmissores ESP32-C3 do projeto

## Ligacao padrao

Deck A / DAC 1:

```text
ESP32-S3 GPIO2   -> DAC A BCK / BCLK
ESP32-S3 GPIO1   -> DAC A LRCK / LCK / WS
ESP32-S3 GPIO42  -> DAC A DIN / DATA
```

Deck B / DAC 2:

```text
ESP32-S3 GPIO14  -> DAC B BCK / BCLK
ESP32-S3 GPIO13  -> DAC B LRCK / LCK / WS
ESP32-S3 GPIO12  -> DAC B DIN / DATA
```

Comum aos dois DACs:

```text
ESP32-S3 GND     -> GND dos dois DACs
ESP32-S3 3V3/5V  -> VCC conforme o modulo DAC
```

Se algum GPIO nao estiver disponivel na sua placa, altere no `.ino`:

```cpp
#define DAC_A_BCK_PIN   2
#define DAC_A_LRCK_PIN  1
#define DAC_A_DATA_PIN  42

#define DAC_B_BCK_PIN   14
#define DAC_B_LRCK_PIN  13
#define DAC_B_DATA_PIN  12
```

## Saidas

- `I2S_NUM_0`: Deck A, transmissor `DECK_ID 1`
- `I2S_NUM_1`: Deck B, transmissor `DECK_ID 2`

Cada DAC gera um par stereo L/R de timecode.

## Funcionamento

- O receptor gerencia a conexao ESP-NOW.
- Transmissores enviam `HELLO`.
- Receptor responde `WELCOME`.
- Receptor envia `PING` periodico.
- Pacotes `DATA` atualizam RPM de cada deck.
- Duas tasks de audio geram CV02 em tempo real, uma por DAC.
- A resposta de pitch usa `DEADZONE_RPM 0.08` e `RPM_SMOOTHING 0.22`.

## Config Arduino IDE

- Board: `ESP32S3 Dev Module`
- CPU: `240MHz`
- USB CDC on boot: opcional
- ESP32 Arduino core: `3.0.x` ou `3.1.x`

Nao precisa de biblioteca externa alem do pacote ESP32 Arduino.

## Debug serial

Esta versao esta com debug serial ativo por padrao:

```cpp
#define DEBUG_SERIAL 1
#define DEBUG_BAUD 115200
```

No Arduino IDE:

- abra o Monitor Serial em `115200 baud`;
- em ESP32-S3 USB nativo, habilite `USB CDC On Boot`;
- aperte `Reset/EN` depois de abrir o Monitor Serial.

Mensagens esperadas no boot:

```text
DVS dual I2S CV02 receiver boot
Building CV02 bit table...
CV02 bit table OK
Starting I2S outputs...
I2S deck 1 OK...
I2S deck 2 OK...
Starting ESP-NOW...
ESP-NOW OK
Audio tasks OK
```

Depois ele imprime periodicamente:

```text
D1 ON  rpm=  33.33 seq=... rx=... lost=... age=...ms  D2 OFF ...
```

Para desligar debug depois dos testes:

```cpp
#define DEBUG_SERIAL 0
```

# DVS Wireless DIY DJ System

Sistema DIY experimental de DVS wireless usando transmissores ESP32-C3, comunicacao ESP-NOW e receiver ESP32-S3 gerando timecode CV02 por DAC I2S.

Este projeto e para estudo, prototipagem e desenvolvimento. Nao e um substituto comercial para sistemas profissionais de DVS wireless.

## Estrutura Publicada

```text
firmware/transmitter_c3_mpu6050_rgb_catodo/
firmware/transmitter_c3_bmi160_rgb_catodo/
firmware/receiver_s3_i2s_dual_cv02/
assets/
docs/
legacy/
```

As pastas `apps/`, `tools/`, `drivers/` e firmwares de teste internos ficam fora deste branch.

## Firmwares Principais

### Transmitter ESP32-C3 + MPU6050 + LED RGB Catodo

Pasta:

```text
firmware/transmitter_c3_mpu6050_rgb_catodo/
```

Recursos:

- MPU6050 via I2C;
- LED RGB catodo comum;
- ESP-NOW em canal fixo;
- envio em taxa fixa de 500 Hz;
- pacote pequeno com `seq` e `timestampMicros`;
- auto-calibracao por janela estavel;
- DLPF no MPU6050 para reduzir ruido mecanico.

### Transmitter ESP32-C3 Super Mini + BMI160 + LED RGB Catodo

Pasta:

```text
firmware/transmitter_c3_bmi160_rgb_catodo/
```

Recursos:

- BMI160 via I2C;
- LED RGB catodo comum;
- ESP-NOW em canal fixo;
- envio em taxa fixa de 500 Hz;
- pacote pequeno com `seq` e `timestampMicros`;
- auto-calibracao por janela estavel.

### Receiver ESP32-S3 Dual I2S CV02

Pasta:

```text
firmware/receiver_s3_i2s_dual_cv02/
```

Recursos:

- receiver ESP-NOW coordenando transmissores;
- dois DACs I2S stereo independentes;
- Deck A em `I2S_NUM_0`;
- Deck B em `I2S_NUM_1`;
- geracao CV02 em tempo real;
- debug serial opcional.

## Fluxo Basico

```text
Transmitter Deck A/B
        -> ESP-NOW
Receiver ESP32-S3
        -> I2S DAC A/B
Mixer/interface DVS
        -> Software DJ
```

## Aviso

Projeto independente para pesquisa e aprendizado. Nao e afiliado a Phase DJ, Serato, VirtualDJ, Traktor, Pioneer DJ, Native Instruments ou qualquer outra marca citada.

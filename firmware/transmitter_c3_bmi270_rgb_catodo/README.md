# ESP32-C3 Super Mini Transmitter BMI270 com LED RGB Catodo

Firmware DVS para `ESP32-C3 Super Mini`, IMU `BMI270` e LED RGB
catodo comum.

Codigo: Felipe Alme
Revisao: Doc GNA

## Recursos

- ESP-NOW bidirecional no canal 1;
- handshake `HELLO` -> `WELCOME` e supervisao por `PING`;
- pacotes de RPM em ate 250 Hz;
- somente um pacote ESP-NOW em voo;
- auto-calibracao do offset com o prato parado;
- leitura de bateria no boot e a cada 5 minutos;
- recuperacao do I2C com reinicializacao completa do BMI270;
- protocolo compativel com `receiver_s3_i2s_dual_cv02`.

## Ligacao

BMI270:

```text
BMI270 VCC -> ESP32-C3 3V3
BMI270 GND -> ESP32-C3 GND
BMI270 SDA -> ESP32-C3 GPIO6
BMI270 SCL -> ESP32-C3 GPIO7
```

O endereco I2C `0x68` ou `0x69` e detectado automaticamente.
Use alimentacao e nivel logico de `3,3 V`.

LED RGB catodo comum:

```text
R      -> resistor -> GPIO2
G      -> resistor -> GPIO3
B      -> resistor -> GPIO4
catodo -> GND
```

O firmware usa:

```cpp
#define LED_COMMON_ANODE 0
```

Bateria Li-ion:

```text
BAT+  -> 220k -> GPIO0
GPIO0 -> 220k -> GND
GPIO0 -> 100 nF -> GND
BAT-  -> GND comum
```

Para ajustar a tensao usando um multimetro, altere:

```cpp
const float BATTERY_CALIBRATION_FACTOR = 1.0f;
```

O fator e `tensao_multimetro / tensao_lida`. Em placas nas quais
GPIO0 interfere no boot, escolha outro pino ADC disponivel.

## Inicializacao do BMI270

O BMI270 perde sua configuracao interna quando fica sem energia. Em
todo boot o firmware executa automaticamente:

1. inicia o I2C em 400 kHz;
2. procura o sensor em `0x68` e `0x69`;
3. chama `bmi270_init()`;
4. carrega no sensor o arquivo de configuracao oficial da Bosch;
5. configura o giroscopio em 800 Hz, modo de alto desempenho e
   faixa de `+/-500 dps`;
6. habilita o giroscopio;
7. calibra o offset do eixo Z;
8. inicia o envio de RPM.

A mesma sequencia e repetida se a recuperacao do barramento I2C for
acionada. A sensibilidade usada em `+/-500 dps` e `65.536 LSB/dps`.

## LEDs

```text
azul piscando  -> aguardando receptor
azul fixo      -> receptor respondeu
verde piscando -> calibrando com o prato parado
verde fixo     -> pronto e transmitindo
vermelho       -> erro de sensor ou inicializacao
```

Ligue o transmissor com o prato parado e so comece a girar quando o
LED ficar verde fixo.

## Driver Bosch incluido

Os arquivos `bmi2.c`, `bmi2.h`, `bmi2_defs.h`, `bmi270.c` e
`bmi270.h` vieram do repositorio oficial
[Bosch BMI270 SensorAPI](https://github.com/boschsensortec/BMI270_SensorAPI),
revisao `41129fcfe39c583ee5462d79195741945d51c1fe`.

Eles usam a licenca BSD-3-Clause preservada em
`LICENSE-BOSCH-SENSORAPI`. Nao e necessario instalar uma biblioteca
BMI270 adicional no Arduino IDE.

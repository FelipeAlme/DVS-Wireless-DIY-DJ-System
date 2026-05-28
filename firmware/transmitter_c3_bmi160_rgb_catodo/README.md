# ESP32-C3 Super Mini Transmitter BMI160 com LED RGB Catodo

Firmware para transmissor usando `ESP32-C3 Super Mini`, modulo IMU `BMI160` e LED RGB catodo comum.

Ele usa o mesmo protocolo do transmissor MPU6050:

- ESP-NOW bidirecional;
- `HELLO` -> `WELCOME`;
- `PING` do receptor;
- envio fixo em `500 Hz`;
- pacote `DVS1` compativel com o receptor e o app JUCE.

## Pinagem padrao

```cpp
#define SDA_PIN 6
#define SCL_PIN 7

#define LED_R 2
#define LED_G 3
#define LED_B 4
#define LED_COMMON_ANODE 0
```

BMI160:

```text
VCC -> 3V3
GND -> GND
SDA -> GPIO 6
SCL -> GPIO 7
```

O firmware detecta automaticamente o endereco I2C `0x68` ou `0x69`.
Se o BMI160 nao responder, o LED pisca vermelho continuamente.

LED RGB KY-016 ou catodo comum:

```text
R -> GPIO 2
G -> GPIO 3
B -> GPIO 4
GND -> GND
```

O firmware vem configurado para catodo comum:

```cpp
#define LED_COMMON_ANODE 0
```

Se usar LED anodo comum:

```cpp
#define LED_COMMON_ANODE 1
```

## BMI160

O firmware configura:

- gyro normal mode;
- ODR de gyro em `800 Hz`;
- faixa `+/-500 dps`;
- sensibilidade `65.6 LSB/dps`.
- resposta de pitch mais sensivel: `SMOOTHING 0.16` e `DEADZONE_RPM 0.20`.

A calibracao de offset do gyro Z acontece automaticamente depois do handshake com o receptor. O transmissor espera uma janela de leituras estaveis com o toca-discos parado; se detectar movimento, reinicia a janela para nao calibrar durante o giro.

```text
azul piscando -> aguardando receptor
azul fixo -> receptor respondeu
verde piscando -> auto-calibrando com o prato parado
verde fixo -> transmitindo
```

Uso recomendado:

1. Ligue o ESP com o toca-discos parado.
2. Aguarde o LED verde parar de piscar e ficar fixo.
3. Comece a girar o prato.

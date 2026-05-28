# ESP32-C3 Transmitter MPU6050 com LED RGB Catodo

Firmware para transmissor usando `ESP32-C3`, modulo IMU `MPU6050` e LED RGB catodo comum.

Ele usa o mesmo protocolo do projeto:

- ESP-NOW bidirecional;
- `HELLO` -> `WELCOME`;
- `PING` do receptor;
- envio fixo em `500 Hz`;
- pacote pequeno com `seq` e `timestampMicros`;
- auto-calibracao por janela estavel.

## Pinagem padrao

```cpp
#define SDA_PIN 8
#define SCL_PIN 9

#define LED_R 2
#define LED_G 3
#define LED_B 4
#define LED_COMMON_ANODE 0
```

MPU6050:

```text
VCC -> 3V3
GND -> GND
SDA -> GPIO 8
SCL -> GPIO 9
```

LED RGB catodo comum:

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

## MPU6050

O firmware configura:

- gyro `+/-500 dps`;
- sensibilidade `65.5 LSB/dps`;
- DLPF no nivel `3` (`0x1A = 0x03`) para reduzir ruido mecanico;
- sample rate do gyro em `1 kHz`;
- leitura I2C em `400 kHz`;
- envio ESP-NOW em `500 Hz`.
- resposta de pitch mais sensivel: `SMOOTHING 0.16` e `DEADZONE_RPM 0.20`.

## Calibracao e LED

A calibracao de offset do gyro Z acontece automaticamente depois do handshake com o receptor. O transmissor espera uma janela de leituras estaveis com o toca-discos parado; se detectar movimento, reinicia a janela para nao calibrar durante o giro.

```text
azul piscando -> aguardando receiver
azul fixo -> receiver respondeu
verde piscando -> auto-calibrando com o prato parado
verde fixo -> transmitindo
vermelho piscando continuo -> erro MPU6050 ou ESP-NOW
```

Uso recomendado:

1. Ligue o ESP com o toca-discos parado.
2. Aguarde o LED verde parar de piscar e ficar fixo.
3. Comece a girar o prato.

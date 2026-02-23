# 📡 Estación Base — AgroMqueen

Estación base del sistema **AgroMqueen** basada en **Heltec WiFi LoRa 32 V2** (ESP32). Permite introducir waypoints por puerto serie, enviarlos al vehículo autónomo vía LoRa con confirmaciones (ACK), y recibir y mostrar telemetría en tiempo real.

> **Repositorio:** [https://git.pandauniverse.es/panda/AgroMqueen](https://git.pandauniverse.es/panda/AgroMqueen)

---

## 📋 Índice

- [Descripción General](#-descripción-general)
- [Hardware](#-hardware)
- [Pinout](#-pinout)
- [Protocolo LoRa](#-protocolo-lora)
- [Guía de Uso](#-guía-de-uso)
- [Arquitectura del Software](#-arquitectura-del-software)
- [Dependencias](#-dependencias)
- [Compilar y Subir](#-compilar-y-subir)
- [Notas Técnicas](#-notas-técnicas)

---

## 🔍 Descripción General

El sistema funciona en dos partes:

| Componente | Hardware | Función |
|---|---|---|
| **Estación Base (este repo)** | Heltec WiFi LoRa 32 **V2** | Recibe waypoints por Serial, los envía por LoRa, muestra telemetría |
| **Vehículo** | Heltec WiFi LoRa 32 **V3** | Recibe waypoints, navega con GPS+brújula, controla motores, envía telemetría |

### Flujo general

```
     ESTACIÓN BASE                        VEHÍCULO
     ─────────────                        ────────
  1. Usuario introduce waypoints
     por Serial Monitor (lat,lon)
  2. Escribe "start" para confirmar
  3. Espera la solicitud del vehículo
                                     4. El vehículo arranca e inicializa
                                        todos los subsistemas (GPS, IMU,
                                        LoRa, motores)
                                     5. Solicita waypoints por LoRa
     ◄── REQUEST_WAYPOINTS (0x01) ────
  6. Envía START_TRANSMISSION
     ── START_TRANSMISSION (0x05) ──►
  7. Envía waypoints uno a uno
     ── WAYPOINT_DATA (0x02) ──────►
     ◄── ACK (0x04) ───────────────    8. Confirma cada waypoint
     ── WAYPOINT_DATA (0x02) ──────►
     ◄── ACK (0x04) ───────────────
  9. Envía fin de transmisión
     ── END_TRANSMISSION (0x03) ───►
                                    10. El vehículo comienza la misión
 11. Recibe y muestra telemetría
     ◄── TELEMETRY (0x06) ──────────   12. Envía telemetría cada 2s
```

---

## 🔧 Hardware

| Componente | Modelo | Función |
|---|---|---|
| MCU | Heltec WiFi LoRa 32 V2 (ESP32) | Controlador + LoRa SX1276 integrado |
| Pantalla | OLED SSD1306 128×64 | Muestra estado y telemetría |
| Comunicación | LoRa SX1276 | Enlace bidireccional con el vehículo |
| Entrada | USB Serial (115200 baud) | Introducción de waypoints |

---

## 📌 Pinout

### OLED SSD1306 (I2C — integrado en la placa V2)
| Señal | GPIO |
|---|---|
| SDA | 4 |
| SCL | 15 |
| RST | 16 |

### LoRa SX1276 (SPI — integrado en la placa V2)
| Señal | GPIO |
|---|---|
| SCK | 5 |
| MISO | 19 |
| MOSI | 27 |
| SS (CS) | 18 |
| RST | 14 |
| DIO0 | 26 |

---

## 📡 Protocolo LoRa

### Configuración de Radio
| Parámetro | Valor |
|---|---|
| Frecuencia | 868.0 MHz (Europa) |
| Ancho de Banda | 125 kHz |
| Spreading Factor | 7 |
| Potencia TX | 14 dBm |
| CRC | Habilitado |
| Sync Word | 0x12 |

### Tipos de Mensaje
| Código | Nombre | Dirección | Descripción |
|---|---|---|---|
| `0x01` | REQUEST_WAYPOINTS | Vehículo → Estación | El vehículo solicita waypoints |
| `0x02` | WAYPOINT_DATA | Estación → Vehículo | Un waypoint con lat/lon |
| `0x03` | END_TRANSMISSION | Estación → Vehículo | Fin de transmisión |
| `0x04` | ACK | Vehículo → Estación | Confirmación de recepción |
| `0x05` | START_TRANSMISSION | Estación → Vehículo | Inicio + número de waypoints |
| `0x06` | TELEMETRY | Vehículo → Estación | Datos de telemetría |

### Estructura WaypointPacket (12 bytes)
```cpp
struct __attribute__((packed)) WaypointPacket {
  uint8_t msgType;         // 0x02
  uint8_t waypointId;      // 1-based
  uint8_t totalWaypoints;
  float   latitude;        // 4 bytes
  float   longitude;       // 4 bytes
  uint8_t checksum;        // XOR de los bytes anteriores
};
```

### Estructura TelemetryPacket
```cpp
struct __attribute__((packed)) TelemetryPacket {
  uint8_t msgType;              // 0x06
  float   latitude;
  float   longitude;
  uint8_t satellites;
  uint8_t gpsFixed;
  uint8_t currentWaypoint;
  float   distanceToWaypoint;
  float   headingToWaypoint;
  int16_t rssi;
  float   snr;
  uint8_t batteryLevel;
  uint8_t checksum;             // Suma de todos los bytes
};
```

### Protocolo ACK con reintentos
- Cada waypoint se envía individualmente
- La estación espera un **ACK** durante **8 segundos** por cada waypoint
- Si no recibe ACK, **reintenta hasta 5 veces**
- Si se agotan los reintentos, la transmisión falla y se puede reiniciar

---

## 📖 Guía de Uso

### 1. Conectar la estación

Conecta la placa Heltec V2 por USB y abre el **Serial Monitor** a **115200 baud**.

### 2. Introducir waypoints

La pantalla OLED mostrará `"Esperando waypoints..."`. Introduce waypoints en el Serial Monitor con el formato:

```
latitud,longitud
```

Ejemplo:
```
37.3891,-5.9845
37.3895,-5.9840
37.3900,-5.9835
```

### 3. Confirmar con "start"

Cuando hayas introducido todos los waypoints, escribe:

```
start
```

La pantalla mostrará `"X WP listos"` y `"Esperando vehiculo..."`.

### 4. Esperar al vehículo

La estación queda esperando la solicitud del vehículo (`REQUEST_WAYPOINTS`). Cuando el vehículo enciende y completa su inicialización, solicitará los waypoints automáticamente.

### 5. Transmisión

La estación envía los waypoints uno a uno con confirmación. La pantalla OLED muestra el progreso:
```
Enviando WP 1/3...
Enviando WP 2/3...
Enviando WP 3/3...
Transmision OK!
```

### 6. Monitorizar telemetría

Una vez completada la transmisión, la estación pasa a **modo recepción de telemetría**. La pantalla OLED muestra:

```
=== TELEMETRIA ===
Lat: 37.3891
Lon: -5.9845
Sat: 8  Fix: Si
WP: 1
Dist: 15.3m
Hdg: 45.2°
RSSI: -67 SNR: 9.5
Bat: 85%
```

---

## 🏗 Arquitectura del Software

```
main.cpp
├── setup()
│   ├── Serial (115200)
│   ├── Inicializa OLED (SSD1306Wire)
│   ├── Inicializa LoRa (868 MHz)
│   └── waitForWaypoints()         → Lee waypoints del Serial
│
├── loop()
│   ├── Estado: WAITING_FOR_REQUEST
│   │   └── Escucha REQUEST_WAYPOINTS
│   │
│   ├── Estado: SENDING_WAYPOINTS
│   │   └── sendWaypointsViaLoRa() → Envía con ACK y reintentos
│   │
│   └── Estado: LISTENING_TELEMETRY
│       └── listenForTelemetry()   → Recibe y muestra telemetría
│
├── Funciones auxiliares
│   ├── waitForWaypoints()         → Parsea "lat,lon" del Serial
│   ├── sendWaypointsViaLoRa()     → Protocolo con ACK, 5 reintentos
│   ├── calculateChecksum()        → XOR de un buffer
│   ├── processTelemetry()         → Desempaqueta TelemetryPacket
│   └── displayTelemetry()         → Muestra datos en la OLED
```

### Máquina de Estados

```
  ┌─────────────────────┐
  │  WAITING_FOR_REQUEST │ ◄── REQUEST_WAYPOINTS
  └──────────┬──────────┘
             │ Recibe solicitud
             ▼
  ┌─────────────────────┐
  │  SENDING_WAYPOINTS   │ ── Envía WP con ACKs ──►
  └──────────┬──────────┘
             │ Todos enviados OK
             ▼
  ┌─────────────────────┐
  │ LISTENING_TELEMETRY  │ ◄── TELEMETRY (cada 2s)
  └─────────────────────┘
```

---

## 📦 Dependencias

Definidas en `platformio.ini`:

```ini
[env:heltec_wifi_lora_32_V2]
platform = espressif32
board = heltec_wifi_lora_32_V2
framework = arduino
lib_deps = 
    thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays@^4.6.1
    sandeepmistry/LoRa@^0.8.0
```

| Librería | Uso |
|---|---|
| **SSD1306Wire** (ThingPulse) | Pantalla OLED por I2C (hardware) |
| **LoRa** (sandeepmistry) | Módulo LoRa SX1276 (Heltec V2) |

> ⚠️ La estación usa la librería **sandeepmistry/LoRa** (SX1276), mientras que el vehículo usa **RadioLib** (SX1262). Ambas son compatibles a nivel de protocolo LoRa con la misma configuración.

---

## 🔨 Compilar y Subir

```bash
# Compilar
pio run

# Subir (ajusta el puerto COM)
pio run --target upload --upload-port COM13

# Monitor serial
pio device monitor --port COM13 --baud 115200
```

---

## 📝 Notas Técnicas

### Compatibilidad SX1276 ↔ SX1262
Los chips LoRa de la V2 (SX1276) y V3 (SX1262) son compatibles si usan la **misma configuración**:
- Frecuencia: 868 MHz
- Spreading Factor: 7
- Bandwidth: 125 kHz
- CRC: Activado
- Sync Word: 0x12

### Diferencias entre librerías LoRa
| Característica | sandeepmistry/LoRa (Estación) | RadioLib (Vehículo) |
|---|---|---|
| Chip | SX1276 | SX1262 |
| API | `LoRa.beginPacket()` / `LoRa.write()` | `radio.transmit()` / `radio.receive()` |
| Recepción | Callback `onReceive()` o polling | Polling con `radio.receive()` |
| Potencia | `LoRa.setTxPower(14)` | Se configura en `radio.begin()` |

### Formato de entrada de waypoints
- Cada waypoint en una línea: `latitud,longitud`
- Formato decimal (no DMS): `37.3891,-5.9845`
- Máximo: limitado por la RAM (pero en la práctica, 10-20 waypoints)
- Se confirma con `start` (case-insensitive)

### Telemetría
- Se recibe automáticamente cada ~2 segundos
- Solo se muestra si el checksum es válido (suma de todos los bytes)
- Los datos incluyen RSSI y SNR del enlace LoRa (medidos en el vehículo)

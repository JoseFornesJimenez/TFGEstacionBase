# 📡 Estación Base — AgroMqueen

Firmware de la **estación base** del sistema AgroMqueen, basado en **Heltec WiFi LoRa 32 V2** (ESP32). Actúa como pasarela entre el ordenador de control (GCS) y el vehículo: recibe los waypoints por USB, los transmite al robot por LoRa con confirmación, y reenvía la telemetría que llega del robot.

> Trabajo de Fin de Grado — José Fornés Jiménez.
> Repositorio: <https://github.com/JoseFornesJimenez/TFGEstacionBase>

## Proyecto AgroMqueen

El sistema completo se compone de tres repositorios:

- **Estación Base (este repo)** — pasarela LoRa ↔ USB (Heltec V2).
- **[AgroMqueen (vehículo)](../AgroMqueen/)** — robot autónomo (Heltec V3).
- **[GCS web (mapa)](../../mapa/)** — interfaz Flask + Leaflet para planificar y monitorizar misiones.

```
GCS web  ◄─ USB ─►  Estación base (este repo)  ◄─ LoRa 868 MHz ─►  Vehículo
```

## Hardware

| Componente | Modelo | Función |
|---|---|---|
| MCU | Heltec WiFi LoRa 32 V2 (ESP32) | Control + LoRa SX1276 integrado |
| Pantalla | OLED SSD1306 128×64 | Estado y telemetría |
| Comunicación | LoRa SX1276 (868 MHz) | Enlace bidireccional con el vehículo |
| Entrada | USB Serial (115 200 baud) | Waypoints y comandos desde la GCS |

## Funcionalidad

- **Recepción de waypoints** por puerto serie en formato `latitud,longitud`, confirmados con `start`.
- **Envío al vehículo por LoRa** con ACK por waypoint y hasta 5 reintentos.
- **Recepción y volcado de telemetría** del robot (posición, satélites, distancia al waypoint, rumbo, RSSI/SNR, batería) cada ~2 s.
- **Visualización en OLED** del estado actual de la misión y los datos recibidos.
- **Compatibilidad SX1276 ↔ SX1262** ajustando sync word OTA (`0x14`) y desactivando el CRC hardware (se usa checksum propio en el protocolo).

## Compilar y subir

```bash
pio run
pio run --target upload --upload-port COM13
pio device monitor --port COM13 --baud 115200
```

## Dependencias

- `thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays` — pantalla OLED.
- `jgromes/RadioLib` — LoRa SX1276 (misma librería que el vehículo).

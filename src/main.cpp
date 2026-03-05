#include <Arduino.h>
#include <vector>
#include <Wire.h>
#include "SSD1306Wire.h"
#include <SPI.h>
#include <RadioLib.h>

void waitForWaypoints();
void updateDisplay();
bool sendWaypointsViaLoRa();
void processTelemetry(uint8_t* data, int size);
void displayTelemetry();
void listenForTelemetry();
uint8_t calculateChecksum(uint8_t* data, size_t len);

// Pines para Heltec WiFi LoRa 32 V2
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

// Pines LoRa para Heltec WiFi LoRa 32 V2
#define LORA_SCK  5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS   18
#define LORA_RST  14
#define LORA_DIO0 26
#define LORA_DIO1 35

// Protocolo
#define MSG_REQUEST_WAYPOINTS  0x01
#define MSG_WAYPOINT_DATA      0x02
#define MSG_END_TRANSMISSION   0x03
#define MSG_ACK                0x04
#define MSG_START_TRANSMISSION 0x05
#define MSG_TELEMETRY          0x06

#define MAX_RETRIES      5
#define ACK_TIMEOUT      8000
#define POST_SEND_DELAY  800

SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL);
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);

struct Waypoint {
  float latitude;
  float longitude;
};

struct __attribute__((packed)) WaypointPacket {
  uint8_t msgType;
  uint8_t waypointId;
  uint8_t totalWaypoints;
  float   latitude;
  float   longitude;
  uint8_t checksum;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t  msgType;
  float    latitude;
  float    longitude;
  uint8_t  satellites;
  uint8_t  gpsFixed;
  uint8_t  currentWaypoint;
  float    distanceToWaypoint;
  float    headingToWaypoint;
  int16_t  rssi;
  float    snr;
  uint8_t  batteryLevel;
  uint8_t  checksum;
};

std::vector<Waypoint> waypoints;

float    robotLatitude           = 0.0;
float    robotLongitude          = 0.0;
uint8_t  robotSatellites         = 0;
bool     robotGpsFixed           = false;
uint8_t  robotCurrentWaypoint    = 0;
float    robotDistanceToWaypoint = 0.0;
float    robotHeadingToWaypoint  = 0.0;
int16_t  robotRssi               = 0;
float    robotSnr                = 0.0;
uint8_t  robotBatteryLevel       = 0;
unsigned long lastTelemetryReceived = 0;
bool     missionActive           = false;

// ── RadioLib: enviar un paquete y volver a RX ──────────────────
static void loraSend(uint8_t* buf, size_t len) {
  radio.transmit(buf, len);   // bloqueante, espera TX_DONE
  radio.startReceive();       // volver a RX inmediatamente
}

// ── RadioLib: recibir con timeout (ms), devuelve bytes leídos o -1 ──
static int loraReceive(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (digitalRead(LORA_DIO0) == HIGH || radio.available()) {
      size_t pktLen = radio.getPacketLength();
      size_t readLen = (pktLen > maxLen) ? maxLen : pktLen;
      int st = radio.readData(buf, readLen);
      radio.startReceive();
      if (st == RADIOLIB_ERR_NONE) return (int)readLen;
      return -1;
    }
    delay(5);
  }
  return -1;  // timeout
}

void setup() {
  Serial.begin(115200);

  // Reset OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(0, 0, "Inicializando...");
  display.display();

  // Inicializar LoRa con RadioLib
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // SF7, BW125, CR5, sync=0x14, pwr=14, preamble=8
  int st = radio.begin(868.0f, 125.0f, 7, 5, 0x14, 14, 8);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("Error LoRa: %d\r\n", st);
    display.clear();
    display.drawString(0, 0, "Error LoRa!");
    display.display();
    while (1);
  }
  radio.setCRC(false);
  radio.startReceive();

  Serial.println("LoRa OK (RadioLib SX1276)");
  display.clear();
  display.drawString(0, 0, "LoRa OK - 868MHz");
  display.drawString(0, 12, "Esperando waypoints...");
  display.display();
  delay(1000);
}

void loop() {
  waitForWaypoints();

  display.clear();
  display.drawString(0, 0, "Waypoints listos");
  display.drawString(0, 12, String(waypoints.size()) + " waypoints");
  display.drawString(0, 24, "Esperando solicitud");
  display.drawString(0, 36, "del vehículo...");
  display.display();

  Serial.println("Esperando solicitud del vehículo...");

  while (true) {
    uint8_t rxBuf[64];
    int rxLen = loraReceive(rxBuf, sizeof(rxBuf), 100);
    if (rxLen >= 1 && rxBuf[0] == MSG_REQUEST_WAYPOINTS) {
      Serial.println("Solicitud recibida. Enviando waypoints...");
      display.clear();
      display.drawString(0, 0, "Solicitud recibida!");
      display.drawString(0, 12, "Enviando waypoints...");
      display.display();

      while (!sendWaypointsViaLoRa()) {
        Serial.println("[WP] Transmision interrumpida — esperando nueva solicitud...");
        bool gotRequest = false;
        unsigned long t0 = millis();
        while (millis() - t0 < 30000) {
          int r = loraReceive(rxBuf, sizeof(rxBuf), 100);
          if (r >= 1 && rxBuf[0] == MSG_REQUEST_WAYPOINTS) { gotRequest = true; break; }
        }
        if (!gotRequest) break;
        Serial.println("[WP] Nueva solicitud — reenviando waypoints...");
      }

      missionActive = true;
      display.clear();
      display.drawString(0, 0, "Waypoints enviados");
      display.drawString(0, 12, "Esperando telemetria");
      display.display();
      delay(2000);

      listenForTelemetry();
    }
  }
}

void waitForWaypoints() {
  Serial.println("Esperando waypoints (formato: lat,lon). Escribe 'start' para comenzar.");

  display.clear();
  display.drawString(0, 0, "Esperando waypoints");
  display.drawString(0, 12, "Formato: lat,lon");
  display.display();

  while (true) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input.equalsIgnoreCase("start")) {
        Serial.println("Iniciando con " + String(waypoints.size()) + " waypoints.");
        updateDisplay();
        break;
      }

      int commaIndex = input.indexOf(',');
      if (commaIndex > 0) {
        float lat = input.substring(0, commaIndex).toFloat();
        float lon = input.substring(commaIndex + 1).toFloat();
        waypoints.push_back({lat, lon});
        Serial.println("Waypoint añadido: Lat=" + String(lat, 6) + ", Lon=" + String(lon, 6));
        display.clear();
        display.drawString(0, 0, "Waypoints: " + String(waypoints.size()));
        display.drawString(0, 12, "Último:");
        display.drawString(0, 24, "Lat: " + String(lat, 4));
        display.drawString(0, 36, "Lon: " + String(lon, 4));
        display.drawString(0, 52, "Escribe 'start'");
        display.display();
      } else {
        Serial.println("Formato inválido. Use: lat,lon");
      }
    }
  }
}

void updateDisplay() {
  display.clear();
  display.drawString(0, 0, "Total: " + String(waypoints.size()) + " waypoints");
  int linea = 12;
  for (int i = 0; i < (int)waypoints.size() && i < 4; i++) {
    String wp = String(i+1) + ": " + String(waypoints[i].latitude, 2) + "," + String(waypoints[i].longitude, 2);
    display.drawString(0, linea, wp);
    linea += 12;
  }
  if (waypoints.size() > 4) display.drawString(0, 52, "... y mas");
  display.display();
}

bool sendWaypointsViaLoRa() {
  delay(500);

  uint8_t startPkt[2] = {MSG_START_TRANSMISSION, (uint8_t)waypoints.size()};
  loraSend(startPkt, sizeof(startPkt));
  Serial.println("Enviando START_TRANSMISSION con " + String(waypoints.size()) + " waypoints");
  delay(1000);

  for (int i = 0; i < (int)waypoints.size(); i++) {
    WaypointPacket packet;
    packet.msgType        = MSG_WAYPOINT_DATA;
    packet.waypointId     = i + 1;
    packet.totalWaypoints = waypoints.size();
    packet.latitude       = waypoints[i].latitude;
    packet.longitude      = waypoints[i].longitude;
    packet.checksum       = calculateChecksum((uint8_t*)&packet, sizeof(WaypointPacket) - 1);

    bool ackReceived = false;
    int  retries     = 0;

    while (!ackReceived && retries < MAX_RETRIES) {
      loraSend((uint8_t*)&packet, sizeof(WaypointPacket));
      Serial.println("Enviando waypoint " + String(packet.waypointId) + "/" + String(packet.totalWaypoints) +
                     " (intento " + String(retries + 1) + ")");
      display.clear();
      display.drawString(0, 0, "Enviando WP " + String(packet.waypointId) + "/" + String(packet.totalWaypoints));
      display.drawString(0, 12, "Intento: " + String(retries + 1));
      display.drawString(0, 24, "Lat: " + String(packet.latitude, 4));
      display.drawString(0, 36, "Lon: " + String(packet.longitude, 4));
      display.display();

      delay(POST_SEND_DELAY);

      unsigned long startTime = millis();
      while (millis() - startTime < ACK_TIMEOUT && !ackReceived) {
        uint8_t rxBuf[64];
        int rxLen = loraReceive(rxBuf, sizeof(rxBuf), 50);
        if (rxLen >= 1) {
          uint8_t msgType    = rxBuf[0];
          uint8_t secondByte = (rxLen >= 2) ? rxBuf[1] : 0;
          Serial.printf("RX: msgType=0x%02X waypointId=%d\r\n", msgType, secondByte);

          if (msgType == MSG_ACK && secondByte == packet.waypointId) {
            ackReceived = true;
            Serial.println("ACK recibido para waypoint " + String(packet.waypointId));
            display.clear();
            display.drawString(0, 0, "ACK recibido!");
            display.drawString(0, 12, "WP " + String(packet.waypointId) + "/" + String(packet.totalWaypoints));
            display.display();
            delay(300);
          } else if (msgType == MSG_TELEMETRY) {
            processTelemetry(rxBuf, rxLen);
          } else if (msgType == MSG_REQUEST_WAYPOINTS) {
            Serial.println("[WP] Robot re-solicito waypoints — reiniciando transmision");
            return false;
          }
        }
      }

      if (!ackReceived) {
        retries++;
        Serial.println("Timeout ACK. Reintento " + String(retries) + "/" + String(MAX_RETRIES));
        delay(300);
      }
    }

    if (!ackReceived) {
      Serial.println("ERROR: No ACK para waypoint " + String(packet.waypointId));
      display.clear();
      display.drawString(0, 0, "ERROR!");
      display.drawString(0, 12, "WP " + String(packet.waypointId) + " sin ACK");
      display.display();
      delay(2000);
    }
  }

  delay(500);
  uint8_t endPkt[1] = {MSG_END_TRANSMISSION};
  loraSend(endPkt, sizeof(endPkt));
  Serial.println("Enviando END_TRANSMISSION");
  display.clear();
  display.drawString(0, 0, "Transmision");
  display.drawString(0, 12, "completa!");
  display.display();
  delay(2000);
  return true;
}

uint8_t calculateChecksum(uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) checksum ^= data[i];
  return checksum;
}

void processTelemetry(uint8_t* data, int size) {
  if (size != sizeof(TelemetryPacket)) {
    Serial.println("[Telemetry] Tamaño incorrecto");
    return;
  }
  TelemetryPacket* packet = (TelemetryPacket*)data;
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) sum += data[i];
  if (sum != packet->checksum) {
    Serial.println("[Telemetry] Checksum invalido");
    return;
  }

  robotLatitude           = packet->latitude;
  robotLongitude          = packet->longitude;
  robotSatellites         = packet->satellites;
  robotGpsFixed           = packet->gpsFixed;
  robotCurrentWaypoint    = packet->currentWaypoint;
  robotDistanceToWaypoint = packet->distanceToWaypoint;
  robotHeadingToWaypoint  = packet->headingToWaypoint;
  robotRssi               = packet->rssi;
  robotSnr                = packet->snr;
  robotBatteryLevel       = packet->batteryLevel;
  lastTelemetryReceived   = millis();

  Serial.println("\n========== TELEMETRIA ROBOT ==========");
  Serial.printf("Posicion: %.6f, %.6f\r\n", robotLatitude, robotLongitude);
  Serial.printf("Satelites: %d | GPS Fix: %s\r\n", robotSatellites, robotGpsFixed ? "SI" : "NO");
  Serial.printf("Waypoint actual: %d | Distancia: %.1f m\r\n", robotCurrentWaypoint + 1, robotDistanceToWaypoint);
  Serial.printf("Rumbo: %.1f°\r\n", robotHeadingToWaypoint);
  Serial.printf("RSSI: %d dBm | SNR: %.1f dB\r\n", robotRssi, robotSnr);
  Serial.printf("Bateria: %d%%\r\n", robotBatteryLevel);
  Serial.println("======================================\n");

  displayTelemetry();
}

void displayTelemetry() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "TELEMETRIA ROBOT");
  if (robotGpsFixed) {
    String pos = String(robotLatitude, 4) + "," + String(robotLongitude, 4);
    if (pos.length() > 20) pos = String(robotLatitude, 3) + "," + String(robotLongitude, 3);
    display.drawString(0, 12, pos);
    display.drawString(0, 24, "WP" + String(robotCurrentWaypoint + 1) + " " + String(robotDistanceToWaypoint, 0) + "m");
    display.drawString(0, 36, String(robotHeadingToWaypoint, 0) + "° S:" + String(robotSatellites));
    display.drawString(0, 48, "R:" + String(robotRssi) + " B:" + String(robotBatteryLevel) + "%");
  } else {
    display.drawString(0, 28, "Sin GPS Fix");
    display.drawString(0, 40, "Sats: " + String(robotSatellites));
  }
  display.display();
}

void listenForTelemetry() {
  Serial.println("\n[Sistema] Escuchando telemetria del robot...");
  while (true) {
    uint8_t rxBuf[256];
    int rxLen = loraReceive(rxBuf, sizeof(rxBuf), 100);
    if (rxLen >= 1) {
      uint8_t msgType = rxBuf[0];
      if (msgType == MSG_TELEMETRY) {
        processTelemetry(rxBuf, rxLen);
      } else if (msgType == MSG_REQUEST_WAYPOINTS) {
        Serial.println("[LoRa] Nueva solicitud de waypoints — reenviando...");
        display.clear();
        display.drawString(0, 0, "Nueva solicitud WP");
        display.drawString(0, 12, "Reenviando...");
        display.display();
        sendWaypointsViaLoRa();
        Serial.println("[Sistema] Volviendo a escuchar telemetria...");
      } else {
        Serial.printf("[LoRa] Mensaje desconocido: 0x%02X\r\n", msgType);
      }
    }

    if (missionActive && millis() - lastTelemetryReceived > 10000 && lastTelemetryReceived > 0) {
      display.clear();
      display.drawString(0, 0, "WARNING");
      display.drawString(0, 16, "Sin telemetria");
      display.drawString(0, 28, "> 10 segundos");
      display.display();
    }
  }
}

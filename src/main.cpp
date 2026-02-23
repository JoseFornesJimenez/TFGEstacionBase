#include <Arduino.h>
#include <vector>
#include <Wire.h>
#include "SSD1306Wire.h"
#include <LoRa.h>

void waitForWaypoints();
void updateDisplay();
void sendWaypointsViaLoRa();
void processTelemetry(uint8_t* data, int size);
void displayTelemetry();
void listenForTelemetry();
uint8_t calculateChecksum(uint8_t* data, size_t len);

// Pines para Heltec WiFi LoRa 32 V2
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

// Pines LoRa para Heltec WiFi LoRa 32 V2
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// Configuración LoRa
#define LORA_FREQUENCY 868E6  // 868 MHz (Europa)
#define LORA_BANDWIDTH 125E3
#define LORA_SPREADING_FACTOR 7

// Protocolo
#define MSG_REQUEST_WAYPOINTS 0x01
#define MSG_WAYPOINT_DATA 0x02
#define MSG_END_TRANSMISSION 0x03
#define MSG_ACK 0x04
#define MSG_START_TRANSMISSION 0x05
#define MSG_TELEMETRY 0x06        // Telemetría del robot
#define MSG_TELEMETRY_ACK 0x07    // ACK de telemetría (opcional)

#define MAX_RETRIES 5
#define ACK_TIMEOUT 8000  // 8 segundos (aumentado para dar más tiempo al vehículo)
#define POST_SEND_DELAY 800  // 800ms después de enviar antes de buscar ACK

SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL);

struct Waypoint {
  float latitude;
  float longitude;
};

struct __attribute__((packed)) WaypointPacket {
  uint8_t msgType;      // Tipo de mensaje
  uint8_t waypointId;   // ID del waypoint (1-based)
  uint8_t totalWaypoints; // Total de waypoints
  float latitude;
  float longitude;
  uint8_t checksum;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t msgType;              // MSG_TELEMETRY (0x06)
  float latitude;               // Latitud actual del robot
  float longitude;              // Longitud actual del robot
  uint8_t satellites;           // Número de satélites GPS
  uint8_t gpsFixed;             // 1 si tiene fix GPS, 0 si no
  uint8_t currentWaypoint;      // Waypoint actual al que se dirige
  float distanceToWaypoint;     // Distancia al waypoint en metros
  float headingToWaypoint;      // Rumbo al waypoint en grados (0-360)
  int16_t rssi;                 // RSSI de la última recepción LoRa
  float snr;                    // SNR de la última recepción LoRa
  uint8_t batteryLevel;         // Nivel de batería en % (0-100)
  uint8_t checksum;             // Checksum simple
};

std::vector<Waypoint> waypoints;

// Variables para almacenar última telemetría
float robotLatitude = 0.0;
float robotLongitude = 0.0;
uint8_t robotSatellites = 0;
bool robotGpsFixed = false;
uint8_t robotCurrentWaypoint = 0;
float robotDistanceToWaypoint = 0.0;
float robotHeadingToWaypoint = 0.0;
int16_t robotRssi = 0;
float robotSnr = 0.0;
uint8_t robotBatteryLevel = 0;
unsigned long lastTelemetryReceived = 0;
bool missionActive = false;

void setup() {
  Serial.begin(115200);
  
  // Reset OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  
  // Inicializar la pantalla OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(0, 0, "Inicializando...");
  display.display();
  
  // Inicializar LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Error al inicializar LoRa");
    display.clear();
    display.drawString(0, 0, "Error LoRa!");
    display.display();
    while (1);
  }
  
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  
  Serial.println("LoRa inicializado correctamente");
  display.clear();
  display.drawString(0, 0, "LoRa OK - 868MHz");
  display.drawString(0, 12, "Esperando waypoints...");
  display.display();
  delay(1000);
}

void loop() {
  waitForWaypoints();
  
  // Esperar solicitud del vehículo
  display.clear();
  display.drawString(0, 0, "Waypoints listos");
  display.drawString(0, 12, String(waypoints.size()) + " waypoints");
  display.drawString(0, 24, "Esperando solicitud");
  display.drawString(0, 36, "del vehículo...");
  display.display();
  
  Serial.println("Esperando solicitud del vehículo...");
  
  while (true) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      uint8_t msgType = LoRa.read();
      
      if (msgType == MSG_REQUEST_WAYPOINTS) {
        Serial.println("Solicitud recibida. Enviando waypoints...");
        display.clear();
        display.drawString(0, 0, "Solicitud recibida!");
        display.drawString(0, 12, "Enviando waypoints...");
        display.display();
        
        sendWaypointsViaLoRa();
        
        // Activar modo de escucha de telemetría
        missionActive = true;
        display.clear();
        display.drawString(0, 0, "Waypoints enviados");
        display.drawString(0, 12, "Esperando telemetría");
        display.display();
        delay(2000);
        
        // Entrar en modo de monitoreo de telemetría
        listenForTelemetry();
      }
    }
    delay(100);
  }
}

void waitForWaypoints() {
  Serial.println("Esperando waypoints (formato: lat,lon). Escribe 'start' para comenzar.");
  
  display.clear();
  display.drawString(0, 0, "Esperando waypoints");
  display.drawString(0, 12, "Formato: lat,lon");
  display.display();
  
  float lat, lon;
  
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
        lat = input.substring(0, commaIndex).toFloat();
        lon = input.substring(commaIndex + 1).toFloat();
        
        Waypoint wp = {lat, lon};
        waypoints.push_back(wp);
        
        Serial.println("Waypoint añadido: Lat=" + String(lat, 6) + ", Lon=" + String(lon, 6));
        
        // Actualizar pantalla con el waypoint añadido
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
  
  // Mostrar los primeros waypoints en la pantalla
  int linea = 12;
  for (int i = 0; i < waypoints.size() && i < 4; i++) {
    String wp = String(i+1) + ": " + String(waypoints[i].latitude, 2) + "," + String(waypoints[i].longitude, 2);
    display.drawString(0, linea, wp);
    linea += 12;
  }
  
  if (waypoints.size() > 4) {
    display.drawString(0, 52, "... y mas");
  }
  
  display.display();
}

void sendWaypointsViaLoRa() {
  // Enviar mensaje de inicio de transmisión
  LoRa.beginPacket();
  LoRa.write(MSG_START_TRANSMISSION);
  LoRa.write((uint8_t)waypoints.size());
  LoRa.endPacket();
  
  Serial.println("Enviando START_TRANSMISSION con " + String(waypoints.size()) + " waypoints");
  delay(1000);  // Aumentado de 500ms a 1s para dar tiempo al vehículo
  
  // Enviar cada waypoint
  for (int i = 0; i < waypoints.size(); i++) {
    WaypointPacket packet;
    packet.msgType = MSG_WAYPOINT_DATA;
    packet.waypointId = i + 1;
    packet.totalWaypoints = waypoints.size();
    packet.latitude = waypoints[i].latitude;
    packet.longitude = waypoints[i].longitude;
    
    // Calcular checksum (excluye el campo checksum)
    packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(WaypointPacket) - 1);
    
    bool ackReceived = false;
    int retries = 0;
    
    while (!ackReceived && retries < MAX_RETRIES) {
      // Enviar waypoint
      LoRa.beginPacket();
      LoRa.write((uint8_t*)&packet, sizeof(WaypointPacket));
      LoRa.endPacket();
      
      Serial.println("Enviando waypoint " + String(packet.waypointId) + "/" + String(packet.totalWaypoints) + 
                     " (intento " + String(retries + 1) + ")");
      
      display.clear();
      display.drawString(0, 0, "Enviando WP " + String(packet.waypointId) + "/" + String(packet.totalWaypoints));
      display.drawString(0, 12, "Intento: " + String(retries + 1));
      display.drawString(0, 24, "Lat: " + String(packet.latitude, 4));
      display.drawString(0, 36, "Lon: " + String(packet.longitude, 4));
      display.display();
      
      // Delay para dar tiempo al vehículo a procesar y preparar ACK
      delay(POST_SEND_DELAY);
      
      // Esperar ACK
      unsigned long startTime = millis();
      bool timeout = false;
      
      while (!timeout && !ackReceived) {
        // Verificar timeout
        if (millis() - startTime >= ACK_TIMEOUT) {
          timeout = true;
          break;
        }
        
        // Buscar paquete ACK
        int packetSize = LoRa.parsePacket();
        if (packetSize >= 2) {
          uint8_t msgType = LoRa.read();
          uint8_t ackWaypointId = LoRa.read();
          
          Serial.print("RX: msgType=0x");
          Serial.print(msgType, HEX);
          Serial.print(", waypointId=");
          Serial.println(ackWaypointId);
          
          if (msgType == MSG_ACK && ackWaypointId == packet.waypointId) {
            ackReceived = true;
            Serial.println("✓ ACK recibido para waypoint " + String(packet.waypointId));
            display.clear();
            display.drawString(0, 0, "ACK recibido!");
            display.drawString(0, 12, "WP " + String(packet.waypointId) + "/" + String(packet.totalWaypoints));
            display.display();
            delay(300);
            break;
          }
        }
        
        delay(50);  // Aumentado de 10ms a 50ms para dar más tiempo entre chequeos
      }
      
      if (!ackReceived) {
        retries++;
        Serial.println("✗ Timeout esperando ACK. Reintento " + String(retries) + "/" + String(MAX_RETRIES));
        delay(300);  // Reducido de 200ms a 100ms entre reintentos
      }
    }
    
    if (!ackReceived) {
      Serial.println("ERROR: No se recibió ACK para waypoint " + String(packet.waypointId) + " después de " + String(MAX_RETRIES) + " intentos");
      display.clear();
      display.drawString(0, 0, "ERROR!");
      display.drawString(0, 12, "WP " + String(packet.waypointId) + " sin ACK");
      display.display();
      delay(2000);
    }
  }
  
  // Enviar mensaje de fin de transmisión
  delay(500);  // Pequeña pausa antes del END
  LoRa.beginPacket();
  LoRa.write(MSG_END_TRANSMISSION);
  LoRa.endPacket();
  
  Serial.println("Enviando END_TRANSMISSION");
  display.clear();
  display.drawString(0, 0, "Transmisión");
  display.drawString(0, 12, "completa!");
  display.display();
  delay(2000);
}

uint8_t calculateChecksum(uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void processTelemetry(uint8_t* data, int size) {
  if (size != sizeof(TelemetryPacket)) {
    Serial.println("[Telemetry] Tamaño de paquete incorrecto");
    return;
  }
  
  TelemetryPacket* packet = (TelemetryPacket*)data;
  
  // Validar checksum
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
    sum += data[i];
  }
  
  if (sum != packet->checksum) {
    Serial.println("[Telemetry] Checksum inválido");
    return;
  }
  
  // Actualizar variables globales
  robotLatitude = packet->latitude;
  robotLongitude = packet->longitude;
  robotSatellites = packet->satellites;
  robotGpsFixed = packet->gpsFixed;
  robotCurrentWaypoint = packet->currentWaypoint;
  robotDistanceToWaypoint = packet->distanceToWaypoint;
  robotHeadingToWaypoint = packet->headingToWaypoint;
  robotRssi = packet->rssi;
  robotSnr = packet->snr;
  robotBatteryLevel = packet->batteryLevel;
  lastTelemetryReceived = millis();
  
  // Mostrar en Serial
  Serial.println("\n========== TELEMETRÍA ROBOT ==========");
  Serial.print("Posición: ");
  Serial.print(robotLatitude, 6);
  Serial.print(", ");
  Serial.println(robotLongitude, 6);
  Serial.print("Satélites: ");
  Serial.print(robotSatellites);
  Serial.print(" | GPS Fix: ");
  Serial.println(robotGpsFixed ? "SI" : "NO");
  Serial.print("Waypoint actual: ");
  Serial.print(robotCurrentWaypoint + 1);
  Serial.print(" | Distancia: ");
  Serial.print(robotDistanceToWaypoint, 1);
  Serial.println(" m");
  Serial.print("Rumbo: ");
  Serial.print(robotHeadingToWaypoint, 1);
  Serial.println("°");
  Serial.print("RSSI: ");
  Serial.print(robotRssi);
  Serial.print(" dBm | SNR: ");
  Serial.print(robotSnr, 1);
  Serial.println(" dB");
  Serial.print("Batería: ");
  Serial.print(robotBatteryLevel);
  Serial.println("%");
  Serial.println("======================================\n");
  
  // Mostrar en OLED
  displayTelemetry();
}

void displayTelemetry() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  
  // Título
  display.drawString(0, 0, "TELEMETRIA ROBOT");
  
  if (robotGpsFixed) {
    // Línea 1: Posición (fuente más pequeña)
    String pos = String(robotLatitude, 4) + "," + String(robotLongitude, 4);
    display.setFont(ArialMT_Plain_10);
    // Si el texto es muy largo, usar coordenadas más cortas
    if (pos.length() > 20) {
      pos = String(robotLatitude, 3) + "," + String(robotLongitude, 3);
    }
    display.drawString(0, 12, pos);
    
    // Línea 2: Waypoint y distancia
    String wp = "WP" + String(robotCurrentWaypoint + 1) + " " + String(robotDistanceToWaypoint, 0) + "m";
    display.drawString(0, 24, wp);
    
    // Línea 3: Rumbo y satélites
    String nav = String(robotHeadingToWaypoint, 0) + "° S:" + String(robotSatellites);
    display.drawString(0, 36, nav);
    
    // Línea 4: Radio y batería
    String radio = "R:" + String(robotRssi) + " B:" + String(robotBatteryLevel) + "%";
    display.drawString(0, 48, radio);
    
  } else {
    display.drawString(0, 28, "Sin GPS Fix");
    String sats = "Sats: " + String(robotSatellites);
    display.drawString(0, 40, sats);
  }
  
  display.display();
}

void listenForTelemetry() {
  Serial.println("\n[Sistema] Escuchando telemetría del robot...");
  
  while (true) {
    int packetSize = LoRa.parsePacket();
    
    if (packetSize) {
      // Leer todo el paquete
      uint8_t rxBuffer[256];
      int bytesRead = 0;
      
      while (LoRa.available() && bytesRead < 256) {
        rxBuffer[bytesRead++] = LoRa.read();
      }
      
      if (bytesRead > 0) {
        uint8_t msgType = rxBuffer[0];
        
        if (msgType == MSG_TELEMETRY) {
          processTelemetry(rxBuffer, bytesRead);
        } else {
          Serial.print("[LoRa] Mensaje desconocido: 0x");
          Serial.println(msgType, HEX);
        }
      }
    }
    
    // Verificar timeout de telemetría (10 segundos sin señal)
    if (missionActive && millis() - lastTelemetryReceived > 10000 && lastTelemetryReceived > 0) {
      display.clear();
      display.drawString(0, 0, "WARNING");
      display.drawString(0, 16, "Sin telemetria");
      display.drawString(0, 28, "> 10 segundos");
      display.display();
    }
    
    delay(10);
  }
}

//Ejemplo de latitud longitud 
// 37.7749,-122.4195//Ejemplo de latitud longitud 
// 37.7749,-122.4195
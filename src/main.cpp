#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"

// --- Configuración MPU ---
const int MPU_ADDR = 0x68; 

// --- Configuración GPS ---
#define RXD2 16
#define TXD2 17
HardwareSerial neogps(2);
TinyGPSPlus gps;

// --- Bluetooth ---
BluetoothSerial SerialBT;

// Variables para control de tiempo
unsigned long lastTelemetria = 0;
const float UMBRAL_IMPACTO = 1.5; // Ajusta este valor (en Gs) según tus pruebas

void setup() {
  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  SerialBT.begin("MotoGuard_ESP32");
  
  // Iniciar I2C en pines 21 y 22
  Wire.begin(21, 22);
  
  // Despertar al MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  
  Serial.println("¡MotoGuard: Sensores y Bluetooth Listos!");
}

void loop() {
  // 1. LEER GPS
  while (neogps.available() > 0) {
    gps.encode(neogps.read());
  }

  // 2. LEER MPU (Tu lógica directa)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); 

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  // Convertir a Gs
  float x = ax / 16384.0;
  float y = ay / 16384.0;
  float z = az / 16384.0;

  // Calcular Fuerza G Resultante (Magnitud del vector)
  float gTotal = sqrt(x*x + y*y + z*z);

  // 3. DETECTAR IMPACTO
  if (gTotal > UMBRAL_IMPACTO) {
    Serial.printf("!!! IMPACTO DETECTADO: %.2f G !!!\n", gTotal);
    
    // Enviar alerta inmediata a la App por Bluetooth
    SerialBT.print("EVENTO:CHOQUE,FUERZA:");
    SerialBT.print(gTotal);
    SerialBT.print(",LAT:");
    SerialBT.print(gps.location.lat(), 6);
    SerialBT.print(",LNG:");
    SerialBT.println(gps.location.lng(), 6);
  }

  // 4. ENVIAR TELEMETRÍA PERIÓDICA (Cada 1 segundo)
  if (millis() - lastTelemetria > 1000) {
    if (gps.location.isValid()) {
      // Trama para que la App muestre velocidad y posición en tiempo real
      SerialBT.print("DATA:");
      SerialBT.print(gps.speed.kmph());
      SerialBT.print(",");
      SerialBT.print(gps.location.lat(), 6);
      SerialBT.print(",");
      SerialBT.println(gps.location.lng(), 6);
    }
    lastTelemetria = millis();
  }
}
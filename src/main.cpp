#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"

const int MPU_ADDR = 0x68; 

#define RXD2 16
#define TXD2 17
HardwareSerial neogps(2);
TinyGPSPlus gps;
BluetoothSerial SerialBT;

unsigned long lastTelemetria = 0;

// --- CONFIGURACIÓN DE UMBRALES ---
const float UMBRAL_IMPACTO = 2.2;       
const float UMBRAL_INCLINACION = 60.0;   // Grados máximos de desviación permitidos desde el encendido

// --- VARIABLES DE CALIBRACIÓN INICIAL (POSICIÓN SAFE) ---
float safeX = 0.0, safeY = 0.0, safeZ = 0.0;

// --- VARIABLES PARA VENTANA DE CONFIRMACIÓN ---
bool alertaEnProgreso = false;
unsigned long tiempoInicioAnomalia = 0;
const unsigned long TIEMPO_GRACIA = 4000; 
float gMaximoRegistrado = 0.0;            

// Función auxiliar para leer los valores limpios de G del MPU
void leerG_Sensores(float &x, float &y, float &z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); 

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  x = ax / 16384.0;
  y = ay / 16384.0;
  z = az / 16384.0;
}

void setup() {
  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  SerialBT.begin("MotoGuard_ESP32");
  
  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  
  delay(500); // Esperar a que el sensor se estabilice

  // --- PROCESO DE CALIBRACIÓN AUTOMÁTICA ---
  Serial.println("--> Calibrando posición SAFE de la moto. No mover...");
  float sumaX = 0, sumaY = 0, sumaZ = 0;
  int muestras = 50;
  
  for (int i = 0; i < muestras; i++) {
    float tx, ty, tz;
    leerG_Sensores(tx, ty, tz);
    sumaX += tx;
    sumaY += ty;
    sumaZ += tz;
    delay(20);
  }
  
  // Promedio de la posición inicial
  float posX = sumaX / muestras;
  float posY = sumaY / muestras;
  float posZ = sumaZ / muestras;
  
  // Normalizar el vector "Safe" (convertirlo a magnitud 1)
  float magSafe = sqrt(posX*posX + posY*posY + posZ*posZ);
  safeX = posX / magSafe;
  safeY = posY / magSafe;
  safeZ = posZ / magSafe;

  Serial.printf("--> ¡Calibración Exitosa! Vector Safe establecido: [%.2f, %.2f, %.2f]\n", safeX, safeY, safeZ);
  Serial.println("¡MotoGuard Listo y Protegido!");
}

void loop() {
  // 1. LEER GPS
  while (neogps.available() > 0) {
    gps.encode(neogps.read());
  }

  // 2. LEER MPU ACTUAL
  float currentX, currentY, currentZ;
  leerG_Sensores(currentX, currentY, currentZ);

  // Calcular Fuerza G Resultante (Magnitud actual)
  float gTotal = sqrt(currentX*currentX + currentY*currentY + currentZ*currentZ);

  // --- CALCULAR DESVIACIÓN DE ÁNGULO RESPECTO AL SAFE ---
  // Normalizar vector actual
  float currentX_n = currentX / gTotal;
  float currentY_n = currentY / gTotal;
  float currentZ_n = currentZ / gTotal;

  // Producto punto entre el vector Safe y el Vector Actual
  float productoPunto = (safeX * currentX_n) + (safeY * currentY_n) + (safeZ * currentZ_n);
  
  // Asegurar límites matemáticos para evitar errores de acos
  if (productoPunto > 1.0) productoPunto = 1.0;
  if (productoPunto < -1.0) productoPunto = -1.0;

  // Ángulo de desviación en grados en cualquier dirección (3D)
  float anguloDesviacion = acos(productoPunto) * 180.0 / M_PI;

  // Evaluamos las condiciones de riesgo
  // Si hay un impacto fuerte O si la moto se desvía más de 60 grados de su posición de encendido
  bool condicionInsegura = (gTotal > UMBRAL_IMPACTO || anguloDesviacion > UMBRAL_INCLINACION);

  // 3. CONTROL DE EMERGENCIA CON RETARDO DE SEGURIDAD
  if (condicionInsegura) {
    if (!alertaEnProgreso) {
      alertaEnProgreso = true;
      tiempoInicioAnomalia = millis();
      gMaximoRegistrado = gTotal; 
      Serial.printf("--> Anomalía detectada. Ángulo Desviación: %.1f°. Esperando confirmación...\n", anguloDesviacion);
    } else {
      if (gTotal > gMaximoRegistrado) {
        gMaximoRegistrado = gTotal;
      }

      // Si pasan los 4 segundos y la moto sigue desviada o sufriendo la anomalía
      if (millis() - tiempoInicioAnomalia > TIEMPO_GRACIA) {
        Serial.println("!!! EMERGENCIA CONFIRMADA, ENVIANDO SEÑAL !!!");
        
        // --- TU CADENA ORIGINAL EXACTA PARA TU APP DE REACT ---
        SerialBT.print("EVENTO:CHOQUE,FUERZA:");
        SerialBT.print(gMaximoRegistrado);
        SerialBT.print(",LAT:");
        SerialBT.print(gps.location.lat(), 6);
        SerialBT.print(",LNG:");
        SerialBT.println(gps.location.lng(), 6);
        // -----------------------------------------------------

        alertaEnProgreso = false; 
        delay(5000); 
      }
    }
  } else {
    if (alertaEnProgreso) {
      Serial.println("<-- Cancelado automáticamente: La moto regresó a su posición inicial 'Safe'.");
      alertaEnProgreso = false;
    }
  }

  // 4. TELEMETRÍA PERIÓDICA (Cada 1 segundo)
  if (millis() - lastTelemetria > 1000) {
    if (gps.location.isValid()) {
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
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
// Umbral para CONFIRMACIÓN por ángulo sostenido (moto caída/volcada)
const float UMBRAL_INCLINACION = 60.0; // grados de desviación respecto al "safe"

// Umbral para PRE-ALERTA instantánea por impacto fuerte.
// Esto NO necesita sostenerse en el tiempo, es un pico puntual.
// Ajusta este valor según tus pruebas reales de choque (no el de vibración normal).
const float UMBRAL_IMPACTO_INSTANTANEO = 2.5;

// --- VARIABLES DE CALIBRACIÓN INICIAL (POSICIÓN SAFE) ---
float safeX = 0.0, safeY = 0.0, safeZ = 0.0;

// --- ESTADOS DEL SISTEMA ---
enum EstadoSistema {
  ESTADO_NORMAL,
  ESTADO_PREALERTA,   // pico de G detectado, esperando confirmación o cancelación
  ESTADO_CONFIRMADO   // evento ya enviado, en cooldown
};

EstadoSistema estado = ESTADO_NORMAL;
unsigned long tiempoInicioAnomalia = 0;
float gMaximoRegistrado = 0.0;

const unsigned long TIEMPO_GRACIA = 4000;     // ventana para confirmar por ángulo sostenido
const unsigned long TIEMPO_COOLDOWN = 5000;   // tiempo de espera tras confirmar, no bloqueante
unsigned long tiempoCooldownInicio = 0;

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

void enviarPrealerta(float gPico) {
  SerialBT.print("ALERTA:POSIBLE_CHOQUE,FUERZA:");
  SerialBT.print(gPico);
  SerialBT.print(",LAT:");
  SerialBT.print(gps.location.lat(), 6);
  SerialBT.print(",LNG:");
  SerialBT.println(gps.location.lng(), 6);
}

void enviarConfirmacion(float gMax) {
  SerialBT.print("EVENTO:CHOQUE,FUERZA:");
  SerialBT.print(gMax);
  SerialBT.print(",LAT:");
  SerialBT.print(gps.location.lat(), 6);
  SerialBT.print(",LNG:");
  SerialBT.println(gps.location.lng(), 6);
}

void enviarCancelacion() {
  SerialBT.println("EVENTO:CANCELADO");
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

  delay(500);

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

  float posX = sumaX / muestras;
  float posY = sumaY / muestras;
  float posZ = sumaZ / muestras;

  float magSafe = sqrt(posX * posX + posY * posY + posZ * posZ);
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

  // 2. LEER COMANDOS DE LA APP (ej: "CANCELAR" cuando el usuario toca "Estoy bien")
  if (SerialBT.available()) {
    String comando = SerialBT.readStringUntil('\n');
    comando.trim();
    if (comando == "CANCELAR" && (estado == ESTADO_PREALERTA)) {
      Serial.println("<-- Alerta cancelada por el usuario desde la app.");
      enviarCancelacion();
      estado = ESTADO_NORMAL;
    }
  }

  // 3. LEER MPU ACTUAL
  float currentX, currentY, currentZ;
  leerG_Sensores(currentX, currentY, currentZ);

  float gTotal = sqrt(currentX * currentX + currentY * currentY + currentZ * currentZ);

  float currentX_n = currentX / gTotal;
  float currentY_n = currentY / gTotal;
  float currentZ_n = currentZ / gTotal;

  float productoPunto = (safeX * currentX_n) + (safeY * currentY_n) + (safeZ * currentZ_n);
  if (productoPunto > 1.0) productoPunto = 1.0;
  if (productoPunto < -1.0) productoPunto = -1.0;
  float anguloDesviacion = acos(productoPunto) * 180.0 / M_PI;

  bool inclinacionAnormal = (anguloDesviacion > UMBRAL_INCLINACION);
  bool picoImpacto = (gTotal > UMBRAL_IMPACTO_INSTANTANEO);

  // 4. MÁQUINA DE ESTADOS
  switch (estado) {

    case ESTADO_NORMAL:
      // Cualquiera de los dos dispara el inicio de una pre-alerta
      if (picoImpacto || inclinacionAnormal) {
        estado = ESTADO_PREALERTA;
        tiempoInicioAnomalia = millis();
        gMaximoRegistrado = gTotal;
        Serial.printf("--> Anomalía detectada (G=%.2f, Ángulo=%.1f°). Enviando pre-alerta...\n", gTotal, anguloDesviacion);
        enviarPrealerta(gTotal);
      }
      break;

    case ESTADO_PREALERTA:
      if (gTotal > gMaximoRegistrado) {
        gMaximoRegistrado = gTotal;
      }

      if (inclinacionAnormal) {
        // Confirmación rápida: si la moto sigue inclinada, no hace falta
        // esperar los 4 segundos completos para algo que ya es evidente,
        // pero igual respetamos un mínimo de persistencia para evitar ruido.
        if (millis() - tiempoInicioAnomalia > TIEMPO_GRACIA) {
          Serial.println("!!! EMERGENCIA CONFIRMADA (ángulo sostenido) !!!");
          enviarConfirmacion(gMaximoRegistrado);
          estado = ESTADO_CONFIRMADO;
          tiempoCooldownInicio = millis();
        }
      } else {
        // La moto volvió a estar erguida / no detecta inclinación.
        // Si tampoco hay pico de G activo, lo más probable es que haya
        // sido un golpe o vibración pasajera. Dejamos la pre-alerta
        // viva un tiempo corto por si la app no cancela, pero si pasan
        // los 4 segundos sin inclinación sostenida, se descarta sola.
        if (millis() - tiempoInicioAnomalia > TIEMPO_GRACIA) {
          Serial.println("<-- Pre-alerta descartada: sin inclinación sostenida.");
          enviarCancelacion();
          estado = ESTADO_NORMAL;
        }
      }
      break;

    case ESTADO_CONFIRMADO:
      // Cooldown no bloqueante (antes había un delay(5000) que congelaba todo)
      if (millis() - tiempoCooldownInicio > TIEMPO_COOLDOWN) {
        estado = ESTADO_NORMAL;
      }
      break;
  }

  // 5. TELEMETRÍA PERIÓDICA (cada 1 segundo)
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

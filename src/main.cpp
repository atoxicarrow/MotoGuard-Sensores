#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"

const int MPU_ADDR = 0x68;
const int MPU_ADDR = 0x68;

#define RXD2 16
#define TXD2 17
HardwareSerial neogps(2);
TinyGPSPlus gps;
BluetoothSerial SerialBT;

unsigned long lastTelemetria = 0;

// ─────────────────────────────────────────────────────────────────────────────
// THRESHOLDS — MODO DEMO
// Dos formas de activar:
//   1. INCLINACIÓN: tumbar el dispositivo >40° durante 3 segundos
//   2. GOLPE:       sacudida brusca que supere 2.0G de delta instantáneo
// ─────────────────────────────────────────────────────────────────────────────

// ── Demo 1: Inclinación ──
const float          UMBRAL_IMPACTO         = 10.0; // Muy alto — no activa por G en Path B
const float          UMBRAL_INCLINACION     = 40.0; // Tumbar ~40° es fácil y visible
const unsigned long  TIEMPO_GRACIA          = 3000; // 3s inclinado → alerta

// ── Demo 2: Golpe brusco (Path 0 spike) ──
const float          UMBRAL_SPIKE_INMEDIATO = 2.0;  // Golpe firme con la mano lo supera

// ── Path A desactivado en demo (evita activaciones accidentales) ──
const int            MUESTRAS_CONFIRMACION  = 20;   // Muy alto — prácticamente no dispara
const float          UMBRAL_G_CONFIRMACION  = 8.0;
const float          VELOCIDAD_MINIMA_CRASH = 99.0; // Nunca se cumple sin GPS real
const float          ANGULO_DESVIO_CRITICO  = 80.0;

// Cooldown corto para repetir la demo rápidamente
const unsigned long  COOLDOWN_MS            = 6000;

// ─────────────────────────────────────────────────────────────────────────────
// STATE
// ─────────────────────────────────────────────────────────────────────────────

float safeX = 0.0, safeY = 0.0, safeZ = 0.0;

// Static path
bool          alertaEnProgreso     = false;
unsigned long tiempoInicioAnomalia = 0;
float         gMaximoRegistrado    = 0.0;

// High-speed confirmation window
int           contadorConfirmacion = 0;
unsigned long tiempoInicioVentana  = 0;
bool          ventanaActiva        = false;
const unsigned long VENTANA_MAX_MS = 800; // window expires after 800ms if not confirmed

// Global cooldown
unsigned long ultimaAlerta         = 0;

// Previous G reading — used to detect sudden spike delta
float         gAnterior            = 1.0;

// ─────────────────────────────────────────────────────────────────────────────
// HELPERS
// ─────────────────────────────────────────────────────────────────────────────

void leerG_Sensores(float &x, float &y, float &z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  x = ax / 16384.0;
  y = ay / 16384.0;
  z = az / 16384.0;
}

void enviarAlerta(float gFuerza, const char* tipo) {
  // Global cooldown check — won't send twice within COOLDOWN_MS
  if (millis() - ultimaAlerta < COOLDOWN_MS) {
    Serial.println("<-- Alerta suprimida por cooldown.");
    return;
  }
  ultimaAlerta = millis();

  Serial.printf("!!! ALERTA [%s]: %.2fG !!!\n", tipo, gFuerza);

  SerialBT.print("EVENTO:CHOQUE,FUERZA:");
  SerialBT.print(gFuerza);
  SerialBT.print(",LAT:");
  SerialBT.print(gps.location.lat(), 6);
  SerialBT.print(",LNG:");
  SerialBT.println(gps.location.lng(), 6);

  // Reset all state
  alertaEnProgreso     = false;
  ventanaActiva        = false;
  contadorConfirmacion = 0;
  gMaximoRegistrado    = 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  SerialBT.begin("MotoGuard_ESP32");


  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(500);

  delay(500);

  // Calibrate safe orientation
  Serial.println("--> Calibrando posicion SAFE. No mover...");
  float sumaX = 0, sumaY = 0, sumaZ = 0;
  const int muestras = 50;
  for (int i = 0; i < muestras; i++) {
    float tx, ty, tz;
    leerG_Sensores(tx, ty, tz);
    sumaX += tx; sumaY += ty; sumaZ += tz;
    delay(20);
  }
  float posX = sumaX / muestras;
  float posY = sumaY / muestras;
  float posZ = sumaZ / muestras;
  float mag  = sqrt(posX*posX + posY*posY + posZ*posZ);
  safeX = posX / mag;
  safeY = posY / mag;
  safeZ = posZ / mag;

  Serial.println("--> Calibracion exitosa. MotoGuard listo.");
}

// ─────────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────────

void loop() {

  // 0. CHECK FOR REMOTE COMMANDS FROM APP
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd == "CMD:RESET") {
      Serial.println("--> Reset remoto solicitado por la App. Reiniciando...");
      SerialBT.println("ACK:RESET");
      delay(200); // Let ACK transmit before reset
      ESP.restart();
    }
  }
  while (neogps.available() > 0) gps.encode(neogps.read());

  // 2. READ IMU
  float cx, cy, cz;
  leerG_Sensores(cx, cy, cz);

  float gTotal = sqrt(cx*cx + cy*cy + cz*cz);

  // Tilt angle vs calibrated safe vector
  float nx = cx / gTotal, ny = cy / gTotal, nz = cz / gTotal;
  float dot = safeX*nx + safeY*ny + safeZ*nz;
  dot = constrain(dot, -1.0f, 1.0f);
  float angulo = acos(dot) * 180.0 / M_PI;

  float velocidad = gps.speed.isValid() ? gps.speed.kmph() : 0.0;

  // ── PATH 0: INSTANT SPIKE ────────────────────────────────────────────────
  // If G jumps by more than UMBRAL_SPIKE_INMEDIATO in a single sample,
  // it's physically impossible from a bump or pothole — fires immediately.
  // No confirmation window needed: the delta itself is the confirmation.
  float gDelta = gTotal - gAnterior;
  if (gDelta > UMBRAL_SPIKE_INMEDIATO) {
    Serial.printf("!!! SPIKE INMEDIATO: delta=%.2fG (%.2f->%.2fG) !!!\n",
                  gDelta, gAnterior, gTotal);
    enviarAlerta(gTotal, "SPIKE");
    gAnterior = gTotal;
    delay(5000);
    return;
  }
  gAnterior = gTotal;
  // Conditions (ALL must be true simultaneously):
  //   • Was moving faster than VELOCIDAD_MINIMA_CRASH (not a GPS glitch)
  //   • Speed suddenly dropped below 10 km/h  (deceleration event)
  //   • Tilt exceeds ANGULO_DESVIO_CRITICO     (bike is displaced)
  //   • G-force exceeds UMBRAL_G_CONFIRMACION  (real impact, not gentle lean)
  //
  // A confirmation COUNTER must reach MUESTRAS_CONFIRMACION consecutive
  // samples within VENTANA_MAX_MS ms — a bump lasts 1-2 samples, a crash
  // keeps all conditions true for many samples.

  bool condicionAltaVelocidad = (velocidad < 10.0 &&
                                  velocidad >= VELOCIDAD_MINIMA_CRASH - 5.0 &&
                                  angulo    > ANGULO_DESVIO_CRITICO &&
                                  gTotal    > UMBRAL_G_CONFIRMACION);

  if (condicionAltaVelocidad) {
    if (!ventanaActiva) {
      // Open confirmation window
      ventanaActiva        = true;
      tiempoInicioVentana  = millis();
      contadorConfirmacion = 1;
      gMaximoRegistrado    = gTotal;
      Serial.println("--> Ventana de confirmacion alta velocidad abierta...");
    } else {
      // Window already open — keep counting
      contadorConfirmacion++;
      if (gTotal > gMaximoRegistrado) gMaximoRegistrado = gTotal;

      if (contadorConfirmacion >= MUESTRAS_CONFIRMACION) {
        // Confirmed crash
        enviarAlerta(gMaximoRegistrado, "ALTA_VELOCIDAD");
        delay(5000);
        return;
      }
    }
  } else {
    // Condition broke — check if window expired without confirmation
    if (ventanaActiva) {
      if (millis() - tiempoInicioVentana > VENTANA_MAX_MS) {
        // Window expired → it was just a bump
        Serial.printf("<-- Falsa alarma descartada (%d muestras, %.2fG)\n",
                      contadorConfirmacion, gMaximoRegistrado);
        ventanaActiva        = false;
        contadorConfirmacion = 0;
        gMaximoRegistrado    = 0.0;
      }
      // If window hasn't expired yet, keep it open — condition might return
    }
  }

  // ── PATH B: STATIC / LOW-SPEED FALL ─────────────────────────────────────
  // Conditions: high G OR high tilt, must persist for TIEMPO_GRACIA ms.
  // This covers: stationary fall, low-speed drop, parking accident.

  bool condicionEstatica = (gTotal > UMBRAL_IMPACTO || angulo > UMBRAL_INCLINACION);

  if (condicionEstatica) {
    if (!alertaEnProgreso) {
      alertaEnProgreso     = true;
      tiempoInicioAnomalia = millis();
      gMaximoRegistrado    = gTotal;
      Serial.println("--> Anomalia detectada. Esperando gracia...");
    } else {
      if (gTotal > gMaximoRegistrado) gMaximoRegistrado = gTotal;

      if (millis() - tiempoInicioAnomalia > TIEMPO_GRACIA) {
        enviarAlerta(gMaximoRegistrado, "CONFIRMADO");
        delay(5000);
        return;
      }
    }
  } else {
    if (alertaEnProgreso) {
      Serial.println("<-- Cancelado: moto reposicionada dentro del tiempo de gracia.");
      alertaEnProgreso  = false;
      gMaximoRegistrado = 0.0;
    }
  }

  // 3. PERIODIC TELEMETRY (every 1 second)
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

  // Small delay gives a ~50ms loop period → each sample = ~50ms
  delay(50);
}

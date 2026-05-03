#include <Arduino.h>
#include <Wire.h>

// Dirección I2C del MPU
const int MPU_ADDR = 0x68; 

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  // Despertar al MPU (por defecto inicia en modo sueño)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Registro de gestión de energía
  Wire.write(0);    // Poner a 0 para despertar
  Wire.endTransmission(true);
  
  Serial.println("¡MotoGuard: Sensor Despierto!");
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Empezar a leer desde el registro de aceleración X
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // Pedir 6 bytes (X, Y, Z)

  // Leer los datos (cada eje son 2 bytes)
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  // Convertir a valores legibles (G's aproximadamente)
  float x = ax / 16384.0;
  float y = ay / 16384.0;
  float z = az / 16384.0;

  Serial.print("Inclinación -> X: "); Serial.print(x);
  Serial.print(" | Y: "); Serial.print(y);
  Serial.print(" | Z: "); Serial.println(z);

  // Lógica simple de alarma para probar
  if (abs(x) > 0.5 || abs(y) > 0.5) {
    Serial.println("!!! ALERTA: MOTO EN MOVIMIENTO !!!");
  }

  delay(200);
}
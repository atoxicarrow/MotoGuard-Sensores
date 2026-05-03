#include <Arduino.h>
#include <unity.h>
#include <Wire.h>

void test_i2c_communication(void) {
    Wire.begin(21, 22);
    Wire.beginTransmission(0x68);
    uint8_t error = Wire.endTransmission();
    
    // Si error es 0, el dispositivo respondió
    TEST_ASSERT_EQUAL_UINT8(0, error);
}

void setup() {
    delay(2000); // Esperar a que el hardware estabilice
    UNITY_BEGIN();
    RUN_TEST(test_i2c_communication);
    UNITY_END();
}

void loop() {}
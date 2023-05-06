#include <M5AtomS3.h>
#include "flight_control.hpp"

//VL53L0X_ADDRESS           0x29
//MPU6886_ADDRESS           0x68
//BMP280_ADDRESS            0x76

void setup() {
  M5.begin(true, true , false, true);
  init_atomfly();
  delay(100);
}

void loop() {
  loop_400Hz();
}

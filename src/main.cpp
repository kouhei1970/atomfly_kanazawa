#include <Arduino.h>
#include <M5Atom.h>
#include "rc.hpp"
#include "flight_control.hpp"

//VL53L0X_ADDRESS           0x29
//MPU6886_ADDRESS           0x68
//BMP280_ADDRESS            0x76


void setup() {  
  M5.begin(true, false, true);
  init_atomfly();
  delay(1000);
}

//float Pitch,Roll,Yaw;

void loop() {
  loop_400Hz();
}

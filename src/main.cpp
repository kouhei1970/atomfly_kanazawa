#include <Arduino.h>
#include <M5Atom.h>
#include "rc.hpp"
#include "control.hpp"

//VL53L0X_ADDRESS           0x29
//MPU6886_ADDRESS           0x69
//BMP280_ADDRESS            0x76


void setup() {  
  M5.begin(true, false, true);
  //SerialBT.begin("ATOMFLY");
  init_atomfly();
  delay(1000);
}

//float Pitch,Roll,Yaw;

void loop() {
  loop_400Hz();
  //rc_demo();
  //atomfly_main();
  //M5.dis.drawpix(0, 0xff00ff);
  //delay(33);
}
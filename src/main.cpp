#include <Arduino.h>
#include <M5Atom.h>
#include "control.hpp"

//VL53L0X_ADDRESS           0x29
//MPU6886_ADDRESS           0x69
//BMP280_ADDRESS            0x76


void setup() {
  unsigned long start_time, end_time;  
  start_time = micros();

  M5.begin(true, true, true);
  Serial.begin(115200);  // start serial for output
  Serial.println();
  Serial.println(start_time);


  init_atomfly();



  end_time = micros();
  Serial.println(end_time);
  Serial.println(end_time - start_time);
}

void loop() {
  M5.update();
  atomfly_main();
  M5.update();
  M5.dis.drawpix(0, 0xff00ff);
  delay(10);
}
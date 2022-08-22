#include <Arduino.h>
#include <M5Atom.h>
#include "vl53l0x.h"
#include "M5_ENV.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "control.hpp"

//VL53L0X_ADDRESS           0x29
//MPU6886_ADDRESS           0x69
//BMP280_ADDRESS            0x76

const int pwmA = 22;
const int pwmB = 19;
const int pwmC = 23;
const int pwmD = 33;

const int freq = 5000;
const int ledChannelA = 1;
const int ledChannelB = 2;
const int ledChannelC = 3;
const int ledChannelD = 4;
const int ledChannel1 = 0;
const int ledChannel2 = 5;
const int resolution = 8;

double pitch, roll;  // Stores attitude related variables.  存储姿态相关变量
double r_rand = 180 / PI;

Adafruit_BMP280 bme;

float pressure = 0.0;

void setup() {
  M5.begin(true, true, true);
  Wire.begin();          // join i2c bus (address optional for master)
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");

  M5.IMU.Init();
  Serial.begin(115200);  // start serial for output
  Serial.println("VLX53LOX test started.");
  Serial.println(F("BMP280 test started...\n"));
  M5.dis.drawpix(0, 0xff0000);
  delay(1000);
  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcSetup(ledChannelA, freq, resolution);
  ledcSetup(ledChannelB, freq, resolution);
  ledcSetup(ledChannelC, freq, resolution);
  ledcSetup(ledChannelD, freq, resolution);
  ledcAttachPin(pwmA, ledChannelA);
  ledcAttachPin(pwmB, ledChannelB);
  ledcAttachPin(pwmC, ledChannelC);
  ledcAttachPin(pwmD, ledChannelD);
  M5.dis.drawpix(0, 0x0000f0);
}

void loop() {
  M5.update();

  if (true /*M5.Btn.wasReleased() || M5.Btn.pressedFor(500)*/) {
    M5.dis.drawpix(0, 0xfff000);
#if 0 
    write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

    byte val = 0;
    int cnt  = 0;
    while (cnt < 100) {  // 1 second waiting time max
        delay(10);
        val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
        if (val & 0x01) break;
        cnt++;
    }
    if (val & 0x01)
        Serial.println("VL53L0X is ready");
    else
        Serial.println("VL53L0X is not ready");

    read_block_data_at(0x14, 12);
    uint16_t acnt                  = makeuint16(gbuf[7], gbuf[6]);
    uint16_t scnt                  = makeuint16(gbuf[9], gbuf[8]);
    uint16_t dist                  = makeuint16(gbuf[11], gbuf[10]);
    byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
    Serial.print("ambient count: ");
    Serial.println(acnt);
    Serial.print("signal count: ");
    Serial.println(scnt);
    Serial.print("ambient count: ");
    Serial.println(acnt);
    Serial.print("distance: ");
    Serial.println(dist);
    Serial.print("status: ");
    Serial.println(DeviceRangeStatusInternal);
    delay(1000);


    Serial.println("IMU Ready");
    M5.IMU.getAttitude(&pitch,
                       &roll);  // Read the attitude (pitch, heading) of the IMU
                                // and store it in relevant variables.
                                // 读取IMU的姿态（俯仰、航向）并存储至相关变量
    double arc = atan2(pitch, roll) * r_rand + 180;
    double valIMU = sqrt(pitch * pitch + roll * roll);
    Serial.printf("%.2f,%.2f,%.2f,%.2f\n", pitch, roll, arc,
                  valIMU);  // serial port output the formatted string.  串口输出

    while (!bme.begin(0x76)) {  //初始化bme传感器.  Init the sensor of bme
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        M5.dis.drawpix(0, 0xff0000);
    }
    M5.dis.drawpix(0, 0xfff000);
    pressure = bme.readPressure();  // Stores the pressure gained by BMP.
                                    // 存储bmp获取到的压强
    Serial.printf("Pressure:%2.0fPa\n",
                  pressure);
  
    delay(500);

    ledcAttachPin(pwmA, ledChannelA);
    ledcAttachPin(pwmB, ledChannelB);
    ledcAttachPin(pwmC, ledChannelC);
    ledcAttachPin(pwmD, ledChannelD);
    M5.dis.drawpix(0, 0x00ff00);
    delay(1000);

    ledcWrite(ledChannelA, 100);
    delay(100);
    ledcWrite(ledChannelA, 0);

    ledcWrite(ledChannelB, 100);
    delay(100);
    ledcWrite(ledChannelB, 0);

    ledcWrite(ledChannelC, 100);
    delay(100);
    ledcWrite(ledChannelC, 0);

    ledcWrite(ledChannelD, 100);
    delay(100);
    ledcWrite(ledChannelD, 0);

    delay(2000);
#endif
    ledcAttachPin(pwmA, ledChannel1);
    ledcAttachPin(pwmB, ledChannel1);
    ledcAttachPin(pwmC, ledChannel2);
    ledcAttachPin(pwmD, ledChannel2);


    for(float dutyCycle = 5; dutyCycle <= 250; dutyCycle++){   
      ledcWrite(ledChannel1, dutyCycle);
      ledcWrite(ledChannel2, dutyCycle + 0.1);
      delay(5);
    }
    delay(500);
    for(float dutyCycle = 250; dutyCycle >= 5; dutyCycle--){
      ledcWrite(ledChannel1, dutyCycle); 
      ledcWrite(ledChannel2, dutyCycle - 0.1);  
      delay(5);
    }

    delay(500);
  }
  M5.update();
  M5.dis.drawpix(0, 0x0000f0);
}
#include "sensor.hpp"


MPU6886 IMU;
//Adafruit_BMP280 bme;
Madgwick Drone_ahrs;

// Set I2C address to 0x40 (A0 pin -> GND)
INA3221 ina3221(INA3221_ADDR40_GND);
Filter acc_filter;
Filter voltage_filter;

//Sensor data
volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
volatile float Phi=0.0f, Theta=0.0f, Psi=0.0f;
volatile float Voltage;
float Acc_norm=0.0f;
//quat_t Quat;
float Over_g=0.0f, Over_rate=0.0f;
uint8_t OverG_flag = 0;
volatile float Pbias=0.0f, Qbias=0.0f, Rbias=0.0f;
volatile uint8_t Power_flag = 0;

uint8_t init_i2c()
{
  Wire1.begin(SDA_PIN, SCL_PIN,400000UL);
  USBSerial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  for (short i = 0; i < 256; i++)
  {
    Wire1.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire1.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      USBSerial.print ("Found address: ");
      USBSerial.print (i, DEC);
      USBSerial.print (" (0x");
      USBSerial.print (i, HEX);     // PCF8574 7 bit address
      USBSerial.println (")");
      count++;
    }
  }
  USBSerial.print ("Found ");      
  USBSerial.print (count, DEC);        // numbers of devices
  USBSerial.println (" device(s).");
  return count;
}

uint8_t mpu6886_byte_read(uint8_t reg_addr)
{
  uint8_t data;
  Wire1.beginTransmission (MPU6886_ADDRESS);
  Wire1.write(reg_addr);
  Wire1.endTransmission();
  Wire1.requestFrom(MPU6886_ADDRESS, 1);
  data = Wire1.read();
  return data;
}

void mpu6886_byte_write(uint8_t reg_addr, uint8_t data)
{
  Wire1.beginTransmission (MPU6886_ADDRESS);
  Wire1.write(reg_addr);
  Wire1.write(data);
  Wire1.endTransmission();
}


void imu_init(void)
{
  //Cutoff frequency
  //filter_config Gyro Accel
  //0 250    218.1 log140　Bad
  //1 176    218.1 log141　Bad
  //2 92     99.0  log142 Bad これはヨーガカクカクする log256
  //3 41     44.8  log143 log188　Good! log257
  //4 20     21.2
  //5 10     10.2
  //6 5      5.1
  //7 3281   420.0
  uint8_t data;
  const uint8_t filter_config = 2;//(今の所2はノイズが多くてダメ、log188は3)

  //Mdgwick filter 実験
  // filter_config=0において実施
  //beta =0 次第に角度増大（角速度の積分のみに相当する）
  //beta=0.5

  IMU.Init();
  //IMUのデフォルトI2C周波数が100kHzなので400kHzに上書き

  MPU6886 imu;

  Wire1.begin(SDA_PIN, SCL_PIN, 400000UL);

 //F_CHOICE_B
  data = mpu6886_byte_read(MPU6886_GYRO_CONFIG);
  USBSerial.printf("GYRO_CONFIG %d\r\n", data);
  mpu6886_byte_write(MPU6886_GYRO_CONFIG, data & 0b11111100);
  data = mpu6886_byte_read(MPU6886_GYRO_CONFIG);
  USBSerial.printf("Update GYRO_CONFIG %d\r\n", data);

  //Gyro
  //DLPG_CFG
  data = mpu6886_byte_read(MPU6886_CONFIG);
  USBSerial.printf("CONFIG %d\r\n", data);
  mpu6886_byte_write(MPU6886_CONFIG, (data&0b11111100)|filter_config);
  data = mpu6886_byte_read(MPU6886_CONFIG);
  USBSerial.printf("Update CONFIG %d\r\n", data);

  //Accel
  //ACCEL_FCHOCE_B & A_DLPF_CFG
  data = mpu6886_byte_read(MPU6886_ACCEL_CONFIG2);
  USBSerial.printf("ACCEL_CONFIG2 %d\r\n", data);
  mpu6886_byte_write(MPU6886_ACCEL_CONFIG2, (data & 0b11110111) | filter_config);
  data = mpu6886_byte_read(MPU6886_ACCEL_CONFIG2);
  USBSerial.printf("Update ACCEL_CONFIG2 %d\r\n", data);

}

void test_rangefinder(void)
{
  #if 1
  USBSerial.println("VLX53LOX test started.");
  //USBSerial.println(F("BMP280 test started...\n"));

  //Begin Range finder Test
  //USBSerial.println(read_byte_data_at(VL53L0X_REG_IDENTIFICATION_MODEL_ID));
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
      USBSerial.printf("VL53L0X is ready. cnt=%d\n",cnt);
  else
      USBSerial.println("VL53L0X is not ready");

  read_block_data_at(0x14, 12);
  uint16_t acnt = makeuint16(gbuf[7], gbuf[6]);
  uint16_t scnt = makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
  byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
  USBSerial.print("ambient count: ");
  USBSerial.println(acnt);
  USBSerial.print("signal count: ");
  USBSerial.println(scnt);
  USBSerial.print("ambient count: ");
  USBSerial.println(acnt);
  USBSerial.print("distance: ");
  USBSerial.println(dist);
  USBSerial.print("status: ");
  USBSerial.println(DeviceRangeStatusInternal);
  //End Range finder Test
  #endif
}

void sensor_init()
{
  if(init_i2c()==0)
  {
    USBSerial.printf("No I2C device!\r\n");
    USBSerial.printf("Can not boot AtomFly2.\r\n");
    while(1);
  }

  imu_init();
  //test_rangefinder();
  Drone_ahrs.begin(400.0);
  ina3221.begin(&Wire1);
  ina3221.reset();  
  //voltage_filter.set_parameter(0.005, 0.0025);
  //Acceleration filter
  acc_filter.set_parameter(0.005, 0.0025);

}


uint16_t get_distance(void)
{
  #if 0
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

  byte val = 0;
  int cnt  = 0;
  while (cnt < 1000) {  // 1 second waiting time max
      val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
      if (val & 0x01) break;
      cnt++;
  }
  //if (val & 0x01)
  //    USBSerial.printf("VL53L0X is ready. cnt=%d\n",cnt);
  //else
  //    USBSerial.println("VL53L0X is not ready");

  read_block_data_at(0x14, 12);
  //uint16_t acnt                  = makeuint16(gbuf[7], gbuf[6]);
  //uint16_t scnt                  = makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
  //byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
  return dist;
  #endif
  return 0;
}

void sensor_read(void)
{
  float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;
  float filterd_v;
  static float dp, dq, dr; 

  IMU.getAccelData(&ax, &ay, &az);
  IMU.getGyroData(&gx, &gy, &gz);
  //ax = ax;
  //ay = ay;
  if(Mode > AVERAGE_MODE)
  {
    Drone_ahrs.updateIMU(gx-Qbias*(float)RAD_TO_DEG, gy-Pbias*(float)RAD_TO_DEG, gz-Rbias*(float)RAD_TO_DEG, ax, ay, az);
    //Drone_ahrs.updateIMU(gx, gy, gz, ax, ay, az);
    Theta = Drone_ahrs.getRoll()*(float)DEG_TO_RAD;
    Phi =   Drone_ahrs.getPitch()*(float)DEG_TO_RAD;
    Psi =   Drone_ahrs.getYaw()*(float)DEG_TO_RAD;
  }

  Ax = ay;
  Ay = ax;
  Az = az;
  dp = Wp;
  dq = Wq;
  dr = Wr;
  Wp = gy*(float)DEG_TO_RAD;
  Wq = gx*(float)DEG_TO_RAD;
  Wr = gz*(float)DEG_TO_RAD;

  if(Wp>8.0||Wp<-8.0)Wp = dp;
  if(Wq>8.0||Wq<-8.0)Wq = dq;
  if(Wr>8.0||Wr<-8.0)Wr = dr;

  #if 1
  acc_norm = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  Acc_norm = acc_filter.update(acc_norm);
  if (Acc_norm>9.0) 
  {
    OverG_flag = 1;
    if (Over_g == 0.0)Over_g = acc_norm;
  }
  #endif
  
  #if 0
  rate_norm = sqrt((Wp-Pbias)*(Wp-Pbias) + (Wq-Qbias)*(Wq-Qbias) + (Wr-Rbias)*(Wr-Rbias));
  if (rate_norm > 800.0)
  {
    OverG_flag = 0;
    if (Over_rate == 0.0) Over_rate =rate_norm;
  } 
  #endif
  
  #if 1
  Voltage = ina3221.getVoltage(INA3221_CH2);
  filterd_v = voltage_filter.update(Voltage);

  if(Power_flag != POWER_FLG_MAX){
    if (filterd_v < POWER_LIMIT) Power_flag ++;
    else Power_flag = 0;
    if ( Power_flag > POWER_FLG_MAX) Power_flag = POWER_FLG_MAX;
  }
  #endif

  #if 0
  if(Stick[BUTTON_A]==1)
  USBSerial.printf("%9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f \r\n", 
    Elapsed_time, Elapsed_time - Old_Elapsed_time ,v1, v2, v3, 
    (Phi-Phi_bias)*180/PI, (Theta-Theta_bias)*180/PI, (Psi-Psi_bias)*180/PI);
  #endif
}


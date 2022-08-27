#include <Arduino.h>
#include <M5Atom.h>
#include "vl53l0x.h"
#include "M5_ENV.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <MadgwickAHRS.h>

#include "control.hpp"

const int pwmFL = 22;
const int pwmFR = 19;
const int pwmRL = 23;
const int pwmRR = 33;

const int freq = 300000;
const int FL_motor = 1;
const int FR_motor = 2;
const int RL_motor = 3;
const int RR_motor = 4;
const int ledChannel1 = 0;
const int ledChannel2 = 5;
const int resolution = 8;

double pitch, roll;  // Stores attitude related variables.  存储姿态相关变量
double r_rand = 180 / PI;

Adafruit_BMP280 bme;

float pressure = 0.0;

//Sensor data
float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
float Acc_norm=0.0;
quat_t Quat;

//Times
float Elapsed_time=0.0;
uint32_t S_time=0,E_time=0,D_time=0,S_time2=0,E_time2=0,D_time2=0;

//Counter
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t BiasCounter=0;
uint16_t LedBlinkCounter=0;

//Control 
float FR_duty, FL_duty, RR_duty, RL_duty;
float P_com, Q_com, R_com;
float T_ref;
float Pbias=0.0,Qbias=0.0,Rbias=0.0;
float Phi_bias=0.0,Theta_bias=0.0,Psi_bias=0.0;  
float Phi,Theta,Psi;
float Phi_ref=0.0,Theta_ref=0.0,Psi_ref=0.0;
float Elevator_center=0.0, Aileron_center=0.0, Rudder_center=0.0;
float Pref=0.0,Qref=0.0,Rref=0.0;
const float Phi_trim   = 0.01;
const float Theta_trim = 0.02;
const float Psi_trim   = 0.0;

//RC
uint16_t Chdata[18];

//Log
uint16_t LogdataCounter=0;
uint8_t Logflag=0;
volatile uint8_t Logoutputflag=0;
float Log_time=0.0;
const uint8_t DATANUM=38; //Log Data Number
const uint32_t LOGDATANUM=48000;
float Logdata[LOGDATANUM]={0.0};

//Machine state
uint8_t LockMode=0;
float Disable_duty =0.10;
float Flight_duty  =0.18;//0.2/////////////////
uint8_t OverG_flag = 0;
uint8_t Arm_flag = 0;

//PID object and etc.
Filter acc_filter;
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;


void loop_400Hz(void);
void rate_control(void);
void sensor_read(void);
void angle_control(void);
void output_data(void);
void output_sensor_raw_data(void);
//void kalman_filter(void);
void logging(void);
void motor_stop(void);
uint8_t lock_com(void);
uint8_t logdata_out_com(void);
void printPQR(void);


#define AVERAGE 2000
#define KALMANWAIT 6000

void gpio_put(uint8_t p, uint8_t state)
{
  return;
}

void init_atomfly(void)
{
  init_i2c();
  M5.IMU.Init();
  Serial.println("VLX53LOX test started.");
  Serial.println(F("BMP280 test started...\n"));
  M5.dis.drawpix(0, 0xff0000);
  delay(1000);
  test_rangefinder();
  init_pwm();
  M5.dis.drawpix(0, 0x0000f0);
}

void init_i2c()
{
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

}

uint16_t get_distance(void)
{
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);
  read_block_data_at(0x14, 12);
  uint16_t dist                  = makeuint16(gbuf[11], gbuf[10]);
  return dist;
}

void init_pwm(void)
{
  //ledcSetup(ledChannel1, freq, resolution);
  //ledcSetup(ledChannel2, freq, resolution);
  ledcSetup(FL_motor, freq, resolution);
  ledcSetup(FR_motor, freq, resolution);
  ledcSetup(RL_motor, freq, resolution);
  ledcSetup(RR_motor, freq, resolution);
  ledcAttachPin(pwmFL, FL_motor);
  ledcAttachPin(pwmFR, FR_motor);
  ledcAttachPin(pwmRL, RL_motor);
  ledcAttachPin(pwmRR, RR_motor);
}

void test_rangefinder(void)
{
  //Begin Range finder Test
  //Serial.println(read_byte_data_at(VL53L0X_REG_IDENTIFICATION_MODEL_ID));
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
  //End Range finder Test
}


void atomfly_main(void)
{

  uint16_t dist;
  dist = get_distance();
  Serial.println(dist);
  #if 0
  if (M5.Btn.wasReleased() || M5.Btn.pressedFor(500)) {
    M5.dis.drawpix(0, 0xfff000);

    

    //Serial.println("IMU Ready");
    M5.IMU.getAttitude(&pitch,
                       &roll);  // Read the attitude (pitch, heading) of the IMU
                                // and store it in relevant variables.
                                // 读取IMU的姿态（俯仰、航向）并存储至相关变量
    double arc = atan2(pitch, roll) * r_rand + 180;
    double valIMU = sqrt(pitch * pitch + roll * roll);
    //Serial.println("hoge");
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
    delay(20);
    
    ledcAttachPin(pwmFL, FL_motor);
    ledcAttachPin(pwmFR, FR_motor);
    ledcAttachPin(pwmRL, RL_motor);
    ledcAttachPin(pwmRR, RR_motor);
    M5.dis.drawpix(0, 0x00ff00);
    delay(1000);

    ledcWrite(FL_motor, 100);
    delay(100);
    ledcWrite(FL_motor, 0);

    ledcWrite(FR_motor, 100);
    delay(100);
    ledcWrite(FR_motor, 0);

    ledcWrite(RL_motor, 100);
    delay(100);
    ledcWrite(RL_motor, 0);

    ledcWrite(RR_motor, 100);
    delay(100);
    ledcWrite(RR_motor, 0);

    delay(2000);
    float duty=64;
    ledcWrite(FL_motor, duty);
    ledcWrite(FR_motor, duty);
    ledcWrite(RL_motor, duty);
    ledcWrite(RR_motor, duty);
    delay(5000);
    ledcWrite(FL_motor, 0);
    ledcWrite(FR_motor, 0);
    ledcWrite(RL_motor, 0);
    ledcWrite(RR_motor, 0);

  }
  #endif

}



void set_duty_fr(double duty){}
void set_duty_fl(double duty){}
void set_duty_rr(double duty){}
void set_duty_rl(double duty){}

void imu_mag_data_read(float* ax, float* ay, float* az, float* gx, float* gy, float* gz){}
void madgwick_filter(quat_t* quat){}




//Main loop
//This function is called from PWM Intrupt on 400Hz.
void loop_400Hz(void)
{
  static uint8_t led=1;
  //S_time=time_us_32();
  
  //割り込みフラグリセット
  //pwm_clear_irq(2);


  if (Arm_flag==0)
  {
      //motor_stop();
      Elevator_center = 0.0;
      Aileron_center = 0.0;
      Rudder_center = 0.0;
      Pbias = 0.0;
      Qbias = 0.0;
      Rbias = 0.0;
      Phi_bias = 0.0;
      Theta_bias = 0.0;
      Psi_bias = 0.0;
      return;
  }
  else if (Arm_flag==1)
  {
    motor_stop();
    //Gyro Bias Estimate
    if (BiasCounter < AVERAGE)
    {
      //Sensor Read
      sensor_read();
      Aileron_center  += Chdata[AILERON];
      Elevator_center += Chdata[ELEVATOR];
      Rudder_center   += Chdata[RUDDER];
      Pbias += Wp;
      Qbias += Wq;
      Rbias += Wr;
      Mx_ave += Mx;
      My_ave += My;
      Mz_ave += Mz;
      BiasCounter++;
      return;
    }
    else if(BiasCounter<KALMANWAIT)
    {
      //Sensor Read
      sensor_read();
      if(BiasCounter == AVERAGE)
      {
        Elevator_center = Elevator_center/AVERAGE;
        Aileron_center  = Aileron_center/AVERAGE;
        Rudder_center   = Rudder_center/AVERAGE;
        Pbias = Pbias/AVERAGE;
        Qbias = Qbias/AVERAGE;
        Rbias = Rbias/AVERAGE;
        Mx_ave = Mx_ave/AVERAGE;
        My_ave = My_ave/AVERAGE;
        Mz_ave = Mz_ave/AVERAGE;

        //Xe(4,0) = Pbias;
        //Xe(5,0) = Qbias;
        //Xe(6,0) = Rbias;
        //Xp(4,0) = Pbias;
        //Xp(5,0) = Qbias;
        //Xp(6,0) = Rbias;
        //MN = Mx_ave;
        //ME = My_ave;
        //MD = Mz_ave;
      }
      
      AngleControlCounter++;
      if(AngleControlCounter==4)
      {
        AngleControlCounter=0;
        //Multicore control
        //sem_release(&sem);
      
      }
      Phi_bias   += Phi;
      Theta_bias += Theta;
      Psi_bias   += Psi;
      BiasCounter++;
      return;
    }
    else
    {
      Arm_flag = 3;
      Phi_bias   = Phi_bias/KALMANWAIT;
      Theta_bias = Theta_bias/KALMANWAIT;
      Psi_bias   = Psi_bias/KALMANWAIT;
      return;
    }
  }
  else if( Arm_flag==2)
  {
    if(LockMode==2)
    {
      if(lock_com()==1)
      {
        LockMode=3;//Disenable Flight
        led=0;
        gpio_put(LED_PIN,led);
        return;
      }
      //Goto Flight
    }
    else if(LockMode==3)
    {
      if(lock_com()==0){
        LockMode=0;
        Arm_flag=3;
      }
      return;
    }
    //LED Blink
    gpio_put(LED_PIN, led);
    if(Logflag==1&&LedBlinkCounter<100){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
   
    //Rate Control (400Hz)
    rate_control();
   
    if(AngleControlCounter==4)
    {
      AngleControlCounter=0;
      //Angle Control (100Hz)
      //sem_release(&sem);
    }
    AngleControlCounter++;
  }
  else if(Arm_flag==3)
  {
    motor_stop();
    OverG_flag = 0;
    if(LedBlinkCounter<10){
      gpio_put(LED_PIN, 1);
      LedBlinkCounter++;
    }
    else if(LedBlinkCounter<100)
    {
      gpio_put(LED_PIN, 0);
      LedBlinkCounter++;
    }
    else LedBlinkCounter=0;
    
    //Get Stick Center 
    Aileron_center  = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center   = Chdata[0];
  
    if(LockMode==0)
    {
      if( lock_com()==1)
      {
        LockMode=1;
        return;
      }
      //Wait  output log
    }
    else if(LockMode==1)
    {
      if(lock_com()==0)
      {
        LockMode=2;//Enable Flight
        Arm_flag=2;
      }
      return;
    }

    if(logdata_out_com()==1)
    {
      Arm_flag=4;
      return;
    }
  }
  else if(Arm_flag==4)
  {
    motor_stop();
    Logoutputflag=1;
    //LED Blink
    gpio_put(LED_PIN, led);
    if(LedBlinkCounter<400){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
  }
  //E_time=time_us_32();
  //D_time=E_time-S_time;
}

void control_init(void)
{
  acc_filter.set_parameter(0.005, 0.0025);
  //Rate control
  p_pid.set_parameter( 2.0, 0.145, 0.028, 0.015, 0.0025);//3.4
  q_pid.set_parameter( 2.1, 0.125, 0.028, 0.015, 0.0025);//3.8
  r_pid.set_parameter(12.0, 0.5, 0.008, 0.015, 0.0025);//9.4
  //Angle control
  phi_pid.set_parameter  ( 5.5, 9.5, 0.025, 0.018, 0.01);//6.0
  theta_pid.set_parameter( 5.5, 9.5, 0.025, 0.018, 0.01);//6.0
  psi_pid.set_parameter  ( 0.0, 10.0, 0.010, 0.03, 0.01);
  //Rate control
  //p_pid.set_parameter(3.3656, 0.1, 0.0112, 0.01, 0.0025);
  //q_pid.set_parameter(3.8042, 0.1, 0.0111, 0.01, 0.0025);
  //r_pid.set_parameter(9.4341, 0.11, 0.0056, 0.01, 0.0025);
  //Angle control
  //phi_pid.set_parameter  ( 9.0   , 0.07, 0.0352,  0.01, 0.01);
  //theta_pid.set_parameter( 8.5583, 0.1 , 0.0552,  0.01, 0.01);
  //psi_pid.set_parameter  ( 9.0256, 0.11, 0.0034,  0.01, 0.01);
}

uint8_t lock_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Chdata[2]<CH3MIN+80 
   && Chdata[0]>CH1MAX-80
   && Chdata[3]<CH4MIN+80 
   && Chdata[1]>CH2MAX-80)
  { 
    chatta++;
    if(chatta>50){
      chatta=50;
      state=1;
    }
  }
  else 
  {
    chatta=0;
    state=0;
  }

  return state;

}

uint8_t logdata_out_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Chdata[4]<(CH5MAX+CH5MIN)*0.5 
   && Chdata[2]<CH3MIN+80 
   && Chdata[0]<CH1MIN+80
   && Chdata[3]>CH4MAX-80 
   && Chdata[1]>CH2MAX-80)
  {
    chatta++;
    if(chatta>50){
      chatta=50;
      state=1;
    }
  }
  else 
  {
    chatta=0;
    state=0;
  }

  return state;
}

void motor_stop(void)
{
  set_duty_fr(0.0);
  set_duty_fl(0.0);
  set_duty_rr(0.0);
  set_duty_rl(0.0);
}

void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_ref, q_ref, r_ref;
  float p_err, q_err, r_err;

  //Read Sensor Value
  sensor_read();

  //Get Bias
  //Pbias = Xe(4, 0);
  //Qbias = Xe(5, 0);
  //Rbias = Xe(6, 0);

  //Control angle velocity
  p_rate = Wp - Pbias;
  q_rate = Wq - Qbias;
  r_rate = Wr - Rbias;

  //Get reference
  p_ref = Pref;
  q_ref = Qref;
  r_ref = Rref;
  T_ref = 0.6 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);

  //Error
  p_err = p_ref - p_rate;
  q_err = q_ref - q_rate;
  r_err = r_ref - r_rate;

  //PID
  P_com = p_pid.update(p_err);
  Q_com = q_pid.update(q_err);
  R_com = r_pid.update(r_err);

  //Motor Control
  // 1250/11.1=112.6
  // 1/11.1=0.0901
  
  FR_duty = (T_ref +(-P_com +Q_com -R_com)*0.25)*0.0901;
  FL_duty = (T_ref +( P_com +Q_com +R_com)*0.25)*0.0901;
  RR_duty = (T_ref +(-P_com -Q_com +R_com)*0.25)*0.0901;
  RL_duty = (T_ref +( P_com -Q_com -R_com)*0.25)*0.0901;
  //FR_duty = (T_ref)*0.0901;
  //FL_duty = (T_ref)*0.0901;
  //RR_duty = (T_ref)*0.0901;
  //RL_duty = (T_ref)*0.0901;
  
  float minimum_duty=0.1;
  const float maximum_duty=0.95;
  minimum_duty = Disable_duty;

  if (FR_duty < minimum_duty) FR_duty = minimum_duty;
  if (FR_duty > maximum_duty) FR_duty = maximum_duty;

  if (FL_duty < minimum_duty) FL_duty = minimum_duty;
  if (FL_duty > maximum_duty) FL_duty = maximum_duty;

  if (RR_duty < minimum_duty) RR_duty = minimum_duty;
  if (RR_duty > maximum_duty) RR_duty = maximum_duty;

  if (RL_duty < minimum_duty) RL_duty = minimum_duty;
  if (RL_duty > maximum_duty) RL_duty = maximum_duty;

  //Duty set
  if(T_ref/BATTERY_VOLTAGE < Disable_duty)
  {
    motor_stop();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    Pref=0.0;
    Qref=0.0;
    Rref=0.0;
    Aileron_center  = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center   = Chdata[0];
    Phi_bias   = Phi;
    Theta_bias = Theta;
    Psi_bias   = Psi;
  }
  else
  {
    if (OverG_flag==0){
      set_duty_fr(FR_duty);
      set_duty_fl(FL_duty);
      set_duty_rr(RR_duty);
      set_duty_rl(RL_duty);
    }
    else motor_stop();
    //printf("%12.5f %12.5f %12.5f %12.5f\n",FR_duty, FL_duty, RR_duty, RL_duty);
  }
 
  //printf("\n");

  //printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n", 
  //    Elapsed_time, fr_duty, fl_duty, rr_duty, rl_duty, p_rate, q_rate, r_rate);
  //printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n", 
  //    Elapsed_time, p_com, q_com, r_com, p_ref, q_ref, r_ref);
  //printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n", 
  //    Elapsed_time, Phi, Theta, Psi, Phi_bias, Theta_bias, Psi_bias);
  //Elapsed_time = Elapsed_time + 0.0025;
  //Logging
  //logging();
}

void angle_control(void)
{
  float phi_err,theta_err,psi_err;
  float q0,q1,q2,q3;
  float e23,e33,e13,e11,e12;
  while(1)
  {
    //sem_acquire_blocking(&sem);
    //sem_reset(&sem, 0);
    //S_time2=time_us_32();
    //kalman_filter();
    madgwick_filter(&Quat);
    q0 = Quat.q0;
    q1 = Quat.q1;
    q2 = Quat.q2;
    q3 = Quat.q3;
    e11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    e12 = 2*(q1*q2 + q0*q3);
    e13 = 2*(q1*q3 - q0*q2);
    e23 = 2*(q2*q3 + q0*q1);
    e33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    Phi = atan2(e23, e33);
    Theta = atan2(-e13, sqrt(e23*e23+e33*e33));
    Psi = atan2(e12,e11);

    //Get angle ref 
    Phi_ref   = Phi_trim   + 0.3 * M_PI *(float)(Chdata[3] - (CH4MAX+CH4MIN)*0.5)*2/(CH4MAX-CH4MIN);
    Theta_ref = Theta_trim + 0.3 * M_PI *(float)(Chdata[1] - (CH2MAX+CH2MIN)*0.5)*2/(CH2MAX-CH2MIN);
    Psi_ref   = Psi_trim   + 0.8 * M_PI *(float)(Chdata[0] - (CH1MAX+CH1MIN)*0.5)*2/(CH1MAX-CH1MIN);

    //Error
    phi_err   = Phi_ref   - (Phi   - Phi_bias);
    theta_err = Theta_ref - (Theta - Theta_bias);
    psi_err   = Psi_ref   - (Psi   - Psi_bias);
    
    //PID Control
    if (T_ref/BATTERY_VOLTAGE < Flight_duty)
    {
      Pref=0.0;
      Qref=0.0;
      Rref=0.0;
      phi_pid.reset();
      theta_pid.reset();
      psi_pid.reset();
      Aileron_center  = Chdata[AILERON];
      Elevator_center = Chdata[ELEVATOR];
      Rudder_center   = Chdata[RUDDER];
      /////////////////////////////////////
      Phi_bias   = Phi;
      Theta_bias = Theta;
      Psi_bias   = Psi;
      /////////////////////////////////////
    }
    else
    {
      Pref = phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);
      Rref = Psi_ref;//psi_pid.update(psi_err);//Yawは角度制御しない
    }

    //Logging
    logging();

    //E_time2=time_us_32();
    //D_time2=E_time2-S_time2;

  }
}

void logging(void)
{  
  //Logging
  if(Chdata[4]>(CH5MAX+CH5MIN)*0.5)
  { 
    if(Logflag==0)
    {
      Logflag=1;
      LogdataCounter=0;
    }
    if(LogdataCounter+DATANUM<LOGDATANUM)
    {
      Logdata[LogdataCounter++]=Quat.q0;                  //1
      Logdata[LogdataCounter++]=Quat.q1;                  //2
      Logdata[LogdataCounter++]=Quat.q2;                  //3
      Logdata[LogdataCounter++]=Quat.q3;                  //4
      Logdata[LogdataCounter++]=Quat.q0;                  //5
      Logdata[LogdataCounter++]=Quat.q0;                  //6
      Logdata[LogdataCounter++]=Quat.q0;                  //7
      Logdata[LogdataCounter++]=Wp;//-Pbias;              //8
      Logdata[LogdataCounter++]=Wq;//-Qbias;              //9
      Logdata[LogdataCounter++]=Wr;//-Rbias;              //10

      Logdata[LogdataCounter++]=Ax;                       //11
      Logdata[LogdataCounter++]=Ay;                       //12
      Logdata[LogdataCounter++]=Az;                       //13
      Logdata[LogdataCounter++]=Mx;                       //14
      Logdata[LogdataCounter++]=My;                       //15
      Logdata[LogdataCounter++]=Mz;                       //16
      Logdata[LogdataCounter++]=Pref;                     //17
      Logdata[LogdataCounter++]=Qref;                     //18
      Logdata[LogdataCounter++]=Rref;                     //19
      Logdata[LogdataCounter++]=Phi-Phi_bias;             //20

      Logdata[LogdataCounter++]=Theta-Theta_bias;         //21
      Logdata[LogdataCounter++]=Psi-Psi_bias;             //22
      Logdata[LogdataCounter++]=Phi_ref;                  //23
      Logdata[LogdataCounter++]=Theta_ref;                //24
      Logdata[LogdataCounter++]=Psi_ref;                  //25
      Logdata[LogdataCounter++]=P_com;                    //26
      Logdata[LogdataCounter++]=Q_com;                    //27
      Logdata[LogdataCounter++]=R_com;                    //28
      Logdata[LogdataCounter++]=p_pid.m_integral;//m_filter_output;    //29
      Logdata[LogdataCounter++]=q_pid.m_integral;//m_filter_output;    //30

      Logdata[LogdataCounter++]=r_pid.m_integral;//m_filter_output;    //31
      Logdata[LogdataCounter++]=phi_pid.m_integral;//m_filter_output;  //32
      Logdata[LogdataCounter++]=theta_pid.m_integral;//m_filter_output;//33
      Logdata[LogdataCounter++]=Pbias;                    //34
      Logdata[LogdataCounter++]=Qbias;                    //35

      Logdata[LogdataCounter++]=Rbias;                    //36
      Logdata[LogdataCounter++]=T_ref;                    //37
      Logdata[LogdataCounter++]=Acc_norm;                 //38

   
    }
    else Logflag=2;
  }
  else
  { 
    if(Logflag>0)
    {
      Logflag=0;
      LogdataCounter=0;
    }
  }
}

void log_output(void)
{
  if(LogdataCounter==0)
  {
    printPQR();
    printf("#Roll rate PID gain\n");
    p_pid.printGain();
    printf("#Pitch rate PID gain\n");
    q_pid.printGain();
    printf("#Yaw rate PID gain\n");
    r_pid.printGain();
    printf("#Roll angle PID gain\n");
    phi_pid.printGain();
    printf("#Pitch angle PID gain\n");
    theta_pid.printGain();
  }
  if(LogdataCounter+DATANUM<LOGDATANUM)
  {
    //LockMode=0;
    printf("%10.2f ", Log_time);
    Log_time=Log_time + 0.01;
    for (uint8_t i=0;i<DATANUM;i++)
    {
      printf("%12.5f",Logdata[LogdataCounter+i]);
    }
    printf("\n");
    LogdataCounter=LogdataCounter + DATANUM;
  }
  else 
  {
    Arm_flag=3;
    Logoutputflag=0;
    LockMode=0;
    Log_time=0.0;
    LogdataCounter=0;
  }
}


void gyroCalibration(void)
{
  float wp,wq,wr;
  float sump,sumq,sumr;
  uint16_t N=400;
  for(uint16_t i=0;i<N;i++)
  {
    sensor_read();
    sump=sump+Wp;
    sumq=sumq+Wq;
    sumr=sumr+Wr;
  }
  Pbias=sump/N;
  Qbias=sumq/N;
  Rbias=sumr/N;
}

void sensor_read(void)
{
  float mx1, my1, mz1, mag_norm, acc_norm, rate_norm;

  imu_mag_data_read(&Ax, &Ay, &Az, &Wp, &Wq, &Wr);
  //Ax =-acceleration_mg[0]*GRAV*0.001;
  //Ay =-acceleration_mg[1]*GRAV*0.001;
  //Az = acceleration_mg[2]*GRAV*0.001;
  //Wp = angular_rate_mdps[0]*M_PI*5.55555555e-6;//5.5.....e-6=1/180/1000
  //Wq = angular_rate_mdps[1]*M_PI*5.55555555e-6;
  //Wr =-angular_rate_mdps[2]*M_PI*5.55555555e-6;
  //Mx0 =-magnetic_field_mgauss[0];
  //My0 = magnetic_field_mgauss[1];
  //Mz0 =-magnetic_field_mgauss[2];

  
  acc_norm = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  if (acc_norm>250.0) OverG_flag = 1;
  Acc_norm = acc_filter.update(acc_norm);
  rate_norm = sqrt(Wp*Wp + Wq*Wq + Wr*Wr);
  if (rate_norm > 6.0) OverG_flag =1;

}

void variable_init(void)
{
}

void printPQR(void)
{
}

void output_data(void)
{
  printf("%9.3f,"
         "%13.8f,%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%6u,%6u,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f"
         //"%13.8f"
         "\n"
            ,Elapsed_time//1
            ,Quat.q0, Quat.q1, Quat.q2, Quat.q3//2~5 
            ,Quat.q0, Quat.q1, Quat.q2//6~8
            //,Phi-Phi_bias, Theta-Theta_bias, Psi-Psi_bias//6~8
            ,D_time, D_time2//10,11
            ,Ax, Ay, Az//11~13
            ,Wp, Wq, Wr//14~16
            ,Mx, My, Mz//17~19
            //,mag_norm
        ); //20
}
void output_sensor_raw_data(void)
{
  printf("%9.3f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f"
         "\n"
            ,Elapsed_time//1
            ,Ax, Ay, Az//2~4
            ,Wp, Wq, Wr//5~7
            ,Mx, My, Mz//8~10
        ); //20
}

#if 0
void kalman_filter(void)
{
  //Kalman Filter
  float dt=0.01;
  Omega_m << Wp, Wq, Wr;
  Z << Ax, Ay, Az, Mx, My, Mz;
  ekf(Xp, Xe, P, Z, Omega_m, Q, R, G*dt, Beta, dt);
}
#endif

PID::PID()
{
  m_kp=1.0e-8;
  m_ti=1.0e8;
  m_td=0.0;
  m_integral=0.0;
  m_filter_time_constant=0.01;
  m_filter_output=0.0;
  m_err=0.0;
  m_h=0.01;
}

void PID::set_parameter(
    float kp, 
    float ti, 
    float td,
    float filter_time_constant, 
    float h)
{
  m_kp=kp;
  m_ti=ti;
  m_td=td;
  m_filter_time_constant=filter_time_constant;
  m_h=h;
}

void PID::reset(void)
{
  m_integral=0.0;
  m_filter_output=0.0;
  m_err=0.0;
  m_err2=0.0;
  m_err3=0.0;
}

void PID::i_reset(void)
{
  m_integral=0.0;
}
void PID::printGain(void)
{
  printf("#Kp:%8.4f Ti:%8.4f Td:%8.4f Filter T:%8.4f h:%8.4f\n",m_kp,m_ti,m_td,m_filter_time_constant,m_h);
}

float PID::filter(float x)
{
  m_filter_output = m_filter_output * m_filter_time_constant/(m_filter_time_constant + m_h) 
                  + x * m_h/(m_filter_time_constant + m_h);   
  return m_filter_output;
}

float PID::update(float err)
{
  float d;
  m_integral = m_integral + m_h * err;
  if(m_integral> 30000.0)m_integral = 30000.0;
  if(m_integral<-30000.0)m_integral =-30000.0;
  m_filter_output = filter((err-m_err3)/m_h);
  m_err3 = m_err2;
  m_err2 = m_err;
  m_err  = err;
  return m_kp*(err + m_integral/m_ti + m_td * m_filter_output); 
}

Filter::Filter()
{
  m_state = 0.0;
  m_T = 0.0025;
  m_h = 0.0025;
}

void Filter::reset(void)
{
  m_state = 0.0;
}

void Filter::set_parameter(float T, float h)
{
  m_T = T;
  m_h = h;
}

float Filter::update(float u)
{
  m_state = m_state * m_T /(m_T + m_h) + u * m_h/(m_T + m_h);
  m_out = m_state;
  return m_out;
}

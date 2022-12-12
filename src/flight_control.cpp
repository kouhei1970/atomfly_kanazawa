#include <Arduino.h>
#include <M5Atom.h>
#include <INA3221.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <MadgwickAHRS.h>
#include "vl53l0x.h"
#include "Adafruit_Sensor.h"
#include "rc.hpp"
#include "flight_control.hpp"
#include "pid.hpp"


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

volatile float Roll, Pitch, Yaw;  // Stores attitude related variables.
double r_rand = 180 / PI;

Adafruit_BMP280 bme;
Madgwick Drone_ahrs;

float pressure = 0.0;

//Sensor data
volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
float Acc_norm=0.0;
quat_t Quat;
float Over_g=0.0, Over_rate=0.0;

//Times
float Elapsed_time=0.0;
//
volatile uint32_t S_time=0,E_time=0,D_time=0,S_time2=0,E_time2=0,D_time2=0;

//Counter
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t BiasCounter=0;
uint16_t LedBlinkCounter=0;

//Control 
volatile float FR_duty=0.0, FL_duty=0.0, RR_duty=0.0, RL_duty=0.0;
volatile float P_com=0.0, Q_com=0.0, R_com=0.0;
volatile float Phi_com=0.0, Tht_com=0.0, Psi_com=0.0;
volatile float T_ref=0.0;
volatile float Pbias=0.0,Qbias=0.0,Rbias=0.0;
volatile float Phi_bias=0.0,Theta_bias=0.0,Psi_bias=0.0;  
volatile float Phi=0.0,Theta=0.0,Psi=0.0;
volatile float Phi_ref=0.0,Theta_ref=0.0,Psi_ref=0.0;
volatile float Elevator_center=0.0, Aileron_center=0.0, Rudder_center=0.0;
volatile float Pref=0.0,Qref=0.0,Rref=0.0;
volatile float Phi_trim   =  0.0;
volatile float Theta_trim =  0.0;
volatile float Psi_trim   = 0.0;

//Log
uint16_t LogdataCounter=0;
uint8_t Logflag=0;
volatile uint8_t Logoutputflag=0;
float Log_time=0.0;
const uint8_t DATANUM=28; //Log Data Number
const uint32_t LOGDATANUM=DATANUM*700;
float Logdata[LOGDATANUM];

//Machine state
uint8_t Mode = INIT_MODE;
volatile uint8_t LockMode=0;
float Motor_on_duty_threshold = 0.1;
float Angle_control_on_duty_threshold = 0.5;
uint8_t OverG_flag = 0;
volatile uint8_t Loop_flag = 0;
volatile uint8_t Angle_control_flag = 0;
CRGB Led_color = 0x000000;

//PID object and etc.
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
Filter acc_filter;

void test_rangefinder(void);
void init_i2c();
void init_pwm();
void control_init();
void gyro_calibration(void);
void variable_init(void);
void log_output(void);
void m5_atom_led(CRGB p, uint8_t state);
void sensor_read(void);
void get_command(void);
void angle_control(void);
void rate_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void logging(void);
void motor_stop(void);
uint8_t lock_com(void);
uint8_t logdata_out_com(void);
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);

hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer() {
  Loop_flag = 1;
}

void init_atomfly(void)
{

  Mode = INIT_MODE;
  M5.dis.drawpix(0, WHITE);
  init_pwm();
  init_i2c();
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8O1, 26, 32);
  rc_init();
  //while(!rc_isconnected());
  M5.IMU.Init();
  //IMUのデフォルトI2C周波数が100kHzなので400kHzに上書き
  Wire.begin(25,21,400000UL);
  test_rangefinder();
  Drone_ahrs.begin(400.0);
  control_init();
  
  //割り込み設定
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2500, true);
  timerAlarmEnable(timer);
  delay(500);

  //Mode = AVERAGE_MODE;
}

//Main loop
void loop_400Hz(void)
{
  static uint8_t led=1;

  while(Loop_flag==0);
  Loop_flag = 0;

  //Begin Mode select
  if (Mode == INIT_MODE)
  {
      motor_stop();
      Elevator_center = 0.0;
      Aileron_center = 0.0;
      Rudder_center = 0.0;
      Pbias = 0.0;
      Qbias = 0.0;
      Rbias = 0.0;
      Phi_bias = 0.0;
      Theta_bias = 0.0;
      Psi_bias = 0.0;
      Mode = AVERAGE_MODE;
      return;
  }
  else if (Mode == AVERAGE_MODE)
  {
    motor_stop();
    //Gyro Bias Estimate
    if (BiasCounter < AVERAGENUM)
    {
      //Sensor Read
      M5.dis.drawpix(0, PERPLE);
      sensor_read();
      Pbias += Wp;
      Qbias += Wq;
      Rbias += Wr;
      Phi_bias   += Phi;
      Theta_bias += Theta;
      Psi_bias   += Psi;
      BiasCounter++;
      return;
    }
    else if(BiasCounter == AVERAGENUM)
    {
      //Average calc
      Pbias = Pbias/AVERAGENUM;
      Qbias = Qbias/AVERAGENUM;
      Rbias = Rbias/AVERAGENUM;
      Phi_bias   = Phi_bias/AVERAGENUM;
      Theta_bias = Theta_bias/AVERAGENUM;
      Psi_bias   = Psi_bias/AVERAGENUM;

      //Mode change
      Mode = STAY_MODE;
    }
    return;
  }
  else if( Mode == FLIGHT_MODE)
  {
    if(LockMode==2)
    {
      if(lock_com()==1)
      {
        LockMode=3;//Disenable Flight
        led=0;
        m5_atom_led(GREEN,led);
        return;
      }
      //Goto Flight
    }
    else if(LockMode==3)
    {
      if(lock_com()==0){
        LockMode=0;
        Mode=STAY_MODE;
      }
      return;
    }
    //LED Blink
    m5_atom_led(Led_color, led);
    if(Logflag==1&&LedBlinkCounter<100){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      if(Logflag==1)led=!led;
      else led=1;
    }
   
    //Read Sensor Value
    sensor_read();

    //Get command
    get_command();

    //Angle Control
    angle_control();

    //Rate Control
    rate_control();
  }
  else if(Mode == STAY_MODE)
  {
    motor_stop();
    OverG_flag = 0;
    Angle_control_flag = 0;
    if(LedBlinkCounter<10){
      m5_atom_led(GREEN, 1);
      LedBlinkCounter++;
    }
    else if(LedBlinkCounter<100)
    {
      m5_atom_led(GREEN, 0);
      LedBlinkCounter++;
    }
    else LedBlinkCounter=0;
      
    if(LockMode==0)
    {
      if( lock_com()==1)
      {
        LockMode=1;
        return;
      }
      //Wait output log
    }
    else if(LockMode==1)
    {
      if(lock_com()==0)
      {
        LockMode=2;//Enable Flight
        Mode=FLIGHT_MODE;
      }
      return;
    }

    if(logdata_out_com()==1)
    {
      Mode=LOG_MODE;
      return;
    }
  }
  else if(Mode == LOG_MODE)
  {
    motor_stop();
    Logoutputflag=1;
    log_output();
    //LED Blink
    m5_atom_led(BLUE, led);
    if(LedBlinkCounter<10){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
  }
  //End Mode select
  //End of Loop_400Hz function
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
//  PID control gain setting
//
//  Sets the gain of PID control.
//  
//  Function usage
//  PID.set_parameter(PGAIN, IGAIN, DGAIN, TC, STEP)
//
//  PGAIN: PID Proportional Gain
//  IGAIN: PID Integral Gain
//   *The larger the value of integral gain, the smaller the effect of integral control.
//  DGAIN: PID Differential Gain
//  TC:    Time constant for Differential control filter
//  STEP:  Control period
//
//  Example
//  Set roll rate control PID gain
//  p_pid.set_parameter(2.5, 10.0, 0.45, 0.01, 0.001); 

void control_init(void)
{
  //Acceleration filter
  acc_filter.set_parameter(0.005, 0.0025);
  //Rate control
  p_pid.set_parameter(0.8, 0.7, 0.010, 0.002, 0.0025);//Roll rate control gain
  q_pid.set_parameter(0.9, 0.7, 0.006, 0.002, 0.0025);//Pitch rate control gain
  r_pid.set_parameter(3.0, 1.0, 0.000, 0.015, 0.0025);//Yaw rate control gain
  //Angle control
  phi_pid.set_parameter  ( 15.0, 7.0, 0.005, 0.002, 0.0025);//これも中々良い
  theta_pid.set_parameter( 15.0, 7.0, 0.005, 0.002, 0.0025);

  //phi_pid.set_parameter  ( 12.0, 8.0, 0.005, 0.002, 0.0025);//これも中々良い
  //theta_pid.set_parameter( 12.0, 8.0, 0.005, 0.002, 0.0025);

  //phi_pid.set_parameter  ( 6.0, 8.0, 0.005, 0.002, 0.0025);//中々良い
  //theta_pid.set_parameter( 6.0, 8.0, 0.005, 0.002, 0.0025);
  
  //phi_pid.set_parameter  ( 1.0, 1.0, 0.001, 0.002, 0.0025);//Roll angle control gain
  //theta_pid.set_parameter( 1.0, 1.0, 0.001, 0.002, 0.0025);//Pitch angle control gain

  //phi_pid.set_parameter  ( 6.5, 8.5, 0.005, 0.002, 0.0025);
  //theta_pid.set_parameter( 6.5, 8.5, 0.005, 0.002, 0.0025);

  //phi_pid.set_parameter  ( 6.0, 9.0, 0.005, 0.002, 0.0025);//中々良い
  //theta_pid.set_parameter( 6.0, 9.0, 0.005, 0.002, 0.0025);

  //phi_pid.set_parameter  ( 6.0, 12.0, 0.005, 0.002, 0.0025);//NG 左右にすーっと動く
  //theta_pid.set_parameter( 6.0, 12.0, 0.005, 0.002, 0.0025);

  //phi_pid.set_parameter  ( 7.0, 9.5, 0.005, 0.002, 0.0025);
  //theta_pid.set_parameter( 7.0, 9.5, 0.005, 0.002, 0.0025);

  //phi_pid.set_parameter  ( 10.0, 9.5, 0.005, 0.002, 0.0025);
  //theta_pid.set_parameter( 10.0, 9.5, 0.005, 0.002, 0.0025);

  //phi_pid.set_parameter  ( 5.0, 9.5, 0.005, 0.002, 0.0025);
  //theta_pid.set_parameter( 5.0, 9.5, 0.005, 0.002, 0.0025);

  //phi_pid.set_parameter  ( 2.5, 9.5, 0.005, 0.002, 0.0025);
  //theta_pid.set_parameter( 2.5, 9.5, 0.005, 0.002, 0.0025);

  //phi_pid.set_parameter  ( 2.5, 4.5, 0.005, 0.002, 0.0025);
  //theta_pid.set_parameter( 2.5, 4.5, 0.005, 0.002, 0.0025);
 
  //phi_pid.set_parameter  ( 25.0, 0.4, 0.004, 0.002, 0.0025);//Roll angle control gain
  //theta_pid.set_parameter( 22.0, 0.4, 0.004, 0.002, 0.0025);//Pitch angle control gain
  //psi_pid.set_parameter  ( 3.0, 10000, 0.0, 0.030, 0.0025);//Yaw angle control gain
  
  //phi_pid.set_parameter  ( 19.0, 0.2, 0.005, 0.002, 0.0025);//Roll angle control gain
  //theta_pid.set_parameter( 17.0, 0.2, 0.002, 0.002, 0.0025);//Pitch angle control gain
  //psi_pid.set_parameter  ( 3.0, 10000, 0.0, 0.030, 0.0025);//Yaw angle control gain

}
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

void get_command(void)
{
  //Throttle curve conversion　スロットルカーブ補正
  float thlo = Stick[THROTTLE];
  T_ref = (3.27*thlo -5.31*thlo*thlo + 3.04*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  Phi_com = Stick[AILERON];
  Tht_com = Stick[ELEVATOR];
  Psi_com = Stick[RUDDER];
}

void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_ref, q_ref, r_ref;
  float p_err, q_err, r_err;

  //Control main
  if(rc_isconnected())
  {
    if(T_ref/BATTERY_VOLTAGE < Motor_on_duty_threshold)
    {
      Led_color=0xffff00;
      motor_stop();
      p_pid.reset();
      q_pid.reset();
      r_pid.reset();
      Pref=0.0;
      Qref=0.0;
      Rref=0.0;
      Rudder_center   = Psi_com;
    }
    else
    {
      //Yaw control
      Rref   = 0.8 * M_PI * (Psi_com - Rudder_center);

      //Control angle velocity
      p_rate = Wp - Pbias;
      q_rate = Wq - Qbias;
      r_rate = Wr - Rbias;

      //Get reference
      p_ref = Pref;
      q_ref = Qref;
      r_ref = Rref;

      //Error
      p_err = p_ref - p_rate;
      q_err = q_ref - q_rate;
      r_err = r_ref - r_rate;

      //Rate Control PID
      P_com = p_pid.update(p_err);
      Q_com = q_pid.update(q_err);
      R_com = r_pid.update(r_err);

      //Motor Control
      //正規化Duty
      FR_duty = (T_ref +(-P_com +Q_com -R_com)*0.25)/BATTERY_VOLTAGE;
      FL_duty = (T_ref +( P_com +Q_com +R_com)*0.25)/BATTERY_VOLTAGE;
      RR_duty = (T_ref +(-P_com -Q_com +R_com)*0.25)/BATTERY_VOLTAGE;
      RL_duty = (T_ref +( P_com -Q_com -R_com)*0.25)/BATTERY_VOLTAGE;
      
      const float minimum_duty=0.0;
      const float maximum_duty=0.99;

      if (FR_duty < minimum_duty) FR_duty = minimum_duty;
      if (FR_duty > maximum_duty) FR_duty = maximum_duty;

      if (FL_duty < minimum_duty) FL_duty = minimum_duty;
      if (FL_duty > maximum_duty) FL_duty = maximum_duty;

      if (RR_duty < minimum_duty) RR_duty = minimum_duty;
      if (RR_duty > maximum_duty) RR_duty = maximum_duty;

      if (RL_duty < minimum_duty) RL_duty = minimum_duty;
      if (RL_duty > maximum_duty) RL_duty = maximum_duty;

      //Duty set
      if (OverG_flag==0){
        set_duty_fr(FR_duty);
        set_duty_fl(FL_duty);
        set_duty_rr(RR_duty);
        set_duty_rl(RL_duty);      
      }
      else 
      {
        motor_stop();
        OverG_flag=0;
        LockMode = 0;
        Mode = STAY_MODE;
      }
      //Serial.printf("%12.5f %12.5f %12.5f %12.5f\n",FR_duty, FL_duty, RR_duty, RL_duty);
    }
  }
  else{
    motor_stop();
    
  } 
}

void angle_control(void)
{
  float phi_err,theta_err;
  static uint8_t cnt=0;
  static double timeval=0.0;

  //PID Control
  if ((T_ref/BATTERY_VOLTAGE < Angle_control_on_duty_threshold))
  {
    Pref=0.0;
    Qref=0.0;
    phi_err = 0.0;
    theta_err = 0.0;
    phi_pid.reset();
    theta_pid.reset();

    /////////////////////////////////////
    // 以下の処理で、角度制御が有効になった時に
    // 急激な目標値が発生して機体が不安定になるのを防止する
    Aileron_center  = Phi_com;
    Elevator_center = Tht_com;

    Phi_bias   = Phi;
    Theta_bias = Theta;
    /////////////////////////////////////
  
  }
  else
  {
    //Angle_control_flag = 1;
    Led_color = RED;
    //Get Roll and Pitch angle ref 
    Phi_ref   = 0.5 * M_PI * (Phi_com - Aileron_center);
    Theta_ref = 0.5 * M_PI * (Tht_com - Elevator_center);

    //Error
    phi_err   = Phi_ref   - (Phi   - Phi_bias);
    theta_err = Theta_ref - (Theta - Theta_bias);
  
    //PID
    Pref = phi_pid.update(phi_err);
    Qref = theta_pid.update(theta_err);
  }
  #if 0
  //For debug
  Serial.printf("%9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f\r\n", 
    timeval, 
    Pref, Qref, 
    phi_err, theta_err, 
    Aileron_center, Phi_com, 
    Elevator_center, Tht_com);

  timeval += 0.0025;
  #endif

  //Logging(100Hz)
  if (cnt==0)logging();
  cnt++;
  if(cnt==4 )cnt=0;
}

void m5_atom_led(CRGB p, uint8_t state)
{
  if (state ==1) M5.dis.drawpix(0, p);
  else M5.dis.drawpix(0, 0x000000);
  return;
}

void init_i2c()
{
  //Wire.begin(25,21);          // join i2c bus (address optional for master)
  Wire.begin(25,21,400000UL);
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  for (short i = 0; i < 256; i++)
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

void init_pwm(void)
{
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
  Serial.println("VLX53LOX test started.");
  //Serial.println(F("BMP280 test started...\n"));

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
      Serial.printf("VL53L0X is ready. cnt=%d\n",cnt);
  else
      Serial.println("VL53L0X is not ready");

  read_block_data_at(0x14, 12);
  uint16_t acnt = makeuint16(gbuf[7], gbuf[6]);
  uint16_t scnt = makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
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

uint16_t get_distance(void)
{
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

  byte val = 0;
  int cnt  = 0;
  while (cnt < 1000) {  // 1 second waiting time max
      val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
      if (val & 0x01) break;
      cnt++;
  }
  //if (val & 0x01)
  //    Serial.printf("VL53L0X is ready. cnt=%d\n",cnt);
  //else
  //    Serial.println("VL53L0X is not ready");

  read_block_data_at(0x14, 12);
  //uint16_t acnt                  = makeuint16(gbuf[7], gbuf[6]);
  //uint16_t scnt                  = makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
  //byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
  return dist;
}

void set_duty_fr(float duty){ledcWrite(FR_motor, (uint32_t)(255*duty));}
void set_duty_fl(float duty){ledcWrite(FL_motor, (uint32_t)(255*duty));}
void set_duty_rr(float duty){ledcWrite(RR_motor, (uint32_t)(255*duty));}
void set_duty_rl(float duty){ledcWrite(RL_motor, (uint32_t)(255*duty));}

void sensor_read(void)
{
  float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;

  M5.IMU.getAccelData(&ax, &ay, &az);
  M5.IMU.getGyroData(&gx, &gy, &gz);
  ax = ax;
  ay = ay;
  Drone_ahrs.updateIMU(gx-Qbias*RAD_TO_DEG, gy-Pbias*RAD_TO_DEG, gz-Rbias*RAD_TO_DEG, ax, ay, az);

  Ax = ay;
  Ay = ax;
  Az = az;
  Wp = gy*DEG_TO_RAD;
  Wq = gx*DEG_TO_RAD;
  Wr = gz*DEG_TO_RAD;

  Theta = Drone_ahrs.getRoll()*DEG_TO_RAD;
  Phi =   Drone_ahrs.getPitch()*DEG_TO_RAD;
  Psi =   Drone_ahrs.getYaw()*DEG_TO_RAD;
  
  #if 1
  acc_norm = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  if (acc_norm>12.0) 
  {
    OverG_flag = 1;
    if (Over_g == 0.0)Over_g = acc_norm;
  }
  Acc_norm = acc_filter.update(acc_norm);
  rate_norm = sqrt((Wp-Pbias)*(Wp-Pbias) + (Wq-Qbias)*(Wq-Qbias) + (Wr-Rbias)*(Wr-Rbias));
  if (rate_norm > 17.0)
  {
    OverG_flag = 1;
    if (Over_rate == 0.0) Over_rate =rate_norm;
  } 
  #endif

#if 0
  Serial.printf("%7.3f  %6d %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\r\n",
   Elapsed_time, D_time, Ax, Ay, Az, Wp, Wq, Wr, Phi, Theta, Psi);
#endif
}

uint8_t lock_com(void)
{
  static uint8_t chatta=0,state=0;
  if( (int)Stick[BUTTON] == 0 )
  { 
    chatta++;
    if(chatta>20){
      chatta=20;
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
  if( Stick[LOG] == 0 
   && Stick[THROTTLE] == 0 
   && Stick[RUDDER]  > 0.4
   && Stick[AILERON] > 0.4 
   && Stick[ELEVATOR] > 0.4)
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

void logging(void)
{  
  //Logging
  if(Stick[LOG] == 1)
  { 
    if(Logflag==0)
    {
      Logflag=1;
      LogdataCounter=0;
    }
    if(LogdataCounter+DATANUM<LOGDATANUM)
    {
      Logdata[LogdataCounter++]=Wp;//-Pbias;              //1
      Logdata[LogdataCounter++]=Wq;//-Qbias;              //2
      Logdata[LogdataCounter++]=Wr;//-Rbias;              //3
      Logdata[LogdataCounter++]=Ax;                       //4
      Logdata[LogdataCounter++]=Ay;                       //5
      Logdata[LogdataCounter++]=Az;                       //6
      Logdata[LogdataCounter++]=Pref;                     //7
      Logdata[LogdataCounter++]=Qref;                     //8
      Logdata[LogdataCounter++]=Rref;                     //9
      Logdata[LogdataCounter++]=Phi-Phi_bias;             //10

      Logdata[LogdataCounter++]=Theta-Theta_bias;         //11
      Logdata[LogdataCounter++]=Psi-Psi_bias;             //12
      Logdata[LogdataCounter++]=Phi_ref;                  //13
      Logdata[LogdataCounter++]=Theta_ref;                //14
      Logdata[LogdataCounter++]=Psi_ref;                  //15
      Logdata[LogdataCounter++]=P_com;                    //16
      Logdata[LogdataCounter++]=Q_com;                    //17
      Logdata[LogdataCounter++]=R_com;                    //18
      Logdata[LogdataCounter++]=p_pid.m_filter_output;    //19
      Logdata[LogdataCounter++]=q_pid.m_filter_output;    //20

      Logdata[LogdataCounter++]=r_pid.m_filter_output;    //21
      Logdata[LogdataCounter++]=phi_pid.m_filter_output;  //22
      Logdata[LogdataCounter++]=theta_pid.m_filter_output;//23
      Logdata[LogdataCounter++]=Pbias;                    //24
      Logdata[LogdataCounter++]=Qbias;                    //25
      Logdata[LogdataCounter++]=Rbias;                    //26
      Logdata[LogdataCounter++]=T_ref;                    //27
      Logdata[LogdataCounter++]=Acc_norm;                 //28
      
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
    Serial2.printf("#Roll rate PID gain\r\n");
    p_pid.printGain();
    Serial2.printf("#Pitch rate PID gain\r\n");
    q_pid.printGain();
    Serial2.printf("#Yaw rate PID gain\r\n");
    r_pid.printGain();
    Serial2.printf("#Roll angle PID gain\r\n");
    phi_pid.printGain();
    Serial2.printf("#Pitch angle PID gain\r\n");
    theta_pid.printGain();
    Serial2.printf("#Phi_trim:%f Theta_trim:%f\r\n", Phi_trim, Theta_trim);
    Serial2.printf("#Over_g:%f Over_rate:%f\r\n", Over_g, Over_rate);
  }
  if(LogdataCounter+DATANUM<LOGDATANUM)
  {
    //LockMode=0;
    Serial2.printf("%10.2f ", Log_time);
    Log_time=Log_time + 0.01;
    for (uint8_t i=0;i<DATANUM;i++)
    {
      Serial2.printf("%12.5f",Logdata[LogdataCounter+i]);
    }
    Serial2.printf("\r\n");
    LogdataCounter=LogdataCounter + DATANUM;
  }
  else 
  {
    Mode = STAY_MODE;
    Logoutputflag = 0;
    LockMode = 0;
    Log_time = 0.0;
    LogdataCounter = 0;
  }
}

void variable_init(void)
{
}

void output_data(void)
{
  Serial.printf("%9.3f,"
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
  Serial.printf("%9.3f,"
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


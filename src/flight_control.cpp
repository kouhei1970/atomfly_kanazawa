//log214
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

//Control period
const float Control_period = 0.0025f;//400Hz

//PID Gain
//Rate control PID gain
const float P_kp = 0.8f;
const float P_ti = 0.7f;
const float P_td = 0.03f;
const float P_eta = 0.125f;

const float Q_kp = 0.8f;
const float Q_ti = 0.7f;
const float Q_td = 0.03f;
const float Q_eta = 0.125f;

const float R_kp = 3.0f;
const float R_ti = 5.0f;
const float R_td = 0.0f;
const float R_eta = 0.125f;

//Angle control PID gain
const float Phi_kp = 12.0f;
const float Phi_ti = 1000.0f;
const float Phi_td = 0.04f;
const float Phi_eta = 0.125f;

const float Tht_kp = 17.0f;
const float Tht_ti = 1000.0f;
const float Tht_td = 0.04f;
const float Tht_eta = 0.125f;

//volatile float Roll, Pitch, Yaw;  // Stores attitude related variables.
float r_rand = 180 / PI;

//Adafruit_BMP280 bme;
Madgwick Drone_ahrs;

// Set I2C address to 0x41 (A0 pin -> VCC)
INA3221 ina3221(INA3221_ADDR40_GND);

float pressure = 0.0f;

//Sensor data
volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
volatile float Voltage;
float Acc_norm=0.0f;
quat_t Quat;
float Over_g=0.0f, Over_rate=0.0f;

//Times
volatile float Elapsed_time=0.0f;
volatile float Old_Elapsed_time=0.0f;

//
volatile uint32_t S_time=0,E_time=0,D_time=0,S_time2=0,E_time2=0,Dt_time=0;

//Counter
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t BiasCounter=0;
uint16_t LedBlinkCounter=0;

//Control 
volatile float FR_duty=0.0f, FL_duty=0.0f, RR_duty=0.0f, RL_duty=0.0f;
volatile float P_com=0.0f, Q_com=0.0f, R_com=0.0f;
volatile float Phi_com=0.0f, Tht_com=0.0f, Psi_com=0.0f;
volatile float T_ref=0.0f;
volatile float Pbias=0.0f, Qbias=0.0f, Rbias=0.0f;
volatile float Phi_bias=0.0f, Theta_bias=0.0f, Psi_bias=0.0f;  
volatile float Phi=0.0f, Theta=0.0f, Psi=0.0f;
volatile float Phi_ref=0.0f, Theta_ref=0.0f, Psi_ref=0.0f;
volatile float Elevator_center=0.0f, Aileron_center=0.0f, Rudder_center=0.0f;
volatile float Pref=0.0f, Qref=0.0f, Rref=0.0f;
volatile float Phi_trim   =  0.8f*M_PI/180.0f;
volatile float Theta_trim = -0.2f*M_PI/180.0f;
volatile float Psi_trim   =  0.0f;

//Log
uint8_t Logflag=0;
uint8_t Telem_cnt = 0;
//uint16_t LogdataCounter=0;
//volatile uint8_t Logoutputflag=0;
//float Log_time=0.0f;
//const uint8_t DATANUM=28; //Log Data Number
//const uint32_t LOGDATANUM=DATANUM*700;
//float Logdata[LOGDATANUM];

//Machine state
float Timevalue=0.0f;
uint8_t Mode = INIT_MODE;
uint8_t Control_mode = ANGLECONTROL;
volatile uint8_t LockMode=0;
float Motor_on_duty_threshold = 0.1f;
float Angle_control_on_duty_threshold = 0.5f;
uint8_t OverG_flag = 0;
uint8_t Flip_flag = 0;
uint16_t Flip_counter = 0; 
float Flip_time = 2.0;
volatile uint8_t Loop_flag = 0;
volatile uint8_t Angle_control_flag = 0;
volatile uint8_t Power_flag = 0;
CRGB Led_color = 0x000000;

//PID object and etc.
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
Filter acc_filter;
Filter voltage_filter;

void test_rangefinder(void);
uint8_t init_i2c();
void init_pwm();
void control_init();
void gyro_calibration(void);
void variable_init(void);
void m5_atom_led(CRGB p, uint8_t state);
void sensor_read(void);
void get_command(void);
void angle_control(void);
void rate_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void motor_stop(void);
uint8_t lock_com(void);
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);

void telemetry(void);
void float2byte(float x, uint8_t* dst);
void append_data(uint8_t* data , uint8_t* newdata, uint8_t index, uint8_t len);
void imu_init(void);
uint8_t mpu6886_byte_read(uint8_t reg_addr);
void mpu6886_byte_write(uint8_t reg_addr, uint8_t data);

hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer() 
{
  Loop_flag = 1;
}

void init_atomfly(void)
{

  Mode = INIT_MODE;
  M5.dis.drawpix(0, WHITE);
  init_pwm();
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8O1, 26, 32);
  rc_init();
  if(init_i2c()==0)
  {
    Serial.printf("No I2C device!\r\n");
    Serial.printf("Can not boot AtomFly2.\r\n");
    while(1);
  }
  //while(!rc_isconnected());
  
  imu_init();
  //test_rangefinder();
  Drone_ahrs.begin(400.0);
  ina3221.begin();
  ina3221.reset();  
  control_init();
  voltage_filter.set_parameter(0.005, 0.0025);


  //割り込み設定
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2500, true);
  timerAlarmEnable(timer);
  delay(500);

  //Mode = AVERAGE_MODE;
}

uint8_t mpu6886_byte_read(uint8_t reg_addr)
{
  uint8_t data;
  Wire.beginTransmission (MPU6886_ADDRESS);
  Wire.write(reg_addr);
  Wire.endTransmission();
  Wire.requestFrom(MPU6886_ADDRESS, 1);
  data = Wire.read();
  return data;
}

void mpu6886_byte_write(uint8_t reg_addr, uint8_t data)
{
  Wire.beginTransmission (MPU6886_ADDRESS);
  Wire.write(reg_addr);
  Wire.write(data);
  Wire.endTransmission();
}


void imu_init(void)
{
  //Cutoff frequency
  //filter_config Gyro Accel
  //0 250    218.1 log140　Bad
  //1 176    218.1 log141　Bad
  //2 92     99.0  log142 Bad これはヨーガカクカクする
  //3 41     44.8  log143 log188　Good!
  //4 20     21.2
  //5 10     10.2
  //6 5      5.1
  //7 3281   420.0
  uint8_t data;
  const uint8_t filter_config = 4;//今の所2はノイズが多くてダメ、log188は3

  //Mdgwick filter 実験
  // filter_config=0において実施
  //beta =0 次第に角度増大（角速度の積分のみに相当する）
  //beta=0.5

  M5.IMU.Init();
  //IMUのデフォルトI2C周波数が100kHzなので400kHzに上書き
  Wire.begin(25,21,400000UL);

 //F_CHOICE_B
  data = mpu6886_byte_read(MPU6886_GYRO_CONFIG);
  Serial.printf("GYRO_CONFIG %d\r\n", data);
  mpu6886_byte_write(MPU6886_GYRO_CONFIG, data & 0b11111100);
  data = mpu6886_byte_read(MPU6886_GYRO_CONFIG);
  Serial.printf("Update GYRO_CONFIG %d\r\n", data);

  //Gyro
  //DLPG_CFG
  data = mpu6886_byte_read(MPU6886_CONFIG);
  Serial.printf("CONFIG %d\r\n", data);
  mpu6886_byte_write(MPU6886_CONFIG, (data&0b11111100)|filter_config);
  data = mpu6886_byte_read(MPU6886_CONFIG);
  Serial.printf("Update CONFIG %d\r\n", data);

  //Accel
  //ACCEL_FCHOCE_B & A_DLPF_CFG
  data = mpu6886_byte_read(MPU6886_ACCEL_CONFIG2);
  Serial.printf("ACCEL_CONFIG2 %d\r\n", data);
  mpu6886_byte_write(MPU6886_ACCEL_CONFIG2, (data & 0b11110111) | filter_config);
  data = mpu6886_byte_read(MPU6886_ACCEL_CONFIG2);
  Serial.printf("Update ACCEL_CONFIG2 %d\r\n", data);

}

//Main loop
void loop_400Hz(void)
{
  static uint8_t led=1;

  while(Loop_flag==0);
  E_time = micros();
  Old_Elapsed_time = Elapsed_time;
  Elapsed_time = 1e-6*(E_time - S_time);
  Loop_flag = 0;
  Timevalue+=0.0025f;

  //Read Sensor Value
  sensor_read();


  //Begin Mode select
  if (Mode == INIT_MODE)
  {
      motor_stop();
      Elevator_center = 0.0f;
      Aileron_center = 0.0f;
      Rudder_center = 0.0f;
      Pbias = 0.0f;
      Qbias = 0.0f;
      Rbias = 0.0f;
      Phi_bias = 0.0f;
      Theta_bias = 0.0f;
      Psi_bias = 0.0f;
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
      //sensor_read();
      Pbias += Wp;
      Qbias += Wq;
      Rbias += Wr;
      BiasCounter++;
      return;
    }
    else if(BiasCounter == AVERAGENUM)
    {
      //Average calc
      Pbias = Pbias/AVERAGENUM;
      Qbias = Qbias/AVERAGENUM;
      Rbias = Rbias/AVERAGENUM;

      //Mode change
      Mode = STAY_MODE;
      S_time = micros();
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
        if(Power_flag==0)m5_atom_led(GREEN,led);
        else m5_atom_led(POWEROFFCOLOR,led);
        //if( (Elapsed_time - Old_Elapsed_time)>0.00251) m5_atom_led(0xffffff,led);
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
    if (Power_flag == 0) m5_atom_led(Led_color, led);
    else m5_atom_led(POWEROFFCOLOR,led);
    //if( (Elapsed_time - Old_Elapsed_time)>0.00251) m5_atom_led(0xffffff,led);
    if(Logflag==1&&LedBlinkCounter<100){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      if(Logflag==1)led=!led;
      else led=1;
    }
   
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
      if (Power_flag == 0) m5_atom_led(GREEN, 1);
      else m5_atom_led(POWEROFFCOLOR,1);
      LedBlinkCounter++;
    }
    else if(LedBlinkCounter<100)
    {
      if (Power_flag == 0) m5_atom_led(GREEN, 0);
      else m5_atom_led(POWEROFFCOLOR,0);
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
  }

  //Telemetry
  if (Telem_cnt == 0)telemetry();
  Telem_cnt++;
  if (Telem_cnt>10-1)Telem_cnt = 0;

  D_time = micros();
  if(Telem_cnt == 1)Dt_time = D_time - E_time;
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
  p_pid.set_parameter(P_kp, P_ti, P_td, P_eta, Control_period);//Roll rate control gain
  q_pid.set_parameter(Q_kp, Q_ti, Q_td, Q_eta, Control_period);//Pitch rate control gain
  r_pid.set_parameter(R_kp, R_ti, R_td, R_eta, Control_period);//Yaw rate control gain
  //Roll P gain を挙げてみて分散が減るかどうか考える
  //Roll Ti を大きくしてみる

  //Angle control
  phi_pid.set_parameter  (Phi_kp, Phi_ti, Phi_td, Phi_eta, Control_period);//Roll angle control gain
  theta_pid.set_parameter(Tht_kp, Tht_ti, Tht_td, Tht_eta, Control_period);//Pitch angle control gain

  //phi_pid.set_parameter  ( 10.0f, 7.0f, 0.005f, 0.002f, 0.0025f);//振動
  //theta_pid.set_parameter( 10.0f, 7.0f, 0.005f, 0.002f, 0.0025f);

  //phi_pid.set_parameter  ( 12.0, 8.0, 0.005, 0.002, 0.0025);//これも中々良い
  //theta_pid.set_parameter( 12.0, 8.0, 0.005, 0.002, 0.0025);

  //phi_pid.set_parameter  ( 6.0, 8.0, 0.005, 0.002, 0.0025);//中々良い
  //theta_pid.set_parameter( 6.0, 8.0, 0.005, 0.002, 0.0025);
  
  //phi_pid.set_parameter  ( 6.0, 12.0, 0.005, 0.002, 0.0025);//NG 左右にすーっと動く
  //theta_pid.set_parameter( 6.0, 12.0, 0.005, 0.002, 0.0025);

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
  Control_mode = Stick[CONTROLMODE];

  //Throttle curve conversion　スロットルカーブ補正
  float thlo = Stick[THROTTLE];
  if (thlo>1.0f) thlo = 1.0f;
  if (thlo<0.0f) thlo = 0.0f;
  //T_ref = (3.27f*thlo -5.31f*thlo*thlo + 3.04f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  //T_ref = (2.92f*thlo -4.90f*thlo*thlo + 2.88f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  //T_ref = (3.01f*thlo -5.20f*thlo*thlo + 3.14f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  //T_ref = (3.46f*thlo -5.74f*thlo*thlo + 3.23f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  //T_ref = (3.42f*thlo -6.00f*thlo*thlo + 3.58f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
  T_ref = (3.32f*thlo -5.40f*thlo*thlo + 3.03f*thlo*thlo*thlo)*BATTERY_VOLTAGE;


  Phi_com = 0.4*Stick[AILERON];
  if (Phi_com<-1.0f)Phi_com = -1.0f;
  if (Phi_com> 1.0f)Phi_com =  1.0f;  
  Tht_com = 0.4*Stick[ELEVATOR];
  if (Tht_com<-1.0f)Tht_com = -1.0f;
  if (Tht_com> 1.0f)Tht_com =  1.0f;  
  Psi_com = Stick[RUDDER];
  if (Psi_com<-1.0f)Psi_com = -1.0f;
  if (Psi_com> 1.0f)Psi_com =  1.0f;  
  //Yaw control
  Rref   = 0.8f * M_PI * (Psi_com - Rudder_center);

  if (Control_mode == RATECONTROL)
  {
    Pref = 240*PI/180*Phi_com;
    Qref = 240*PI/180*Tht_com;
  }

  // A button
  if (Stick[BUTTON_A]==1)
  {
    Flip_counter++;
    if (Flip_counter>20)Flip_counter=20;
  }
  else if (Flip_counter == 20)
  {
    if (Flip_flag == 0)Flip_flag =1;
    else Flip_flag = 0;
    Flip_counter = 0;
  }
  else if (Flip_counter<20)
  {
    Flip_counter = 0;
  }

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
      if(Control_mode == ANGLECONTROL)
      {
        if(Flip_flag==0)Led_color=0xffff00;
        else Led_color = 0xFF9933;
      }
      else Led_color = 0xDC669B;
      FR_duty = 0.0;
      FL_duty = 0.0;
      RR_duty = 0.0;
      RL_duty = 0.0;
      motor_stop();
      p_pid.reset();
      q_pid.reset();
      r_pid.reset();
      Pref=0.0f;
      Qref=0.0f;
      Rref=0.0f;
      Rudder_center   = Psi_com;
    }
    else
    {

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
      FR_duty = (T_ref +(-P_com +Q_com -R_com)*0.25f)/BATTERY_VOLTAGE;
      FL_duty = (T_ref +( P_com +Q_com +R_com)*0.25f)/BATTERY_VOLTAGE;
      RR_duty = (T_ref +(-P_com -Q_com +R_com)*0.25f)/BATTERY_VOLTAGE;
      RL_duty = (T_ref +( P_com -Q_com -R_com)*0.25f)/BATTERY_VOLTAGE;
      
      const float minimum_duty=0.0f;
      const float maximum_duty=0.95f;

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
        FR_duty = 0.0;
        FL_duty = 0.0;
        RR_duty = 0.0;
        RL_duty = 0.0;
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
  static float timeval=0.0f;

  if (Control_mode == RATECONTROL) return;

  //PID Control
  if ((T_ref/BATTERY_VOLTAGE < Motor_on_duty_threshold))//Angle_control_on_duty_threshold))
  {
    Pref=0.0f;
    Qref=0.0f;
    phi_err = 0.0f;
    theta_err = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Phi_ref);
    theta_pid.set_error(Theta_ref);
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
    
    //Flip
    if ( Flip_flag == 1 )
    { 
      Led_color = 0xFF9933;
      #if 0
      if (Flip_flag == 0)Flip_flag = 1;
      if (Flip_counter >400)
      {
        Flip_flag = 0;
        Flip_counter = 0;
      }
      Pref = 2.0*PI;
      Qref = 0.0;
      Flip_counter++;

      #if 0
      Flip_flag = 1;

      //PID Reset
      phi_pid.reset();
      theta_pid.reset();
    
      Flip_time = 2.0;
      Pref = 2.0*PI/Flip_time;
      Qref = 0.0;
      if (Flip_counter> (Flip_time/0.0025) )
      {
        Flip_flag = 0;
        Flip_counter = 0;
      }
      Flip_counter++;
      #endif
    #endif

      //Get Roll and Pitch angle ref 
      Phi_ref   = 0.0f;//
      Theta_ref = 0.0f;//

      //Error
      phi_err   = Phi_ref   - (Phi   - Phi_bias);
      theta_err = Theta_ref - (Theta - Theta_bias);
    
      //PID
      Pref = phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);

    }
    
    //Angle Control
    else
    {
      Led_color = RED;
      //Get Roll and Pitch angle ref 
      Phi_ref   = 0.5f * M_PI * (Phi_com - Aileron_center);
      Theta_ref = 0.5f * M_PI * (Tht_com - Elevator_center);
      if (Phi_ref > (30.0f*M_PI/180.0f) ) Phi_ref = 30.0f*M_PI/180.0f;
      if (Phi_ref <-(30.0f*M_PI/180.0f) ) Phi_ref =-30.0f*M_PI/180.0f;
      if (Theta_ref > (30.0f*M_PI/180.0f) ) Theta_ref = 30.0f*M_PI/180.0f;
      if (Theta_ref <-(30.0f*M_PI/180.0f) ) Theta_ref =-30.0f*M_PI/180.0f;

      //Error
      phi_err   = Phi_ref   - (Phi   - Phi_bias );
      theta_err = Theta_ref - (Theta - Theta_bias);
    
      //PID
      Pref = phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);
    }
    
  }
}

void m5_atom_led(CRGB p, uint8_t state)
{
  if (state ==1) M5.dis.drawpix(0, p);
  else M5.dis.drawpix(0, 0x000000);
  return;
}

uint8_t init_i2c()
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
  return count;
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
  float filterd_v;

  M5.IMU.getAccelData(&ax, &ay, &az);
  M5.IMU.getGyroData(&gx, &gy, &gz);
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
  Wp = gy*(float)DEG_TO_RAD;
  Wq = gx*(float)DEG_TO_RAD;
  Wr = gz*(float)DEG_TO_RAD;

  #if 1
  acc_norm = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  Acc_norm = acc_filter.update(acc_norm);
  if (Acc_norm>7.5) 
  {
    OverG_flag = 1;
    if (Over_g == 0.0)Over_g = acc_norm;
  }
  
  rate_norm = sqrt((Wp-Pbias)*(Wp-Pbias) + (Wq-Qbias)*(Wq-Qbias) + (Wr-Rbias)*(Wr-Rbias));
  if (rate_norm > 800.0)
  {
    OverG_flag = 0;
    if (Over_rate == 0.0) Over_rate =rate_norm;
  } 
  #endif

  Voltage = ina3221.getVoltage(INA3221_CH2);
  filterd_v = voltage_filter.update(Voltage);

  if(Power_flag != POWER_FLG_MAX){
    if (filterd_v < POWER_LIMIT) Power_flag ++;
    else Power_flag = 0;
    if ( Power_flag > POWER_FLG_MAX) Power_flag = POWER_FLG_MAX;
  }


  #if 0
  if(Stick[BUTTON_A]==1)
  Serial.printf("%9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f \r\n", 
    Elapsed_time, Elapsed_time - Old_Elapsed_time ,v1, v2, v3, 
    (Phi-Phi_bias)*180/PI, (Theta-Theta_bias)*180/PI, (Psi-Psi_bias)*180/PI);
  #endif
}

void telemetry(void)
{
  //Telemetry
  float d_float;
  uint8_t d_int[4];
  uint8_t senddata[92]; 
  uint8_t index=0;  

  if(Mode > AVERAGE_MODE)
  {
    //1 Time
    d_float = Elapsed_time;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //2 delta Time
    d_float = 1e-6*Dt_time;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //3 Phi
    d_float = (Phi-Phi_bias)*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //4 Theta
    d_float = (Theta-Theta_bias)*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //5 Psi
    d_float = (Psi-Psi_bias)*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //6 P
    d_float = (Wp-Pbias)*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //7 Q
    d_float = (Wq-Qbias)*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //8 R
    d_float = (Wr-Rbias)*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //9 Phi_ref
    d_float = Phi_ref*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //10 Theta_ref
    d_float = Theta_ref*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //11 P ref
    d_float = Pref*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //12 Q ref
    d_float = Qref*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //13 R ref
    d_float = Rref*180/PI;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //14 T ref
    d_float = T_ref/BATTERY_VOLTAGE;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //15 Voltage
    d_float = Voltage;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //16 Ax
    d_float = Ax;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //17 Ay
    d_float = Ay;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //18 Az
    d_float = Az;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //19 Acc Norm
    d_float = Acc_norm;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //20 FR_duty
    d_float = FR_duty;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //21 FL_duty
    d_float = FL_duty;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //22 RR_duty
    d_float = RR_duty;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;
    //23 RL_duty
    d_float = RL_duty;
    float2byte(d_float, d_int);
    append_data(senddata, d_int, index, 4);
    index = index + 4;

    //Send !
    telemetry_send(senddata, sizeof(senddata));
  }
}


void float2byte(float x, uint8_t* dst)
{
  uint8_t* dummy;
  dummy = (uint8_t*)&x;
  dst[0]=dummy[0];
  dst[1]=dummy[1];
  dst[2]=dummy[2];
  dst[3]=dummy[3];
}

void append_data(uint8_t* data , uint8_t* newdata, uint8_t index, uint8_t len)
{
  for(uint8_t i=index;i<index+len;i++)
  {
    data[i]=newdata[i-index];
  }
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

void motor_stop(void)
{
  set_duty_fr(0.0);
  set_duty_fl(0.0);
  set_duty_rr(0.0);
  set_duty_rl(0.0);
}


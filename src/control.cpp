#include <Arduino.h>
#include <M5Atom.h>
#include "vl53l0x.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include <math.h>
#include <MadgwickAHRS.h>
#include "rc.hpp"
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

float Roll, Pitch, Yaw;  // Stores attitude related variables.
double r_rand = 180 / PI;

Adafruit_BMP280 bme;
Madgwick Drone_ahrs;

float pressure = 0.0;

//Sensor data
float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
float Acc_norm=0.0;
quat_t Quat;

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
float FR_duty, FL_duty, RR_duty, RL_duty;
float P_com, Q_com, R_com;
float T_ref;
float Pbias=0.0,Qbias=0.0,Rbias=0.0;
float Phi_bias=0.0,Theta_bias=0.0,Psi_bias=0.0;  
float Phi,Theta,Psi;
float Phi_ref=0.0,Theta_ref=0.0,Psi_ref=0.0;
float Elevator_center=0.0, Aileron_center=0.0, Rudder_center=0.0;
float Pref=0.0,Qref=0.0,Rref=0.0;
float Phi_trim   = 0.0;
float Theta_trim = 0.0;
float Psi_trim   = 0.0;

//Log
uint16_t LogdataCounter=0;
uint8_t Logflag=0;
volatile uint8_t Logoutputflag=0;
float Log_time=0.0;
const uint8_t DATANUM=28; //Log Data Number
const uint32_t LOGDATANUM=DATANUM*700;
float Logdata[LOGDATANUM];

//Machine state
volatile uint8_t LockMode=0;
float Disable_duty =0.10;
float Flight_duty  =0.18;//0.2/////////////////
uint8_t OverG_flag = 0;
uint8_t Arm_flag = 0;
volatile uint8_t Loop_flag = 0;

//PID object and etc.
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
Filter acc_filter;

void loop_400Hz(void);
void rate_control(void);
void sensor_read(void);
void angle_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void logging(void);
void motor_stop(void);
uint8_t lock_com(void);
uint8_t logdata_out_com(void);
void printPQR(void);
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);

hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer() {
  Loop_flag = 1;
}

void gpio_put(CRGB p, uint8_t state)
{
  if (state ==1) M5.dis.drawpix(0, p);
  else M5.dis.drawpix(0, 0x000000);
  return;
}

void init_atomfly(void)
{
  init_i2c();
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8O1, 26, 32);
  rc_init();
  M5.IMU.Init();
  Wire.begin(25,21,400000UL);
  Serial.println("VLX53LOX test started.");
  Serial.println(F("BMP280 test started...\n"));
  M5.dis.drawpix(0, BLUE);
  test_rangefinder();
  init_pwm();
  Drone_ahrs.begin(400.0);
  control_init();

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2500, true);
  timerAlarmEnable(timer);
  
  delay(500);
  Arm_flag = 1;
}

void init_i2c()
{
  Wire.begin(25,21);          // join i2c bus (address optional for master)
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
  return cnt;
}

void set_duty_fr(float duty){ledcWrite(FR_motor, (uint32_t)(255*duty));}
void set_duty_fl(float duty){ledcWrite(FL_motor, (uint32_t)(255*duty));}
void set_duty_rr(float duty){ledcWrite(RR_motor, (uint32_t)(255*duty));}
void set_duty_rl(float duty){ledcWrite(RL_motor, (uint32_t)(255*duty));}
void imu_mag_data_read(float* ax, float* ay, float* az, float* gx, float* gy, float* gz){}

void sensor_read(void)
{
  float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;

  M5.IMU.getAccelData(&ax, &ay, &az);
  M5.IMU.getGyroData(&gx, &gy, &gz);
  Drone_ahrs.updateIMU(gx-Qbias*RAD_TO_DEG, gy-Pbias*RAD_TO_DEG, gz-Rbias*RAD_TO_DEG, ax, ay, az);

  Ax = ay;
  Ay = ax;
  Az = az;
  Wp = gy*DEG_TO_RAD;
  Wq = gx*DEG_TO_RAD;
  Wr = gz*DEG_TO_RAD;

  Theta = Drone_ahrs.getRoll()*DEG_TO_RAD;
  Phi = Drone_ahrs.getPitch()*DEG_TO_RAD;
  Psi = Drone_ahrs.getYaw()*DEG_TO_RAD;
  
  acc_norm = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  if (acc_norm>25.0) OverG_flag = 1;
  Acc_norm = acc_filter.update(acc_norm);
  rate_norm = sqrt((Wp-Pbias)*(Wp-Pbias) + (Wq-Qbias)*(Wq-Qbias) + (Wr-Rbias)*(Wr-Rbias));
  if (rate_norm > 50.0) OverG_flag =1;

#if 0
  Serial.printf("%7.3f  %6d %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\r\n",
   Elapsed_time, D_time, Ax, Ay, Az, Wp, Wq, Wr, Phi, Theta, Psi);
#endif
}




//Main loop
//This function is called from PWM Intrupt on 400Hz.
void loop_400Hz(void)
{

  static uint8_t led=1;

  while(Loop_flag==0);
  Loop_flag = 0;

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
      Aileron_center  += Stick[AILERON];
      Elevator_center += Stick[ELEVATOR];
      Rudder_center   += Stick[RUDDER];
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
      }

      //AngleControl control      

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
        gpio_put(GREEN,led);
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
    gpio_put(RED, led);
    if(Logflag==1&&LedBlinkCounter<100){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      if(Logflag==1)led=!led;
      else led=1;
    }
   
    //Angle Control
    angle_control();

    //Rate Control
    rate_control();
  }
  else if(Arm_flag==3)
  {
    motor_stop();
    OverG_flag = 0;
    if(LedBlinkCounter<10){
      gpio_put(GREEN, 1);
      LedBlinkCounter++;
    }
    else if(LedBlinkCounter<100)
    {
      gpio_put(GREEN, 0);
      LedBlinkCounter++;
    }
    else LedBlinkCounter=0;
    
    //Get Stick Center 
    Aileron_center  = Stick[AILERON];
    Elevator_center = Stick[ELEVATOR];
    Rudder_center   = Stick[RUDDER];
  
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
    log_output();
    //LED Blink
    gpio_put(BLUE, led);
    if(LedBlinkCounter<10){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
  }
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
  p_pid.set_parameter(1.0, 10000.0, 0.0, 0.015, 0.0025);//Roll rate control gain
  q_pid.set_parameter(1.0, 10000.0, 0.0, 0.015, 0.0025);//Pitch rate control gain
  r_pid.set_parameter(3.0, 10000.0, 0.0, 0.015, 0.0025);//Yaw rate control gain
  //Angle control
  phi_pid.set_parameter  ( 1.0, 10000, 0.0, 0.018, 0.0025);//Roll angle control gain
  theta_pid.set_parameter( 1.0, 10000, 0.0, 0.018, 0.0025);//Pitch angle control gain
  psi_pid.set_parameter  ( 3.0, 10000, 0.0, 0.030, 0.0025);//Yaw angle control gain
}
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

uint8_t lock_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Stick[LOG] == 0 
   && Stick[RUDDER]   < -0.4
   && Stick[AILERON]  < -0.4 
   && Stick[ELEVATOR] > 0.4 )
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


#define RUDDER 0
#define ELEVATOR 1
#define THROTTLE 2
#define AILERON 3
#define LOG 4


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

void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_ref, q_ref, r_ref;
  float p_err, q_err, r_err;

  //Read Sensor Value
  sensor_read();

  //Control angle velocity
  p_rate = Wp - Pbias;
  q_rate = Wq - Qbias;
  r_rate = Wr - Rbias;

  //Get reference
  p_ref = Pref;
  q_ref = Qref;
  r_ref = Rref;
  //調整値0.8*電池電圧公称値*正規化スロットル値
  T_ref = 0.8 * BATTERY_VOLTAGE*Stick[THROTTLE];

  //Error
  p_err = p_ref - p_rate;
  q_err = q_ref - q_rate;
  r_err = r_ref - r_rate;

  //PID
  P_com = p_pid.update(p_err);
  Q_com = q_pid.update(q_err);
  R_com = r_pid.update(r_err);

  //Trim
  Phi_trim = Phi_trim + Stick[DPAD_RIGHT]*0.00001 - Stick[DPAD_LEFT]*0.00001;
  Theta_trim = Theta_trim - Stick[DPAD_UP]*0.00001 + Stick[DPAD_DOWN]*0.00001; 

  //Motor Control
  //正規化Duty
  FR_duty = (T_ref +(-P_com +Q_com -R_com)*0.25)/BATTERY_VOLTAGE-Phi_trim+Theta_trim;
  FL_duty = (T_ref +( P_com +Q_com +R_com)*0.25)/BATTERY_VOLTAGE+Phi_trim+Theta_trim;
  RR_duty = (T_ref +(-P_com -Q_com +R_com)*0.25)/BATTERY_VOLTAGE-Phi_trim-Theta_trim;
  RL_duty = (T_ref +( P_com -Q_com -R_com)*0.25)/BATTERY_VOLTAGE+Phi_trim-Theta_trim;
  
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
    Aileron_center  = Stick[AILERON];
    Elevator_center = Stick[ELEVATOR];
    Rudder_center   = Stick[RUDDER];
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
    //Serial.printf("%12.5f %12.5f %12.5f %12.5f\n",FR_duty, FL_duty, RR_duty, RL_duty);
  } 
}

void angle_control(void)
{
  float phi_err,theta_err,psi_err;
  float q0,q1,q2,q3;
  float e23,e33,e13,e11,e12;
  static uint8_t cnt=0;

  if (true)
  {
    //Get angle ref 
    Phi_ref   = 0.6 * M_PI *Stick[AILERON];
    Theta_ref = 0.6 * M_PI *Stick[ELEVATOR];
    Psi_ref   = 0.8 * M_PI *Stick[RUDDER];

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
      Aileron_center  = Stick[AILERON];
      Elevator_center = Stick[ELEVATOR];
      Rudder_center   = Stick[RUDDER];
      /////////////////////////////////////
      Phi_bias   = Phi;
      Theta_bias = Theta;
      Psi_bias   = Psi;
      /////////////////////////////////////
      //Serial.println("Disenable angle  control");
    }
    else
    {
      Pref = phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);
      Rref = Psi_ref;//psi_pid.update(psi_err);//Yawは角度制御しない
      //Serial.println("Enable angle  control");

    }

    //Logging
    if (cnt==0)logging();
    cnt++;
    if(cnt==4 )cnt=0;
  }
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
      Logdata[LogdataCounter++]=Phi;                      //10

      Logdata[LogdataCounter++]=Theta;                    //11
      Logdata[LogdataCounter++]=Psi;                      //12
      Logdata[LogdataCounter++]=Phi_ref;                  //13
      Logdata[LogdataCounter++]=Theta_ref;                //14
      Logdata[LogdataCounter++]=Psi_ref;                  //15
      Logdata[LogdataCounter++]=P_com;                    //16
      Logdata[LogdataCounter++]=Q_com;                    //17
      Logdata[LogdataCounter++]=R_com;                    //18
      Logdata[LogdataCounter++]=p_pid.m_integral;//m_filter_output;    //19
      Logdata[LogdataCounter++]=q_pid.m_integral;//m_filter_output;    //20

      Logdata[LogdataCounter++]=r_pid.m_integral;//m_filter_output;    //21
      Logdata[LogdataCounter++]=phi_pid.m_integral;//m_filter_output;  //22
      Logdata[LogdataCounter++]=theta_pid.m_integral;//m_filter_output;//23
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
  uint16_t N=1000;
  for(uint16_t i=0;i<N;i++)
  {
    sensor_read();
    sump=sump+Wp;
    sumq=sumq+Wq;
    sumr=sumr+Wr;
    delay(1);
  } 
  Pbias=sump/N;
  Qbias=sumq/N;
  Rbias=sumr/N;
}

void variable_init(void)
{
}

void printPQR(void)
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
  Serial.printf("#Kp:%8.4f Ti:%8.4f Td:%8.4f Filter T:%8.4f h:%8.4f\n",m_kp,m_ti,m_td,m_filter_time_constant,m_h);
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

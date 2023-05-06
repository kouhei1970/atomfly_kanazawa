#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "flight_control.hpp"
#include "pid.hpp"
#include <INA3221.h>
#include <M5Atom.h>
#include "MadgwickAHRS.h"
#include "vl53l0x.hpp"
#include <stdint.h>

#define I2C_SCL 21
#define I2C_SDA 25


typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
} quat_t;

typedef struct
{
  uint16_t distance;
  uint16_t cnt;  
} distance_t;

//Sensor data
extern volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
extern volatile float Phi, Theta, Psi;
extern volatile float Altitude;
extern volatile float Voltage;
extern float Acc_norm;
extern quat_t Quat;
extern float Over_g, Over_rate;
extern uint8_t OverG_flag;
extern volatile float Pbias, Qbias, Rbias;
extern volatile uint8_t Power_flag;

void sensor_init(void);
void sensor_read(void);
void ahrs_reset(void);

#endif
#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <M5AtomS3.h>
#include "flight_control.hpp"
#include "pid.hpp"
#include <INA3221.h>
//#include <Adafruit_BMP280.h>
#include "MadgwickAHRS.h"
#include "vl53l0x.hpp"
#include <stdint.h>
//#include "Adafruit_Sensor.h"

#define I2C_SCL 39
#define I2C_SDA 38
#define MPU6886_ADDRESS 0x68
#define MPU6886_ADDRESS           0x68 
#define MPU6886_WHOAMI            0x75
#define MPU6886_ACCEL_INTEL_CTRL  0x69
#define MPU6886_SMPLRT_DIV        0x19
#define MPU6886_INT_PIN_CFG       0x37
#define MPU6886_INT_ENABLE        0x38
#define MPU6886_ACCEL_XOUT_H      0x3B
#define MPU6886_ACCEL_XOUT_L      0x3C
#define MPU6886_ACCEL_YOUT_H      0x3D
#define MPU6886_ACCEL_YOUT_L      0x3E
#define MPU6886_ACCEL_ZOUT_H      0x3F
#define MPU6886_ACCEL_ZOUT_L      0x40

#define MPU6886_TEMP_OUT_H        0x41
#define MPU6886_TEMP_OUT_L        0x42

#define MPU6886_GYRO_XOUT_H       0x43
#define MPU6886_GYRO_XOUT_L       0x44
#define MPU6886_GYRO_YOUT_H       0x45
#define MPU6886_GYRO_YOUT_L       0x46
#define MPU6886_GYRO_ZOUT_H       0x47
#define MPU6886_GYRO_ZOUT_L       0x48

#define MPU6886_USER_CTRL         0x6A
#define MPU6886_PWR_MGMT_1        0x6B
#define MPU6886_PWR_MGMT_2        0x6C
#define MPU6886_CONFIG            0x1A
#define MPU6886_GYRO_CONFIG       0x1B
#define MPU6886_ACCEL_CONFIG      0x1C
#define MPU6886_ACCEL_CONFIG2     0x1D
#define MPU6886_FIFO_EN           0x23

//#define G (9.8)
#define RtA     57.324841
#define AtR    	0.0174533	
#define Gyro_Gr	0.0010653


typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
} quat_t;

//Sensor data
extern volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
extern volatile float Phi, Theta, Psi;

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
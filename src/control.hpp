#ifndef CONTROL_HPP
#define CONTROL_HPP

#define BATTERY_VOLTAGE (3.7)
#define BLUE 0x0000ff
#define RED 0xff0000
#define GREEN 0x00ff00

#define AVERAGE 200
#define KALMANWAIT 600

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
} quat_t;

class PID
{
  private:
    float m_kp;
    float m_ti;
    float m_td;
    float m_filter_time_constant;
    float m_err,m_err2,m_err3;
    float m_h;
  public:
    float m_filter_output;
    float m_integral;
    PID();
    void set_parameter(
        float kp, 
        float ti, 
        float td,
        float filter_time_constant, 
        float h);
    void reset(void);
    void i_reset(void);
    void printGain(void);
    float filter(float x);
    float update(float err);
};

class Filter
{
  private:
    float m_state;
    float m_T;
    float m_h;
  public:
    float m_out;
    Filter();
    void set_parameter(
        float T,
        float h);
    void reset(void);
    float update(float u);
};

//グローバル関数の宣言
void init_atomfly(void);
void test_rangefinder(void);
void init_i2c();
void init_pwm();

void loop_400Hz(void);
void control_init();
void rate_control(void);
void angle_control(void);
void gyro_calibration(void);
void variable_init(void);
void log_output(void);
void gpio_put(CRGB p, uint8_t state);
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);
void imu_mag_data_read(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);

//グローバル変数
extern volatile uint8_t LockMode;
extern volatile uint8_t Logoutputflag;
extern volatile uint32_t S_time, E_time, D_time, S_time2, E_time2, D_time2;

#endif

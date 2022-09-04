#ifndef PID_HPP
#define PID_HPP

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

#endif
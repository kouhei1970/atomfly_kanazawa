#ifndef RC_HPP
#define RC_HPP

#define RUDDER 0
#define ELEVATOR 1
#define THROTTLE 2
#define AILERON 3
#define LOG 4
#define LED_PIN 13
#define RUDDER_MAX 127
#define RUDDER_MIN -128
#define ELEVATOR_MAX 127
#define ELEVATOR_MIN -128
#define THROTTLE_MAX 127
#define THROTTLE_MIN -128
#define AILERON_MAX 127
#define AILERON_MIN -128
#define LOG_MAX 1
#define LOG_MIN 0
#define CH6MAX 127
#define CH6MIN -128

#define BTID "4c:75:25:d5:b2:8e"

void rc_init(void);
void rc_demo(void);
void rc_end(void);

extern volatile float Stick_throttle;
extern volatile float Stick_roll;
extern volatile float Stick_pitch;
extern volatile float Stick_yaw;
extern volatile uint8_t Log_sw;

#endif
#include "AppDebug.h"
#include "EspOTA.h"
#include <Arduino.h>
#include "ArduinoJson.h"
#include "Ticker.h"
#include "KalmanMPU6050.h"

#include "soc/soc.h"  //Brownout detector was triggered
#include "soc/rtc_cntl_reg.h"

#define OFFSET_ROLL 1    //+1 deg

#define PWM_MOTOR_LEFT 14
#define DIR_MOTOR_LEFT 16
#define FG_MOTOR_LEFT 13
#define PWM_MOTOR_RIGHT 18
#define DIR_MOTOR_RIGHT 19
#define FG_MOTOR_RIGHT 17

#define MOTOR_CHANNEL_LEFT 1
#define MOTOR_CHANNEL_RIGHT 2

#define TRIG_DISTANT_SENSOR 23
#define ECHO_DISTANT_SENSOR 25

#define LED_TEST 32

#define SAMPLE_TIME_CALCU_SPEED 0.01    //10ms
#define PULSE_FG_MOTOR 6    //xung feedback
#define RADIUS 0.03         //ban kinh banh xe 3cm
#define LONG_SEGWAY 0.2     //chieu dai banh xe 20cm


const double K[] = {320.85, 70.62, 50, 4, 1, 4, 320.31, 70.62, -50, 4, 1, -4};   //  [24.85, 70.62, 14.77, 37, -7.27, -2.23,
                                                                                //  -18.31, 5, -14.66, 356, -3.44, 2.23]

double kalman_Roll;
double set_position_x = 0;
double set_position_csi = 0;
double set_position_title = 0;
double err_x, pre_err_x = 0;
double err_csi, pre_err_csi = 0;
double err_title, pre_err_title = 0;
double dev_x, dev_csi, dev_title;   //thanh phan dao ham kd
double output_pwm_motor_left, output_pwm_motor_right;
int pul_FG_motor_left = 0, pul_FG_motor_right = 0;


void calculateSpeed();
void setupPinMode();
void IRAM_ATTR inputSpeedMotorLeft();
void IRAM_ATTR inputSpeedMotorRight();

Ticker tickerCalculateSpeed(calculateSpeed, SAMPLE_TIME_CALCU_SPEED*1000);   //every 10ms
// Ticker SetPWMspeed(setpwmMotor, 1, 0, MICROS_MICROS);  //MICROS_MICROS
// Ticker SetPWMStopSpeed(setpwmStopMotor, 1, 0, MICROS_MICROS);
// Ticker tickerSetApMode(setLedApMode, 200, 0);   //every 200ms
// Ticker tickerSetPwmLedLightOn(setPwmLedLighton, 20, 255);	//every 20ms
// Ticker tickerSetPwmLedLightOff(setPwmLedLightoff, 10, 255);
// Ticker tickerSetPwmLedLightChange(setPwmLedLightChange, 10, 255);

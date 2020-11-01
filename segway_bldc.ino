#include "segway_bldc.h"

void calculateSpeed(){
	/* Reads the data from the MPU and processes it with the Kalman Filter */
    IMU::read();
    kalman_Roll = IMU::getRoll() + OFFSET_ROLL;

	//bien X
	err_x = set_position_x - ((pul_FG_motor_left+pul_FG_motor_right)/2)*2*M_PI*RADIUS/PULSE_FG_MOTOR;
	dev_x = (err_x - pre_err_x)/SAMPLE_TIME_CALCU_SPEED;         //dev trong thanh phan Kd
	pre_err_x = err_x;

	//bien title
	err_title = set_position_title - kalman_Roll;         //mong muon goc do la 0, nen se lay 0 tru di goc do dc
	dev_title = (err_title - pre_err_title)/SAMPLE_TIME_CALCU_SPEED;         //dev trong thanh phan Kd
    pre_err_title = err_title;

	//bien csi
    err_csi = set_position_csi - ((pul_FG_motor_left-pul_FG_motor_right)/2)*2*M_PI*RADIUS/PULSE_FG_MOTOR;         //mong muon goc do la 0, nen se lay 0 tru di goc do dc, xe co 12 xung.
    dev_csi = (err_csi - pre_err_csi)/SAMPLE_TIME_CALCU_SPEED;         //dev trong thanh phan Kd
    pre_err_csi = err_csi;

	output_pwm_motor_left = K[0]*err_x + K[1]*err_title + K[2]*err_csi + K[3]*dev_x+ K[4]*dev_title + K[5]*dev_csi;
	output_pwm_motor_right = K[6]*err_x + K[7]*err_title + K[8]*err_csi + K[9]*dev_x+ K[10]*dev_title + K[11]*dev_csi;

	ECHO(kalman_Roll);
	ECHO("       ");
	ECHO(output_pwm_motor_left);
	ECHO("       ");
	ECHOLN(output_pwm_motor_right);
}

void setupPinMode(){
    pinMode(DIR_MOTOR_LEFT, INPUT_PULLUP);
	pinMode(FG_MOTOR_LEFT, INPUT_PULLUP);
	pinMode(DIR_MOTOR_RIGHT, INPUT_PULLUP);
	pinMode(FG_MOTOR_RIGHT, INPUT_PULLUP);
	pinMode(LED_TEST, OUTPUT);
}

void IRAM_ATTR inputSpeedMotorLeft(){
    if(output_pwm_motor_left >= 0){
		pul_FG_motor_left++;
	}else{
		pul_FG_motor_left--;
	}
}

void IRAM_ATTR inputSpeedMotorRight(){
    if(pul_FG_motor_right >= 0){
		pul_FG_motor_right++;
	}else{
		pul_FG_motor_right--;
	}
}

void setup() {
  // put your setup code here, to run once:
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
	delay(10);
    Serial.begin(115200);

	ledcSetup(MOTOR_CHANNEL_LEFT, 30000, 8); // 30 kHz PWM, 8-bit resolution
	ledcSetup(MOTOR_CHANNEL_RIGHT, 30000, 8); // 30 kHz PWM, 8-bit resolution

	ledcAttachPin(PWM_MOTOR_LEFT, MOTOR_CHANNEL_LEFT);              // analog pin to channel Motor MOTOR_CHANNEL_LEFT
	ledcAttachPin(PWM_MOTOR_RIGHT, MOTOR_CHANNEL_RIGHT);              // analog pin to channel Motor

    setupPinMode();

	attachInterrupt(digitalPinToInterrupt(FG_MOTOR_LEFT), inputSpeedMotorLeft, FALLING);
	attachInterrupt(digitalPinToInterrupt(FG_MOTOR_RIGHT), inputSpeedMotorRight, FALLING);

    IMU::init();

	tickerCalculateSpeed.start();


}

void loop() {
   
    
    tickerCalculateSpeed.update();

}

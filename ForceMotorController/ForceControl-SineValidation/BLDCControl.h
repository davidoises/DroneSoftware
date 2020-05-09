/*!
 * @file BLDCControl.h
 * @brief Motor structures and variables for PID speed control
 *
 * @copyright	GNU Lesser General Public License
 
 * @author [DDM]
 * @version  V1.0
 * @date  2020-04-21
 */
 
#ifndef __BLDCControl_H
#define __BLDCControl_H

#include <Arduino.h>

// PWM properties
#define FREQ 50
#define RESOLUTION 12

// Constants for ESC control
#define MAX_PWM (1<<RESOLUTION)-1
#define MAX_PERIOD 1000.0f/((double)FREQ)
#define MS_TO_PWM(x) ((double)x)*((double)MAX_PWM)/(((double)MAX_PERIOD)*1000.0f)

// Struct with all the variables related to encoder reading, motor direction and position PID controller
typedef struct {
	// Variables related to hardware: ADC, PWM and input pins
  uint8_t adc_pin;
	uint8_t pwm_pin;
  uint8_t pwm_channel;

	// Variables for PID controller
  double pwm = 0;
  double sampling_time;
	double setpoint = 0;
  double voltage;
  double force;
	double prev_pid = 0;
  double prev2_pid = 0;
	double prev_error = 0;
  double prev2_error = 0;
  double kp = 269.725;
  double ki = 519.075;
  double kd = 35.0375;
  
  //double kp = 1436/3.0;
  //double ki = 0.0;
  //double kd = 104.82/3.0;
  //double kp = 210;
  //double ki = 0.0;
  //ouble kd = 60;
  
}MotorData;

class BLDCController	
{
	public:
		BLDCController(uint8_t adc_pin, uint8_t pwm_pin, uint8_t pwm_channel, double sampling_time);
		void init();
    double calculatePID();
    void controlMotor();
    void setpoint(double setpoint);
    double getVoltage();
    double getForce();
    double getSetpoint();
	private:
    MotorData vars;
		
};

#endif

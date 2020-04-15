/*!
 * @file MotorControl.h
 * @brief Motor structures and variables for direction and PID position control
 *
 * @copyright	GNU Lesser General Public License
 
 * @author [DDM]
 * @version  V1.0
 * @date  2019-11-12
 */
 
#ifndef __MotorControl_H
#define __MotorControl_H

#include <Arduino.h>

// Struct with all the variables related to encoder reading, motor direction and position PID controller
typedef struct {
	
	// Variables related to hardware: Encoder, PWM and input pins
	uint8_t a_channel;
	uint8_t b_channel;
	uint8_t pwm_pin1;
	uint8_t pwm_pin2;
	uint8_t in_channel1;
	uint8_t in_channel2;
	uint8_t res = 8;
	uint8_t freq = 100;
  uint8_t sense_factor;
	// Variables for encoder reading
	double counts_to_deg;
  double deg_to_pwm = (pow(2, res)-1)/360.0;
	long count = 0;
  int temp_count = 0;
	bool A,B;
	uint8_t state;
	uint8_t previous_state;
	// Variables for PID controller
	double setpoint = 0;
	double prev_integral = 0;
	double prev_error = 0;
	double kp;
  double ki;
  double kd;
  // Variables for trayectory planning
  double theta_zero;
  double theta_f;
  double theta_current;
  double lapse;
  double t_current;	
}MotorDetails;

class MotorController	
{
	public:
		MotorDetails vars;
		MotorController(uint8_t a_channel, uint8_t b_channel, uint8_t pwm_pin1, uint8_t pwm_pin2, uint8_t in_channel1, uint8_t in_channel2, double counts_to_deg, uint8_t sense_factor=1);
		~MotorController();
		void init();
    void interruptHandler();
    double calculatePID(double delta_time);
    int controlMotor(double control_law, double from);
		void setTrayectory(double theta_f, double lapse);
    boolean updateSetpoint(double dt);
	private:
		
};

#endif

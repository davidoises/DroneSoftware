#include "BLDCControl.h"

BLDCController::BLDCController(uint8_t adc_pin, uint8_t pwm_pin, uint8_t pwm_channel, double sampling_time)
{
	vars.adc_pin = adc_pin;
	vars.pwm_pin = pwm_pin;
	vars.pwm_channel = pwm_channel;
	vars.sampling_time = sampling_time;
}

void BLDCController::init()
{
  ledcSetup(vars.pwm_channel, FREQ, RESOLUTION);
  ledcAttachPin(vars.pwm_pin, vars.pwm_channel);
  ledcWrite(vars.pwm_channel, MS_TO_PWM(1000.0));
}

double BLDCController::calculatePID()
{
  double N = 10;
  vars.voltage = analogRead(vars.adc_pin)*33.0/4095.0;
  vars.force = 0.0095*sq(vars.voltage) + 0.0068*vars.voltage;
  double error = vars.setpoint - vars.force;
  
  // use this for PI with Kd = 0
  //double u = vars.prev_pid + + vars.kp*(error - vars.prev_error) + vars.ki*vars.sampling_time*error + (vars.kd/vars.sampling_time)*(error - 2*vars.prev_error + vars.prev2_error);
  
  // use this for PID with low pass filtered D
  double u = (2-vars.sampling_time*N)*vars.prev_pid + (vars.sampling_time*N-1)*vars.prev2_pid + vars.kp*(error + (vars.sampling_time*N-2)*vars.prev_error + (1-vars.sampling_time*N)*vars.prev2_error) + vars.ki*vars.sampling_time*(vars.prev_error+(vars.sampling_time*N-1)*vars.prev2_error) + vars.kd*N*(error - 2*vars.prev_error + vars.prev2_error);
  
  // use this for PD with low pass filtered D
  //double u = (1-vars.sampling_time*N)*vars.prev_pid + vars.kp*(error + (vars.sampling_time*N-1)*vars.prev_error) + vars.kd*N*(error - vars.prev_error);
  
  vars.prev2_error = vars.prev_error;
  vars.prev_error = error;
  vars.prev2_pid = vars.prev_pid;
  vars.prev_pid = u;
  
  vars.pwm = constrain(1000.0 + u, 1000.0, 2000.0);
  if(vars.pwm < 1100)
    vars.pwm = 1000;
  return vars.pwm;
}

void BLDCController::controlMotor()
{
  ledcWrite(vars.pwm_channel, MS_TO_PWM(vars.pwm));
}

void BLDCController::setpoint(double setpoint)
{
  // Make sure minimum force is 0.4 since accomplishing less results difficult for ESCs
  if(setpoint < 0.4)
    vars.setpoint = 0;
  else
    vars.setpoint = setpoint;
}

double BLDCController::getVoltage()
{
  return vars.voltage;
}

double BLDCController::getForce()
{
  return vars.force;
}

double BLDCController::getSetpoint()
{
  return vars.setpoint;
}

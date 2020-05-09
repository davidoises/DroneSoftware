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
  vars.voltage = analogRead(vars.adc_pin)*33.0/4095.0;
  double error = vars.setpoint - vars.voltage;
  double u = vars.prev_pid + (vars.kp + vars.ki*vars.sampling_time)*error - vars.kp*vars.prev_error;
  vars.prev_error = error;
  vars.prev_pid = u;
  
  vars.pwm = constrain(1000.0 + u, 1000.0, 2000.0);
  return vars.pwm;
}

void BLDCController::controlMotor()
{
  ledcWrite(vars.pwm_channel, MS_TO_PWM(vars.pwm));
}

void BLDCController::setpoint(double setpoint)
{
  vars.setpoint = setpoint;
}

double BLDCController::getVoltage()
{
  return vars.voltage;
}

double BLDCController::getSetpoint()
{
  return vars.setpoint;
}

#include "MotorControl.h"

MotorController::MotorController(uint8_t a_channel, uint8_t b_channel, uint8_t pwm_pin1, uint8_t pwm_pin2, uint8_t in_channel1, uint8_t in_channel2, double counts_to_deg, uint8_t sense_factor)
{
	vars.a_channel = a_channel;
	vars.b_channel = b_channel;
	vars.pwm_pin1 = pwm_pin1;
	vars.pwm_pin2 = pwm_pin2;
	vars.in_channel1 = in_channel1;
	vars.in_channel2 = in_channel2;
  vars.sense_factor = sense_factor;
  vars.counts_to_deg = counts_to_deg*sense_factor;
}

MotorController::~MotorController()
{}

void MotorController::init()
{
	pinMode(vars.a_channel, INPUT);
	pinMode(vars.b_channel, INPUT);
	
	ledcSetup(vars.in_channel1, vars.freq, vars.res);
	ledcSetup(vars.in_channel2, vars.freq, vars.res);
	ledcAttachPin(vars.pwm_pin1, vars.in_channel1);
	ledcAttachPin(vars.pwm_pin2, vars.in_channel2);
}


void MotorController::interruptHandler()
{
    vars.A = digitalRead(vars.a_channel);
    vars.B = digitalRead(vars.b_channel);
    
    
    if ((vars.A==HIGH)&&(vars.B==HIGH)) vars.state = 1;
    if ((vars.A==HIGH)&&(vars.B==LOW)) vars.state = 2;
    if ((vars.A==LOW)&&(vars.B==LOW)) vars.state = 3;
    if((vars.A==LOW)&&(vars.B==HIGH)) vars.state = 4;

    switch (vars.state)
    {
      case 1:
      {
        if (vars.previous_state == 2) vars.temp_count++;
        if (vars.previous_state == 4) vars.temp_count--;
        break;
      }
      case 2:
      {
        if (vars.previous_state == 1) vars.temp_count--;
        if (vars.previous_state == 3) vars.temp_count++;
        break;
      }
      case 3:
      {
        if (vars.previous_state == 2) vars.temp_count --;
        if (vars.previous_state == 4) vars.temp_count ++;
        break;
      }
      default:
      {
        if (vars.previous_state == 1) vars.temp_count++;
        if (vars.previous_state == 3) vars.temp_count--;
      }
    }
    if(vars.temp_count == vars.sense_factor)
    {
      vars.temp_count = 0;
      vars.count++;
    }
    if(vars.temp_count == -vars.sense_factor)
    {
      vars.temp_count = 0;
      vars.count--;
    }
    vars.previous_state = vars.state;
}

double MotorController::calculatePID(double delta_time)
{
  double error = ((double)vars.count)*vars.counts_to_deg - vars.setpoint;
  double integral = (delta_time/2.0)*(error + vars.prev_error)+ vars.prev_integral;
  double diff = (error - vars.prev_error)/(delta_time);
  vars.prev_error = error;
  vars.prev_integral = integral;
  
  return vars.kp*error + vars.ki*integral + vars.kd*diff;
  
}

int MotorController::controlMotor(double control_law, double from)
{
  int pwm = constrain(abs(control_law), 0, 360);
  
  if(pwm != 0)
  {
    pwm = map(pwm, 0, 360, from, pow(2, vars.res)-1);
  }
  
  if(control_law < 0)
  {
    ledcWrite(vars.in_channel1, 0);
    ledcWrite(vars.in_channel2, pwm);
  }
  else
  {
    ledcWrite(vars.in_channel1, pwm);
    ledcWrite(vars.in_channel2, 0);
  }

  return pwm;
}

void MotorController::setTrayectory(double theta_f, double lapse)
{
  vars.theta_zero = vars.count*vars.counts_to_deg;
  vars.theta_f = theta_f;
  vars.theta_current = vars.theta_zero;
  vars.lapse = lapse;
  vars.t_current = 0;
}

boolean MotorController::updateSetpoint(double dt)
{
  if(vars.setpoint < vars.theta_f)
  {
    vars.t_current += dt;
    double trayectory = vars.theta_zero - (10*pow(vars.t_current,3)*(vars.theta_zero - vars.theta_f))/pow(vars.lapse,3) + (15*pow(vars.t_current,4)*(vars.theta_zero - vars.theta_f))/pow(vars.lapse,4) - (6*pow(vars.t_current,5)*(vars.theta_zero - vars.theta_f))/pow(vars.lapse,5);
    vars.setpoint = trayectory; 
  }
  else
  {
    vars.setpoint = vars.theta_f;
    if(abs(vars.count*vars.counts_to_deg-vars.setpoint) < .001)
    {
      return 1; 
    }
  }
  return 0;
}

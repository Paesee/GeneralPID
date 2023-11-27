#include "GeneralPID.h"

void GeneralPID_init(GeneralPID *self, float sampling_time, float kp, float ki, float kd)
{
}

void GeneralPID_setGains(GeneralPID *self, float kp, float ki, float kd)
{
  // store PID gains inserted by the user
  self->KP = kp;
  self->KI = ki;
  self->KD = kd;
  // calculate discrete coefficients based on sampling
  float discrete_kp = self->KP;
  float discrete_ki = self->KI * self->sampling_time * 0.5;
  float discrete_kd = self->KD * self->sampling_time * 2;
  // store gains used to calculate the equivalent difference equation
  self->K1 = discrete_kp + discrete_ki + discrete_kd;
  self->K2 = 2 * discrete_ki - 2 * discrete_kd;
  self->K3 = -discrete_kp + discrete_ki + discrete_kd;
  // store gains without considering integrator coeffiecient in case of output saturation
  self->K1_anti_windup = discrete_kp + discrete_ki * 0 + discrete_kd;
  self->K2_anti_windup = 2 * discrete_ki * 0 - 2 * discrete_kd;
  self->K3_anti_windup = -discrete_kp + discrete_ki * 0 + discrete_kd;
}

int GeneralPID_setSetpoint(GeneralPID *self, float setpoint);

float GeneralPID_execute(GeneralPID *self, float measurement)
{
  // variables definition
  float output = 0;

  // error calculation
  float error = self->setpoint - measurement;

  // calculate differences equation
  if (self->output_n1 < self->max_output)
  {
    output = self->output_n2 + self->K1 * error + self->K2 * self->error_n1 + self->K3 * self->error_n2;
  }
  else
  {
    output = self->output_n2 + self->K1_anti_windup * error + self->K2_anti_windup * self->error_n1 + self->K3_anti_windup * self->error_n2;
  }

  // store last control values
  self->output_n2 = self->output_n1;
  self->output_n1 = output;
  self->error_n2 = self->error_n1;
  self->error_n1 = error;

  // return control action
  return output;
}
#include "GeneralPID.h"

int GeneralPID_init(GeneralPID *self, float sampling_time, float kp, float ki, float kd)
{
  self->setpoint = 0;
  self->sampling_time = sampling_time;
  GeneralPID_setGains(self, kp, ki, kd);
  self->max_output = 0;
  self->min_output = 0;
  self->last_error = 0;
  self->second_last_error = 0;
  self->last_output = 0;
  self->second_last_output = 0;
  self->setSetpoint = GeneralPID_setSetpoint;
  self->getSetpoint = GeneralPID_getSetpoint;
  self->setSamplingTime = GeneralPID_setSamplingTime;
  self->getSamplingTime = GeneralPID_getSamplingTime;
  self->setGains = GeneralPID_setGains;
  self->getGains = GeneralPID_getGains;
  self->setMatlabGains = GeneralPID_setMatlabGains;
  self->getMatlabGains = GeneralPID_getMatlabGains;
  self->setLimits = GeneralPID_setLimits;
  self->getLimits = GeneralPID_getLimits;
  self->execute = GeneralPID_execute;
}

int GeneralPID_setSetpoint(GeneralPID *self, float setpoint){
  self->setpoint = setpoint;
  return 0;
}

float GeneralPID_getSetpoint(GeneralPID *self){
  return self->setpoint;
}

int GeneralPID_setSamplingTime(GeneralPID *self, float sampling_time){
  
}
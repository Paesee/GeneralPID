#include "GeneralPID.h"

int GeneralPID_init(GeneralPID *self, float sampling_time, float kp, float ki, float kd)
{
#include <stdlib.h> // For memory allocation functions

  int GeneralPID_init(GeneralPID * self, float sampling_time, float kp, float ki, float kd)
  {
    if (self == 0)
      return 1;

    // Initialize struct members with provided values
    self->setpoint = 0.0;
    self->sampling_time = sampling_time;
    //GeneralPID_getGains;
    self->max_output = 0.0;
    self->min_output = 0.0;
    self->last_error = 0.0;
    self->second_last_error = 0.0;
    self->last_output = 0.0;
    self->second_last_output = 0.0;

    // Set function pointers to zero
    self->setSetpoint = 0;
    self->getSetpoint = 0;
    self->setSamplingTime = 0;
    self->getSamplingTime = 0;
    self->setGains = 0;
    self->getGains = 0;
    self->setMatlabGains = 0;
    self->getMatlabGains = 0;
    self->setLimits = 0;
    self->getLimits = 0;
    self->execute = 0;

    return 0; // Success
  }
}
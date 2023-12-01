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
  self->execute2 = GeneralPID_execute2;
}

int GeneralPID_setSetpoint(GeneralPID *self, float setpoint)
{
  self->setpoint = setpoint;
  return 0;
}

float GeneralPID_getSetpoint(GeneralPID *self)
{
  return self->setpoint;
}

int GeneralPID_setSamplingTime(GeneralPID *self, float sampling_time)
{
  self->sampling_time = sampling_time;
  GeneralPID_setGains(self, self->KP, self->KI, self->KD);
  return 0;
}

float GeneralPID_getSamplingTime(GeneralPID *self)
{
  return self->sampling_time;
}

int GeneralPID_setGains(GeneralPID *self, float kp, float ki, float kd)
{
  // store PID gains inserted by the user
  self->KP = kp;
  self->KI = ki;
  self->KD = kd;
  // calculate the gains according to tustin method
  float discrete_kp = self->KP;
  float discrete_ki = self->KI * self->sampling_time / 2.0;
  float discrete_kd = self->KD * self->sampling_time * 2.0;
  // define gains to execute PID
  self->K1 = discrete_kp + discrete_ki + discrete_kd;
  self->K2 = (2 * discrete_ki) - (2 * discrete_kd);
  self->K3 = -discrete_kp + discrete_ki + discrete_kd;
  // define gains to execute PD when control action saturates
  self->K1_AW = discrete_kp + discrete_kd;
  self->K2_AW = discrete_kp - discrete_kd;
  // return 0 for success
  return 0;
}

int GeneralPID_getGains(GeneralPID *self, float *kp, float *ki, float *kd)
{
  if (kp == NULL || ki == NULL || kd == NULL)
    return 1;
  *kp = self->KP;
  *ki = self->KI;
  *kd = self->KD;
  return 0;
}

int GeneralPID_setMatlabGains(GeneralPID *self, float K1, float K2, float K3, float K1_AW, float K2_AW)
{
  // reset PID gains
  self->KP = 0;
  self->KI = 0;
  self->KD = 0;
  // set K1, K2 and K3 gains
  self->K1 = K1;
  self->K2 = K2;
  self->K3 = K3;
  // reset antiwindup
  self->K1_AW = K1_AW;
  self->K2_AW = K2_AW;
  return 0;
}

int GeneralPID_getMatlabGains(GeneralPID *self, float *K1, float *K2, float *K3, float *K1_AW, float *K2_AW)
{
  if (K1 == NULL || K2 == NULL || K3 == NULL || K1_AW == NULL || K2_AW == NULL)
    return 1;
  *K1 = self->K1;
  *K2 = self->K2;
  *K3 = self->K3;
  *K1_AW = self->K1_AW;
  *K2_AW = self->K2_AW;
  return 0;
}

int GeneralPID_setLimits(GeneralPID *self, float max, float min)
{
  if (min >= max)
    return 1;
  self->max_output = max;
  self->min_output = min;
}

int GeneralPID_getLimits(GeneralPID *self, float *max, float *min)
{
  if (max == NULL || min == NULL)
    return 1;
  *max = self->max_output;
  *min = self->min_output;
}

float GeneralPID_execute(GeneralPID *self, float measurement)
{
  bool is_saturated = 0;
  float output = 0;
  // calcula do erro
  float error = self->setpoint - measurement;
  // verifica se esta saturado
  if (self->last_output >= self->max_output || self->last_output <= self->min_output)
    is_saturated = 1;
  // calcula a equacao de diferenças
  if (is_saturated)
  {
    output = self->last_output + self->K1 * error + self->K2 * self->second_last_error;
  }
  else
  {
    output = self->last_output + self->K1 * error + self->K2 * self->last_error + self->K3 * self->second_last_error;
  }
  // estabeleces as limitacoes de saida
  if (output > self->max_output)
    output = self->max_output;
  if (output < self->min_output)
    output = self->min_output;
  // armazena os valores anteriores de erro e saida
  self->second_last_output = self->last_output;
  self->last_output = output;
  self->second_last_error = self->last_error;
  self->last_error = error;
  // retorna o valor da acao de controle
  return output;
}

float GeneralPID_execute2(GeneralPID *self, float measurement)
{
  bool is_saturated = 0;
  float output = 0;
  // calcula do erro
  float error = self->setpoint - measurement;
  // verifica se esta saturado
  if (self->last_output >= self->max_output || self->last_output <= self->min_output)
    is_saturated = 1;
  // calcula a equacao de diferenças
  output = (!is_saturated) * (self->last_output + self->K1 * error + self->K2 * self->last_error + self->K3 * self->second_last_error) + (is_saturated) * (self->last_output + self->K1 * error + self->K2 * self->second_last_error);
  //  estabeleces as limitacoes de saida
  output = (output > self->max_output) ? self->max_output : (output < self->min_output) ? self->min_output : output;
  // armazena os valores anteriores de erro e saida
  self->second_last_output = self->last_output;
  self->last_output = output;
  self->second_last_error = self->last_error;
  self->last_error = error;
  // retorna o valor da acao de controle
  return output;
}

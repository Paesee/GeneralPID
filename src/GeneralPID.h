#ifndef GENERALPID_H
#define GENERALPID_H

#include <stdlib.h>
#include <stdbool.h>

/// @brief General PID struct meant to be used for any control loop that controls a DC value
typedef struct GeneralPID
{
  // setpoint for control routine
  float setpoint;
  // sampling time
  float sampling_time;
  // PID gains
  float KP;
  float KI;
  float KD;
  // PID gains converted according to tustin discretization method
  float K1;
  float K2;
  float K3;
  // anti-windup PD gains converted according to tustin discretization method
  float K1_AW;
  float K2_AW;
  // define max and min output values
  float max_output;
  float min_output;
  // variables used to store past erros and past outputs
  float last_error;
  float second_last_error;
  float last_output;
  float second_last_output;
  // pointers to functions
  int (*setSetpoint)(struct GeneralPID *self, float setpoint);
  float (*getSetpoint)(struct GeneralPID *self);
  int (*setSamplingTime)(struct GeneralPID *self, float sampling_time);
  float (*getSamplingTime)(struct GeneralPID *self);
  int (*setGains)(struct GeneralPID *self, float kp, float ki, float kd);
  int (*getGains)(struct GeneralPID *self, float *kp, float *ki, float *kd);
  int (*setMatlabGains)(struct GeneralPID *self, float K1, float K2, float K3, float K1AW, float K2AW);
  int (*getMatlabGains)(struct GeneralPID *self, float *K1, float *K2, float *K3, float *K1_AW, float *K2_AW);
  int (*setLimits)(struct GeneralPID *self, float max, float min);
  int (*getLimits)(struct GeneralPID *self, float *max, float *min);
  float (*execute)(struct GeneralPID *self, float measurement);
  float (*execute2)(struct GeneralPID *self, float measurement);
} GeneralPID;

/// @brief initialize the PID struct
/// @param self pointer to the instance of the struct
/// @param sampling_time sampling time (in seconds | ts = 1/f)
/// @param kp proportional gain
/// @param ki integral gain
/// @param kd derivative gain
/// @return 0 for success and 1 for errors
int GeneralPID_init(GeneralPID *self, float sampling_time, float kp, float ki, float kd);

/// @brief set the setpoint of the control loop
/// @param self pointer to the instance of the struct
/// @param setpoint numerical value of the setpoint
/// @return 0 for success and 1 for errors
int GeneralPID_setSetpoint(GeneralPID *self, float setpoint);

/// @brief get setpoint value
/// @param self pointer to the instance of the struct
/// @return setpoint
float GeneralPID_getSetpoint(GeneralPID *self);

/// @brief set sampling time on the fly, this function also call GeneralPID_setGains
/// @param self pointer to the instance of the struct
/// @param sampling_time
/// @return 0 for success and 1 for errors
int GeneralPID_setSamplingTime(GeneralPID *self, float sampling_time);

/// @brief get sampling time
/// @param self pointer to the instance of the struct
/// @return sampling time
float GeneralPID_getSamplingTime(GeneralPID *self);

/// @brief set the PID gains, converting them according to Tustin Discretization Method
/// @param self pointer to the instance of the struct
/// @param kp proportional gain
/// @param ki integral gain
/// @param kd derivative gain
/// @return 0 for success and 1 for errors
int GeneralPID_setGains(GeneralPID *self, float kp, float ki, float kd);

/// @brief get PID gains through pointers
/// @param self pointer to the instance of the struct
/// @param kp a pointer to variable that will receive proportional gain
/// @param ki a pointer to variable that will receive integral gain
/// @param kd a pointer to variable that will receive derivative gain
int GeneralPID_getGains(GeneralPID *self, float *kp, float *ki, float *kd);

/// @brief set the gains according to matlab function c2d(model_tf, ts, 'tustin'), see examples for further explanation
/// @param self pointer to the instance of the struct
/// @param K1 value that multiplies z^2 -> K1 * z^2
/// @param K2 value that multiplies z^1 -> K2 * z^1
/// @param K3 value that multiplies z^0 -> K3 * z^0
/// @param K1AW see anti-windup implementation for further explanation
/// @param K2AW see anti-windup implementation for further explanation
/// @return 0 for success and 1 for errors
int GeneralPID_setMatlabGains(GeneralPID *self, float K1, float K2, float K3, float K1AW, float K2AW);

/// @brief get PID gains through pointers
/// @param self pointer to the instance of the struct
/// @param K1 a pointer to variable that will receive K1 gain
/// @param K2 a pointer to variable that will receive K2 gain
/// @param K3 a pointer to variable that will receive K3 gain
/// @param K1AW a pointer to variable that will receive K1AW gain
/// @param K2AW a pointer to variable that will receive K2AW gain
int GeneralPID_getMatlabGains(GeneralPID *self, float *K1, float *K2, float *K3, float *K1_AW, float *K2_AW);

/// @brief set the maximum and minimum output of the control loop
/// @param self pointer to the instance of the struct
/// @param max maximum output value of GeneralPID_execute function
/// @param min minimum output value of GeneralPID_execute function
/// @return 0 for success and 1 for errors
int GeneralPID_setLimits(GeneralPID *self, float max, float min);

/// @brief get output limits
/// @param self pointer to the instance of the struct
/// @param max a pointer to variable that will receive the maximum output
/// @param min a pointer to variable that will receive the minimum output
int GeneralPID_getLimits(GeneralPID *self, float *max, float *min);

/// @brief execute PID control loop
/// @param self pointer to the instance of the struct
/// @param measurement
/// @return control loop action
float GeneralPID_execute(GeneralPID *self, float measurement);

/// @brief execute PID control loop
/// @param self pointer to the instance of the struct
/// @param measurement
/// @return control loop action
float GeneralPID_execute2(GeneralPID *self, float measurement);

#endif
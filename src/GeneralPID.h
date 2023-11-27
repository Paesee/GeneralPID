#ifndef GENERAL_PID_H // diretiva de inclusão condicional para evitar múltiplas inclusões do mesmo arquivo de cabeçalho
#define GENERAL_PID_H

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
  // Gains definition
  float K1;
  float K2;
  float K3;
  float K1_anti_windup;
  float K2_anti_windup;
  float K3_anti_windup;
  // variables used to store past erros and outputs
  float error_n1;
  float error_n2;
  float output_n1;
  float output_n2;
  // max output values
  float max_output;
  float min_output;
  // pointers to functions
  int (*setSetpoint)(struct GeneralPID *self, float setpoint);
  int (*setBoundaries)(struct GeneralPID *self, float max, float min);
  float (*execute)(struct GeneralPID *self, float meas_variable);
} GeneralPID;

void GeneralPID_init(GeneralPID *self, float sampling_time, float kp, float ki, float kd);
void GeneralPID_setGains(GeneralPID *self, float kp, float ki, float kd);
int GeneralPID_setSetpoint(GeneralPID *self, float setpoint);
float GeneralPID_execute(GeneralPID *self, float measurement);

#endif // fecha a diretiva de inclusão condicional
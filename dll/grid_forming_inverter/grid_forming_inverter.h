#define EXPORT __declspec(dllexport)

#ifndef GRID_FORMING_INVERTER
#define GRID_FORMING_INVERTER

/* DROOP CONTROLLER */

// Droop Controller Struct
typedef struct
{
  float p_max;
  float q_max;
  float p_ref;
  float q_ref;
  float v0;
  float w0;
  float m;
  float n;
  float last_pf_meas;
  float last_qf_meas;
} Droop;

// Droop Controller Functions
EXPORT void DroopControlInit(Droop *droop, float p_max, float q_max, float v0, float w0, float delta_v, float delta_w);
EXPORT void calculateDroopPQ(Droop *pc, float v_alpha, float v_beta, float i_alpha, float i_beta, float *active_power, float *reactive_power);
EXPORT void setDroopPQ(Droop *droop, float p_ref, float q_ref);
EXPORT void setDroopV0(Droop *droop, float v0);
EXPORT void setDroopW0(Droop *droop, float w0);
EXPORT void setDroopDeltaW(Droop *droop, float delta_w);
EXPORT void setDroopDeltaV(Droop *droop, float delta_v);
EXPORT void executeDroopControl(Droop *droop, float *v_ref, float *w_ref);

// Droop Controller Gains

/*
Having in mind that the implemented power calculation method uses a Low Pass Filter and its difference equation looks like this:
  P = LPF_GAIN_1 * p + LPF_GAIN_2 * ep->last_pf;
  Q = LPF_GAIN_1 * q + LPF_GAIN_2 * ep->last_qf;
Please, define the gains below.
*/

#define LPF_GAIN_1 0.01559
#define LPF_GAIN_2 0.9844

/* VOLTAGE CONTROLLER */

// Voltage Controller Struct

typedef struct
{
  float last_u;
  float sec_last_u;
  float last_error;
  float sec_last_error;
  float last_theta;
} VoltageControl;

// Voltage Controller Gains
#define V_GAIN_1 0.02148
#define V_GAIN_2 -0.01848
#define V_GAIN_3 -1.994
#define V_GAIN_4 1
// Voltage Controller Definitions
#define TSV 0.0002
#define TWO_PI 6.2831853
#define SQRT2 1.41421356237

// Voltage Controller Functions
EXPORT void VoltageControlInit(VoltageControl *vc);
EXPORT void generateReference(VoltageControl *vc, float v_rms, float w, float *sin_wt);
EXPORT void executeVoltageControl(VoltageControl *vc, float v_ref, float v_meas, float *u);

/* Current CONTROLLER */

// Current Controller Struct

//*names had to be changed due to similarity to grid_following current control

typedef struct
{
  float last_u;
  float sec_last_u;
  float last_error;
  float sec_last_error;
} GFormCurrentControl;

// Current Controller Gains
#define GFC_GAIN 3.7

// Current Controller Functions
EXPORT void GFormCurrentControlInit(GFormCurrentControl *cc);
EXPORT void GFormExecuteCurrentControl(GFormCurrentControl *cc, float i_ref, float i_meas, float *u);

#endif
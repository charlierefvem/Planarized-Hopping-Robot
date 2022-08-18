/*
 * PHRControl.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "PHRControl".
 *
 * Model version              : 1.67
 * Simulink Coder version : 9.7 (R2022a) 13-Nov-2021
 * C++ source code generated on : Thu Aug 11 15:04:13 2022
 *
 * Target selection: slrealtime.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "PHRControl.h"
#include "rtwtypes.h"
#include "PHRControl_cal.h"
#include "PHRControl_private.h"
#include <cmath>

extern "C" {

#include "rt_nonfinite.h"

}
/* Named constants for Chart: '<S9>/Command1.HSW_Controller' */
  const uint32_T PHRControl_IN_Flight = 1U;
const uint32_T PHRControl_IN_HSW = 2U;
const uint32_T PHRControl_IN_Idle = 3U;
const uint32_T PHRControl_IN_Load = 4U;
const uint8_T PHRControl_IN_NO_ACTIVE_CHILD = 0U;
const uint32_T PHRControl_IN_Unload = 5U;

/* Named constants for Chart: '<S9>/Command1.Raibert' */
const uint32_T PHRControl_IN_Compress = 1U;
const uint32_T PHRControl_IN_Flight_c = 2U;
const uint32_T PHRControl_IN_Thrust = 5U;
const uint32_T PHRControl_IN_Unload_l = 6U;

/* Named constants for Chart: '<S9>/Command1.SLIP' */
const uint32_T PHRControl_IN_Idle_j = 2U;
const uint32_T PHRControl_IN_Load_m = 3U;
const uint32_T PHRControl_IN_Spring = 4U;

/* Named constants for Chart: '<Root>/State-Transition Diagram' */
const uint32_T PHRControl_IN_Command1 = 1U;
const uint32_T PHRControl_IN_Deinitiate = 2U;
const uint32_T PHRControl_IN_FinalShutdown = 3U;
const uint32_T PHRControl_IN_HSW_Controller = 2U;
const uint32_T PHRControl_IN_Init = 4U;
const uint32_T PHRControl_IN_Raibert = 3U;
const uint32_T PHRControl_IN_SLIP = 4U;
const uint32_T PHRControl_IN_SlowDown = 5U;
const uint32_T PHRControl_IN_Stance = 5U;
const uint32_T PHRControl_IN_Stand = 6U;
const uint32_T PHRControl_IN_Start = 6U;
const uint32_T PHRControl_IN_Zero = 7U;

/* Block signals (default storage) */
B_PHRControl_T PHRControl_B;

/* Block states (default storage) */
DW_PHRControl_T PHRControl_DW;

/* Real-time model */
RT_MODEL_PHRControl_T PHRControl_M_ = RT_MODEL_PHRControl_T();
RT_MODEL_PHRControl_T *const PHRControl_M = &PHRControl_M_;

/* Forward declaration for local functions */
static void PHRControl_dec2bin(real_T d, char_T s_data[], int32_T s_size[2]);

/* Function for MATLAB Function: '<S6>/floats -> bytes' */
static void PHRControl_dec2bin(real_T d, char_T s_data[], int32_T s_size[2])
{
  int32_T idx;
  int32_T pmax;
  int32_T pmin;
  char_T b[64];
  char_T sfull[64];
  boolean_T exitg1;
  if (d < 0.0) {
    real_T b_d;
    boolean_T carry;
    b_d = -d;
    for (idx = 0; idx < 64; idx++) {
      b[idx] = '0';
    }

    idx = 64;
    exitg1 = false;
    while ((!exitg1) && (idx > 0)) {
      real_T olddi;
      olddi = b_d;
      b_d /= 2.0;
      b_d = std::floor(b_d);
      if (2.0 * b_d < olddi) {
        b[idx - 1] = '1';
      }

      if (!(b_d > 0.0)) {
        exitg1 = true;
      } else {
        idx--;
      }
    }

    for (idx = 0; idx < 64; idx++) {
      sfull[idx] = '1';
      if (b[idx] == '1') {
        sfull[idx] = '0';
      }
    }

    carry = true;
    for (idx = 63; idx >= 0; idx--) {
      char_T sfull_0;
      sfull_0 = sfull[idx];
      if (carry) {
        if (sfull_0 == '1') {
          sfull_0 = '0';
        } else {
          sfull_0 = '1';
          carry = false;
        }
      }

      sfull[idx] = sfull_0;
    }
  } else {
    real_T b_d;
    b_d = d;
    for (idx = 0; idx < 64; idx++) {
      sfull[idx] = '0';
    }

    idx = 64;
    exitg1 = false;
    while ((!exitg1) && (idx > 0)) {
      real_T olddi;
      olddi = b_d;
      b_d /= 2.0;
      b_d = std::floor(b_d);
      if (2.0 * b_d < olddi) {
        sfull[idx - 1] = '1';
      }

      if (!(b_d > 0.0)) {
        exitg1 = true;
      } else {
        idx--;
      }
    }
  }

  if (d < 0.0) {
    idx = 0;
    pmax = 0;
    exitg1 = false;
    while ((!exitg1) && (pmax < 64)) {
      if (sfull[pmax] == '0') {
        idx = pmax + 1;
        exitg1 = true;
      } else {
        pmax++;
      }
    }

    if (idx == 0) {
      pmax = 55;
      idx = 8;
    } else {
      idx = 66 - idx;
      if (idx <= 4) {
        idx = 4;
      }

      pmax = 31;
      pmin = 0;
      exitg1 = false;
      while ((!exitg1) && (pmax - pmin > 1)) {
        int32_T p;
        int32_T pow2p;
        p = (pmin + pmax) >> 1;
        pow2p = 1 << p;
        if (pow2p == idx) {
          pmax = p;
          exitg1 = true;
        } else if (pow2p > idx) {
          pmax = p;
        } else {
          pmin = p;
        }
      }

      idx = 1 << pmax;
      if (idx >= 64) {
        idx = 64;
      }

      pmax = 63 - idx;
    }
  } else {
    idx = 0;
    pmax = 0;
    exitg1 = false;
    while ((!exitg1) && (pmax < 64)) {
      if (sfull[pmax] == '1') {
        idx = pmax + 1;
        exitg1 = true;
      } else {
        pmax++;
      }
    }

    if (idx == 0) {
      idx = 64;
    }

    pmax = idx - 2;
    idx = 65 - idx;
  }

  if (idx <= 1) {
    s_size[0] = 1;
    s_size[1] = 1;
  } else {
    s_size[0] = 1;
    s_size[1] = idx;
  }

  for (pmin = 0; pmin < idx; pmin++) {
    s_data[pmin] = sfull[(pmax + pmin) + 1];
  }
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Output and update for atomic system:
 *    '<S6>/floats -> bytes'
 *    '<S7>/floats -> bytes'
 */
void PHRControl_floatsbytes(real_T rtu_position, real_T rtu_velocity, real_T
  rtu_K_p, real_T rtu_K_d, real_T rtu_T_ff, B_floatsbytes_PHRControl_T *localB)
{
  real_T B[8];
  real_T y;
  int32_T loop_ub;
  int32_T loop_ub_0;
  char_T p_data[79];
  char_T kd_data[75];
  char_T kp_data[75];
  char_T t_data[75];
  char_T v_data[75];
  char_T c_data[64];
  int8_T outsize_idx_1;
  static const boolean_T b[128] = { false, false, false, false, false, false,
    false, false, false, true, true, true, true, true, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    true, true, true, true, true, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false };

  int32_T c_size[2];
  if (rtu_position <= 95.5) {
    y = rtu_position;
  } else {
    y = 95.5;
  }

  if (!(y >= -95.5)) {
    y = -95.5;
  }

  PHRControl_dec2bin(std::floor((y - -95.5) * 65535.0 / 191.0), c_data, c_size);
  outsize_idx_1 = static_cast<int8_T>(16 - c_size[1]);
  loop_ub = outsize_idx_1;
  loop_ub_0 = c_size[1] - 1;
  for (int32_T i = 0; i < loop_ub; i++) {
    p_data[i] = '0';
  }

  for (int32_T i = 0; i <= loop_ub_0; i++) {
    p_data[i + loop_ub] = c_data[i];
  }

  if (rtu_velocity <= 45.0) {
    y = rtu_velocity;
  } else {
    y = 45.0;
  }

  if (!(y >= -45.0)) {
    y = -45.0;
  }

  PHRControl_dec2bin(std::floor((y - -45.0) * 4095.0 / 90.0), c_data, c_size);
  outsize_idx_1 = static_cast<int8_T>(12 - c_size[1]);
  loop_ub = outsize_idx_1;
  loop_ub_0 = c_size[1] - 1;
  for (int32_T i = 0; i < loop_ub; i++) {
    v_data[i] = '0';
  }

  for (int32_T i = 0; i <= loop_ub_0; i++) {
    v_data[i + loop_ub] = c_data[i];
  }

  if (rtu_K_p <= 500.0) {
    y = rtu_K_p;
  } else {
    y = 500.0;
  }

  if (!(y >= 0.0)) {
    y = 0.0;
  }

  PHRControl_dec2bin(std::floor(y * 4095.0 / 500.0), c_data, c_size);
  outsize_idx_1 = static_cast<int8_T>(12 - c_size[1]);
  loop_ub = outsize_idx_1;
  loop_ub_0 = c_size[1] - 1;
  for (int32_T i = 0; i < loop_ub; i++) {
    kp_data[i] = '0';
  }

  for (int32_T i = 0; i <= loop_ub_0; i++) {
    kp_data[i + loop_ub] = c_data[i];
  }

  if (rtu_K_d <= 45.0) {
    y = rtu_K_d;
  } else {
    y = 45.0;
  }

  if (!(y >= -45.0)) {
    y = -45.0;
  }

  PHRControl_dec2bin(std::floor(y * 4095.0 / 5.0), c_data, c_size);
  outsize_idx_1 = static_cast<int8_T>(12 - c_size[1]);
  loop_ub = outsize_idx_1;
  loop_ub_0 = c_size[1] - 1;
  for (int32_T i = 0; i < loop_ub; i++) {
    kd_data[i] = '0';
  }

  for (int32_T i = 0; i <= loop_ub_0; i++) {
    kd_data[i + loop_ub] = c_data[i];
  }

  if (rtu_T_ff <= 45.0) {
    y = rtu_T_ff;
  } else {
    y = 45.0;
  }

  if (!(y >= -45.0)) {
    y = -45.0;
  }

  PHRControl_dec2bin(std::floor((y - -18.0) * 4095.0 / 36.0), c_data, c_size);
  outsize_idx_1 = static_cast<int8_T>(12 - c_size[1]);
  loop_ub = outsize_idx_1;
  loop_ub_0 = c_size[1] - 1;
  for (int32_T i = 0; i < loop_ub; i++) {
    t_data[i] = '0';
  }

  for (int32_T i = 0; i <= loop_ub_0; i++) {
    t_data[i + loop_ub] = c_data[i];
  }

  for (int32_T i = 0; i < 64; i++) {
    c_data[i] = '0';
  }

  for (int32_T i = 0; i < 8; i++) {
    c_data[i << 3] = p_data[i];
    c_data[(i << 3) + 1] = p_data[i + 8];
    c_data[(i << 3) + 2] = v_data[i];
  }

  c_data[3] = v_data[8];
  c_data[35] = kp_data[0];
  c_data[11] = v_data[9];
  c_data[43] = kp_data[1];
  c_data[19] = v_data[10];
  c_data[51] = kp_data[2];
  c_data[27] = v_data[11];
  c_data[59] = kp_data[3];
  for (int32_T i = 0; i < 8; i++) {
    c_data[(i << 3) + 4] = kp_data[i + 4];
    c_data[(i << 3) + 5] = kd_data[i];
  }

  c_data[6] = kd_data[8];
  c_data[38] = t_data[0];
  c_data[14] = kd_data[9];
  c_data[46] = t_data[1];
  c_data[22] = kd_data[10];
  c_data[54] = t_data[2];
  c_data[30] = kd_data[11];
  c_data[62] = t_data[3];
  for (int32_T i = 0; i < 8; i++) {
    c_data[(i << 3) + 7] = t_data[i + 4];
  }

  for (int32_T i = 0; i < 8; i++) {
    uint64_T a;
    uint64_T p2;
    boolean_T b_p;
    boolean_T exitg1;
    loop_ub = 1;
    exitg1 = false;
    while ((!exitg1) && (loop_ub < 8)) {
      b_p = b[static_cast<int32_T>(c_data[((loop_ub - 1) << 3) + i])];
      if (b_p) {
        loop_ub++;
      } else {
        exitg1 = true;
      }
    }

    loop_ub_0 = 7;
    exitg1 = false;
    while ((!exitg1) && (loop_ub_0 + 1 > loop_ub)) {
      b_p = b[static_cast<int32_T>(c_data[(loop_ub_0 << 3) + i])];
      if (b_p) {
        loop_ub_0--;
      } else {
        exitg1 = true;
      }
    }

    p2 = 1UL;
    a = 0UL;
    exitg1 = false;
    while ((!exitg1) && (loop_ub < loop_ub_0 + 1)) {
      b_p = b[static_cast<int32_T>(c_data[((loop_ub - 1) << 3) + i])];
      if (b_p) {
        loop_ub++;
      } else {
        exitg1 = true;
      }
    }

    while (loop_ub_0 + 1 >= loop_ub) {
      if (c_data[(loop_ub_0 << 3) + i] == '1') {
        a += p2;
        p2 += p2;
      } else if (c_data[(loop_ub_0 << 3) + i] == '0') {
        p2 += p2;
      }

      loop_ub_0--;
    }

    B[i] = static_cast<real_T>(a);
  }

  for (int32_T i = 0; i < 8; i++) {
    uint8_T tmp;
    y = rt_roundd_snf(B[i]);
    if (y < 256.0) {
      if (y >= 0.0) {
        tmp = static_cast<uint8_T>(y);
      } else {
        tmp = 0U;
      }
    } else {
      tmp = MAX_uint8_T;
    }

    localB->b[i] = tmp;
  }
}

/*
 * Output and update for atomic system:
 *    '<S8>/bytes -> floats'
 *    '<S8>/bytes -> floats1'
 */
void PHRControl_bytesfloats(B_bytesfloats_PHRControl_T *localB)
{
  real_T I_ff;
  real_T position;
  real_T velocity;
  I_ff = localB->I_ff;
  velocity = localB->velocity;
  position = localB->position;
  localB->I_ff = I_ff;
  localB->velocity = velocity;
  localB->position = position;
  localB->position = localB->position * 191.0 / 65535.0 + -95.5;
  localB->velocity = localB->velocity * 90.0 / 4095.0 + -45.0;
  localB->I_ff = localB->I_ff * 80.0 / 4095.0 + -40.0;
}

/*
 * System initialize for action system:
 *    '<S16>/Idle'
 *    '<S17>/Idle'
 */
void PHRControl_Idle_Init(DW_Idle_PHRControl_T *localDW,
  PHRControl_Idle_cal_type *PHRControl_PageSwitching_arg)
{
  /* InitializeConditions for DiscreteIntegrator: '<S250>/Filter' */
  localDW->Filter_DSTATE =
    PHRControl_PageSwitching_arg->PIDController_InitialConditionForFilter;

  /* InitializeConditions for DiscreteIntegrator: '<S255>/Integrator' */
  localDW->Integrator_DSTATE =
    PHRControl_PageSwitching_arg->PIDController_InitialConditionForIntegrator;

  /* InitializeConditions for DiscreteIntegrator: '<S298>/Filter' */
  localDW->Filter_DSTATE_f =
    PHRControl_PageSwitching_arg->PIDController1_InitialConditionForFilter;

  /* InitializeConditions for DiscreteIntegrator: '<S303>/Integrator' */
  localDW->Integrator_DSTATE_g =
    PHRControl_PageSwitching_arg->PIDController1_InitialConditionForIntegrator;
}

/*
 * Output and update for action system:
 *    '<S16>/Idle'
 *    '<S17>/Idle'
 */
void PHRControl_Idle(const real_T rtu_measLeg[2], real_T *rty_T1, real_T *rty_T2,
                     real_T *rty_Machine_State, B_Idle_PHRControl_T *localB,
                     DW_Idle_PHRControl_T *localDW, PHRControl_Idle_cal_type
                     *PHRControl_PageSwitching_arg)
{
  real_T u0;
  real_T u1;
  real_T u2;

  /* Sum: '<S21>/Sum1' incorporates:
   *  Constant: '<S21>/Constant2'
   */
  localB->Sum1 = PHRControl_cal->initHip - rtu_measLeg[0];

  /* Gain: '<S249>/Derivative Gain' */
  localB->DerivativeGain = PHRControl_PageSwitching_arg->PIDController_D *
    localB->Sum1;

  /* DiscreteIntegrator: '<S250>/Filter' */
  localB->Filter = localDW->Filter_DSTATE;

  /* Sum: '<S250>/SumD' */
  localB->SumD = localB->DerivativeGain - localB->Filter;

  /* Gain: '<S252>/Integral Gain' */
  localB->IntegralGain = PHRControl_PageSwitching_arg->PIDController_I *
    localB->Sum1;

  /* DiscreteIntegrator: '<S255>/Integrator' */
  localB->Integrator = localDW->Integrator_DSTATE;

  /* Gain: '<S258>/Filter Coefficient' */
  localB->FilterCoefficient = PHRControl_PageSwitching_arg->PIDController_N *
    localB->SumD;

  /* Gain: '<S260>/Proportional Gain' */
  localB->ProportionalGain = PHRControl_PageSwitching_arg->PIDController_P *
    localB->Sum1;

  /* Sum: '<S264>/Sum' */
  localB->Sum = (localB->ProportionalGain + localB->Integrator) +
    localB->FilterCoefficient;

  /* Sum: '<S21>/Sum6' */
  localB->Sum6 = rtu_measLeg[0] + rtu_measLeg[1];

  /* Sum: '<S21>/Sum2' incorporates:
   *  Constant: '<S21>/Constant1'
   */
  localB->Sum2 = PHRControl_cal->initKnee - localB->Sum6;

  /* Gain: '<S297>/Derivative Gain' */
  localB->DerivativeGain_f = PHRControl_PageSwitching_arg->PIDController1_D *
    localB->Sum2;

  /* DiscreteIntegrator: '<S298>/Filter' */
  localB->Filter_k = localDW->Filter_DSTATE_f;

  /* Sum: '<S298>/SumD' */
  localB->SumD_l = localB->DerivativeGain_f - localB->Filter_k;

  /* Gain: '<S300>/Integral Gain' */
  localB->IntegralGain_m = PHRControl_PageSwitching_arg->PIDController1_I *
    localB->Sum2;

  /* DiscreteIntegrator: '<S303>/Integrator' */
  localB->Integrator_j = localDW->Integrator_DSTATE_g;

  /* Gain: '<S306>/Filter Coefficient' */
  localB->FilterCoefficient_i = PHRControl_PageSwitching_arg->PIDController1_N *
    localB->SumD_l;

  /* Gain: '<S308>/Proportional Gain' */
  localB->ProportionalGain_h = PHRControl_PageSwitching_arg->PIDController1_P *
    localB->Sum2;

  /* Sum: '<S312>/Sum' */
  localB->Sum_b = (localB->ProportionalGain_h + localB->Integrator_j) +
    localB->FilterCoefficient_i;

  /* Saturate: '<S21>/Saturation2' */
  u1 = -PHRControl_cal->max_torque;
  u0 = localB->Sum;
  u2 = PHRControl_cal->max_torque;
  if (u0 > u2) {
    u0 = u2;
  } else if (u0 < u1) {
    u0 = u1;
  }

  *rty_T1 = u0;

  /* End of Saturate: '<S21>/Saturation2' */

  /* Saturate: '<S21>/Saturation3' */
  u1 = -PHRControl_cal->max_torque;
  u0 = localB->Sum_b;
  u2 = PHRControl_cal->max_torque;
  if (u0 > u2) {
    u0 = u2;
  } else if (u0 < u1) {
    u0 = u1;
  }

  *rty_T2 = u0;

  /* End of Saturate: '<S21>/Saturation3' */

  /* SignalConversion generated from: '<S21>/Machine_State' incorporates:
   *  Constant: '<S21>/Constant'
   */
  *rty_Machine_State = PHRControl_PageSwitching_arg->Constant_Value;

  /* Update for DiscreteIntegrator: '<S250>/Filter' */
  localDW->Filter_DSTATE += PHRControl_PageSwitching_arg->Filter_gainval *
    localB->FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S255>/Integrator' */
  localDW->Integrator_DSTATE += PHRControl_PageSwitching_arg->Integrator_gainval
    * localB->IntegralGain;

  /* Update for DiscreteIntegrator: '<S298>/Filter' */
  localDW->Filter_DSTATE_f += PHRControl_PageSwitching_arg->Filter_gainval_o *
    localB->FilterCoefficient_i;

  /* Update for DiscreteIntegrator: '<S303>/Integrator' */
  localDW->Integrator_DSTATE_g +=
    PHRControl_PageSwitching_arg->Integrator_gainval_b * localB->IntegralGain_m;
}

/*
 * Output and update for action system:
 *    '<S16>/Load'
 *    '<S17>/Load'
 *    '<S18>/Load'
 */
void PHRControl_Load(real_T *rty_T1, real_T *rty_T2, real_T *rty_Machine_State,
                     PHRControl_Load_cal_type *PHRControl_PageSwitching_arg)
{
  /* SignalConversion generated from: '<S22>/T1' incorporates:
   *  Constant: '<S22>/Constant'
   */
  *rty_T1 = PHRControl_PageSwitching_arg->Constant_Value;

  /* SignalConversion generated from: '<S22>/T2' incorporates:
   *  Constant: '<S22>/Constant1'
   */
  *rty_T2 = PHRControl_PageSwitching_arg->Constant1_Value;

  /* SignalConversion generated from: '<S22>/Machine_State' incorporates:
   *  Constant: '<S22>/Constant2'
   */
  *rty_Machine_State = PHRControl_PageSwitching_arg->Constant2_Value;
}

/*
 * Output and update for action system:
 *    '<S17>/Unload'
 *    '<S18>/Unload'
 */
void PHRControl_Unload(real_T *rty_T1, real_T *rty_T2, real_T *rty_Machine_State,
  PHRControl_Unload_cal_type *PHRControl_PageSwitching_arg)
{
  /* SignalConversion generated from: '<S325>/T1' incorporates:
   *  Constant: '<S325>/Constant'
   */
  *rty_T1 = PHRControl_PageSwitching_arg->Constant_Value;

  /* SignalConversion generated from: '<S325>/T2' incorporates:
   *  Constant: '<S325>/Constant1'
   */
  *rty_T2 = PHRControl_PageSwitching_arg->Constant1_Value;

  /* SignalConversion generated from: '<S325>/Machine_State' incorporates:
   *  Constant: '<S325>/Constant2'
   */
  *rty_Machine_State = PHRControl_PageSwitching_arg->Constant2_Value;
}

/* Model step function */
void PHRControl_step(void)
{
  real_T T_idx_0;
  real_T th1;
  int32_T s8_iter;

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.EnabledSubsystem_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Flight_SubsysRanBC_f);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.HSW_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Idle.Idle_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Load.Load_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Unload_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Compress_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Flight_SubsysRanBC_m);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Thrust_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Unload_l.Unload_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Flight_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Idle_SubsysRanBC);

  /* Reset subsysRan breadcrumbs */
  srClearBC(PHRControl_DW.Spring_SubsysRanBC);

  /* S-Function (sg_IO602_IO691_setup_s): '<Root>/CAN Setup ' */

  /* Level2 S-Function Block: '<Root>/CAN Setup ' (sg_IO602_IO691_setup_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[5];
    sfcnOutputs(rts,0);
  }

  /* Clock: '<Root>/Clock' */
  PHRControl_B.Clock = PHRControl_M->Timing.t[0];

  /* RateTransition: '<Root>/Rate Transition' */
  PHRControl_B.RateTransition = PHRControl_B.Clock;

  /* DataStoreWrite: '<Root>/Data Store Write6' */
  PHRControl_DW.simTime = PHRControl_B.RateTransition;

  /* Delay: '<Root>/Delay2' */
  PHRControl_B.Delay2 = PHRControl_DW.Delay2_DSTATE;

  /* Outputs for Iterator SubSystem: '<Root>/Read All' incorporates:
   *  WhileIterator: '<S8>/While Iterator'
   */
  s8_iter = 1;
  do {
    PHRControl_B.WhileIterator = s8_iter;

    /* Level2 S-Function Block: '<S8>/CAN Read' (sg_IO602_IO691_read_s) */
    {
      SimStruct *rts = PHRControl_M->childSfunctions[4];
      sfcnOutputs(rts,0);
    }

    {
      /* S-Function (scanunpack): '<S8>/CAN Unpack' */
      uint8_T msgReceived = 0;
      if ((6 == PHRControl_B.CANRead_o2.Length) && (PHRControl_B.CANRead_o2.ID
           != INVALID_CAN_ID) ) {
        if ((1 == PHRControl_B.CANRead_o2.ID) && (0U ==
             PHRControl_B.CANRead_o2.Extended) ) {
          msgReceived = 1;

          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 16
             *  length                  = 16
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (PHRControl_B.CANRead_o2.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (PHRControl_B.CANRead_o2.Data[1]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                PHRControl_B.CANUnpack_o1 = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 36
             *  length                  = 12
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)((uint16_T)((uint16_T)
                      (PHRControl_B.CANRead_o2.Data[4]) & (uint16_T)(0xF0U)) >>
                      4);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (PHRControl_B.CANRead_o2.Data[3]) << 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                PHRControl_B.CANUnpack_o2 = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 40
             *  length                  = 12
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (PHRControl_B.CANRead_o2.Data[5]);
                    tempValue = tempValue | (uint16_T)((uint16_T)((uint16_T)
                      (PHRControl_B.CANRead_o2.Data[4]) & (uint16_T)(0xFU)) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                PHRControl_B.CANUnpack_o3 = result;
              }
            }

            /* --------------- START Unpacking signal 3 ------------------
             *  startBit                = 0
             *  length                  = 8
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */
          }
        }
      }

      /* Status port */
      PHRControl_B.CANUnpack_o5 = msgReceived;
    }

    {
      /* S-Function (scanunpack): '<S8>/CAN Unpack1' */
      uint8_T msgReceived = 0;
      if ((6 == PHRControl_B.CANRead_o2.Length) && (PHRControl_B.CANRead_o2.ID
           != INVALID_CAN_ID) ) {
        if ((3 == PHRControl_B.CANRead_o2.ID) && (0U ==
             PHRControl_B.CANRead_o2.Extended) ) {
          msgReceived = 1;

          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 0
             *  length                  = 32
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = SIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int32_T unpackedValue = 0;

                {
                  uint32_T tempValue = (uint32_T) (0);
                  int32_T* tempValuePtr = (int32_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint32_T)
                      (PHRControl_B.CANRead_o2.Data[0]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (PHRControl_B.CANRead_o2.Data[1]) << 8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (PHRControl_B.CANRead_o2.Data[2]) << 16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (PHRControl_B.CANRead_o2.Data[3]) << 24);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                PHRControl_B.pos = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 32
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = SIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (PHRControl_B.CANRead_o2.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (PHRControl_B.CANRead_o2.Data[5]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                PHRControl_B.vel = result;
              }
            }
          }
        }
      }

      /* Status port */
      PHRControl_B.CANUnpack1_o3 = msgReceived;
    }

    {
      /* S-Function (scanunpack): '<S8>/CAN Unpack2' */
      uint8_T msgReceived = 0;
      if ((6 == PHRControl_B.CANRead_o2.Length) && (PHRControl_B.CANRead_o2.ID
           != INVALID_CAN_ID) ) {
        if ((4 == PHRControl_B.CANRead_o2.ID) && (0U ==
             PHRControl_B.CANRead_o2.Extended) ) {
          msgReceived = 1;

          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 0
             *  length                  = 32
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = SIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int32_T unpackedValue = 0;

                {
                  uint32_T tempValue = (uint32_T) (0);
                  int32_T* tempValuePtr = (int32_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint32_T)
                      (PHRControl_B.CANRead_o2.Data[0]);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (PHRControl_B.CANRead_o2.Data[1]) << 8);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (PHRControl_B.CANRead_o2.Data[2]) << 16);
                    tempValue = tempValue | (uint32_T)((uint32_T)
                      (PHRControl_B.CANRead_o2.Data[3]) << 24);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                PHRControl_B.pos_h = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 32
             *  length                  = 16
             *  desiredSignalByteLayout = LITTLEENDIAN
             *  dataType                = SIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                int16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);
                  int16_T* tempValuePtr = (int16_T*)&tempValue;

                  {
                    tempValue = tempValue | (uint16_T)
                      (PHRControl_B.CANRead_o2.Data[4]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (PHRControl_B.CANRead_o2.Data[5]) << 8);
                  }

                  unpackedValue = *tempValuePtr;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                PHRControl_B.vel_e = result;
              }
            }
          }
        }
      }

      /* Status port */
      PHRControl_B.CANUnpack2_o3 = msgReceived;
    }

    {
      /* S-Function (scanunpack): '<S8>/CAN Unpack3' */
      uint8_T msgReceived = 0;
      if ((6 == PHRControl_B.CANRead_o2.Length) && (PHRControl_B.CANRead_o2.ID
           != INVALID_CAN_ID) ) {
        if ((2 == PHRControl_B.CANRead_o2.ID) && (0U ==
             PHRControl_B.CANRead_o2.Extended) ) {
          msgReceived = 1;

          {
            /* --------------- START Unpacking signal 0 ------------------
             *  startBit                = 16
             *  length                  = 16
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (PHRControl_B.CANRead_o2.Data[2]);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (PHRControl_B.CANRead_o2.Data[1]) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                PHRControl_B.CANUnpack3_o1 = result;
              }
            }

            /* --------------- START Unpacking signal 1 ------------------
             *  startBit                = 36
             *  length                  = 12
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)((uint16_T)((uint16_T)
                      (PHRControl_B.CANRead_o2.Data[4]) & (uint16_T)(0xF0U)) >>
                      4);
                    tempValue = tempValue | (uint16_T)((uint16_T)
                      (PHRControl_B.CANRead_o2.Data[3]) << 4);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                PHRControl_B.CANUnpack3_o2 = result;
              }
            }

            /* --------------- START Unpacking signal 2 ------------------
             *  startBit                = 40
             *  length                  = 12
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            {
              real64_T outValue = 0;

              {
                uint16_T unpackedValue = 0;

                {
                  uint16_T tempValue = (uint16_T) (0);

                  {
                    tempValue = tempValue | (uint16_T)
                      (PHRControl_B.CANRead_o2.Data[5]);
                    tempValue = tempValue | (uint16_T)((uint16_T)((uint16_T)
                      (PHRControl_B.CANRead_o2.Data[4]) & (uint16_T)(0xFU)) << 8);
                  }

                  unpackedValue = tempValue;
                }

                outValue = (real64_T) (unpackedValue);
              }

              {
                real64_T result = (real64_T) outValue;
                PHRControl_B.CANUnpack3_o3 = result;
              }
            }

            /* --------------- START Unpacking signal 3 ------------------
             *  startBit                = 0
             *  length                  = 8
             *  desiredSignalByteLayout = BIGENDIAN
             *  dataType                = UNSIGNED
             *  factor                  = 1.0
             *  offset                  = 0.0
             * -----------------------------------------------------------------------*/
            /*
             * Signal is not connected or connected to terminator.
             * No unpacking code generated.
             */
          }
        }
      }

      /* Status port */
      PHRControl_B.CANUnpack3_o5 = msgReceived;
    }

    th1 = PHRControl_B.pos * 2.0 * 3.1415926535897931 / (4.0 *
      PHRControl_cal->CPR);
    PHRControl_B.z = PHRControl_cal->r * std::sin(th1) + PHRControl_cal->h;
    PHRControl_B.zd = PHRControl_B.vel * 2.0 * 3.1415926535897931 / (4.0 *
      PHRControl_cal->CPR) * PHRControl_cal->r * std::cos(th1);
    th1 = PHRControl_B.pos_h * 2.0 * 3.1415926535897931 / (4.0 *
      PHRControl_cal->CPR);
    PHRControl_B.x = PHRControl_cal->r * std::sin(th1);
    PHRControl_B.xd = PHRControl_B.vel_e / 1000.0 * PHRControl_cal->r * std::cos
      (th1);
    PHRControl_B.sf_bytesfloats.position = PHRControl_B.CANUnpack_o1;
    PHRControl_B.sf_bytesfloats.velocity = PHRControl_B.CANUnpack_o2;
    PHRControl_B.sf_bytesfloats.I_ff = PHRControl_B.CANUnpack_o3;
    PHRControl_bytesfloats(&PHRControl_B.sf_bytesfloats);
    PHRControl_B.Gain = PHRControl_cal->Gain_Gain *
      PHRControl_B.sf_bytesfloats.position;
    PHRControl_B.sf_bytesfloats1.position = PHRControl_B.CANUnpack3_o1;
    PHRControl_B.sf_bytesfloats1.velocity = PHRControl_B.CANUnpack3_o2;
    PHRControl_B.sf_bytesfloats1.I_ff = PHRControl_B.CANUnpack3_o3;
    PHRControl_bytesfloats(&PHRControl_B.sf_bytesfloats1);
    PHRControl_B.Gain2 = PHRControl_cal->Gain2_Gain *
      PHRControl_B.sf_bytesfloats1.position;
    s8_iter++;
  } while (PHRControl_B.CANRead_o1);

  /* End of Outputs for SubSystem: '<Root>/Read All' */

  /* SampleTimeMath: '<S4>/TSamp'
   *
   * About '<S4>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  PHRControl_B.TSamp = PHRControl_B.sf_bytesfloats.I_ff *
    PHRControl_cal->TSamp_WtEt;

  /* UnitDelay: '<S4>/UD' */
  PHRControl_B.Uk1 = PHRControl_DW.UD_DSTATE;

  /* Sum: '<S4>/Diff' */
  PHRControl_B.Diff = PHRControl_B.TSamp - PHRControl_B.Uk1;

  /* SampleTimeMath: '<S3>/TSamp'
   *
   * About '<S3>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  PHRControl_B.TSamp_j = PHRControl_B.sf_bytesfloats1.I_ff *
    PHRControl_cal->TSamp_WtEt_n;

  /* UnitDelay: '<S3>/UD' */
  PHRControl_B.Uk1_d = PHRControl_DW.UD_DSTATE_n;

  /* Sum: '<S3>/Diff' */
  PHRControl_B.Diff_l = PHRControl_B.TSamp_j - PHRControl_B.Uk1_d;

  /* MATLAB Function: '<Root>/Collision Detector' */
  T_idx_0 = std::abs(PHRControl_B.Diff);
  th1 = std::abs(PHRControl_B.Diff_l);
  if ((T_idx_0 >= th1) || rtIsNaN(th1)) {
    th1 = T_idx_0;
  }

  if (th1 > 50.0) {
    PHRControl_B.ground = !PHRControl_B.Delay2;
  } else {
    PHRControl_B.ground = PHRControl_B.Delay2;
  }

  /* End of MATLAB Function: '<Root>/Collision Detector' */

  /* DataStoreWrite: '<Root>/Data Store Write' */
  PHRControl_DW.Simscape_Transitions = PHRControl_B.ground;

  /* Constant: '<Root>/isReady' */
  PHRControl_B.isReady = PHRControl_cal->isReady_Value;

  /* DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  if (((PHRControl_B.isReady > 0.0) &&
       (PHRControl_DW.DiscreteTimeIntegrator_PrevResetState <= 0)) ||
      ((PHRControl_B.isReady <= 0.0) &&
       (PHRControl_DW.DiscreteTimeIntegrator_PrevResetState == 1))) {
    PHRControl_DW.DiscreteTimeIntegrator_DSTATE =
      PHRControl_cal->DiscreteTimeIntegrator_IC_m;
  }

  /* DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  PHRControl_B.DiscreteTimeIntegrator =
    PHRControl_DW.DiscreteTimeIntegrator_DSTATE;

  /* Switch: '<Root>/Switch' */
  if (PHRControl_B.DiscreteTimeIntegrator > PHRControl_cal->Switch_Threshold) {
    /* Switch: '<Root>/Switch' incorporates:
     *  Constant: '<Root>/Constant5'
     */
    PHRControl_B.Switch = PHRControl_cal->Constant5_Value;
  } else {
    /* Switch: '<Root>/Switch' incorporates:
     *  Constant: '<Root>/Constant6'
     */
    PHRControl_B.Switch = PHRControl_cal->Constant6_Value;
  }

  /* End of Switch: '<Root>/Switch' */

  /* DataStoreWrite: '<Root>/Data Store Write1' */
  PHRControl_DW.run = PHRControl_B.Switch;

  /* DataStoreWrite: '<Root>/Data Store Write4' incorporates:
   *  Constant: '<Root>/Constant10'
   */
  PHRControl_DW.knee_bend_factor = PHRControl_cal->knee_bend_factor;

  /* DataStoreWrite: '<Root>/Data Store Write2' incorporates:
   *  Constant: '<Root>/Constant7'
   */
  PHRControl_DW.hip_range = PHRControl_cal->hip_fuzz;

  /* DataStoreWrite: '<Root>/Data Store Write5' incorporates:
   *  Constant: '<Root>/Constant8'
   */
  PHRControl_DW.hip_bend_factor = PHRControl_cal->hip_bend_factor;

  /* DataStoreWrite: '<Root>/Data Store Write3' incorporates:
   *  Constant: '<Root>/Constant9'
   */
  PHRControl_DW.knee_range = PHRControl_cal->knee_fuzz;

  /* MATLAB Function: '<Root>/Danger Detector Function' */
  PHRControl_B.danger = 0.0;
  if ((std::abs(PHRControl_B.Gain2) > 2.0) || (std::abs(PHRControl_B.Gain) > 6.0))
  {
    PHRControl_B.danger = 1.0;
  }

  /* End of MATLAB Function: '<Root>/Danger Detector Function' */

  /* Delay: '<Root>/Delay1' */
  PHRControl_B.Delay1[0] = PHRControl_DW.Delay1_DSTATE[0];
  PHRControl_B.Delay1[1] = PHRControl_DW.Delay1_DSTATE[1];

  /* Chart: '<Root>/State-Transition Diagram' incorporates:
   *  Constant: '<S19>/Knee Joint Angle1'
   *  MATLAB Function: '<S20>/Torque Calculator'
   */
  if (PHRControl_DW.temporalCounter_i1 < 1023U) {
    PHRControl_DW.temporalCounter_i1 = static_cast<uint16_T>
      (PHRControl_DW.temporalCounter_i1 + 1U);
  }

  if (PHRControl_DW.temporalCounter_i1_m < 1U) {
    PHRControl_DW.temporalCounter_i1_m = 1U;
  }

  if (PHRControl_DW.temporalCounter_i1_a < 1U) {
    PHRControl_DW.temporalCounter_i1_a = 1U;
  }

  if (PHRControl_DW.temporalCounter_i1_b < 1U) {
    PHRControl_DW.temporalCounter_i1_b = 1U;
  }

  PHRControl_DW.sfEvent = -1;
  if (PHRControl_DW.is_active_c8_PHRControl == 0U) {
    PHRControl_DW.is_active_c8_PHRControl = 1U;
    PHRControl_DW.is_c8_PHRControl = PHRControl_IN_Start;
  } else {
    switch (PHRControl_DW.is_c8_PHRControl) {
     case PHRControl_IN_Command1:
      {
        PHRControl_B.c = 3.0;
        if ((PHRControl_B.isReady == 4.0) || (PHRControl_B.danger == 1.0)) {
          switch (PHRControl_DW.is_Command1) {
           case PHRControl_IN_HSW_Controller:
            PHRControl_DW.is_c5_PHRControl = PHRControl_IN_NO_ACTIVE_CHILD;
            PHRControl_DW.is_Command1 = PHRControl_IN_NO_ACTIVE_CHILD;
            break;

           case PHRControl_IN_Raibert:
            PHRControl_DW.is_c4_PHRControl = PHRControl_IN_NO_ACTIVE_CHILD;
            PHRControl_DW.is_Command1 = PHRControl_IN_NO_ACTIVE_CHILD;
            break;

           case PHRControl_IN_SLIP:
            PHRControl_DW.is_c3_PHRControl = PHRControl_IN_NO_ACTIVE_CHILD;
            PHRControl_DW.is_Command1 = PHRControl_IN_NO_ACTIVE_CHILD;
            break;

           default:
            PHRControl_DW.is_Command1 = PHRControl_IN_NO_ACTIVE_CHILD;
            break;
          }

          PHRControl_DW.is_c8_PHRControl = PHRControl_IN_FinalShutdown;
          PHRControl_B.Kp1 = 0.0;
          PHRControl_B.Kp2 = 0.0;
          PHRControl_B.theta1 = 0.0;
          PHRControl_B.theta2 = 0.0;

          /* Merge: '<S9>/ Merge 2' */
          PHRControl_B.T1 = 0.0;

          /* Merge: '<S9>/ Merge 1' */
          PHRControl_B.T2 = 0.0;
          PHRControl_B.Kd1 = 0.1;
          PHRControl_B.Kd2 = 0.1;
        } else {
          switch (PHRControl_DW.is_Command1) {
           case PHRControl_IN_Flight:
            if (PHRControl_DW.Simscape_Transitions) {
              PHRControl_B.Kp1 = 0.0;
              PHRControl_B.Kp2 = 0.0;
              PHRControl_B.Kd1 = 0.0;
              PHRControl_B.Kd2 = 0.0;
              PHRControl_DW.is_Command1 = PHRControl_IN_Stance;
            } else {
              PHRControl_B.theta1 = PHRControl_cal->initHip;
              PHRControl_B.theta2 = PHRControl_cal->initKnee;
              PHRControl_B.Kp1 = 4.0;
              PHRControl_B.Kp2 = 4.0;
              PHRControl_B.Kd1 = 0.5;
              PHRControl_B.Kd2 = 0.5;
            }
            break;

           case PHRControl_IN_HSW_Controller:
            {
              if ((PHRControl_B.MachineState == 0.0) && (PHRControl_B.isReady ==
                   2.0)) {
                PHRControl_DW.is_c5_PHRControl = PHRControl_IN_NO_ACTIVE_CHILD;
                PHRControl_DW.is_Command1 = PHRControl_IN_Stand;
              } else {
                switch (PHRControl_DW.is_c5_PHRControl) {
                 case PHRControl_IN_Flight:
                  {
                    if (PHRControl_DW.Simscape_Transitions) {
                      PHRControl_DW.SLIP_impact_angle =
                        PHRControl_DW.foot_x_flight /
                        PHRControl_DW.foot_y_flight;
                      PHRControl_DW.SLIP_impact_angle = std::atan
                        (PHRControl_DW.SLIP_impact_angle);
                      PHRControl_DW.landingTime_g = PHRControl_DW.simTime;
                      PHRControl_DW.is_c5_PHRControl = PHRControl_IN_Load;
                      PHRControl_DW.temporalCounter_i1_b = 0U;
                      PHRControl_Load(&PHRControl_B.T1, &PHRControl_B.T2,
                                      &PHRControl_B.MachineState,
                                      &PHRControl_cal->PHRControl_Load_cal);
                    } else {
                      real_T u2;

                      /* DataStoreRead: '<S19>/Data Store Read' */
                      PHRControl_B.DataStoreRead_n = PHRControl_DW.contactTime;
                      PHRControl_B.KneeJointAngle1 = PHRControl_cal->initKnee;

                      /* Sum: '<S19>/Sum3' incorporates:
                       *  Constant: '<S19>/Constant1'
                       */
                      PHRControl_B.Sum3_h = PHRControl_cal->v_des -
                        PHRControl_B.xd;

                      /* Gain: '<S19>/Multiply' */
                      PHRControl_B.NPOffset_l = PHRControl_cal->k_landing_angle *
                        PHRControl_B.Sum3_h;

                      /* Trigonometry: '<S19>/Sin' */
                      PHRControl_B.Sin_k = std::sin(PHRControl_B.Delay1[0]);

                      /* Product: '<S19>/Product1' incorporates:
                       *  Constant: '<S19>/Constant4'
                       */
                      PHRControl_B.kneex_e = PHRControl_B.Sin_k *
                        PHRControl_cal->upper_leg_length;

                      /* Sum: '<S19>/Sum6' */
                      PHRControl_B.Sum6_dy = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Trigonometry: '<S19>/Sin1' */
                      PHRControl_B.Sin1_bn = std::sin(PHRControl_B.Sum6_dy);

                      /* Product: '<S19>/Product2' incorporates:
                       *  Constant: '<S19>/Constant3'
                       */
                      PHRControl_B.footx_o = PHRControl_B.Sin1_bn *
                        PHRControl_cal->lower_leg_length;

                      /* Product: '<S19>/Product' */
                      PHRControl_B.NP_h = PHRControl_B.DataStoreRead_n *
                        PHRControl_B.xd;

                      /* Sum: '<S19>/Sum4' */
                      PHRControl_B.X_desired_e = PHRControl_B.NP_h +
                        PHRControl_B.NPOffset_l;

                      /* Sum: '<S19>/Sum2' */
                      PHRControl_B.x_actual_m = PHRControl_B.kneex_e +
                        PHRControl_B.footx_o;

                      /* Sum: '<S19>/Sum' */
                      PHRControl_B.error_d = PHRControl_B.X_desired_e -
                        PHRControl_B.x_actual_m;

                      /* Gain: '<S51>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_mb =
                        PHRControl_cal->PIDController_D * PHRControl_B.error_d;

                      /* DiscreteIntegrator: '<S52>/Filter' */
                      PHRControl_B.Filter_a = PHRControl_DW.Filter_DSTATE_f;

                      /* Sum: '<S52>/SumD' */
                      PHRControl_B.SumD_k = PHRControl_B.DerivativeGain_mb -
                        PHRControl_B.Filter_a;

                      /* Gain: '<S54>/Integral Gain' */
                      PHRControl_B.IntegralGain_m =
                        PHRControl_cal->PIDController_I * PHRControl_B.error_d;

                      /* DiscreteIntegrator: '<S57>/Integrator' */
                      PHRControl_B.Integrator_ob =
                        PHRControl_DW.Integrator_DSTATE_e;

                      /* Gain: '<S60>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_d =
                        PHRControl_cal->PIDController_N * PHRControl_B.SumD_k;

                      /* Gain: '<S62>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_a5 =
                        PHRControl_cal->PIDController_P * PHRControl_B.error_d;

                      /* Sum: '<S66>/Sum' */
                      PHRControl_B.Sum_a = (PHRControl_B.ProportionalGain_a5 +
                                            PHRControl_B.Integrator_ob) +
                        PHRControl_B.FilterCoefficient_d;

                      /* Sum: '<S19>/Sum5' */
                      PHRControl_B.Sum5_c = PHRControl_B.KneeJointAngle1 -
                        PHRControl_B.Sum6_dy;

                      /* Gain: '<S99>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_l =
                        PHRControl_cal->PIDController1_D * PHRControl_B.Sum5_c;

                      /* DiscreteIntegrator: '<S100>/Filter' */
                      PHRControl_B.Filter_jl = PHRControl_DW.Filter_DSTATE_fx;

                      /* Sum: '<S100>/SumD' */
                      PHRControl_B.SumD_ct = PHRControl_B.DerivativeGain_l -
                        PHRControl_B.Filter_jl;

                      /* Gain: '<S102>/Integral Gain' */
                      PHRControl_B.IntegralGain_j =
                        PHRControl_cal->PIDController1_I * PHRControl_B.Sum5_c;

                      /* DiscreteIntegrator: '<S105>/Integrator' */
                      PHRControl_B.Integrator_p =
                        PHRControl_DW.Integrator_DSTATE_a1;

                      /* Gain: '<S108>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_hu =
                        PHRControl_cal->PIDController1_N * PHRControl_B.SumD_ct;

                      /* Gain: '<S110>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_cc =
                        PHRControl_cal->PIDController1_P * PHRControl_B.Sum5_c;

                      /* Sum: '<S114>/Sum' */
                      PHRControl_B.Sum_bj = (PHRControl_B.ProportionalGain_cc +
                        PHRControl_B.Integrator_p) +
                        PHRControl_B.FilterCoefficient_hu;

                      /* Saturate: '<S19>/Saturation2' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Sum_a;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = T_idx_0;
                      }

                      /* Saturate: '<S19>/Saturation3' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Sum_bj;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = T_idx_0;
                      }

                      /* Merge: '<S9>/ Merge ' incorporates:
                       *  Constant: '<S19>/Constant2'
                       *  SignalConversion generated from: '<S19>/Machine_State'
                       */
                      PHRControl_B.MachineState =
                        PHRControl_cal->Constant2_Value;

                      /* Update for DiscreteIntegrator: '<S52>/Filter' */
                      PHRControl_DW.Filter_DSTATE_f +=
                        PHRControl_cal->Filter_gainval *
                        PHRControl_B.FilterCoefficient_d;

                      /* Update for DiscreteIntegrator: '<S57>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_e +=
                        PHRControl_cal->Integrator_gainval *
                        PHRControl_B.IntegralGain_m;

                      /* Update for DiscreteIntegrator: '<S100>/Filter' */
                      PHRControl_DW.Filter_DSTATE_fx +=
                        PHRControl_cal->Filter_gainval_e *
                        PHRControl_B.FilterCoefficient_hu;

                      /* Update for DiscreteIntegrator: '<S105>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_a1 +=
                        PHRControl_cal->Integrator_gainval_l *
                        PHRControl_B.IntegralGain_j;
                    }
                  }
                  break;

                 case PHRControl_IN_HSW:
                  {
                    if (!PHRControl_DW.Simscape_Transitions) {
                      PHRControl_DW.is_c5_PHRControl = PHRControl_IN_Unload;
                      PHRControl_DW.temporalCounter_i1_b = 0U;

                      /* Merge: '<S9>/ Merge 2' incorporates:
                       *  Constant: '<S23>/Constant'
                       *  SignalConversion generated from: '<S23>/T1'
                       */
                      PHRControl_B.T1 = PHRControl_cal->Constant_Value;

                      /* Merge: '<S9>/ Merge 1' incorporates:
                       *  Constant: '<S23>/Constant1'
                       *  SignalConversion generated from: '<S23>/T2'
                       */
                      PHRControl_B.T2 = PHRControl_cal->Constant1_Value;

                      /* Merge: '<S9>/ Merge ' incorporates:
                       *  Constant: '<S23>/Constant2'
                       *  SignalConversion generated from: '<S23>/Machine_State'
                       */
                      PHRControl_B.MachineState =
                        PHRControl_cal->Constant2_Value_n;
                    } else {
                      real_T T_idx_1;
                      real_T c12;
                      real_T tmp;
                      real_T tmp_0;
                      real_T u2;

                      /* DataStoreRead: '<S20>/Data Store Read' */
                      PHRControl_B.DataStoreRead_h = true;

                      /* DataStoreRead: '<S20>/Data Store Read1' */
                      PHRControl_B.DataStoreRead1 = PHRControl_DW.contactTime;

                      /* Switch: '<S20>/Switch2' incorporates:
                       *  Constant: '<S20>/Constant8'
                       */
                      PHRControl_B.Switch2 = PHRControl_cal->Constant8_Value;

                      /* DiscreteIntegrator: '<S20>/Discrete-Time Integrator' */
                      if ((PHRControl_B.Switch2 > 0.0) &&
                          (PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c
                           <= 0)) {
                        PHRControl_DW.DiscreteTimeIntegrator_DSTATE_c =
                          PHRControl_cal->DiscreteTimeIntegrator_IC;
                      }

                      /* DiscreteIntegrator: '<S20>/Discrete-Time Integrator' */
                      PHRControl_B.DiscreteTimeIntegrator_f =
                        PHRControl_DW.DiscreteTimeIntegrator_DSTATE_c;

                      /* Sum: '<S20>/Sum' incorporates:
                       *  Constant: '<S20>/Constant5'
                       */
                      PHRControl_B.Sum_lf = PHRControl_cal->z_des -
                        PHRControl_B.z;

                      /* Gain: '<S151>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_f =
                        PHRControl_cal->PIDController2_D * PHRControl_B.Sum_lf;

                      /* DiscreteIntegrator: '<S152>/Filter' */
                      PHRControl_B.Filter_i = PHRControl_DW.Filter_DSTATE_ct;

                      /* Sum: '<S152>/SumD' */
                      PHRControl_B.SumD_c = PHRControl_B.DerivativeGain_f -
                        PHRControl_B.Filter_i;

                      /* Gain: '<S154>/Integral Gain' */
                      PHRControl_B.IntegralGain_a =
                        PHRControl_cal->PIDController2_I * PHRControl_B.Sum_lf;

                      /* DiscreteIntegrator: '<S157>/Integrator' */
                      PHRControl_B.Integrator_c =
                        PHRControl_DW.Integrator_DSTATE_l;

                      /* Gain: '<S160>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_m =
                        PHRControl_cal->PIDController2_N * PHRControl_B.SumD_c;

                      /* Gain: '<S162>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_a =
                        PHRControl_cal->PIDController2_P * PHRControl_B.Sum_lf;

                      /* Sum: '<S166>/Sum' */
                      PHRControl_B.Sum_hl = (PHRControl_B.ProportionalGain_a +
                        PHRControl_B.Integrator_c) +
                        PHRControl_B.FilterCoefficient_m;

                      /* Sum: '<S20>/Sum2' incorporates:
                       *  Constant: '<S20>/Constant10'
                       */
                      PHRControl_B.Sum2_p = PHRControl_cal->v_des -
                        PHRControl_B.xd;

                      /* Gain: '<S199>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_g =
                        PHRControl_cal->PIDController3_D * PHRControl_B.Sum2_p;

                      /* DiscreteIntegrator: '<S200>/Filter' */
                      PHRControl_B.Filter_d = PHRControl_DW.Filter_DSTATE_h;

                      /* Sum: '<S200>/SumD' */
                      PHRControl_B.SumD_en = PHRControl_B.DerivativeGain_g -
                        PHRControl_B.Filter_d;

                      /* Gain: '<S202>/Integral Gain' */
                      PHRControl_B.IntegralGain_pj =
                        PHRControl_cal->PIDController3_I * PHRControl_B.Sum2_p;

                      /* DiscreteIntegrator: '<S205>/Integrator' */
                      PHRControl_B.Integrator_g =
                        PHRControl_DW.Integrator_DSTATE_a;

                      /* Gain: '<S208>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_hx =
                        PHRControl_cal->PIDController3_N * PHRControl_B.SumD_en;

                      /* Gain: '<S210>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_p =
                        PHRControl_cal->PIDController3_P * PHRControl_B.Sum2_p;

                      /* Sum: '<S214>/Sum' */
                      PHRControl_B.Sum_d = (PHRControl_B.ProportionalGain_p +
                                            PHRControl_B.Integrator_g) +
                        PHRControl_B.FilterCoefficient_hx;

                      /* MATLAB Function: '<S20>/Sine Generator' incorporates:
                       *  Constant: '<S20>/Constant'
                       */
                      if (PHRControl_B.DataStoreRead1 != 0.0) {
                        PHRControl_B.HSW = std::sin(3.1415926535897931 *
                          PHRControl_B.DiscreteTimeIntegrator_f /
                          PHRControl_B.DataStoreRead1) *
                          PHRControl_cal->amplitude;
                      } else {
                        PHRControl_B.HSW = std::sin(3.1415926535897931 *
                          PHRControl_B.DiscreteTimeIntegrator_f / 0.3) * 0.3;
                      }

                      /* Sum: '<S20>/Sum4' */
                      PHRControl_B.Sum4_f = PHRControl_B.HSW +
                        PHRControl_B.Sum_hl;

                      /* Sum: '<S20>/Sum6' */
                      PHRControl_B.Sum6_dr = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* MATLAB Function: '<S20>/Torque Calculator' */
                      th1 = std::sin(PHRControl_B.Delay1[0] +
                                     PHRControl_B.Sum6_dr);
                      c12 = std::cos(PHRControl_B.Delay1[0] +
                                     PHRControl_B.Sum6_dr);
                      T_idx_0 = std::sin(PHRControl_B.Delay1[0]);
                      u2 = std::cos(PHRControl_B.Delay1[0]);
                      T_idx_0 = -PHRControl_cal->upper_leg_length * T_idx_0 -
                        PHRControl_cal->lower_leg_length * th1;
                      th1 *= -PHRControl_cal->lower_leg_length;
                      u2 = PHRControl_cal->upper_leg_length * u2 +
                        PHRControl_cal->lower_leg_length * c12;
                      c12 *= PHRControl_cal->lower_leg_length;
                      tmp = PHRControl_B.Sum_d;
                      tmp_0 = PHRControl_B.Sum4_f;
                      T_idx_1 = T_idx_0 * tmp;
                      T_idx_1 += u2 * tmp_0;
                      T_idx_0 = T_idx_1;

                      /* MATLAB Function: '<S20>/Torque Calculator' */
                      T_idx_1 = th1 * tmp;
                      T_idx_1 += c12 * tmp_0;
                      T_idx_0 = -T_idx_0;
                      PHRControl_B.T1_f = T_idx_0;
                      PHRControl_B.T2_e = T_idx_1;

                      /* Saturate: '<S20>/Saturation' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.T1_f;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = T_idx_0;
                      }

                      /* Saturate: '<S20>/Saturation1' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.T2_e;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = T_idx_0;
                      }

                      /* Merge: '<S9>/ Merge ' incorporates:
                       *  Constant: '<S20>/Constant2'
                       *  SignalConversion generated from: '<S20>/Machine_State'
                       */
                      PHRControl_B.MachineState =
                        PHRControl_cal->Constant2_Value_h;

                      /* Update for DiscreteIntegrator: '<S20>/Discrete-Time Integrator' incorporates:
                       *  Constant: '<S20>/Constant7'
                       */
                      PHRControl_DW.DiscreteTimeIntegrator_DSTATE_c +=
                        PHRControl_cal->DiscreteTimeIntegrator_gainval *
                        PHRControl_cal->Constant7_Value;
                      if (PHRControl_B.Switch2 > 0.0) {
                        PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c =
                          1;
                      } else if (PHRControl_B.Switch2 < 0.0) {
                        PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c =
                          -1;
                      } else if (PHRControl_B.Switch2 == 0.0) {
                        PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c =
                          0;
                      } else {
                        PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c =
                          2;
                      }

                      /* Update for DiscreteIntegrator: '<S152>/Filter' */
                      PHRControl_DW.Filter_DSTATE_ct +=
                        PHRControl_cal->Filter_gainval_f *
                        PHRControl_B.FilterCoefficient_m;

                      /* Update for DiscreteIntegrator: '<S157>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_l +=
                        PHRControl_cal->Integrator_gainval_b *
                        PHRControl_B.IntegralGain_a;

                      /* Update for DiscreteIntegrator: '<S200>/Filter' */
                      PHRControl_DW.Filter_DSTATE_h +=
                        PHRControl_cal->Filter_gainval_b *
                        PHRControl_B.FilterCoefficient_hx;

                      /* Update for DiscreteIntegrator: '<S205>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_a +=
                        PHRControl_cal->Integrator_gainval_a *
                        PHRControl_B.IntegralGain_pj;
                    }
                  }
                  break;

                 case PHRControl_IN_Idle:
                  if (PHRControl_DW.run == 1.0) {
                    PHRControl_DW.is_c5_PHRControl = PHRControl_IN_Load;
                    PHRControl_DW.temporalCounter_i1_b = 0U;
                    PHRControl_Load(&PHRControl_B.T1, &PHRControl_B.T2,
                                    &PHRControl_B.MachineState,
                                    &PHRControl_cal->PHRControl_Load_cal);
                  } else {
                    PHRControl_Idle(PHRControl_B.Delay1, &PHRControl_B.T1,
                                    &PHRControl_B.T2, &PHRControl_B.MachineState,
                                    &PHRControl_B.Idle, &PHRControl_DW.Idle,
                                    &PHRControl_cal->PHRControl_Idle_cal);
                  }
                  break;

                 case PHRControl_IN_Load:
                  {
                    real_T T_idx_1;
                    real_T c12;
                    real_T tmp;
                    real_T tmp_0;
                    real_T u2;
                    PHRControl_DW.is_c5_PHRControl = PHRControl_IN_HSW;

                    /* DataStoreRead: '<S20>/Data Store Read' */
                    PHRControl_B.DataStoreRead_h =
                      PHRControl_DW.Simscape_Transitions;

                    /* DataStoreRead: '<S20>/Data Store Read1' */
                    PHRControl_B.DataStoreRead1 = PHRControl_DW.contactTime;

                    /* Switch: '<S20>/Switch2' */
                    if (PHRControl_B.DataStoreRead_h) {
                      /* Switch: '<S20>/Switch2' incorporates:
                       *  Constant: '<S20>/Constant8'
                       */
                      PHRControl_B.Switch2 = PHRControl_cal->Constant8_Value;
                    } else {
                      /* Switch: '<S20>/Switch2' incorporates:
                       *  Constant: '<S20>/Constant9'
                       */
                      PHRControl_B.Switch2 = PHRControl_cal->Constant9_Value;
                    }

                    /* DiscreteIntegrator: '<S20>/Discrete-Time Integrator' */
                    if ((PHRControl_B.Switch2 > 0.0) &&
                        (PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c <=
                         0)) {
                      PHRControl_DW.DiscreteTimeIntegrator_DSTATE_c =
                        PHRControl_cal->DiscreteTimeIntegrator_IC;
                    }

                    /* DiscreteIntegrator: '<S20>/Discrete-Time Integrator' */
                    PHRControl_B.DiscreteTimeIntegrator_f =
                      PHRControl_DW.DiscreteTimeIntegrator_DSTATE_c;

                    /* Sum: '<S20>/Sum' incorporates:
                     *  Constant: '<S20>/Constant5'
                     */
                    PHRControl_B.Sum_lf = PHRControl_cal->z_des - PHRControl_B.z;

                    /* Gain: '<S151>/Derivative Gain' */
                    PHRControl_B.DerivativeGain_f =
                      PHRControl_cal->PIDController2_D * PHRControl_B.Sum_lf;

                    /* DiscreteIntegrator: '<S152>/Filter' */
                    PHRControl_B.Filter_i = PHRControl_DW.Filter_DSTATE_ct;

                    /* Sum: '<S152>/SumD' */
                    PHRControl_B.SumD_c = PHRControl_B.DerivativeGain_f -
                      PHRControl_B.Filter_i;

                    /* Gain: '<S154>/Integral Gain' */
                    PHRControl_B.IntegralGain_a =
                      PHRControl_cal->PIDController2_I * PHRControl_B.Sum_lf;

                    /* DiscreteIntegrator: '<S157>/Integrator' */
                    PHRControl_B.Integrator_c =
                      PHRControl_DW.Integrator_DSTATE_l;

                    /* Gain: '<S160>/Filter Coefficient' */
                    PHRControl_B.FilterCoefficient_m =
                      PHRControl_cal->PIDController2_N * PHRControl_B.SumD_c;

                    /* Gain: '<S162>/Proportional Gain' */
                    PHRControl_B.ProportionalGain_a =
                      PHRControl_cal->PIDController2_P * PHRControl_B.Sum_lf;

                    /* Sum: '<S166>/Sum' */
                    PHRControl_B.Sum_hl = (PHRControl_B.ProportionalGain_a +
                      PHRControl_B.Integrator_c) +
                      PHRControl_B.FilterCoefficient_m;

                    /* Sum: '<S20>/Sum2' incorporates:
                     *  Constant: '<S20>/Constant10'
                     */
                    PHRControl_B.Sum2_p = PHRControl_cal->v_des -
                      PHRControl_B.xd;

                    /* Gain: '<S199>/Derivative Gain' */
                    PHRControl_B.DerivativeGain_g =
                      PHRControl_cal->PIDController3_D * PHRControl_B.Sum2_p;

                    /* DiscreteIntegrator: '<S200>/Filter' */
                    PHRControl_B.Filter_d = PHRControl_DW.Filter_DSTATE_h;

                    /* Sum: '<S200>/SumD' */
                    PHRControl_B.SumD_en = PHRControl_B.DerivativeGain_g -
                      PHRControl_B.Filter_d;

                    /* Gain: '<S202>/Integral Gain' */
                    PHRControl_B.IntegralGain_pj =
                      PHRControl_cal->PIDController3_I * PHRControl_B.Sum2_p;

                    /* DiscreteIntegrator: '<S205>/Integrator' */
                    PHRControl_B.Integrator_g =
                      PHRControl_DW.Integrator_DSTATE_a;

                    /* Gain: '<S208>/Filter Coefficient' */
                    PHRControl_B.FilterCoefficient_hx =
                      PHRControl_cal->PIDController3_N * PHRControl_B.SumD_en;

                    /* Gain: '<S210>/Proportional Gain' */
                    PHRControl_B.ProportionalGain_p =
                      PHRControl_cal->PIDController3_P * PHRControl_B.Sum2_p;

                    /* Sum: '<S214>/Sum' */
                    PHRControl_B.Sum_d = (PHRControl_B.ProportionalGain_p +
                                          PHRControl_B.Integrator_g) +
                      PHRControl_B.FilterCoefficient_hx;

                    /* MATLAB Function: '<S20>/Sine Generator' incorporates:
                     *  Constant: '<S20>/Constant'
                     */
                    if (PHRControl_B.DataStoreRead1 != 0.0) {
                      PHRControl_B.HSW = std::sin(3.1415926535897931 *
                        PHRControl_B.DiscreteTimeIntegrator_f /
                        PHRControl_B.DataStoreRead1) * PHRControl_cal->amplitude;
                    } else {
                      PHRControl_B.HSW = std::sin(3.1415926535897931 *
                        PHRControl_B.DiscreteTimeIntegrator_f / 0.3) * 0.3;
                    }

                    /* Sum: '<S20>/Sum4' */
                    PHRControl_B.Sum4_f = PHRControl_B.HSW + PHRControl_B.Sum_hl;

                    /* Sum: '<S20>/Sum6' */
                    PHRControl_B.Sum6_dr = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* MATLAB Function: '<S20>/Torque Calculator' */
                    th1 = std::sin(PHRControl_B.Delay1[0] + PHRControl_B.Sum6_dr);
                    c12 = std::cos(PHRControl_B.Delay1[0] + PHRControl_B.Sum6_dr);
                    T_idx_0 = std::sin(PHRControl_B.Delay1[0]);
                    u2 = std::cos(PHRControl_B.Delay1[0]);
                    T_idx_0 = -PHRControl_cal->upper_leg_length * T_idx_0 -
                      PHRControl_cal->lower_leg_length * th1;
                    th1 *= -PHRControl_cal->lower_leg_length;
                    u2 = PHRControl_cal->upper_leg_length * u2 +
                      PHRControl_cal->lower_leg_length * c12;
                    c12 *= PHRControl_cal->lower_leg_length;
                    tmp = PHRControl_B.Sum_d;
                    tmp_0 = PHRControl_B.Sum4_f;
                    T_idx_1 = T_idx_0 * tmp;
                    T_idx_1 += u2 * tmp_0;
                    T_idx_0 = T_idx_1;

                    /* MATLAB Function: '<S20>/Torque Calculator' */
                    T_idx_1 = th1 * tmp;
                    T_idx_1 += c12 * tmp_0;
                    T_idx_0 = -T_idx_0;
                    PHRControl_B.T1_f = T_idx_0;
                    PHRControl_B.T2_e = T_idx_1;

                    /* Saturate: '<S20>/Saturation' */
                    th1 = -PHRControl_cal->max_torque;
                    T_idx_0 = PHRControl_B.T1_f;
                    u2 = PHRControl_cal->max_torque;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = T_idx_0;
                    }

                    /* Saturate: '<S20>/Saturation1' */
                    th1 = -PHRControl_cal->max_torque;
                    T_idx_0 = PHRControl_B.T2_e;
                    u2 = PHRControl_cal->max_torque;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = T_idx_0;
                    }

                    /* Merge: '<S9>/ Merge ' incorporates:
                     *  Constant: '<S20>/Constant2'
                     *  SignalConversion generated from: '<S20>/Machine_State'
                     */
                    PHRControl_B.MachineState =
                      PHRControl_cal->Constant2_Value_h;

                    /* Update for DiscreteIntegrator: '<S20>/Discrete-Time Integrator' incorporates:
                     *  Constant: '<S20>/Constant7'
                     */
                    PHRControl_DW.DiscreteTimeIntegrator_DSTATE_c +=
                      PHRControl_cal->DiscreteTimeIntegrator_gainval *
                      PHRControl_cal->Constant7_Value;
                    if (PHRControl_B.Switch2 > 0.0) {
                      PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c = 1;
                    } else if (PHRControl_B.Switch2 < 0.0) {
                      PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c = -1;
                    } else if (PHRControl_B.Switch2 == 0.0) {
                      PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c = 0;
                    } else {
                      PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c = 2;
                    }

                    /* Update for DiscreteIntegrator: '<S152>/Filter' */
                    PHRControl_DW.Filter_DSTATE_ct +=
                      PHRControl_cal->Filter_gainval_f *
                      PHRControl_B.FilterCoefficient_m;

                    /* Update for DiscreteIntegrator: '<S157>/Integrator' */
                    PHRControl_DW.Integrator_DSTATE_l +=
                      PHRControl_cal->Integrator_gainval_b *
                      PHRControl_B.IntegralGain_a;

                    /* Update for DiscreteIntegrator: '<S200>/Filter' */
                    PHRControl_DW.Filter_DSTATE_h +=
                      PHRControl_cal->Filter_gainval_b *
                      PHRControl_B.FilterCoefficient_hx;

                    /* Update for DiscreteIntegrator: '<S205>/Integrator' */
                    PHRControl_DW.Integrator_DSTATE_a +=
                      PHRControl_cal->Integrator_gainval_a *
                      PHRControl_B.IntegralGain_pj;
                  }
                  break;

                 default:
                  {
                    real_T u2;

                    /* case IN_Unload: */
                    PHRControl_DW.contactTime = PHRControl_DW.simTime -
                      PHRControl_DW.landingTime_g;
                    PHRControl_DW.hasHopped = 1.0;
                    PHRControl_DW.is_c5_PHRControl = PHRControl_IN_Flight;

                    /* DataStoreRead: '<S19>/Data Store Read' */
                    PHRControl_B.DataStoreRead_n = PHRControl_DW.contactTime;
                    PHRControl_B.KneeJointAngle1 = PHRControl_cal->initKnee;

                    /* Sum: '<S19>/Sum3' incorporates:
                     *  Constant: '<S19>/Constant1'
                     */
                    PHRControl_B.Sum3_h = PHRControl_cal->v_des -
                      PHRControl_B.xd;

                    /* Gain: '<S19>/Multiply' */
                    PHRControl_B.NPOffset_l = PHRControl_cal->k_landing_angle *
                      PHRControl_B.Sum3_h;

                    /* Trigonometry: '<S19>/Sin' */
                    PHRControl_B.Sin_k = std::sin(PHRControl_B.Delay1[0]);

                    /* Product: '<S19>/Product1' incorporates:
                     *  Constant: '<S19>/Constant4'
                     */
                    PHRControl_B.kneex_e = PHRControl_B.Sin_k *
                      PHRControl_cal->upper_leg_length;

                    /* Sum: '<S19>/Sum6' */
                    PHRControl_B.Sum6_dy = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Trigonometry: '<S19>/Sin1' */
                    PHRControl_B.Sin1_bn = std::sin(PHRControl_B.Sum6_dy);

                    /* Product: '<S19>/Product2' incorporates:
                     *  Constant: '<S19>/Constant3'
                     */
                    PHRControl_B.footx_o = PHRControl_B.Sin1_bn *
                      PHRControl_cal->lower_leg_length;

                    /* Product: '<S19>/Product' */
                    PHRControl_B.NP_h = PHRControl_B.DataStoreRead_n *
                      PHRControl_B.xd;

                    /* Sum: '<S19>/Sum4' */
                    PHRControl_B.X_desired_e = PHRControl_B.NP_h +
                      PHRControl_B.NPOffset_l;

                    /* Sum: '<S19>/Sum2' */
                    PHRControl_B.x_actual_m = PHRControl_B.kneex_e +
                      PHRControl_B.footx_o;

                    /* Sum: '<S19>/Sum' */
                    PHRControl_B.error_d = PHRControl_B.X_desired_e -
                      PHRControl_B.x_actual_m;

                    /* Gain: '<S51>/Derivative Gain' */
                    PHRControl_B.DerivativeGain_mb =
                      PHRControl_cal->PIDController_D * PHRControl_B.error_d;

                    /* DiscreteIntegrator: '<S52>/Filter' */
                    PHRControl_B.Filter_a = PHRControl_DW.Filter_DSTATE_f;

                    /* Sum: '<S52>/SumD' */
                    PHRControl_B.SumD_k = PHRControl_B.DerivativeGain_mb -
                      PHRControl_B.Filter_a;

                    /* Gain: '<S54>/Integral Gain' */
                    PHRControl_B.IntegralGain_m =
                      PHRControl_cal->PIDController_I * PHRControl_B.error_d;

                    /* DiscreteIntegrator: '<S57>/Integrator' */
                    PHRControl_B.Integrator_ob =
                      PHRControl_DW.Integrator_DSTATE_e;

                    /* Gain: '<S60>/Filter Coefficient' */
                    PHRControl_B.FilterCoefficient_d =
                      PHRControl_cal->PIDController_N * PHRControl_B.SumD_k;

                    /* Gain: '<S62>/Proportional Gain' */
                    PHRControl_B.ProportionalGain_a5 =
                      PHRControl_cal->PIDController_P * PHRControl_B.error_d;

                    /* Sum: '<S66>/Sum' */
                    PHRControl_B.Sum_a = (PHRControl_B.ProportionalGain_a5 +
                                          PHRControl_B.Integrator_ob) +
                      PHRControl_B.FilterCoefficient_d;

                    /* Sum: '<S19>/Sum5' */
                    PHRControl_B.Sum5_c = PHRControl_B.KneeJointAngle1 -
                      PHRControl_B.Sum6_dy;

                    /* Gain: '<S99>/Derivative Gain' */
                    PHRControl_B.DerivativeGain_l =
                      PHRControl_cal->PIDController1_D * PHRControl_B.Sum5_c;

                    /* DiscreteIntegrator: '<S100>/Filter' */
                    PHRControl_B.Filter_jl = PHRControl_DW.Filter_DSTATE_fx;

                    /* Sum: '<S100>/SumD' */
                    PHRControl_B.SumD_ct = PHRControl_B.DerivativeGain_l -
                      PHRControl_B.Filter_jl;

                    /* Gain: '<S102>/Integral Gain' */
                    PHRControl_B.IntegralGain_j =
                      PHRControl_cal->PIDController1_I * PHRControl_B.Sum5_c;

                    /* DiscreteIntegrator: '<S105>/Integrator' */
                    PHRControl_B.Integrator_p =
                      PHRControl_DW.Integrator_DSTATE_a1;

                    /* Gain: '<S108>/Filter Coefficient' */
                    PHRControl_B.FilterCoefficient_hu =
                      PHRControl_cal->PIDController1_N * PHRControl_B.SumD_ct;

                    /* Gain: '<S110>/Proportional Gain' */
                    PHRControl_B.ProportionalGain_cc =
                      PHRControl_cal->PIDController1_P * PHRControl_B.Sum5_c;

                    /* Sum: '<S114>/Sum' */
                    PHRControl_B.Sum_bj = (PHRControl_B.ProportionalGain_cc +
                      PHRControl_B.Integrator_p) +
                      PHRControl_B.FilterCoefficient_hu;

                    /* Saturate: '<S19>/Saturation2' */
                    th1 = -PHRControl_cal->max_torque;
                    T_idx_0 = PHRControl_B.Sum_a;
                    u2 = PHRControl_cal->max_torque;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = T_idx_0;
                    }

                    /* Saturate: '<S19>/Saturation3' */
                    th1 = -PHRControl_cal->max_torque;
                    T_idx_0 = PHRControl_B.Sum_bj;
                    u2 = PHRControl_cal->max_torque;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = T_idx_0;
                    }

                    /* Merge: '<S9>/ Merge ' incorporates:
                     *  Constant: '<S19>/Constant2'
                     *  SignalConversion generated from: '<S19>/Machine_State'
                     */
                    PHRControl_B.MachineState = PHRControl_cal->Constant2_Value;

                    /* Update for DiscreteIntegrator: '<S52>/Filter' */
                    PHRControl_DW.Filter_DSTATE_f +=
                      PHRControl_cal->Filter_gainval *
                      PHRControl_B.FilterCoefficient_d;

                    /* Update for DiscreteIntegrator: '<S57>/Integrator' */
                    PHRControl_DW.Integrator_DSTATE_e +=
                      PHRControl_cal->Integrator_gainval *
                      PHRControl_B.IntegralGain_m;

                    /* Update for DiscreteIntegrator: '<S100>/Filter' */
                    PHRControl_DW.Filter_DSTATE_fx +=
                      PHRControl_cal->Filter_gainval_e *
                      PHRControl_B.FilterCoefficient_hu;

                    /* Update for DiscreteIntegrator: '<S105>/Integrator' */
                    PHRControl_DW.Integrator_DSTATE_a1 +=
                      PHRControl_cal->Integrator_gainval_l *
                      PHRControl_B.IntegralGain_j;
                  }
                  break;
                }
              }
            }
            break;

           case PHRControl_IN_Raibert:
            {
              if ((PHRControl_B.MachineState == 0.0) && (PHRControl_B.isReady ==
                   2.0)) {
                PHRControl_DW.is_c4_PHRControl = PHRControl_IN_NO_ACTIVE_CHILD;
                PHRControl_DW.is_Command1 = PHRControl_IN_Stand;
              } else {
                switch (PHRControl_DW.is_c4_PHRControl) {
                 case PHRControl_IN_Compress:
                  {
                    if (((PHRControl_DW.hip_range + 1.0) * std::abs(std::abs
                          (PHRControl_DW.Hip_Impact_Angle) -
                          PHRControl_DW.hip_bend_factor) > std::abs
                         (PHRControl_B.Delay1[0])) && (std::abs
                         (PHRControl_B.Delay1[0]) > (1.0 -
                          PHRControl_DW.hip_range) * std::abs(std::abs
                          (PHRControl_DW.Hip_Impact_Angle) -
                          PHRControl_DW.hip_bend_factor))) {
                      real_T u2;
                      PHRControl_DW.is_c4_PHRControl = PHRControl_IN_Thrust;

                      /* DataStoreRead: '<S324>/Data Store Read' */
                      PHRControl_B.impactangle = PHRControl_DW.Hip_Impact_Angle;

                      /* Sum: '<S324>/Sum2' incorporates:
                       *  Constant: '<S324>/Constant'
                       */
                      PHRControl_B.Sum2_o = PHRControl_cal->desired_speed -
                        PHRControl_B.xd;

                      /* Gain: '<S324>/Gain' */
                      PHRControl_B.desiredspeed = PHRControl_cal->Gain_Gain_k *
                        PHRControl_B.Sum2_o;

                      /* Sum: '<S324>/Sum' */
                      PHRControl_B.Sum_l = PHRControl_B.impactangle +
                        PHRControl_B.desiredspeed;

                      /* Gain: '<S597>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_m =
                        PHRControl_cal->PIDController2_D_m * PHRControl_B.Sum_l;

                      /* DiscreteIntegrator: '<S598>/Filter' */
                      PHRControl_B.Filter_c = PHRControl_DW.Filter_DSTATE_k;

                      /* Sum: '<S598>/SumD' */
                      PHRControl_B.SumD_m = PHRControl_B.DerivativeGain_m -
                        PHRControl_B.Filter_c;

                      /* Gain: '<S600>/Integral Gain' */
                      PHRControl_B.IntegralGain_k =
                        PHRControl_cal->PIDController2_I_c * PHRControl_B.Sum_l;

                      /* DiscreteIntegrator: '<S603>/Integrator' */
                      PHRControl_B.Integrator_k =
                        PHRControl_DW.Integrator_DSTATE_c;

                      /* Gain: '<S606>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_l =
                        PHRControl_cal->PIDController2_N_j * PHRControl_B.SumD_m;

                      /* Gain: '<S608>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_m =
                        PHRControl_cal->PIDController2_P_o * PHRControl_B.Sum_l;

                      /* Sum: '<S612>/Sum' */
                      PHRControl_B.Sum_e = (PHRControl_B.ProportionalGain_m +
                                            PHRControl_B.Integrator_k) +
                        PHRControl_B.FilterCoefficient_l;

                      /* Saturate: '<S324>/Saturation1' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Sum_e;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = T_idx_0;
                      }

                      /* Merge: '<S9>/ Merge 1' incorporates:
                       *  Constant: '<S324>/Constant1'
                       *  SignalConversion generated from: '<S324>/T2'
                       */
                      PHRControl_B.T2 = PHRControl_cal->max_torque;

                      /* Merge: '<S9>/ Merge ' incorporates:
                       *  Constant: '<S324>/Constant2'
                       *  SignalConversion generated from: '<S324>/Machine_State'
                       */
                      PHRControl_B.MachineState =
                        PHRControl_cal->Constant2_Value_p;

                      /* Update for DiscreteIntegrator: '<S598>/Filter' */
                      PHRControl_DW.Filter_DSTATE_k +=
                        PHRControl_cal->Filter_gainval_ew *
                        PHRControl_B.FilterCoefficient_l;

                      /* Update for DiscreteIntegrator: '<S603>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_c +=
                        PHRControl_cal->Integrator_gainval_e *
                        PHRControl_B.IntegralGain_k;
                    } else {
                      real_T u2;

                      /* DataStoreRead: '<S320>/Data Store Read' */
                      PHRControl_B.DataStoreRead_e =
                        PHRControl_DW.Hip_Impact_Angle;

                      /* Sum: '<S320>/Sum3' incorporates:
                       *  Constant: '<S320>/Constant4'
                       */
                      PHRControl_B.Sum3_n = PHRControl_B.DataStoreRead_e -
                        PHRControl_cal->hip_bend_factor;

                      /* Sum: '<S320>/Sum2' */
                      PHRControl_B.Sum2_a = PHRControl_B.Sum3_n -
                        PHRControl_B.Delay1[0];

                      /* Gain: '<S352>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_h =
                        PHRControl_cal->PIDController2_D_l * PHRControl_B.Sum2_a;

                      /* DiscreteIntegrator: '<S353>/Filter' */
                      PHRControl_B.Filter_f = PHRControl_DW.Filter_DSTATE_ak;

                      /* Sum: '<S353>/SumD' */
                      PHRControl_B.SumD_fv = PHRControl_B.DerivativeGain_h -
                        PHRControl_B.Filter_f;

                      /* Gain: '<S355>/Integral Gain' */
                      PHRControl_B.IntegralGain_h =
                        PHRControl_cal->PIDController2_I_i * PHRControl_B.Sum2_a;

                      /* DiscreteIntegrator: '<S358>/Integrator' */
                      PHRControl_B.Integrator_o =
                        PHRControl_DW.Integrator_DSTATE_oc;

                      /* Gain: '<S361>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_c =
                        PHRControl_cal->PIDController2_N_l *
                        PHRControl_B.SumD_fv;

                      /* Gain: '<S363>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_mv =
                        PHRControl_cal->PIDController2_P_i * PHRControl_B.Sum2_a;

                      /* Sum: '<S367>/Sum' */
                      PHRControl_B.Sum_g = (PHRControl_B.ProportionalGain_mv +
                                            PHRControl_B.Integrator_o) +
                        PHRControl_B.FilterCoefficient_c;

                      /* Saturate: '<S320>/Saturation1' */
                      T_idx_0 = PHRControl_B.Sum_g;
                      th1 = PHRControl_cal->Saturation1_LowerSat;
                      u2 = PHRControl_cal->Saturation1_UpperSat;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = T_idx_0;
                      }

                      /* Merge: '<S9>/ Merge 1' incorporates:
                       *  Constant: '<S320>/Constant'
                       *  SignalConversion generated from: '<S320>/T2'
                       */
                      PHRControl_B.T2 = PHRControl_cal->Constant_Value_f;

                      /* Merge: '<S9>/ Merge ' incorporates:
                       *  Constant: '<S320>/Constant2'
                       *  SignalConversion generated from: '<S320>/Machine_State'
                       */
                      PHRControl_B.MachineState =
                        PHRControl_cal->Constant2_Value_e;

                      /* Update for DiscreteIntegrator: '<S353>/Filter' */
                      PHRControl_DW.Filter_DSTATE_ak +=
                        PHRControl_cal->Filter_gainval_c *
                        PHRControl_B.FilterCoefficient_c;

                      /* Update for DiscreteIntegrator: '<S358>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_oc +=
                        PHRControl_cal->Integrator_gainval_g *
                        PHRControl_B.IntegralGain_h;
                    }
                  }
                  break;

                 case PHRControl_IN_Flight_c:
                  {
                    if (PHRControl_DW.Simscape_Transitions) {
                      PHRControl_DW.landingTime_m = PHRControl_DW.simTime;
                      PHRControl_DW.Hip_Impact_Angle = PHRControl_B.Delay1[0];
                      PHRControl_DW.Knee_Impact_Angle = PHRControl_B.Delay1[1];
                      PHRControl_DW.is_c4_PHRControl = PHRControl_IN_Load;
                      PHRControl_DW.temporalCounter_i1_m = 0U;
                      PHRControl_Load(&PHRControl_B.T1, &PHRControl_B.T2,
                                      &PHRControl_B.MachineState,
                                      &PHRControl_cal->PHRControl_Load_h_cal);
                    } else {
                      real_T u2;

                      /* Sum: '<S321>/Sum3' incorporates:
                       *  Constant: '<S321>/Constant1'
                       */
                      PHRControl_B.Sum3_b = PHRControl_cal->desired_speed -
                        PHRControl_B.xd;

                      /* Gain: '<S321>/Multiply' */
                      PHRControl_B.offsetfromneutralpointtocontrollocomotionspeed
                        = PHRControl_cal->k_landing_angle * PHRControl_B.Sum3_b;

                      /* Trigonometry: '<S321>/Sin' */
                      PHRControl_B.Sin_o = std::sin(PHRControl_B.Delay1[0]);

                      /* Product: '<S321>/Product1' incorporates:
                       *  Constant: '<S321>/Constant4'
                       */
                      PHRControl_B.Product1_j = PHRControl_B.Sin_o *
                        PHRControl_cal->upper_leg_length;

                      /* Sum: '<S321>/Sum6' */
                      PHRControl_B.Sum6_d = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Trigonometry: '<S321>/Sin1' */
                      PHRControl_B.Sin1_b = std::sin(PHRControl_B.Sum6_d);

                      /* Product: '<S321>/Product2' incorporates:
                       *  Constant: '<S321>/Constant3'
                       */
                      PHRControl_B.Product2_l = PHRControl_B.Sin1_b *
                        PHRControl_cal->lower_leg_length;

                      /* Sum: '<S321>/Sum2' */
                      PHRControl_B.X_actual = PHRControl_B.Product1_j +
                        PHRControl_B.Product2_l;

                      /* Product: '<S321>/Product' incorporates:
                       *  Constant: '<S321>/Constant'
                       */
                      PHRControl_B.Product_h = PHRControl_cal->contactTime *
                        PHRControl_B.xd;

                      /* Sum: '<S321>/Sum4' */
                      PHRControl_B.X_desired_k = PHRControl_B.Product_h +
                        PHRControl_B.offsetfromneutralpointtocontrollocomotionspeed;

                      /* Sum: '<S321>/Sum' */
                      PHRControl_B.Sum_og = PHRControl_B.X_desired_k -
                        PHRControl_B.X_actual;

                      /* Gain: '<S402>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_p =
                        PHRControl_cal->PIDController_D_b * PHRControl_B.Sum_og;

                      /* DiscreteIntegrator: '<S403>/Filter' */
                      PHRControl_B.Filter_kx = PHRControl_DW.Filter_DSTATE_ai;

                      /* Sum: '<S403>/SumD' */
                      PHRControl_B.SumD_l = PHRControl_B.DerivativeGain_p -
                        PHRControl_B.Filter_kx;

                      /* Gain: '<S405>/Integral Gain' */
                      PHRControl_B.IntegralGain_g =
                        PHRControl_cal->PIDController_I_k * PHRControl_B.Sum_og;

                      /* DiscreteIntegrator: '<S408>/Integrator' */
                      PHRControl_B.Integrator_b =
                        PHRControl_DW.Integrator_DSTATE_pp;

                      /* Gain: '<S411>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_e =
                        PHRControl_cal->PIDController_N_l * PHRControl_B.SumD_l;

                      /* Gain: '<S413>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_d =
                        PHRControl_cal->PIDController_P_n * PHRControl_B.Sum_og;

                      /* Sum: '<S417>/Sum' */
                      PHRControl_B.Sum_od = (PHRControl_B.ProportionalGain_d +
                        PHRControl_B.Integrator_b) +
                        PHRControl_B.FilterCoefficient_e;

                      /* Sum: '<S321>/Sum5' incorporates:
                       *  Constant: '<S321>/Constant5'
                       */
                      PHRControl_B.Sum5_h = PHRControl_cal->initKnee -
                        PHRControl_B.Sum6_d;

                      /* Gain: '<S450>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_e =
                        PHRControl_cal->PIDController1_D_j * PHRControl_B.Sum5_h;

                      /* DiscreteIntegrator: '<S451>/Filter' */
                      PHRControl_B.Filter_j = PHRControl_DW.Filter_DSTATE_el;

                      /* Sum: '<S451>/SumD' */
                      PHRControl_B.SumD_o = PHRControl_B.DerivativeGain_e -
                        PHRControl_B.Filter_j;

                      /* Gain: '<S453>/Integral Gain' */
                      PHRControl_B.IntegralGain_c =
                        PHRControl_cal->PIDController1_I_k * PHRControl_B.Sum5_h;

                      /* DiscreteIntegrator: '<S456>/Integrator' */
                      PHRControl_B.Integrator_j =
                        PHRControl_DW.Integrator_DSTATE_b;

                      /* Gain: '<S459>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_f =
                        PHRControl_cal->PIDController1_N_o * PHRControl_B.SumD_o;

                      /* Gain: '<S461>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_dm =
                        PHRControl_cal->PIDController1_P_e * PHRControl_B.Sum5_h;

                      /* Sum: '<S465>/Sum' */
                      PHRControl_B.Sum_cr = (PHRControl_B.ProportionalGain_dm +
                        PHRControl_B.Integrator_j) +
                        PHRControl_B.FilterCoefficient_f;

                      /* Saturate: '<S321>/Saturation2' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Sum_od;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = T_idx_0;
                      }

                      /* Saturate: '<S321>/Saturation3' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Sum_cr;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = T_idx_0;
                      }

                      /* Merge: '<S9>/ Merge ' incorporates:
                       *  Constant: '<S321>/Constant2'
                       *  SignalConversion generated from: '<S321>/Machine_State'
                       */
                      PHRControl_B.MachineState =
                        PHRControl_cal->Constant2_Value_j;

                      /* Update for DiscreteIntegrator: '<S403>/Filter' */
                      PHRControl_DW.Filter_DSTATE_ai +=
                        PHRControl_cal->Filter_gainval_c2 *
                        PHRControl_B.FilterCoefficient_e;

                      /* Update for DiscreteIntegrator: '<S408>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_pp +=
                        PHRControl_cal->Integrator_gainval_gd *
                        PHRControl_B.IntegralGain_g;

                      /* Update for DiscreteIntegrator: '<S451>/Filter' */
                      PHRControl_DW.Filter_DSTATE_el +=
                        PHRControl_cal->Filter_gainval_g *
                        PHRControl_B.FilterCoefficient_f;

                      /* Update for DiscreteIntegrator: '<S456>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_b +=
                        PHRControl_cal->Integrator_gainval_d *
                        PHRControl_B.IntegralGain_c;
                    }
                  }
                  break;

                 case PHRControl_IN_Idle:
                  if (PHRControl_DW.run == 1.0) {
                    PHRControl_DW.is_c4_PHRControl = PHRControl_IN_Load;
                    PHRControl_DW.temporalCounter_i1_m = 0U;
                    PHRControl_Load(&PHRControl_B.T1, &PHRControl_B.T2,
                                    &PHRControl_B.MachineState,
                                    &PHRControl_cal->PHRControl_Load_h_cal);
                  } else {
                    PHRControl_Idle(PHRControl_B.Delay1, &PHRControl_B.T1,
                                    &PHRControl_B.T2, &PHRControl_B.MachineState,
                                    &PHRControl_B.Idle_l, &PHRControl_DW.Idle_l,
                                    &PHRControl_cal->PHRControl_Idle_l_cal);
                  }
                  break;

                 case PHRControl_IN_Load:
                  {
                    real_T u2;
                    PHRControl_DW.is_c4_PHRControl = PHRControl_IN_Compress;

                    /* DataStoreRead: '<S320>/Data Store Read' */
                    PHRControl_B.DataStoreRead_e =
                      PHRControl_DW.Hip_Impact_Angle;

                    /* Sum: '<S320>/Sum3' incorporates:
                     *  Constant: '<S320>/Constant4'
                     */
                    PHRControl_B.Sum3_n = PHRControl_B.DataStoreRead_e -
                      PHRControl_cal->hip_bend_factor;

                    /* Sum: '<S320>/Sum2' */
                    PHRControl_B.Sum2_a = PHRControl_B.Sum3_n -
                      PHRControl_B.Delay1[0];

                    /* Gain: '<S352>/Derivative Gain' */
                    PHRControl_B.DerivativeGain_h =
                      PHRControl_cal->PIDController2_D_l * PHRControl_B.Sum2_a;

                    /* DiscreteIntegrator: '<S353>/Filter' */
                    PHRControl_B.Filter_f = PHRControl_DW.Filter_DSTATE_ak;

                    /* Sum: '<S353>/SumD' */
                    PHRControl_B.SumD_fv = PHRControl_B.DerivativeGain_h -
                      PHRControl_B.Filter_f;

                    /* Gain: '<S355>/Integral Gain' */
                    PHRControl_B.IntegralGain_h =
                      PHRControl_cal->PIDController2_I_i * PHRControl_B.Sum2_a;

                    /* DiscreteIntegrator: '<S358>/Integrator' */
                    PHRControl_B.Integrator_o =
                      PHRControl_DW.Integrator_DSTATE_oc;

                    /* Gain: '<S361>/Filter Coefficient' */
                    PHRControl_B.FilterCoefficient_c =
                      PHRControl_cal->PIDController2_N_l * PHRControl_B.SumD_fv;

                    /* Gain: '<S363>/Proportional Gain' */
                    PHRControl_B.ProportionalGain_mv =
                      PHRControl_cal->PIDController2_P_i * PHRControl_B.Sum2_a;

                    /* Sum: '<S367>/Sum' */
                    PHRControl_B.Sum_g = (PHRControl_B.ProportionalGain_mv +
                                          PHRControl_B.Integrator_o) +
                      PHRControl_B.FilterCoefficient_c;

                    /* Saturate: '<S320>/Saturation1' */
                    T_idx_0 = PHRControl_B.Sum_g;
                    th1 = PHRControl_cal->Saturation1_LowerSat;
                    u2 = PHRControl_cal->Saturation1_UpperSat;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = T_idx_0;
                    }

                    /* Merge: '<S9>/ Merge 1' incorporates:
                     *  Constant: '<S320>/Constant'
                     *  SignalConversion generated from: '<S320>/T2'
                     */
                    PHRControl_B.T2 = PHRControl_cal->Constant_Value_f;

                    /* Merge: '<S9>/ Merge ' incorporates:
                     *  Constant: '<S320>/Constant2'
                     *  SignalConversion generated from: '<S320>/Machine_State'
                     */
                    PHRControl_B.MachineState =
                      PHRControl_cal->Constant2_Value_e;

                    /* Update for DiscreteIntegrator: '<S353>/Filter' */
                    PHRControl_DW.Filter_DSTATE_ak +=
                      PHRControl_cal->Filter_gainval_c *
                      PHRControl_B.FilterCoefficient_c;

                    /* Update for DiscreteIntegrator: '<S358>/Integrator' */
                    PHRControl_DW.Integrator_DSTATE_oc +=
                      PHRControl_cal->Integrator_gainval_g *
                      PHRControl_B.IntegralGain_h;
                  }
                  break;

                 case PHRControl_IN_Thrust:
                  {
                    if (!PHRControl_DW.Simscape_Transitions) {
                      PHRControl_DW.is_c4_PHRControl = PHRControl_IN_Unload_l;
                      PHRControl_DW.temporalCounter_i1_m = 0U;
                      PHRControl_Unload(&PHRControl_B.T1, &PHRControl_B.T2,
                                        &PHRControl_B.MachineState,
                                        &PHRControl_cal->PHRControl_Unload_l_cal);
                    } else {
                      real_T u2;

                      /* DataStoreRead: '<S324>/Data Store Read' */
                      PHRControl_B.impactangle = PHRControl_DW.Hip_Impact_Angle;

                      /* Sum: '<S324>/Sum2' incorporates:
                       *  Constant: '<S324>/Constant'
                       */
                      PHRControl_B.Sum2_o = PHRControl_cal->desired_speed -
                        PHRControl_B.xd;

                      /* Gain: '<S324>/Gain' */
                      PHRControl_B.desiredspeed = PHRControl_cal->Gain_Gain_k *
                        PHRControl_B.Sum2_o;

                      /* Sum: '<S324>/Sum' */
                      PHRControl_B.Sum_l = PHRControl_B.impactangle +
                        PHRControl_B.desiredspeed;

                      /* Gain: '<S597>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_m =
                        PHRControl_cal->PIDController2_D_m * PHRControl_B.Sum_l;

                      /* DiscreteIntegrator: '<S598>/Filter' */
                      PHRControl_B.Filter_c = PHRControl_DW.Filter_DSTATE_k;

                      /* Sum: '<S598>/SumD' */
                      PHRControl_B.SumD_m = PHRControl_B.DerivativeGain_m -
                        PHRControl_B.Filter_c;

                      /* Gain: '<S600>/Integral Gain' */
                      PHRControl_B.IntegralGain_k =
                        PHRControl_cal->PIDController2_I_c * PHRControl_B.Sum_l;

                      /* DiscreteIntegrator: '<S603>/Integrator' */
                      PHRControl_B.Integrator_k =
                        PHRControl_DW.Integrator_DSTATE_c;

                      /* Gain: '<S606>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_l =
                        PHRControl_cal->PIDController2_N_j * PHRControl_B.SumD_m;

                      /* Gain: '<S608>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_m =
                        PHRControl_cal->PIDController2_P_o * PHRControl_B.Sum_l;

                      /* Sum: '<S612>/Sum' */
                      PHRControl_B.Sum_e = (PHRControl_B.ProportionalGain_m +
                                            PHRControl_B.Integrator_k) +
                        PHRControl_B.FilterCoefficient_l;

                      /* Saturate: '<S324>/Saturation1' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Sum_e;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = T_idx_0;
                      }

                      /* Merge: '<S9>/ Merge 1' incorporates:
                       *  Constant: '<S324>/Constant1'
                       *  SignalConversion generated from: '<S324>/T2'
                       */
                      PHRControl_B.T2 = PHRControl_cal->max_torque;

                      /* Merge: '<S9>/ Merge ' incorporates:
                       *  Constant: '<S324>/Constant2'
                       *  SignalConversion generated from: '<S324>/Machine_State'
                       */
                      PHRControl_B.MachineState =
                        PHRControl_cal->Constant2_Value_p;

                      /* Update for DiscreteIntegrator: '<S598>/Filter' */
                      PHRControl_DW.Filter_DSTATE_k +=
                        PHRControl_cal->Filter_gainval_ew *
                        PHRControl_B.FilterCoefficient_l;

                      /* Update for DiscreteIntegrator: '<S603>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_c +=
                        PHRControl_cal->Integrator_gainval_e *
                        PHRControl_B.IntegralGain_k;
                    }
                  }
                  break;

                 default:
                  {
                    real_T u2;

                    /* case IN_Unload: */
                    PHRControl_DW.contactTime_l = PHRControl_DW.simTime -
                      PHRControl_DW.landingTime_m;
                    PHRControl_DW.hasHopped = 1.0;
                    PHRControl_DW.is_c4_PHRControl = PHRControl_IN_Flight_c;

                    /* Sum: '<S321>/Sum3' incorporates:
                     *  Constant: '<S321>/Constant1'
                     */
                    PHRControl_B.Sum3_b = PHRControl_cal->desired_speed -
                      PHRControl_B.xd;

                    /* Gain: '<S321>/Multiply' */
                    PHRControl_B.offsetfromneutralpointtocontrollocomotionspeed =
                      PHRControl_cal->k_landing_angle * PHRControl_B.Sum3_b;

                    /* Trigonometry: '<S321>/Sin' */
                    PHRControl_B.Sin_o = std::sin(PHRControl_B.Delay1[0]);

                    /* Product: '<S321>/Product1' incorporates:
                     *  Constant: '<S321>/Constant4'
                     */
                    PHRControl_B.Product1_j = PHRControl_B.Sin_o *
                      PHRControl_cal->upper_leg_length;

                    /* Sum: '<S321>/Sum6' */
                    PHRControl_B.Sum6_d = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Trigonometry: '<S321>/Sin1' */
                    PHRControl_B.Sin1_b = std::sin(PHRControl_B.Sum6_d);

                    /* Product: '<S321>/Product2' incorporates:
                     *  Constant: '<S321>/Constant3'
                     */
                    PHRControl_B.Product2_l = PHRControl_B.Sin1_b *
                      PHRControl_cal->lower_leg_length;

                    /* Sum: '<S321>/Sum2' */
                    PHRControl_B.X_actual = PHRControl_B.Product1_j +
                      PHRControl_B.Product2_l;

                    /* Product: '<S321>/Product' incorporates:
                     *  Constant: '<S321>/Constant'
                     */
                    PHRControl_B.Product_h = PHRControl_cal->contactTime *
                      PHRControl_B.xd;

                    /* Sum: '<S321>/Sum4' */
                    PHRControl_B.X_desired_k = PHRControl_B.Product_h +
                      PHRControl_B.offsetfromneutralpointtocontrollocomotionspeed;

                    /* Sum: '<S321>/Sum' */
                    PHRControl_B.Sum_og = PHRControl_B.X_desired_k -
                      PHRControl_B.X_actual;

                    /* Gain: '<S402>/Derivative Gain' */
                    PHRControl_B.DerivativeGain_p =
                      PHRControl_cal->PIDController_D_b * PHRControl_B.Sum_og;

                    /* DiscreteIntegrator: '<S403>/Filter' */
                    PHRControl_B.Filter_kx = PHRControl_DW.Filter_DSTATE_ai;

                    /* Sum: '<S403>/SumD' */
                    PHRControl_B.SumD_l = PHRControl_B.DerivativeGain_p -
                      PHRControl_B.Filter_kx;

                    /* Gain: '<S405>/Integral Gain' */
                    PHRControl_B.IntegralGain_g =
                      PHRControl_cal->PIDController_I_k * PHRControl_B.Sum_og;

                    /* DiscreteIntegrator: '<S408>/Integrator' */
                    PHRControl_B.Integrator_b =
                      PHRControl_DW.Integrator_DSTATE_pp;

                    /* Gain: '<S411>/Filter Coefficient' */
                    PHRControl_B.FilterCoefficient_e =
                      PHRControl_cal->PIDController_N_l * PHRControl_B.SumD_l;

                    /* Gain: '<S413>/Proportional Gain' */
                    PHRControl_B.ProportionalGain_d =
                      PHRControl_cal->PIDController_P_n * PHRControl_B.Sum_og;

                    /* Sum: '<S417>/Sum' */
                    PHRControl_B.Sum_od = (PHRControl_B.ProportionalGain_d +
                      PHRControl_B.Integrator_b) +
                      PHRControl_B.FilterCoefficient_e;

                    /* Sum: '<S321>/Sum5' incorporates:
                     *  Constant: '<S321>/Constant5'
                     */
                    PHRControl_B.Sum5_h = PHRControl_cal->initKnee -
                      PHRControl_B.Sum6_d;

                    /* Gain: '<S450>/Derivative Gain' */
                    PHRControl_B.DerivativeGain_e =
                      PHRControl_cal->PIDController1_D_j * PHRControl_B.Sum5_h;

                    /* DiscreteIntegrator: '<S451>/Filter' */
                    PHRControl_B.Filter_j = PHRControl_DW.Filter_DSTATE_el;

                    /* Sum: '<S451>/SumD' */
                    PHRControl_B.SumD_o = PHRControl_B.DerivativeGain_e -
                      PHRControl_B.Filter_j;

                    /* Gain: '<S453>/Integral Gain' */
                    PHRControl_B.IntegralGain_c =
                      PHRControl_cal->PIDController1_I_k * PHRControl_B.Sum5_h;

                    /* DiscreteIntegrator: '<S456>/Integrator' */
                    PHRControl_B.Integrator_j =
                      PHRControl_DW.Integrator_DSTATE_b;

                    /* Gain: '<S459>/Filter Coefficient' */
                    PHRControl_B.FilterCoefficient_f =
                      PHRControl_cal->PIDController1_N_o * PHRControl_B.SumD_o;

                    /* Gain: '<S461>/Proportional Gain' */
                    PHRControl_B.ProportionalGain_dm =
                      PHRControl_cal->PIDController1_P_e * PHRControl_B.Sum5_h;

                    /* Sum: '<S465>/Sum' */
                    PHRControl_B.Sum_cr = (PHRControl_B.ProportionalGain_dm +
                      PHRControl_B.Integrator_j) +
                      PHRControl_B.FilterCoefficient_f;

                    /* Saturate: '<S321>/Saturation2' */
                    th1 = -PHRControl_cal->max_torque;
                    T_idx_0 = PHRControl_B.Sum_od;
                    u2 = PHRControl_cal->max_torque;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = T_idx_0;
                    }

                    /* Saturate: '<S321>/Saturation3' */
                    th1 = -PHRControl_cal->max_torque;
                    T_idx_0 = PHRControl_B.Sum_cr;
                    u2 = PHRControl_cal->max_torque;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = T_idx_0;
                    }

                    /* Merge: '<S9>/ Merge ' incorporates:
                     *  Constant: '<S321>/Constant2'
                     *  SignalConversion generated from: '<S321>/Machine_State'
                     */
                    PHRControl_B.MachineState =
                      PHRControl_cal->Constant2_Value_j;

                    /* Update for DiscreteIntegrator: '<S403>/Filter' */
                    PHRControl_DW.Filter_DSTATE_ai +=
                      PHRControl_cal->Filter_gainval_c2 *
                      PHRControl_B.FilterCoefficient_e;

                    /* Update for DiscreteIntegrator: '<S408>/Integrator' */
                    PHRControl_DW.Integrator_DSTATE_pp +=
                      PHRControl_cal->Integrator_gainval_gd *
                      PHRControl_B.IntegralGain_g;

                    /* Update for DiscreteIntegrator: '<S451>/Filter' */
                    PHRControl_DW.Filter_DSTATE_el +=
                      PHRControl_cal->Filter_gainval_g *
                      PHRControl_B.FilterCoefficient_f;

                    /* Update for DiscreteIntegrator: '<S456>/Integrator' */
                    PHRControl_DW.Integrator_DSTATE_b +=
                      PHRControl_cal->Integrator_gainval_d *
                      PHRControl_B.IntegralGain_c;
                  }
                  break;
                }
              }
            }
            break;

           case PHRControl_IN_SLIP:
            {
              if ((PHRControl_B.MachineState == 0.0) && (PHRControl_B.isReady ==
                   2.0)) {
                PHRControl_DW.is_c3_PHRControl = PHRControl_IN_NO_ACTIVE_CHILD;
                PHRControl_DW.is_Command1 = PHRControl_IN_Stand;
              } else {
                switch (PHRControl_DW.is_c3_PHRControl) {
                 case PHRControl_IN_Flight:
                  {
                    if (PHRControl_DW.Simscape_Transitions) {
                      PHRControl_DW.SLIP_impact_angle =
                        PHRControl_DW.foot_x_flight /
                        PHRControl_DW.foot_y_flight;
                      PHRControl_DW.SLIP_impact_angle = std::atan
                        (PHRControl_DW.SLIP_impact_angle);
                      PHRControl_DW.landingTime = PHRControl_DW.simTime;
                      PHRControl_DW.is_c3_PHRControl = PHRControl_IN_Load_m;
                      PHRControl_DW.temporalCounter_i1_a = 0U;
                      PHRControl_Load(&PHRControl_B.T1, &PHRControl_B.T2,
                                      &PHRControl_B.MachineState,
                                      &PHRControl_cal->PHRControl_Load_k_cal);
                    } else {
                      real_T u2;

                      /* Trigonometry: '<S627>/Sin3' */
                      PHRControl_B.Sin3_o = std::sin(PHRControl_B.Delay1[0]);

                      /* Product: '<S627>/Product4' incorporates:
                       *  Constant: '<S627>/Constant6'
                       */
                      PHRControl_B.Product4_i = PHRControl_cal->upper_leg_length
                        * PHRControl_B.Sin3_o;

                      /* Sum: '<S627>/Sum1' */
                      PHRControl_B.Sum1_n = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Trigonometry: '<S627>/Sin2' */
                      PHRControl_B.Sin2_p = std::sin(PHRControl_B.Sum1_n);

                      /* Product: '<S627>/Product3' incorporates:
                       *  Constant: '<S627>/Constant5'
                       */
                      PHRControl_B.Product3_bq =
                        PHRControl_cal->lower_leg_length * PHRControl_B.Sin2_p;

                      /* Sum: '<S627>/Sum6' */
                      PHRControl_B.footxposition_j = PHRControl_B.Product4_i +
                        PHRControl_B.Product3_bq;

                      /* DataStoreWrite: '<S627>/Data Store Write' */
                      PHRControl_DW.foot_x_flight = PHRControl_B.footxposition_j;

                      /* Trigonometry: '<S627>/Sin5' */
                      PHRControl_B.Sin5_i = std::cos(PHRControl_B.Delay1[0]);

                      /* Product: '<S627>/Product6' incorporates:
                       *  Constant: '<S627>/Constant8'
                       */
                      PHRControl_B.Product6_je =
                        PHRControl_cal->upper_leg_length * PHRControl_B.Sin5_i;

                      /* Trigonometry: '<S627>/Sin4' */
                      PHRControl_B.Sin4_a = std::cos(PHRControl_B.Sum1_n);

                      /* Product: '<S627>/Product5' incorporates:
                       *  Constant: '<S627>/Constant7'
                       */
                      PHRControl_B.Product5_l = PHRControl_cal->lower_leg_length
                        * PHRControl_B.Sin4_a;

                      /* Sum: '<S627>/Sum7' */
                      PHRControl_B.footyposition_k = PHRControl_B.Product6_je +
                        PHRControl_B.Product5_l;

                      /* DataStoreWrite: '<S627>/Data Store Write1' */
                      PHRControl_DW.foot_y_flight = PHRControl_B.footyposition_k;

                      /* Math: '<S627>/Square2' */
                      PHRControl_B.Square2_d = PHRControl_B.footxposition_j *
                        PHRControl_B.footxposition_j;

                      /* Math: '<S627>/Square3' */
                      PHRControl_B.Square3_p = PHRControl_B.footyposition_k *
                        PHRControl_B.footyposition_k;

                      /* Sum: '<S627>/Sum8' */
                      PHRControl_B.Sum8_gi = PHRControl_B.Square2_d +
                        PHRControl_B.Square3_p;

                      /* Sqrt: '<S627>/Square Root1' */
                      PHRControl_B.currentlength_p = std::sqrt
                        (PHRControl_B.Sum8_gi);

                      /* DataStoreRead: '<S620>/Data Store Read' */
                      PHRControl_B.DataStoreRead = PHRControl_DW.contactTime;

                      /* Sum: '<S620>/Sum3' incorporates:
                       *  Constant: '<S620>/Constant1'
                       */
                      PHRControl_B.Sum3 = PHRControl_cal->desired_speed -
                        PHRControl_B.xd;

                      /* Gain: '<S620>/Multiply' */
                      PHRControl_B.NPOffset = PHRControl_cal->k_landing_angle *
                        PHRControl_B.Sum3;

                      /* Trigonometry: '<S620>/Sin' */
                      PHRControl_B.Sin_m = std::sin(PHRControl_B.Delay1[0]);

                      /* Product: '<S620>/Product1' incorporates:
                       *  Constant: '<S620>/Constant4'
                       */
                      PHRControl_B.kneex = PHRControl_B.Sin_m *
                        PHRControl_cal->upper_leg_length;

                      /* Sum: '<S620>/Sum7' */
                      PHRControl_B.Sum7_f = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Trigonometry: '<S620>/Sin1' */
                      PHRControl_B.Sin1_a = std::sin(PHRControl_B.Sum7_f);

                      /* Product: '<S620>/Product2' incorporates:
                       *  Constant: '<S620>/Constant3'
                       */
                      PHRControl_B.footx = PHRControl_B.Sin1_a *
                        PHRControl_cal->lower_leg_length;

                      /* Sum: '<S620>/Sum2' */
                      PHRControl_B.x_actual = PHRControl_B.kneex +
                        PHRControl_B.footx;

                      /* Product: '<S620>/Product' */
                      PHRControl_B.NP = PHRControl_B.DataStoreRead *
                        PHRControl_B.xd;

                      /* Sum: '<S620>/Sum4' */
                      PHRControl_B.X_desired = PHRControl_B.NP +
                        PHRControl_B.NPOffset;

                      /* Sum: '<S620>/Sum' */
                      PHRControl_B.error = PHRControl_B.X_desired -
                        PHRControl_B.x_actual;

                      /* Gain: '<S653>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_o =
                        PHRControl_cal->PIDController_D_l * PHRControl_B.error;

                      /* DiscreteIntegrator: '<S654>/Filter' */
                      PHRControl_B.Filter_g = PHRControl_DW.Filter_DSTATE_e;

                      /* Sum: '<S654>/SumD' */
                      PHRControl_B.SumD_p = PHRControl_B.DerivativeGain_o -
                        PHRControl_B.Filter_g;

                      /* Gain: '<S656>/Integral Gain' */
                      PHRControl_B.IntegralGain_i =
                        PHRControl_cal->PIDController_I_m * PHRControl_B.error;

                      /* DiscreteIntegrator: '<S659>/Integrator' */
                      PHRControl_B.Integrator_n =
                        PHRControl_DW.Integrator_DSTATE_o;

                      /* Gain: '<S662>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_h =
                        PHRControl_cal->PIDController_N_b * PHRControl_B.SumD_p;

                      /* Gain: '<S664>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_n =
                        PHRControl_cal->PIDController_P_j * PHRControl_B.error;

                      /* Sum: '<S668>/Sum' */
                      PHRControl_B.Sum_o = (PHRControl_B.ProportionalGain_n +
                                            PHRControl_B.Integrator_n) +
                        PHRControl_B.FilterCoefficient_h;

                      /* Sum: '<S620>/Sum5' incorporates:
                       *  Constant: '<S620>/Knee Joint Angle1'
                       */
                      PHRControl_B.Sum5_e = PHRControl_cal->equilibrium_length -
                        PHRControl_B.currentlength_p;

                      /* Gain: '<S701>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_d =
                        PHRControl_cal->PIDController1_D_o * PHRControl_B.Sum5_e;

                      /* DiscreteIntegrator: '<S702>/Filter' */
                      PHRControl_B.Filter_n = PHRControl_DW.Filter_DSTATE_a;

                      /* Sum: '<S702>/SumD' */
                      PHRControl_B.SumD_e = PHRControl_B.DerivativeGain_d -
                        PHRControl_B.Filter_n;

                      /* Gain: '<S704>/Integral Gain' */
                      PHRControl_B.IntegralGain_p =
                        PHRControl_cal->PIDController1_I_i * PHRControl_B.Sum5_e;

                      /* DiscreteIntegrator: '<S707>/Integrator' */
                      PHRControl_B.Integrator_n0 =
                        PHRControl_DW.Integrator_DSTATE_p;

                      /* Gain: '<S710>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_a =
                        PHRControl_cal->PIDController1_N_n * PHRControl_B.SumD_e;

                      /* Gain: '<S712>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_c =
                        PHRControl_cal->PIDController1_P_j * PHRControl_B.Sum5_e;

                      /* Sum: '<S716>/Sum' */
                      PHRControl_B.Sum_h = (PHRControl_B.ProportionalGain_c +
                                            PHRControl_B.Integrator_n0) +
                        PHRControl_B.FilterCoefficient_a;

                      /* Saturate: '<S620>/Saturation2' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Sum_o;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = T_idx_0;
                      }

                      /* Saturate: '<S620>/Saturation3' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Sum_h;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = T_idx_0;
                      }

                      /* Merge: '<S9>/ Merge ' incorporates:
                       *  Constant: '<S620>/Constant2'
                       *  SignalConversion generated from: '<S620>/Machine_State'
                       */
                      PHRControl_B.MachineState =
                        PHRControl_cal->Constant2_Value_o;

                      /* Update for DiscreteIntegrator: '<S654>/Filter' */
                      PHRControl_DW.Filter_DSTATE_e +=
                        PHRControl_cal->Filter_gainval_d *
                        PHRControl_B.FilterCoefficient_h;

                      /* Update for DiscreteIntegrator: '<S659>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_o +=
                        PHRControl_cal->Integrator_gainval_p *
                        PHRControl_B.IntegralGain_i;

                      /* Update for DiscreteIntegrator: '<S702>/Filter' */
                      PHRControl_DW.Filter_DSTATE_a +=
                        PHRControl_cal->Filter_gainval_l *
                        PHRControl_B.FilterCoefficient_a;

                      /* Update for DiscreteIntegrator: '<S707>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_p +=
                        PHRControl_cal->Integrator_gainval_o *
                        PHRControl_B.IntegralGain_p;
                    }
                  }
                  break;

                 case PHRControl_IN_Idle_j:
                  {
                    if (PHRControl_DW.run == 1.0) {
                      PHRControl_DW.is_c3_PHRControl = PHRControl_IN_Load_m;
                      PHRControl_DW.temporalCounter_i1_a = 0U;
                      PHRControl_Load(&PHRControl_B.T1, &PHRControl_B.T2,
                                      &PHRControl_B.MachineState,
                                      &PHRControl_cal->PHRControl_Load_k_cal);
                    } else {
                      real_T u2;

                      /* Sum: '<S621>/Sum1' incorporates:
                       *  Constant: '<S621>/Constant2'
                       */
                      PHRControl_B.Sum1_a = PHRControl_cal->initHip -
                        PHRControl_B.Delay1[0];

                      /* Gain: '<S751>/Derivative Gain' */
                      PHRControl_B.DerivativeGain =
                        PHRControl_cal->PIDController_D_f * PHRControl_B.Sum1_a;

                      /* DiscreteIntegrator: '<S752>/Filter' */
                      PHRControl_B.Filter = PHRControl_DW.Filter_DSTATE;

                      /* Sum: '<S752>/SumD' */
                      PHRControl_B.SumD = PHRControl_B.DerivativeGain -
                        PHRControl_B.Filter;

                      /* Gain: '<S754>/Integral Gain' */
                      PHRControl_B.IntegralGain =
                        PHRControl_cal->PIDController_I_e * PHRControl_B.Sum1_a;

                      /* DiscreteIntegrator: '<S757>/Integrator' */
                      PHRControl_B.Integrator = PHRControl_DW.Integrator_DSTATE;

                      /* Gain: '<S760>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient =
                        PHRControl_cal->PIDController_N_n * PHRControl_B.SumD;

                      /* Gain: '<S762>/Proportional Gain' */
                      PHRControl_B.ProportionalGain =
                        PHRControl_cal->PIDController_P_i * PHRControl_B.Sum1_a;

                      /* Sum: '<S766>/Sum' */
                      PHRControl_B.Sum_ct = (PHRControl_B.ProportionalGain +
                        PHRControl_B.Integrator) +
                        PHRControl_B.FilterCoefficient;

                      /* Sum: '<S621>/Sum6' */
                      PHRControl_B.Sum6_i = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Sum: '<S621>/Sum2' incorporates:
                       *  Constant: '<S621>/Constant1'
                       */
                      PHRControl_B.Sum2_k = PHRControl_cal->initKnee -
                        PHRControl_B.Sum6_i;

                      /* Gain: '<S799>/Derivative Gain' */
                      PHRControl_B.DerivativeGain_b =
                        PHRControl_cal->PIDController1_D_l * PHRControl_B.Sum2_k;

                      /* DiscreteIntegrator: '<S800>/Filter' */
                      PHRControl_B.Filter_k = PHRControl_DW.Filter_DSTATE_c;

                      /* Sum: '<S800>/SumD' */
                      PHRControl_B.SumD_f = PHRControl_B.DerivativeGain_b -
                        PHRControl_B.Filter_k;

                      /* Gain: '<S802>/Integral Gain' */
                      PHRControl_B.IntegralGain_f =
                        PHRControl_cal->PIDController1_I_b * PHRControl_B.Sum2_k;

                      /* DiscreteIntegrator: '<S805>/Integrator' */
                      PHRControl_B.Integrator_i =
                        PHRControl_DW.Integrator_DSTATE_j;

                      /* Gain: '<S808>/Filter Coefficient' */
                      PHRControl_B.FilterCoefficient_p =
                        PHRControl_cal->PIDController1_N_j * PHRControl_B.SumD_f;

                      /* Gain: '<S810>/Proportional Gain' */
                      PHRControl_B.ProportionalGain_f =
                        PHRControl_cal->PIDController1_P_ex *
                        PHRControl_B.Sum2_k;

                      /* Sum: '<S814>/Sum' */
                      PHRControl_B.Sum_c1 = (PHRControl_B.ProportionalGain_f +
                        PHRControl_B.Integrator_i) +
                        PHRControl_B.FilterCoefficient_p;

                      /* Saturate: '<S621>/Saturation2' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Sum_ct;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = T_idx_0;
                      }

                      /* Saturate: '<S621>/Saturation3' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Sum_c1;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = T_idx_0;
                      }

                      /* Merge: '<S9>/ Merge ' incorporates:
                       *  Constant: '<S621>/Constant'
                       *  SignalConversion generated from: '<S621>/Machine_State'
                       */
                      PHRControl_B.MachineState =
                        PHRControl_cal->Constant_Value_l;

                      /* Update for DiscreteIntegrator: '<S752>/Filter' */
                      PHRControl_DW.Filter_DSTATE +=
                        PHRControl_cal->Filter_gainval_m *
                        PHRControl_B.FilterCoefficient;

                      /* Update for DiscreteIntegrator: '<S757>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE +=
                        PHRControl_cal->Integrator_gainval_eb *
                        PHRControl_B.IntegralGain;

                      /* Update for DiscreteIntegrator: '<S800>/Filter' */
                      PHRControl_DW.Filter_DSTATE_c +=
                        PHRControl_cal->Filter_gainval_gw *
                        PHRControl_B.FilterCoefficient_p;

                      /* Update for DiscreteIntegrator: '<S805>/Integrator' */
                      PHRControl_DW.Integrator_DSTATE_j +=
                        PHRControl_cal->Integrator_gainval_oy *
                        PHRControl_B.IntegralGain_f;
                    }
                  }
                  break;

                 case PHRControl_IN_Load_m:
                  {
                    real_T u2;
                    PHRControl_DW.is_c3_PHRControl = PHRControl_IN_Spring;

                    /* Trigonometry: '<S822>/Cos' */
                    PHRControl_B.Cos = std::cos(PHRControl_B.Delay1[0]);

                    /* Sum: '<S822>/Sum6' */
                    PHRControl_B.Sum6 = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Sum: '<S822>/Sum5' */
                    PHRControl_B.Sum5 = PHRControl_B.Delay1[0] +
                      PHRControl_B.Sum6;

                    /* Trigonometry: '<S822>/Sin3' */
                    PHRControl_B.Sin3 = std::cos(PHRControl_B.Sum5);

                    /* Product: '<S822>/Product2' incorporates:
                     *  Constant: '<S822>/Constant6'
                     */
                    PHRControl_B.Product2 = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin3;

                    /* Trigonometry: '<S827>/Sin3' */
                    PHRControl_B.Sin3_e = std::sin(PHRControl_B.Delay1[0]);

                    /* Product: '<S827>/Product4' incorporates:
                     *  Constant: '<S827>/Constant6'
                     */
                    PHRControl_B.Product4 = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin3_e;

                    /* Sum: '<S827>/Sum' */
                    PHRControl_B.Sum = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Trigonometry: '<S827>/Sin2' */
                    PHRControl_B.Sin2 = std::sin(PHRControl_B.Sum);

                    /* Product: '<S827>/Product3' incorporates:
                     *  Constant: '<S827>/Constant5'
                     */
                    PHRControl_B.Product3 = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin2;

                    /* Sum: '<S827>/Sum6' */
                    PHRControl_B.footxposition = PHRControl_B.Product4 +
                      PHRControl_B.Product3;

                    /* Math: '<S827>/Square2' */
                    PHRControl_B.Square2 = PHRControl_B.footxposition *
                      PHRControl_B.footxposition;

                    /* Trigonometry: '<S827>/Sin5' */
                    PHRControl_B.Sin5 = std::cos(PHRControl_B.Delay1[0]);

                    /* Product: '<S827>/Product6' incorporates:
                     *  Constant: '<S827>/Constant8'
                     */
                    PHRControl_B.Product6 = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin5;

                    /* Trigonometry: '<S827>/Sin4' */
                    PHRControl_B.Sin4 = std::cos(PHRControl_B.Sum);

                    /* Product: '<S827>/Product5' incorporates:
                     *  Constant: '<S827>/Constant7'
                     */
                    PHRControl_B.Product5 = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin4;

                    /* Sum: '<S827>/Sum7' */
                    PHRControl_B.footyposition = PHRControl_B.Product6 +
                      PHRControl_B.Product5;

                    /* Math: '<S827>/Square3' */
                    PHRControl_B.Square3 = PHRControl_B.footyposition *
                      PHRControl_B.footyposition;

                    /* Sum: '<S827>/Sum8' */
                    PHRControl_B.Sum8 = PHRControl_B.Square2 +
                      PHRControl_B.Square3;

                    /* Sqrt: '<S827>/Square Root1' */
                    PHRControl_B.currentlength = std::sqrt(PHRControl_B.Sum8);

                    /* Sum: '<S822>/Sum2' incorporates:
                     *  Constant: '<S822>/Constant4'
                     */
                    PHRControl_B.Sum2 = PHRControl_B.currentlength -
                      PHRControl_cal->equilibrium_length;

                    /* Trigonometry: '<S825>/Sin6' */
                    PHRControl_B.Sin6 = std::sin(PHRControl_B.Delay1[0]);

                    /* Product: '<S825>/Product2' incorporates:
                     *  Constant: '<S825>/Constant4'
                     */
                    PHRControl_B.Product2_c = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin6;

                    /* Sum: '<S825>/Sum' */
                    PHRControl_B.Sum_b = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Trigonometry: '<S825>/Sin1' */
                    PHRControl_B.Sin1 = std::sin(PHRControl_B.Sum_b);

                    /* Product: '<S825>/Product1' incorporates:
                     *  Constant: '<S825>/Constant3'
                     */
                    PHRControl_B.Product1 = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin1;

                    /* Sum: '<S825>/Sum1' */
                    PHRControl_B.footxposition_h = PHRControl_B.Product2_c +
                      PHRControl_B.Product1;

                    /* Trigonometry: '<S825>/Sin8' */
                    PHRControl_B.Sin8 = std::cos(PHRControl_B.Delay1[0]);

                    /* Product: '<S825>/Product8' incorporates:
                     *  Constant: '<S825>/Constant10'
                     */
                    PHRControl_B.Product8 = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin8;

                    /* Trigonometry: '<S825>/Sin7' */
                    PHRControl_B.Sin7 = std::cos(PHRControl_B.Sum_b);

                    /* Product: '<S825>/Product7' incorporates:
                     *  Constant: '<S825>/Constant9'
                     */
                    PHRControl_B.Product7 = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin7;

                    /* Sum: '<S825>/Sum2' */
                    PHRControl_B.footyposition_c = PHRControl_B.Product8 +
                      PHRControl_B.Product7;

                    /* Product: '<S825>/Divide' */
                    PHRControl_B.Divide = PHRControl_B.footxposition_h /
                      PHRControl_B.footyposition_c;

                    /* Trigonometry: '<S825>/Trigonometric Function' */
                    PHRControl_B.TrigonometricFunction = std::atan
                      (PHRControl_B.Divide);

                    /* Trigonometry: '<S822>/Sin1' */
                    PHRControl_B.Sin1_p = std::cos
                      (PHRControl_B.TrigonometricFunction);

                    /* Product: '<S822>/Product1' incorporates:
                     *  Constant: '<S822>/Constant3'
                     */
                    th1 = -PHRControl_cal->k_slip;

                    /* Product: '<S822>/Product1' */
                    PHRControl_B.Product1_a = th1 * PHRControl_B.Sum2 *
                      PHRControl_B.Sin1_p;

                    /* Product: '<S822>/Divide' */
                    PHRControl_B.Divide_f = 1.0 / PHRControl_B.Product2 *
                      PHRControl_B.Product1_a;

                    /* Trigonometry: '<S826>/Sin3' */
                    PHRControl_B.Sin3_j = std::sin(PHRControl_B.Delay1[0]);

                    /* Product: '<S826>/Product4' incorporates:
                     *  Constant: '<S826>/Constant6'
                     */
                    PHRControl_B.Product4_p = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin3_j;

                    /* Sum: '<S826>/Sum' */
                    PHRControl_B.Sum_p = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Trigonometry: '<S826>/Sin2' */
                    PHRControl_B.Sin2_f = std::sin(PHRControl_B.Sum_p);

                    /* Product: '<S826>/Product3' incorporates:
                     *  Constant: '<S826>/Constant5'
                     */
                    PHRControl_B.Product3_c = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin2_f;

                    /* Sum: '<S826>/Sum6' */
                    PHRControl_B.footxposition_a = PHRControl_B.Product4_p +
                      PHRControl_B.Product3_c;

                    /* Math: '<S826>/Square2' */
                    PHRControl_B.Square2_e = PHRControl_B.footxposition_a *
                      PHRControl_B.footxposition_a;

                    /* Trigonometry: '<S826>/Sin5' */
                    PHRControl_B.Sin5_e = std::cos(PHRControl_B.Delay1[0]);

                    /* Product: '<S826>/Product6' incorporates:
                     *  Constant: '<S826>/Constant8'
                     */
                    PHRControl_B.Product6_i = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin5_e;

                    /* Trigonometry: '<S826>/Sin4' */
                    PHRControl_B.Sin4_m = std::cos(PHRControl_B.Sum_p);

                    /* Product: '<S826>/Product5' incorporates:
                     *  Constant: '<S826>/Constant7'
                     */
                    PHRControl_B.Product5_c = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin4_m;

                    /* Sum: '<S826>/Sum7' */
                    PHRControl_B.footyposition_h = PHRControl_B.Product6_i +
                      PHRControl_B.Product5_c;

                    /* Math: '<S826>/Square3' */
                    PHRControl_B.Square3_l = PHRControl_B.footyposition_h *
                      PHRControl_B.footyposition_h;

                    /* Sum: '<S826>/Sum8' */
                    PHRControl_B.Sum8_b = PHRControl_B.Square2_e +
                      PHRControl_B.Square3_l;

                    /* Sqrt: '<S826>/Square Root1' */
                    PHRControl_B.currentlength_g = std::sqrt(PHRControl_B.Sum8_b);

                    /* Sum: '<S822>/Sum' incorporates:
                     *  Constant: '<S822>/Constant1'
                     */
                    PHRControl_B.Sum_i = PHRControl_B.currentlength_g -
                      PHRControl_cal->equilibrium_length;

                    /* Trigonometry: '<S824>/Sin6' */
                    PHRControl_B.Sin6_p = std::sin(PHRControl_B.Delay1[0]);

                    /* Product: '<S824>/Product2' incorporates:
                     *  Constant: '<S824>/Constant4'
                     */
                    PHRControl_B.Product2_k = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin6_p;

                    /* Sum: '<S824>/Sum' */
                    PHRControl_B.Sum_c = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Trigonometry: '<S824>/Sin1' */
                    PHRControl_B.Sin1_g = std::sin(PHRControl_B.Sum_c);

                    /* Product: '<S824>/Product1' incorporates:
                     *  Constant: '<S824>/Constant3'
                     */
                    PHRControl_B.Product1_p = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin1_g;

                    /* Sum: '<S824>/Sum1' */
                    PHRControl_B.footxposition_he = PHRControl_B.Product2_k +
                      PHRControl_B.Product1_p;

                    /* Trigonometry: '<S824>/Sin8' */
                    PHRControl_B.Sin8_e = std::cos(PHRControl_B.Delay1[0]);

                    /* Product: '<S824>/Product8' incorporates:
                     *  Constant: '<S824>/Constant10'
                     */
                    PHRControl_B.Product8_m = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin8_e;

                    /* Trigonometry: '<S824>/Sin7' */
                    PHRControl_B.Sin7_h = std::cos(PHRControl_B.Sum_c);

                    /* Product: '<S824>/Product7' incorporates:
                     *  Constant: '<S824>/Constant9'
                     */
                    PHRControl_B.Product7_a = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin7_h;

                    /* Sum: '<S824>/Sum2' */
                    PHRControl_B.footyposition_a = PHRControl_B.Product8_m +
                      PHRControl_B.Product7_a;

                    /* Product: '<S824>/Divide' */
                    PHRControl_B.Divide_m = PHRControl_B.footxposition_he /
                      PHRControl_B.footyposition_a;

                    /* Trigonometry: '<S824>/Trigonometric Function' */
                    PHRControl_B.TrigonometricFunction_m = std::atan
                      (PHRControl_B.Divide_m);

                    /* Trigonometry: '<S822>/Sin' */
                    PHRControl_B.Sin = std::sin
                      (PHRControl_B.TrigonometricFunction_m);

                    /* Product: '<S822>/Product' incorporates:
                     *  Constant: '<S822>/Constant'
                     */
                    th1 = -PHRControl_cal->k_slip;

                    /* Product: '<S822>/Product' */
                    PHRControl_B.Product = th1 * PHRControl_B.Sum_i *
                      PHRControl_B.Sin;

                    /* Sum: '<S822>/Sum1' */
                    PHRControl_B.hiptorquenum = PHRControl_B.Product -
                      PHRControl_B.Divide_f;

                    /* Trigonometry: '<S822>/Sin4' */
                    PHRControl_B.mult2 = std::sin(PHRControl_B.Delay1[0]);

                    /* Sum: '<S822>/Sum4' */
                    PHRControl_B.Sum4 = PHRControl_B.Delay1[0] +
                      PHRControl_B.Sum6;

                    /* Trigonometry: '<S822>/Sin5' */
                    PHRControl_B.Sin5_a = std::sin(PHRControl_B.Sum4);

                    /* Product: '<S822>/Product3' incorporates:
                     *  Constant: '<S822>/Constant10'
                     *  Constant: '<S822>/Constant9'
                     */
                    PHRControl_B.Product3_b = PHRControl_B.Cos *
                      PHRControl_B.Sin5_a * PHRControl_cal->upper_leg_length *
                      PHRControl_cal->lower_leg_length;

                    /* Sum: '<S822>/Sum3' incorporates:
                     *  Constant: '<S822>/Constant8'
                     */
                    PHRControl_B.mult3 = PHRControl_cal->Constant8_Value_d +
                      PHRControl_B.Product3_b;

                    /* Product: '<S822>/Product4' incorporates:
                     *  Constant: '<S822>/Constant7'
                     */
                    PHRControl_B.hiptorquedenom =
                      PHRControl_cal->upper_leg_length * PHRControl_B.mult2 *
                      PHRControl_B.mult3;

                    /* Product: '<S822>/Divide1' */
                    PHRControl_B.hiptorque = PHRControl_B.hiptorquenum /
                      PHRControl_B.hiptorquedenom;

                    /* Sum: '<S822>/Minus' incorporates:
                     *  Constant: '<S822>/Constant2'
                     */
                    PHRControl_B.Minus = PHRControl_cal->Constant2_Value_ea -
                      PHRControl_B.hiptorque;

                    /* Trigonometry: '<S828>/Sin6' */
                    PHRControl_B.Sin6_b = std::sin(PHRControl_B.Delay1[0]);

                    /* Product: '<S828>/Product2' incorporates:
                     *  Constant: '<S828>/Constant4'
                     */
                    PHRControl_B.Product2_i = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin6_b;

                    /* Sum: '<S828>/Sum7' */
                    PHRControl_B.Sum7 = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Trigonometry: '<S828>/Sin1' */
                    PHRControl_B.Sin1_k = std::sin(PHRControl_B.Sum7);

                    /* Product: '<S828>/Product1' incorporates:
                     *  Constant: '<S828>/Constant3'
                     */
                    PHRControl_B.Product1_c = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin1_k;

                    /* Sum: '<S828>/Sum1' */
                    PHRControl_B.footxposition_i = PHRControl_B.Product2_i +
                      PHRControl_B.Product1_c;

                    /* Trigonometry: '<S828>/Sin8' */
                    PHRControl_B.Sin8_l = std::cos(PHRControl_B.Delay1[0]);

                    /* Product: '<S828>/Product8' incorporates:
                     *  Constant: '<S828>/Constant10'
                     */
                    PHRControl_B.Product8_a = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin8_l;

                    /* Trigonometry: '<S828>/Sin7' */
                    PHRControl_B.Sin7_f = std::cos(PHRControl_B.Sum7);

                    /* Product: '<S828>/Product7' incorporates:
                     *  Constant: '<S828>/Constant9'
                     */
                    PHRControl_B.Product7_ad = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin7_f;

                    /* Sum: '<S828>/Sum2' */
                    PHRControl_B.footyposition_n = PHRControl_B.Product8_a +
                      PHRControl_B.Product7_ad;

                    /* Product: '<S828>/Divide' */
                    PHRControl_B.Divide_j = PHRControl_B.footxposition_i /
                      PHRControl_B.footyposition_n;

                    /* Trigonometry: '<S828>/Trigonometric Function' */
                    PHRControl_B.TrigonometricFunction_g = std::atan
                      (PHRControl_B.Divide_j);

                    /* Trigonometry: '<S823>/Cos' */
                    PHRControl_B.Cos_f = std::cos
                      (PHRControl_B.TrigonometricFunction_g);

                    /* Trigonometry: '<S823>/Cos1' */
                    PHRControl_B.Cos1 = std::cos(PHRControl_B.Delay1[0]);

                    /* Sum: '<S823>/Sum7' */
                    PHRControl_B.Sum7_k = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Sum: '<S823>/Sum4' */
                    PHRControl_B.Sum4_l = PHRControl_B.Delay1[0] +
                      PHRControl_B.Sum7_k;

                    /* Trigonometry: '<S823>/Cos2' */
                    PHRControl_B.Cos2 = std::cos(PHRControl_B.Sum4_l);

                    /* Product: '<S823>/Product1' incorporates:
                     *  Constant: '<S823>/Constant3'
                     */
                    PHRControl_B.Product1_i = PHRControl_B.Minus *
                      PHRControl_B.Cos1 * PHRControl_cal->upper_leg_length;

                    /* Trigonometry: '<S829>/Sin3' */
                    PHRControl_B.Sin3_g = std::sin(PHRControl_B.Delay1[0]);

                    /* Product: '<S829>/Product4' incorporates:
                     *  Constant: '<S829>/Constant6'
                     */
                    PHRControl_B.Product4_n = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin3_g;

                    /* Sum: '<S829>/Sum1' */
                    PHRControl_B.Sum1 = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Trigonometry: '<S829>/Sin2' */
                    PHRControl_B.Sin2_l = std::sin(PHRControl_B.Sum1);

                    /* Product: '<S829>/Product3' incorporates:
                     *  Constant: '<S829>/Constant5'
                     */
                    PHRControl_B.Product3_j = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin2_l;

                    /* Sum: '<S829>/Sum6' */
                    PHRControl_B.footxposition_m = PHRControl_B.Product4_n +
                      PHRControl_B.Product3_j;

                    /* Math: '<S829>/Square2' */
                    PHRControl_B.Square2_b = PHRControl_B.footxposition_m *
                      PHRControl_B.footxposition_m;

                    /* Trigonometry: '<S829>/Sin5' */
                    PHRControl_B.Sin5_g = std::cos(PHRControl_B.Delay1[0]);

                    /* Product: '<S829>/Product6' incorporates:
                     *  Constant: '<S829>/Constant8'
                     */
                    PHRControl_B.Product6_j = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin5_g;

                    /* Trigonometry: '<S829>/Sin4' */
                    PHRControl_B.Sin4_f = std::cos(PHRControl_B.Sum1);

                    /* Product: '<S829>/Product5' incorporates:
                     *  Constant: '<S829>/Constant7'
                     */
                    PHRControl_B.Product5_m = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin4_f;

                    /* Sum: '<S829>/Sum7' */
                    PHRControl_B.footyposition_g = PHRControl_B.Product6_j +
                      PHRControl_B.Product5_m;

                    /* Math: '<S829>/Square3' */
                    PHRControl_B.Square3_j = PHRControl_B.footyposition_g *
                      PHRControl_B.footyposition_g;

                    /* Sum: '<S829>/Sum8' */
                    PHRControl_B.Sum8_g = PHRControl_B.Square2_b +
                      PHRControl_B.Square3_j;

                    /* Sqrt: '<S829>/Square Root1' */
                    PHRControl_B.currentlength_b = std::sqrt(PHRControl_B.Sum8_g);

                    /* Sum: '<S823>/Sum' incorporates:
                     *  Constant: '<S823>/Constant'
                     */
                    PHRControl_B.Sum_ch = PHRControl_B.currentlength_b -
                      PHRControl_cal->equilibrium_length;

                    /* Product: '<S823>/Product' incorporates:
                     *  Constant: '<S823>/Constant1'
                     */
                    th1 = -PHRControl_cal->k_slip;

                    /* Product: '<S823>/Product' */
                    PHRControl_B.Product_l = PHRControl_B.Cos_f * th1 *
                      PHRControl_B.Sum_ch;

                    /* Sum: '<S823>/Sum1' */
                    PHRControl_B.Sum1_h = PHRControl_B.Product_l -
                      PHRControl_B.Product1_i;

                    /* Product: '<S823>/Product2' incorporates:
                     *  Constant: '<S823>/Constant4'
                     */
                    PHRControl_B.Product2_p = PHRControl_B.Cos2 *
                      PHRControl_cal->lower_leg_length;

                    /* Product: '<S823>/Divide' */
                    PHRControl_B.kneetorque = PHRControl_B.Sum1_h /
                      PHRControl_B.Product2_p;

                    /* Saturate: '<S623>/Saturation' */
                    th1 = -PHRControl_cal->max_torque;
                    T_idx_0 = PHRControl_B.Minus;
                    u2 = PHRControl_cal->max_torque;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = T_idx_0;
                    }

                    /* Saturate: '<S623>/Saturation1' */
                    th1 = -PHRControl_cal->max_torque;
                    T_idx_0 = PHRControl_B.kneetorque;
                    u2 = PHRControl_cal->max_torque;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = T_idx_0;
                    }

                    /* Merge: '<S9>/ Merge ' incorporates:
                     *  Constant: '<S623>/Constant2'
                     *  SignalConversion generated from: '<S623>/Machine_State'
                     */
                    PHRControl_B.MachineState =
                      PHRControl_cal->Constant2_Value_f;
                  }
                  break;

                 case PHRControl_IN_Spring:
                  {
                    if (!PHRControl_DW.Simscape_Transitions) {
                      PHRControl_DW.is_c3_PHRControl = PHRControl_IN_Unload;
                      PHRControl_DW.temporalCounter_i1_a = 0U;
                      PHRControl_Unload(&PHRControl_B.T1, &PHRControl_B.T2,
                                        &PHRControl_B.MachineState,
                                        &PHRControl_cal->PHRControl_Unload_le_cal);
                    } else {
                      real_T u2;

                      /* Trigonometry: '<S822>/Cos' */
                      PHRControl_B.Cos = std::cos(PHRControl_B.Delay1[0]);

                      /* Sum: '<S822>/Sum6' */
                      PHRControl_B.Sum6 = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Sum: '<S822>/Sum5' */
                      PHRControl_B.Sum5 = PHRControl_B.Delay1[0] +
                        PHRControl_B.Sum6;

                      /* Trigonometry: '<S822>/Sin3' */
                      PHRControl_B.Sin3 = std::cos(PHRControl_B.Sum5);

                      /* Product: '<S822>/Product2' incorporates:
                       *  Constant: '<S822>/Constant6'
                       */
                      PHRControl_B.Product2 = PHRControl_cal->lower_leg_length *
                        PHRControl_B.Sin3;

                      /* Trigonometry: '<S827>/Sin3' */
                      PHRControl_B.Sin3_e = std::sin(PHRControl_B.Delay1[0]);

                      /* Product: '<S827>/Product4' incorporates:
                       *  Constant: '<S827>/Constant6'
                       */
                      PHRControl_B.Product4 = PHRControl_cal->upper_leg_length *
                        PHRControl_B.Sin3_e;

                      /* Sum: '<S827>/Sum' */
                      PHRControl_B.Sum = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Trigonometry: '<S827>/Sin2' */
                      PHRControl_B.Sin2 = std::sin(PHRControl_B.Sum);

                      /* Product: '<S827>/Product3' incorporates:
                       *  Constant: '<S827>/Constant5'
                       */
                      PHRControl_B.Product3 = PHRControl_cal->lower_leg_length *
                        PHRControl_B.Sin2;

                      /* Sum: '<S827>/Sum6' */
                      PHRControl_B.footxposition = PHRControl_B.Product4 +
                        PHRControl_B.Product3;

                      /* Math: '<S827>/Square2' */
                      PHRControl_B.Square2 = PHRControl_B.footxposition *
                        PHRControl_B.footxposition;

                      /* Trigonometry: '<S827>/Sin5' */
                      PHRControl_B.Sin5 = std::cos(PHRControl_B.Delay1[0]);

                      /* Product: '<S827>/Product6' incorporates:
                       *  Constant: '<S827>/Constant8'
                       */
                      PHRControl_B.Product6 = PHRControl_cal->upper_leg_length *
                        PHRControl_B.Sin5;

                      /* Trigonometry: '<S827>/Sin4' */
                      PHRControl_B.Sin4 = std::cos(PHRControl_B.Sum);

                      /* Product: '<S827>/Product5' incorporates:
                       *  Constant: '<S827>/Constant7'
                       */
                      PHRControl_B.Product5 = PHRControl_cal->lower_leg_length *
                        PHRControl_B.Sin4;

                      /* Sum: '<S827>/Sum7' */
                      PHRControl_B.footyposition = PHRControl_B.Product6 +
                        PHRControl_B.Product5;

                      /* Math: '<S827>/Square3' */
                      PHRControl_B.Square3 = PHRControl_B.footyposition *
                        PHRControl_B.footyposition;

                      /* Sum: '<S827>/Sum8' */
                      PHRControl_B.Sum8 = PHRControl_B.Square2 +
                        PHRControl_B.Square3;

                      /* Sqrt: '<S827>/Square Root1' */
                      PHRControl_B.currentlength = std::sqrt(PHRControl_B.Sum8);

                      /* Sum: '<S822>/Sum2' incorporates:
                       *  Constant: '<S822>/Constant4'
                       */
                      PHRControl_B.Sum2 = PHRControl_B.currentlength -
                        PHRControl_cal->equilibrium_length;

                      /* Trigonometry: '<S825>/Sin6' */
                      PHRControl_B.Sin6 = std::sin(PHRControl_B.Delay1[0]);

                      /* Product: '<S825>/Product2' incorporates:
                       *  Constant: '<S825>/Constant4'
                       */
                      PHRControl_B.Product2_c = PHRControl_cal->upper_leg_length
                        * PHRControl_B.Sin6;

                      /* Sum: '<S825>/Sum' */
                      PHRControl_B.Sum_b = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Trigonometry: '<S825>/Sin1' */
                      PHRControl_B.Sin1 = std::sin(PHRControl_B.Sum_b);

                      /* Product: '<S825>/Product1' incorporates:
                       *  Constant: '<S825>/Constant3'
                       */
                      PHRControl_B.Product1 = PHRControl_cal->lower_leg_length *
                        PHRControl_B.Sin1;

                      /* Sum: '<S825>/Sum1' */
                      PHRControl_B.footxposition_h = PHRControl_B.Product2_c +
                        PHRControl_B.Product1;

                      /* Trigonometry: '<S825>/Sin8' */
                      PHRControl_B.Sin8 = std::cos(PHRControl_B.Delay1[0]);

                      /* Product: '<S825>/Product8' incorporates:
                       *  Constant: '<S825>/Constant10'
                       */
                      PHRControl_B.Product8 = PHRControl_cal->upper_leg_length *
                        PHRControl_B.Sin8;

                      /* Trigonometry: '<S825>/Sin7' */
                      PHRControl_B.Sin7 = std::cos(PHRControl_B.Sum_b);

                      /* Product: '<S825>/Product7' incorporates:
                       *  Constant: '<S825>/Constant9'
                       */
                      PHRControl_B.Product7 = PHRControl_cal->lower_leg_length *
                        PHRControl_B.Sin7;

                      /* Sum: '<S825>/Sum2' */
                      PHRControl_B.footyposition_c = PHRControl_B.Product8 +
                        PHRControl_B.Product7;

                      /* Product: '<S825>/Divide' */
                      PHRControl_B.Divide = PHRControl_B.footxposition_h /
                        PHRControl_B.footyposition_c;

                      /* Trigonometry: '<S825>/Trigonometric Function' */
                      PHRControl_B.TrigonometricFunction = std::atan
                        (PHRControl_B.Divide);

                      /* Trigonometry: '<S822>/Sin1' */
                      PHRControl_B.Sin1_p = std::cos
                        (PHRControl_B.TrigonometricFunction);

                      /* Product: '<S822>/Product1' incorporates:
                       *  Constant: '<S822>/Constant3'
                       */
                      th1 = -PHRControl_cal->k_slip;

                      /* Product: '<S822>/Product1' */
                      PHRControl_B.Product1_a = th1 * PHRControl_B.Sum2 *
                        PHRControl_B.Sin1_p;

                      /* Product: '<S822>/Divide' */
                      PHRControl_B.Divide_f = 1.0 / PHRControl_B.Product2 *
                        PHRControl_B.Product1_a;

                      /* Trigonometry: '<S826>/Sin3' */
                      PHRControl_B.Sin3_j = std::sin(PHRControl_B.Delay1[0]);

                      /* Product: '<S826>/Product4' incorporates:
                       *  Constant: '<S826>/Constant6'
                       */
                      PHRControl_B.Product4_p = PHRControl_cal->upper_leg_length
                        * PHRControl_B.Sin3_j;

                      /* Sum: '<S826>/Sum' */
                      PHRControl_B.Sum_p = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Trigonometry: '<S826>/Sin2' */
                      PHRControl_B.Sin2_f = std::sin(PHRControl_B.Sum_p);

                      /* Product: '<S826>/Product3' incorporates:
                       *  Constant: '<S826>/Constant5'
                       */
                      PHRControl_B.Product3_c = PHRControl_cal->lower_leg_length
                        * PHRControl_B.Sin2_f;

                      /* Sum: '<S826>/Sum6' */
                      PHRControl_B.footxposition_a = PHRControl_B.Product4_p +
                        PHRControl_B.Product3_c;

                      /* Math: '<S826>/Square2' */
                      PHRControl_B.Square2_e = PHRControl_B.footxposition_a *
                        PHRControl_B.footxposition_a;

                      /* Trigonometry: '<S826>/Sin5' */
                      PHRControl_B.Sin5_e = std::cos(PHRControl_B.Delay1[0]);

                      /* Product: '<S826>/Product6' incorporates:
                       *  Constant: '<S826>/Constant8'
                       */
                      PHRControl_B.Product6_i = PHRControl_cal->upper_leg_length
                        * PHRControl_B.Sin5_e;

                      /* Trigonometry: '<S826>/Sin4' */
                      PHRControl_B.Sin4_m = std::cos(PHRControl_B.Sum_p);

                      /* Product: '<S826>/Product5' incorporates:
                       *  Constant: '<S826>/Constant7'
                       */
                      PHRControl_B.Product5_c = PHRControl_cal->lower_leg_length
                        * PHRControl_B.Sin4_m;

                      /* Sum: '<S826>/Sum7' */
                      PHRControl_B.footyposition_h = PHRControl_B.Product6_i +
                        PHRControl_B.Product5_c;

                      /* Math: '<S826>/Square3' */
                      PHRControl_B.Square3_l = PHRControl_B.footyposition_h *
                        PHRControl_B.footyposition_h;

                      /* Sum: '<S826>/Sum8' */
                      PHRControl_B.Sum8_b = PHRControl_B.Square2_e +
                        PHRControl_B.Square3_l;

                      /* Sqrt: '<S826>/Square Root1' */
                      PHRControl_B.currentlength_g = std::sqrt
                        (PHRControl_B.Sum8_b);

                      /* Sum: '<S822>/Sum' incorporates:
                       *  Constant: '<S822>/Constant1'
                       */
                      PHRControl_B.Sum_i = PHRControl_B.currentlength_g -
                        PHRControl_cal->equilibrium_length;

                      /* Trigonometry: '<S824>/Sin6' */
                      PHRControl_B.Sin6_p = std::sin(PHRControl_B.Delay1[0]);

                      /* Product: '<S824>/Product2' incorporates:
                       *  Constant: '<S824>/Constant4'
                       */
                      PHRControl_B.Product2_k = PHRControl_cal->upper_leg_length
                        * PHRControl_B.Sin6_p;

                      /* Sum: '<S824>/Sum' */
                      PHRControl_B.Sum_c = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Trigonometry: '<S824>/Sin1' */
                      PHRControl_B.Sin1_g = std::sin(PHRControl_B.Sum_c);

                      /* Product: '<S824>/Product1' incorporates:
                       *  Constant: '<S824>/Constant3'
                       */
                      PHRControl_B.Product1_p = PHRControl_cal->lower_leg_length
                        * PHRControl_B.Sin1_g;

                      /* Sum: '<S824>/Sum1' */
                      PHRControl_B.footxposition_he = PHRControl_B.Product2_k +
                        PHRControl_B.Product1_p;

                      /* Trigonometry: '<S824>/Sin8' */
                      PHRControl_B.Sin8_e = std::cos(PHRControl_B.Delay1[0]);

                      /* Product: '<S824>/Product8' incorporates:
                       *  Constant: '<S824>/Constant10'
                       */
                      PHRControl_B.Product8_m = PHRControl_cal->upper_leg_length
                        * PHRControl_B.Sin8_e;

                      /* Trigonometry: '<S824>/Sin7' */
                      PHRControl_B.Sin7_h = std::cos(PHRControl_B.Sum_c);

                      /* Product: '<S824>/Product7' incorporates:
                       *  Constant: '<S824>/Constant9'
                       */
                      PHRControl_B.Product7_a = PHRControl_cal->lower_leg_length
                        * PHRControl_B.Sin7_h;

                      /* Sum: '<S824>/Sum2' */
                      PHRControl_B.footyposition_a = PHRControl_B.Product8_m +
                        PHRControl_B.Product7_a;

                      /* Product: '<S824>/Divide' */
                      PHRControl_B.Divide_m = PHRControl_B.footxposition_he /
                        PHRControl_B.footyposition_a;

                      /* Trigonometry: '<S824>/Trigonometric Function' */
                      PHRControl_B.TrigonometricFunction_m = std::atan
                        (PHRControl_B.Divide_m);

                      /* Trigonometry: '<S822>/Sin' */
                      PHRControl_B.Sin = std::sin
                        (PHRControl_B.TrigonometricFunction_m);

                      /* Product: '<S822>/Product' incorporates:
                       *  Constant: '<S822>/Constant'
                       */
                      th1 = -PHRControl_cal->k_slip;

                      /* Product: '<S822>/Product' */
                      PHRControl_B.Product = th1 * PHRControl_B.Sum_i *
                        PHRControl_B.Sin;

                      /* Sum: '<S822>/Sum1' */
                      PHRControl_B.hiptorquenum = PHRControl_B.Product -
                        PHRControl_B.Divide_f;

                      /* Trigonometry: '<S822>/Sin4' */
                      PHRControl_B.mult2 = std::sin(PHRControl_B.Delay1[0]);

                      /* Sum: '<S822>/Sum4' */
                      PHRControl_B.Sum4 = PHRControl_B.Delay1[0] +
                        PHRControl_B.Sum6;

                      /* Trigonometry: '<S822>/Sin5' */
                      PHRControl_B.Sin5_a = std::sin(PHRControl_B.Sum4);

                      /* Product: '<S822>/Product3' incorporates:
                       *  Constant: '<S822>/Constant10'
                       *  Constant: '<S822>/Constant9'
                       */
                      PHRControl_B.Product3_b = PHRControl_B.Cos *
                        PHRControl_B.Sin5_a * PHRControl_cal->upper_leg_length *
                        PHRControl_cal->lower_leg_length;

                      /* Sum: '<S822>/Sum3' incorporates:
                       *  Constant: '<S822>/Constant8'
                       */
                      PHRControl_B.mult3 = PHRControl_cal->Constant8_Value_d +
                        PHRControl_B.Product3_b;

                      /* Product: '<S822>/Product4' incorporates:
                       *  Constant: '<S822>/Constant7'
                       */
                      PHRControl_B.hiptorquedenom =
                        PHRControl_cal->upper_leg_length * PHRControl_B.mult2 *
                        PHRControl_B.mult3;

                      /* Product: '<S822>/Divide1' */
                      PHRControl_B.hiptorque = PHRControl_B.hiptorquenum /
                        PHRControl_B.hiptorquedenom;

                      /* Sum: '<S822>/Minus' incorporates:
                       *  Constant: '<S822>/Constant2'
                       */
                      PHRControl_B.Minus = PHRControl_cal->Constant2_Value_ea -
                        PHRControl_B.hiptorque;

                      /* Trigonometry: '<S828>/Sin6' */
                      PHRControl_B.Sin6_b = std::sin(PHRControl_B.Delay1[0]);

                      /* Product: '<S828>/Product2' incorporates:
                       *  Constant: '<S828>/Constant4'
                       */
                      PHRControl_B.Product2_i = PHRControl_cal->upper_leg_length
                        * PHRControl_B.Sin6_b;

                      /* Sum: '<S828>/Sum7' */
                      PHRControl_B.Sum7 = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Trigonometry: '<S828>/Sin1' */
                      PHRControl_B.Sin1_k = std::sin(PHRControl_B.Sum7);

                      /* Product: '<S828>/Product1' incorporates:
                       *  Constant: '<S828>/Constant3'
                       */
                      PHRControl_B.Product1_c = PHRControl_cal->lower_leg_length
                        * PHRControl_B.Sin1_k;

                      /* Sum: '<S828>/Sum1' */
                      PHRControl_B.footxposition_i = PHRControl_B.Product2_i +
                        PHRControl_B.Product1_c;

                      /* Trigonometry: '<S828>/Sin8' */
                      PHRControl_B.Sin8_l = std::cos(PHRControl_B.Delay1[0]);

                      /* Product: '<S828>/Product8' incorporates:
                       *  Constant: '<S828>/Constant10'
                       */
                      PHRControl_B.Product8_a = PHRControl_cal->upper_leg_length
                        * PHRControl_B.Sin8_l;

                      /* Trigonometry: '<S828>/Sin7' */
                      PHRControl_B.Sin7_f = std::cos(PHRControl_B.Sum7);

                      /* Product: '<S828>/Product7' incorporates:
                       *  Constant: '<S828>/Constant9'
                       */
                      PHRControl_B.Product7_ad =
                        PHRControl_cal->lower_leg_length * PHRControl_B.Sin7_f;

                      /* Sum: '<S828>/Sum2' */
                      PHRControl_B.footyposition_n = PHRControl_B.Product8_a +
                        PHRControl_B.Product7_ad;

                      /* Product: '<S828>/Divide' */
                      PHRControl_B.Divide_j = PHRControl_B.footxposition_i /
                        PHRControl_B.footyposition_n;

                      /* Trigonometry: '<S828>/Trigonometric Function' */
                      PHRControl_B.TrigonometricFunction_g = std::atan
                        (PHRControl_B.Divide_j);

                      /* Trigonometry: '<S823>/Cos' */
                      PHRControl_B.Cos_f = std::cos
                        (PHRControl_B.TrigonometricFunction_g);

                      /* Trigonometry: '<S823>/Cos1' */
                      PHRControl_B.Cos1 = std::cos(PHRControl_B.Delay1[0]);

                      /* Sum: '<S823>/Sum7' */
                      PHRControl_B.Sum7_k = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Sum: '<S823>/Sum4' */
                      PHRControl_B.Sum4_l = PHRControl_B.Delay1[0] +
                        PHRControl_B.Sum7_k;

                      /* Trigonometry: '<S823>/Cos2' */
                      PHRControl_B.Cos2 = std::cos(PHRControl_B.Sum4_l);

                      /* Product: '<S823>/Product1' incorporates:
                       *  Constant: '<S823>/Constant3'
                       */
                      PHRControl_B.Product1_i = PHRControl_B.Minus *
                        PHRControl_B.Cos1 * PHRControl_cal->upper_leg_length;

                      /* Trigonometry: '<S829>/Sin3' */
                      PHRControl_B.Sin3_g = std::sin(PHRControl_B.Delay1[0]);

                      /* Product: '<S829>/Product4' incorporates:
                       *  Constant: '<S829>/Constant6'
                       */
                      PHRControl_B.Product4_n = PHRControl_cal->upper_leg_length
                        * PHRControl_B.Sin3_g;

                      /* Sum: '<S829>/Sum1' */
                      PHRControl_B.Sum1 = PHRControl_B.Delay1[0] +
                        PHRControl_B.Delay1[1];

                      /* Trigonometry: '<S829>/Sin2' */
                      PHRControl_B.Sin2_l = std::sin(PHRControl_B.Sum1);

                      /* Product: '<S829>/Product3' incorporates:
                       *  Constant: '<S829>/Constant5'
                       */
                      PHRControl_B.Product3_j = PHRControl_cal->lower_leg_length
                        * PHRControl_B.Sin2_l;

                      /* Sum: '<S829>/Sum6' */
                      PHRControl_B.footxposition_m = PHRControl_B.Product4_n +
                        PHRControl_B.Product3_j;

                      /* Math: '<S829>/Square2' */
                      PHRControl_B.Square2_b = PHRControl_B.footxposition_m *
                        PHRControl_B.footxposition_m;

                      /* Trigonometry: '<S829>/Sin5' */
                      PHRControl_B.Sin5_g = std::cos(PHRControl_B.Delay1[0]);

                      /* Product: '<S829>/Product6' incorporates:
                       *  Constant: '<S829>/Constant8'
                       */
                      PHRControl_B.Product6_j = PHRControl_cal->upper_leg_length
                        * PHRControl_B.Sin5_g;

                      /* Trigonometry: '<S829>/Sin4' */
                      PHRControl_B.Sin4_f = std::cos(PHRControl_B.Sum1);

                      /* Product: '<S829>/Product5' incorporates:
                       *  Constant: '<S829>/Constant7'
                       */
                      PHRControl_B.Product5_m = PHRControl_cal->lower_leg_length
                        * PHRControl_B.Sin4_f;

                      /* Sum: '<S829>/Sum7' */
                      PHRControl_B.footyposition_g = PHRControl_B.Product6_j +
                        PHRControl_B.Product5_m;

                      /* Math: '<S829>/Square3' */
                      PHRControl_B.Square3_j = PHRControl_B.footyposition_g *
                        PHRControl_B.footyposition_g;

                      /* Sum: '<S829>/Sum8' */
                      PHRControl_B.Sum8_g = PHRControl_B.Square2_b +
                        PHRControl_B.Square3_j;

                      /* Sqrt: '<S829>/Square Root1' */
                      PHRControl_B.currentlength_b = std::sqrt
                        (PHRControl_B.Sum8_g);

                      /* Sum: '<S823>/Sum' incorporates:
                       *  Constant: '<S823>/Constant'
                       */
                      PHRControl_B.Sum_ch = PHRControl_B.currentlength_b -
                        PHRControl_cal->equilibrium_length;

                      /* Product: '<S823>/Product' incorporates:
                       *  Constant: '<S823>/Constant1'
                       */
                      th1 = -PHRControl_cal->k_slip;

                      /* Product: '<S823>/Product' */
                      PHRControl_B.Product_l = PHRControl_B.Cos_f * th1 *
                        PHRControl_B.Sum_ch;

                      /* Sum: '<S823>/Sum1' */
                      PHRControl_B.Sum1_h = PHRControl_B.Product_l -
                        PHRControl_B.Product1_i;

                      /* Product: '<S823>/Product2' incorporates:
                       *  Constant: '<S823>/Constant4'
                       */
                      PHRControl_B.Product2_p = PHRControl_B.Cos2 *
                        PHRControl_cal->lower_leg_length;

                      /* Product: '<S823>/Divide' */
                      PHRControl_B.kneetorque = PHRControl_B.Sum1_h /
                        PHRControl_B.Product2_p;

                      /* Saturate: '<S623>/Saturation' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.Minus;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 2' */
                        PHRControl_B.T1 = T_idx_0;
                      }

                      /* Saturate: '<S623>/Saturation1' */
                      th1 = -PHRControl_cal->max_torque;
                      T_idx_0 = PHRControl_B.kneetorque;
                      u2 = PHRControl_cal->max_torque;
                      if (T_idx_0 > u2) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = u2;
                      } else if (T_idx_0 < th1) {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = th1;
                      } else {
                        /* Merge: '<S9>/ Merge 1' */
                        PHRControl_B.T2 = T_idx_0;
                      }

                      /* Merge: '<S9>/ Merge ' incorporates:
                       *  Constant: '<S623>/Constant2'
                       *  SignalConversion generated from: '<S623>/Machine_State'
                       */
                      PHRControl_B.MachineState =
                        PHRControl_cal->Constant2_Value_f;
                    }
                  }
                  break;

                 default:
                  {
                    real_T u2;

                    /* case IN_Unload: */
                    PHRControl_DW.contactTime = PHRControl_DW.simTime -
                      PHRControl_DW.landingTime;

                    /* Merge: '<S9>/ Merge 3' */
                    PHRControl_B.n++;
                    PHRControl_DW.hasHopped = 1.0;
                    PHRControl_DW.is_c3_PHRControl = PHRControl_IN_Flight;

                    /* Trigonometry: '<S627>/Sin3' */
                    PHRControl_B.Sin3_o = std::sin(PHRControl_B.Delay1[0]);

                    /* Product: '<S627>/Product4' incorporates:
                     *  Constant: '<S627>/Constant6'
                     */
                    PHRControl_B.Product4_i = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin3_o;

                    /* Sum: '<S627>/Sum1' */
                    PHRControl_B.Sum1_n = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Trigonometry: '<S627>/Sin2' */
                    PHRControl_B.Sin2_p = std::sin(PHRControl_B.Sum1_n);

                    /* Product: '<S627>/Product3' incorporates:
                     *  Constant: '<S627>/Constant5'
                     */
                    PHRControl_B.Product3_bq = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin2_p;

                    /* Sum: '<S627>/Sum6' */
                    PHRControl_B.footxposition_j = PHRControl_B.Product4_i +
                      PHRControl_B.Product3_bq;

                    /* DataStoreWrite: '<S627>/Data Store Write' */
                    PHRControl_DW.foot_x_flight = PHRControl_B.footxposition_j;

                    /* Trigonometry: '<S627>/Sin5' */
                    PHRControl_B.Sin5_i = std::cos(PHRControl_B.Delay1[0]);

                    /* Product: '<S627>/Product6' incorporates:
                     *  Constant: '<S627>/Constant8'
                     */
                    PHRControl_B.Product6_je = PHRControl_cal->upper_leg_length *
                      PHRControl_B.Sin5_i;

                    /* Trigonometry: '<S627>/Sin4' */
                    PHRControl_B.Sin4_a = std::cos(PHRControl_B.Sum1_n);

                    /* Product: '<S627>/Product5' incorporates:
                     *  Constant: '<S627>/Constant7'
                     */
                    PHRControl_B.Product5_l = PHRControl_cal->lower_leg_length *
                      PHRControl_B.Sin4_a;

                    /* Sum: '<S627>/Sum7' */
                    PHRControl_B.footyposition_k = PHRControl_B.Product6_je +
                      PHRControl_B.Product5_l;

                    /* DataStoreWrite: '<S627>/Data Store Write1' */
                    PHRControl_DW.foot_y_flight = PHRControl_B.footyposition_k;

                    /* Math: '<S627>/Square2' */
                    PHRControl_B.Square2_d = PHRControl_B.footxposition_j *
                      PHRControl_B.footxposition_j;

                    /* Math: '<S627>/Square3' */
                    PHRControl_B.Square3_p = PHRControl_B.footyposition_k *
                      PHRControl_B.footyposition_k;

                    /* Sum: '<S627>/Sum8' */
                    PHRControl_B.Sum8_gi = PHRControl_B.Square2_d +
                      PHRControl_B.Square3_p;

                    /* Sqrt: '<S627>/Square Root1' */
                    PHRControl_B.currentlength_p = std::sqrt
                      (PHRControl_B.Sum8_gi);

                    /* DataStoreRead: '<S620>/Data Store Read' */
                    PHRControl_B.DataStoreRead = PHRControl_DW.contactTime;

                    /* Sum: '<S620>/Sum3' incorporates:
                     *  Constant: '<S620>/Constant1'
                     */
                    PHRControl_B.Sum3 = PHRControl_cal->desired_speed -
                      PHRControl_B.xd;

                    /* Gain: '<S620>/Multiply' */
                    PHRControl_B.NPOffset = PHRControl_cal->k_landing_angle *
                      PHRControl_B.Sum3;

                    /* Trigonometry: '<S620>/Sin' */
                    PHRControl_B.Sin_m = std::sin(PHRControl_B.Delay1[0]);

                    /* Product: '<S620>/Product1' incorporates:
                     *  Constant: '<S620>/Constant4'
                     */
                    PHRControl_B.kneex = PHRControl_B.Sin_m *
                      PHRControl_cal->upper_leg_length;

                    /* Sum: '<S620>/Sum7' */
                    PHRControl_B.Sum7_f = PHRControl_B.Delay1[0] +
                      PHRControl_B.Delay1[1];

                    /* Trigonometry: '<S620>/Sin1' */
                    PHRControl_B.Sin1_a = std::sin(PHRControl_B.Sum7_f);

                    /* Product: '<S620>/Product2' incorporates:
                     *  Constant: '<S620>/Constant3'
                     */
                    PHRControl_B.footx = PHRControl_B.Sin1_a *
                      PHRControl_cal->lower_leg_length;

                    /* Sum: '<S620>/Sum2' */
                    PHRControl_B.x_actual = PHRControl_B.kneex +
                      PHRControl_B.footx;

                    /* Product: '<S620>/Product' */
                    PHRControl_B.NP = PHRControl_B.DataStoreRead *
                      PHRControl_B.xd;

                    /* Sum: '<S620>/Sum4' */
                    PHRControl_B.X_desired = PHRControl_B.NP +
                      PHRControl_B.NPOffset;

                    /* Sum: '<S620>/Sum' */
                    PHRControl_B.error = PHRControl_B.X_desired -
                      PHRControl_B.x_actual;

                    /* Gain: '<S653>/Derivative Gain' */
                    PHRControl_B.DerivativeGain_o =
                      PHRControl_cal->PIDController_D_l * PHRControl_B.error;

                    /* DiscreteIntegrator: '<S654>/Filter' */
                    PHRControl_B.Filter_g = PHRControl_DW.Filter_DSTATE_e;

                    /* Sum: '<S654>/SumD' */
                    PHRControl_B.SumD_p = PHRControl_B.DerivativeGain_o -
                      PHRControl_B.Filter_g;

                    /* Gain: '<S656>/Integral Gain' */
                    PHRControl_B.IntegralGain_i =
                      PHRControl_cal->PIDController_I_m * PHRControl_B.error;

                    /* DiscreteIntegrator: '<S659>/Integrator' */
                    PHRControl_B.Integrator_n =
                      PHRControl_DW.Integrator_DSTATE_o;

                    /* Gain: '<S662>/Filter Coefficient' */
                    PHRControl_B.FilterCoefficient_h =
                      PHRControl_cal->PIDController_N_b * PHRControl_B.SumD_p;

                    /* Gain: '<S664>/Proportional Gain' */
                    PHRControl_B.ProportionalGain_n =
                      PHRControl_cal->PIDController_P_j * PHRControl_B.error;

                    /* Sum: '<S668>/Sum' */
                    PHRControl_B.Sum_o = (PHRControl_B.ProportionalGain_n +
                                          PHRControl_B.Integrator_n) +
                      PHRControl_B.FilterCoefficient_h;

                    /* Sum: '<S620>/Sum5' incorporates:
                     *  Constant: '<S620>/Knee Joint Angle1'
                     */
                    PHRControl_B.Sum5_e = PHRControl_cal->equilibrium_length -
                      PHRControl_B.currentlength_p;

                    /* Gain: '<S701>/Derivative Gain' */
                    PHRControl_B.DerivativeGain_d =
                      PHRControl_cal->PIDController1_D_o * PHRControl_B.Sum5_e;

                    /* DiscreteIntegrator: '<S702>/Filter' */
                    PHRControl_B.Filter_n = PHRControl_DW.Filter_DSTATE_a;

                    /* Sum: '<S702>/SumD' */
                    PHRControl_B.SumD_e = PHRControl_B.DerivativeGain_d -
                      PHRControl_B.Filter_n;

                    /* Gain: '<S704>/Integral Gain' */
                    PHRControl_B.IntegralGain_p =
                      PHRControl_cal->PIDController1_I_i * PHRControl_B.Sum5_e;

                    /* DiscreteIntegrator: '<S707>/Integrator' */
                    PHRControl_B.Integrator_n0 =
                      PHRControl_DW.Integrator_DSTATE_p;

                    /* Gain: '<S710>/Filter Coefficient' */
                    PHRControl_B.FilterCoefficient_a =
                      PHRControl_cal->PIDController1_N_n * PHRControl_B.SumD_e;

                    /* Gain: '<S712>/Proportional Gain' */
                    PHRControl_B.ProportionalGain_c =
                      PHRControl_cal->PIDController1_P_j * PHRControl_B.Sum5_e;

                    /* Sum: '<S716>/Sum' */
                    PHRControl_B.Sum_h = (PHRControl_B.ProportionalGain_c +
                                          PHRControl_B.Integrator_n0) +
                      PHRControl_B.FilterCoefficient_a;

                    /* Saturate: '<S620>/Saturation2' */
                    th1 = -PHRControl_cal->max_torque;
                    T_idx_0 = PHRControl_B.Sum_o;
                    u2 = PHRControl_cal->max_torque;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 2' */
                      PHRControl_B.T1 = T_idx_0;
                    }

                    /* Saturate: '<S620>/Saturation3' */
                    th1 = -PHRControl_cal->max_torque;
                    T_idx_0 = PHRControl_B.Sum_h;
                    u2 = PHRControl_cal->max_torque;
                    if (T_idx_0 > u2) {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = u2;
                    } else if (T_idx_0 < th1) {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = th1;
                    } else {
                      /* Merge: '<S9>/ Merge 1' */
                      PHRControl_B.T2 = T_idx_0;
                    }

                    /* Merge: '<S9>/ Merge ' incorporates:
                     *  Constant: '<S620>/Constant2'
                     *  SignalConversion generated from: '<S620>/Machine_State'
                     */
                    PHRControl_B.MachineState =
                      PHRControl_cal->Constant2_Value_o;

                    /* Update for DiscreteIntegrator: '<S654>/Filter' */
                    PHRControl_DW.Filter_DSTATE_e +=
                      PHRControl_cal->Filter_gainval_d *
                      PHRControl_B.FilterCoefficient_h;

                    /* Update for DiscreteIntegrator: '<S659>/Integrator' */
                    PHRControl_DW.Integrator_DSTATE_o +=
                      PHRControl_cal->Integrator_gainval_p *
                      PHRControl_B.IntegralGain_i;

                    /* Update for DiscreteIntegrator: '<S702>/Filter' */
                    PHRControl_DW.Filter_DSTATE_a +=
                      PHRControl_cal->Filter_gainval_l *
                      PHRControl_B.FilterCoefficient_a;

                    /* Update for DiscreteIntegrator: '<S707>/Integrator' */
                    PHRControl_DW.Integrator_DSTATE_p +=
                      PHRControl_cal->Integrator_gainval_o *
                      PHRControl_B.IntegralGain_p;
                  }
                  break;
                }
              }
            }
            break;

           case PHRControl_IN_Stance:
            if (!PHRControl_DW.Simscape_Transitions) {
              /* Merge: '<S9>/ Merge 2' */
              PHRControl_B.T1 = 0.0;

              /* Merge: '<S9>/ Merge 1' */
              PHRControl_B.T2 = 0.0;
              PHRControl_DW.is_Command1 = PHRControl_IN_Flight;
            } else if (PHRControl_B.isReady == 2.0) {
              PHRControl_DW.is_Command1 = PHRControl_IN_Stand;
            } else {
              /* Merge: '<S9>/ Merge 2' */
              PHRControl_B.T1 = 5.0;

              /* Merge: '<S9>/ Merge 1' */
              PHRControl_B.T2 = 5.0;
            }
            break;

           default:
            {
              /* case IN_Stand: */
              if (PHRControl_B.isReady == 3.0) {
                PHRControl_B.Kp1 = 0.0;
                PHRControl_B.Kp2 = 0.0;
                PHRControl_B.Kd1 = 0.0;
                PHRControl_B.Kd2 = 0.0;
                PHRControl_DW.is_Command1 = PHRControl_IN_Raibert;

                /* Chart: '<S9>/Command1.Raibert' */
                PHRControl_DW.is_active_c4_PHRControl = 1U;
                PHRControl_DW.contactTime_l = 0.0;
                PHRControl_DW.hasHopped = 0.0;
                PHRControl_DW.landingTime_m = 0.0;
                PHRControl_DW.is_c4_PHRControl = PHRControl_IN_Idle;
                PHRControl_Idle(PHRControl_B.Delay1, &PHRControl_B.T1,
                                &PHRControl_B.T2, &PHRControl_B.MachineState,
                                &PHRControl_B.Idle_l, &PHRControl_DW.Idle_l,
                                &PHRControl_cal->PHRControl_Idle_l_cal);
              } else if (PHRControl_B.isReady == 6.0) {
                real_T u2;
                PHRControl_B.Kp1 = 0.0;
                PHRControl_B.Kp2 = 0.0;
                PHRControl_B.Kd1 = 0.0;
                PHRControl_B.Kd2 = 0.0;
                PHRControl_DW.is_Command1 = PHRControl_IN_SLIP;

                /* Chart: '<S9>/Command1.SLIP' */
                PHRControl_DW.is_active_c3_PHRControl = 1U;

                /* Merge: '<S9>/ Merge 3' incorporates:
                 *  Chart: '<S9>/Command1.SLIP'
                 */
                PHRControl_B.n = 0.0;

                /* Chart: '<S9>/Command1.SLIP' */
                PHRControl_DW.contactTime = 0.0;
                PHRControl_DW.hasHopped = 0.0;
                PHRControl_DW.landingTime = 0.0;
                PHRControl_DW.is_c3_PHRControl = PHRControl_IN_Idle_j;

                /* Sum: '<S621>/Sum1' incorporates:
                 *  Constant: '<S621>/Constant2'
                 */
                PHRControl_B.Sum1_a = PHRControl_cal->initHip -
                  PHRControl_B.Delay1[0];

                /* Gain: '<S751>/Derivative Gain' */
                PHRControl_B.DerivativeGain = PHRControl_cal->PIDController_D_f *
                  PHRControl_B.Sum1_a;

                /* DiscreteIntegrator: '<S752>/Filter' */
                PHRControl_B.Filter = PHRControl_DW.Filter_DSTATE;

                /* Sum: '<S752>/SumD' */
                PHRControl_B.SumD = PHRControl_B.DerivativeGain -
                  PHRControl_B.Filter;

                /* Gain: '<S754>/Integral Gain' */
                PHRControl_B.IntegralGain = PHRControl_cal->PIDController_I_e *
                  PHRControl_B.Sum1_a;

                /* DiscreteIntegrator: '<S757>/Integrator' */
                PHRControl_B.Integrator = PHRControl_DW.Integrator_DSTATE;

                /* Gain: '<S760>/Filter Coefficient' */
                PHRControl_B.FilterCoefficient =
                  PHRControl_cal->PIDController_N_n * PHRControl_B.SumD;

                /* Gain: '<S762>/Proportional Gain' */
                PHRControl_B.ProportionalGain =
                  PHRControl_cal->PIDController_P_i * PHRControl_B.Sum1_a;

                /* Sum: '<S766>/Sum' */
                PHRControl_B.Sum_ct = (PHRControl_B.ProportionalGain +
                  PHRControl_B.Integrator) + PHRControl_B.FilterCoefficient;

                /* Sum: '<S621>/Sum6' */
                PHRControl_B.Sum6_i = PHRControl_B.Delay1[0] +
                  PHRControl_B.Delay1[1];

                /* Sum: '<S621>/Sum2' incorporates:
                 *  Constant: '<S621>/Constant1'
                 */
                PHRControl_B.Sum2_k = PHRControl_cal->initKnee -
                  PHRControl_B.Sum6_i;

                /* Gain: '<S799>/Derivative Gain' */
                PHRControl_B.DerivativeGain_b =
                  PHRControl_cal->PIDController1_D_l * PHRControl_B.Sum2_k;

                /* DiscreteIntegrator: '<S800>/Filter' */
                PHRControl_B.Filter_k = PHRControl_DW.Filter_DSTATE_c;

                /* Sum: '<S800>/SumD' */
                PHRControl_B.SumD_f = PHRControl_B.DerivativeGain_b -
                  PHRControl_B.Filter_k;

                /* Gain: '<S802>/Integral Gain' */
                PHRControl_B.IntegralGain_f = PHRControl_cal->PIDController1_I_b
                  * PHRControl_B.Sum2_k;

                /* DiscreteIntegrator: '<S805>/Integrator' */
                PHRControl_B.Integrator_i = PHRControl_DW.Integrator_DSTATE_j;

                /* Gain: '<S808>/Filter Coefficient' */
                PHRControl_B.FilterCoefficient_p =
                  PHRControl_cal->PIDController1_N_j * PHRControl_B.SumD_f;

                /* Gain: '<S810>/Proportional Gain' */
                PHRControl_B.ProportionalGain_f =
                  PHRControl_cal->PIDController1_P_ex * PHRControl_B.Sum2_k;

                /* Sum: '<S814>/Sum' */
                PHRControl_B.Sum_c1 = (PHRControl_B.ProportionalGain_f +
                  PHRControl_B.Integrator_i) + PHRControl_B.FilterCoefficient_p;

                /* Saturate: '<S621>/Saturation2' */
                th1 = -PHRControl_cal->max_torque;
                T_idx_0 = PHRControl_B.Sum_ct;
                u2 = PHRControl_cal->max_torque;
                if (T_idx_0 > u2) {
                  /* Merge: '<S9>/ Merge 2' */
                  PHRControl_B.T1 = u2;
                } else if (T_idx_0 < th1) {
                  /* Merge: '<S9>/ Merge 2' */
                  PHRControl_B.T1 = th1;
                } else {
                  /* Merge: '<S9>/ Merge 2' */
                  PHRControl_B.T1 = T_idx_0;
                }

                /* Saturate: '<S621>/Saturation3' */
                th1 = -PHRControl_cal->max_torque;
                T_idx_0 = PHRControl_B.Sum_c1;
                u2 = PHRControl_cal->max_torque;
                if (T_idx_0 > u2) {
                  /* Merge: '<S9>/ Merge 1' */
                  PHRControl_B.T2 = u2;
                } else if (T_idx_0 < th1) {
                  /* Merge: '<S9>/ Merge 1' */
                  PHRControl_B.T2 = th1;
                } else {
                  /* Merge: '<S9>/ Merge 1' */
                  PHRControl_B.T2 = T_idx_0;
                }

                /* Merge: '<S9>/ Merge ' incorporates:
                 *  Constant: '<S621>/Constant'
                 *  SignalConversion generated from: '<S621>/Machine_State'
                 */
                PHRControl_B.MachineState = PHRControl_cal->Constant_Value_l;

                /* Update for DiscreteIntegrator: '<S752>/Filter' */
                PHRControl_DW.Filter_DSTATE += PHRControl_cal->Filter_gainval_m *
                  PHRControl_B.FilterCoefficient;

                /* Update for DiscreteIntegrator: '<S757>/Integrator' */
                PHRControl_DW.Integrator_DSTATE +=
                  PHRControl_cal->Integrator_gainval_eb *
                  PHRControl_B.IntegralGain;

                /* Update for DiscreteIntegrator: '<S800>/Filter' */
                PHRControl_DW.Filter_DSTATE_c +=
                  PHRControl_cal->Filter_gainval_gw *
                  PHRControl_B.FilterCoefficient_p;

                /* Update for DiscreteIntegrator: '<S805>/Integrator' */
                PHRControl_DW.Integrator_DSTATE_j +=
                  PHRControl_cal->Integrator_gainval_oy *
                  PHRControl_B.IntegralGain_f;
              } else if (PHRControl_B.isReady == 7.0) {
                PHRControl_B.Kp1 = 0.0;
                PHRControl_B.Kp2 = 0.0;
                PHRControl_B.Kd1 = 0.0;
                PHRControl_B.Kd2 = 0.0;
                PHRControl_DW.is_Command1 = PHRControl_IN_Stance;
              } else if (PHRControl_B.isReady == 5.0) {
                PHRControl_B.Kp1 = 0.0;
                PHRControl_B.Kp2 = 0.0;
                PHRControl_B.Kd1 = 0.0;
                PHRControl_B.Kd2 = 0.0;
                PHRControl_DW.is_Command1 = PHRControl_IN_HSW_Controller;

                /* Chart: '<S9>/Command1.HSW_Controller' */
                PHRControl_DW.is_active_c5_PHRControl = 1U;
                PHRControl_DW.contactTime = 0.0;
                PHRControl_DW.hasHopped = 0.0;
                PHRControl_DW.landingTime_g = 0.0;
                PHRControl_DW.is_c5_PHRControl = PHRControl_IN_Idle;
                PHRControl_Idle(PHRControl_B.Delay1, &PHRControl_B.T1,
                                &PHRControl_B.T2, &PHRControl_B.MachineState,
                                &PHRControl_B.Idle, &PHRControl_DW.Idle,
                                &PHRControl_cal->PHRControl_Idle_cal);
              } else {
                PHRControl_B.theta1 = PHRControl_cal->initHip;
                PHRControl_B.theta2 = PHRControl_cal->initKnee;
                PHRControl_B.Kp1 = 5.0;
                PHRControl_B.Kp2 = 5.0;
                PHRControl_B.Kd1 = 0.4;
                PHRControl_B.Kd2 = 0.4;

                /* Merge: '<S9>/ Merge 2' */
                PHRControl_B.T1 = 0.0;

                /* Merge: '<S9>/ Merge 1' */
                PHRControl_B.T2 = 0.0;
              }
            }
            break;
          }
        }
      }
      break;

     case PHRControl_IN_Deinitiate:
      PHRControl_B.c = 4.0;
      if (PHRControl_DW.temporalCounter_i1 >= 600U) {
        PHRControl_B.stop = 1.0;
        PHRControl_DW.is_c8_PHRControl = PHRControl_IN_Deinitiate;
        PHRControl_DW.temporalCounter_i1 = 0U;
        PHRControl_B.c = 4.0;
      }
      break;

     case PHRControl_IN_FinalShutdown:
      PHRControl_DW.is_c8_PHRControl = PHRControl_IN_Deinitiate;
      PHRControl_DW.temporalCounter_i1 = 0U;
      PHRControl_B.c = 4.0;
      break;

     case PHRControl_IN_Init:
      PHRControl_B.c = 2.0;
      if ((PHRControl_B.isReady == 4.0) || (PHRControl_B.danger == 1.0)) {
        PHRControl_DW.is_c8_PHRControl = PHRControl_IN_FinalShutdown;
        PHRControl_B.Kp1 = 0.0;
        PHRControl_B.Kp2 = 0.0;
        PHRControl_B.theta1 = 0.0;
        PHRControl_B.theta2 = 0.0;

        /* Merge: '<S9>/ Merge 2' */
        PHRControl_B.T1 = 0.0;

        /* Merge: '<S9>/ Merge 1' */
        PHRControl_B.T2 = 0.0;
        PHRControl_B.Kd1 = 0.1;
        PHRControl_B.Kd2 = 0.1;
      } else if (PHRControl_B.isReady == 2.0) {
        PHRControl_DW.is_c8_PHRControl = PHRControl_IN_Command1;
        PHRControl_B.c = 3.0;
        PHRControl_DW.is_Command1 = PHRControl_IN_Stand;
      }
      break;

     case PHRControl_IN_SlowDown:
      PHRControl_B.c = 4.0;
      if (PHRControl_B.isReady == 1.0) {
        PHRControl_DW.is_c8_PHRControl = PHRControl_IN_Init;
        PHRControl_B.c = 2.0;
      } else if ((PHRControl_DW.temporalCounter_i1 >= 200U) &&
                 (!(PHRControl_B.theta1 == PHRControl_B.theta2))) {
        PHRControl_DW.is_c8_PHRControl = PHRControl_IN_Zero;
        PHRControl_B.c = 1.0;
      }
      break;

     case PHRControl_IN_Start:
      if (PHRControl_B.isReady == 0.0) {
        PHRControl_DW.is_c8_PHRControl = PHRControl_IN_Zero;
        PHRControl_B.c = 1.0;
      } else if (PHRControl_B.isReady == 1.0) {
        PHRControl_DW.is_c8_PHRControl = PHRControl_IN_Init;
        PHRControl_B.c = 2.0;
      }
      break;

     default:
      /* case IN_Zero: */
      PHRControl_DW.is_c8_PHRControl = PHRControl_IN_SlowDown;
      PHRControl_DW.temporalCounter_i1 = 0U;
      PHRControl_B.c = 4.0;
      break;
    }
  }

  /* End of Chart: '<Root>/State-Transition Diagram' */

  /* MultiPortSwitch: '<Root>/Multiport Switch' */
  switch (static_cast<int32_T>(PHRControl_B.c)) {
   case 1:
    /* MultiPortSwitch: '<Root>/Multiport Switch' incorporates:
     *  Constant: '<Root>/Constant'
     */
    PHRControl_B.MultiportSwitch = PHRControl_cal->Constant_Value_c;
    break;

   case 2:
    /* MultiPortSwitch: '<Root>/Multiport Switch' incorporates:
     *  Constant: '<Root>/Constant3'
     */
    PHRControl_B.MultiportSwitch = PHRControl_cal->Constant3_Value_m;
    break;

   case 3:
    /* MultiPortSwitch: '<Root>/Multiport Switch' incorporates:
     *  Constant: '<Root>/Constant1'
     */
    PHRControl_B.MultiportSwitch = PHRControl_cal->Constant1_Value_p;
    break;

   default:
    /* MultiPortSwitch: '<Root>/Multiport Switch' incorporates:
     *  Constant: '<Root>/Constant4'
     */
    PHRControl_B.MultiportSwitch = PHRControl_cal->Constant4_Value;
    break;
  }

  /* End of MultiPortSwitch: '<Root>/Multiport Switch' */

  /* Outputs for Enabled SubSystem: '<Root>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S5>/Enable'
   */
  if (PHRControl_B.MultiportSwitch > 0.0) {
    /* S-Function (scanpack): '<S5>/CAN Pack1' incorporates:
     *  Constant: '<S5>/Constant'
     */
    /* S-Function (scanpack): '<S5>/CAN Pack1' */
    PHRControl_B.CANPack1_j.ID = 5U;
    PHRControl_B.CANPack1_j.Length = 0U;
    PHRControl_B.CANPack1_j.Extended = 0U;
    PHRControl_B.CANPack1_j.Remote = 0;
    PHRControl_B.CANPack1_j.Data[0] = 0;
    PHRControl_B.CANPack1_j.Data[1] = 0;
    PHRControl_B.CANPack1_j.Data[2] = 0;
    PHRControl_B.CANPack1_j.Data[3] = 0;
    PHRControl_B.CANPack1_j.Data[4] = 0;
    PHRControl_B.CANPack1_j.Data[5] = 0;
    PHRControl_B.CANPack1_j.Data[6] = 0;
    PHRControl_B.CANPack1_j.Data[7] = 0;

    {
      (void) std::memcpy((PHRControl_B.CANPack1_j.Data),
                         &PHRControl_cal->Constant_Value_g,
                         1 * sizeof(uint8_T));
    }

    /* S-Function (sg_IO602_IO691_write_s): '<S5>/CAN Write1' */

    /* Level2 S-Function Block: '<S5>/CAN Write1' (sg_IO602_IO691_write_s) */
    {
      SimStruct *rts = PHRControl_M->childSfunctions[0];
      sfcnOutputs(rts,0);
    }

    /* S-Function (scanpack): '<S5>/CAN Pack2' incorporates:
     *  Constant: '<S5>/Constant'
     */
    /* S-Function (scanpack): '<S5>/CAN Pack2' */
    PHRControl_B.CANPack2.ID = 6U;
    PHRControl_B.CANPack2.Length = 0U;
    PHRControl_B.CANPack2.Extended = 0U;
    PHRControl_B.CANPack2.Remote = 0;
    PHRControl_B.CANPack2.Data[0] = 0;
    PHRControl_B.CANPack2.Data[1] = 0;
    PHRControl_B.CANPack2.Data[2] = 0;
    PHRControl_B.CANPack2.Data[3] = 0;
    PHRControl_B.CANPack2.Data[4] = 0;
    PHRControl_B.CANPack2.Data[5] = 0;
    PHRControl_B.CANPack2.Data[6] = 0;
    PHRControl_B.CANPack2.Data[7] = 0;

    {
      (void) std::memcpy((PHRControl_B.CANPack2.Data),
                         &PHRControl_cal->Constant_Value_g,
                         1 * sizeof(uint8_T));
    }

    /* S-Function (sg_IO602_IO691_write_s): '<S5>/CAN Write2' */

    /* Level2 S-Function Block: '<S5>/CAN Write2' (sg_IO602_IO691_write_s) */
    {
      SimStruct *rts = PHRControl_M->childSfunctions[1];
      sfcnOutputs(rts,0);
    }

    srUpdateBC(PHRControl_DW.EnabledSubsystem_SubsysRanBC);
  }

  /* End of Outputs for SubSystem: '<Root>/Enabled Subsystem' */

  /* Gain: '<Root>/Gain10' */
  PHRControl_B.Gain10 = PHRControl_cal->Gain10_Gain * PHRControl_B.theta1;

  /* Gain: '<Root>/Gain8' */
  PHRControl_B.Gain8 = PHRControl_cal->Gain8_Gain * PHRControl_B.T1;

  /* Outputs for Atomic SubSystem: '<Root>/ID1 Write Hip Data' */
  /* MATLAB Function: '<S6>/floats -> bytes' incorporates:
   *  Constant: '<S6>/Constant3'
   */
  PHRControl_floatsbytes(PHRControl_B.Gain10, PHRControl_cal->Constant3_Value,
    PHRControl_B.Kp1, PHRControl_B.Kd1, PHRControl_B.Gain8,
    &PHRControl_B.sf_floatsbytes);

  /* MultiPortSwitch: '<S6>/Multiport Switch' */
  switch (static_cast<int32_T>(PHRControl_B.c)) {
   case 1:
    /* MultiPortSwitch: '<S6>/Multiport Switch' incorporates:
     *  Constant: '<S6>/Constant'
     */
    for (s8_iter = 0; s8_iter < 8; s8_iter++) {
      PHRControl_B.MultiportSwitch_l[s8_iter] =
        PHRControl_cal->Constant_Value_fg[s8_iter];
    }
    break;

   case 2:
    /* MultiPortSwitch: '<S6>/Multiport Switch' incorporates:
     *  Constant: '<S6>/Constant1'
     */
    for (s8_iter = 0; s8_iter < 8; s8_iter++) {
      PHRControl_B.MultiportSwitch_l[s8_iter] =
        PHRControl_cal->Constant1_Value_n[s8_iter];
    }
    break;

   case 3:
    /* MultiPortSwitch: '<S6>/Multiport Switch' */
    for (s8_iter = 0; s8_iter < 8; s8_iter++) {
      PHRControl_B.MultiportSwitch_l[s8_iter] =
        PHRControl_B.sf_floatsbytes.b[s8_iter];
    }
    break;

   default:
    /* MultiPortSwitch: '<S6>/Multiport Switch' incorporates:
     *  Constant: '<S6>/Constant2'
     */
    for (s8_iter = 0; s8_iter < 8; s8_iter++) {
      PHRControl_B.MultiportSwitch_l[s8_iter] =
        PHRControl_cal->Constant2_Value_g[s8_iter];
    }
    break;
  }

  /* End of MultiPortSwitch: '<S6>/Multiport Switch' */

  /* S-Function (slrealtimebytepacking): '<S6>/Byte Packing' */

  /* Byte Packing: <S6>/Byte Packing */
  (void)memcpy((uint8_T*)&PHRControl_B.BytePacking_l[0] + 0, (uint8_T*)
               &PHRControl_B.MultiportSwitch_l[0], 8);

  /* S-Function (scanpack): '<S6>/CAN Pack1' */
  /* S-Function (scanpack): '<S6>/CAN Pack1' */
  PHRControl_B.CANPack1.ID = 1U;
  PHRControl_B.CANPack1.Length = 8U;
  PHRControl_B.CANPack1.Extended = 0U;
  PHRControl_B.CANPack1.Remote = 0;
  PHRControl_B.CANPack1.Data[0] = 0;
  PHRControl_B.CANPack1.Data[1] = 0;
  PHRControl_B.CANPack1.Data[2] = 0;
  PHRControl_B.CANPack1.Data[3] = 0;
  PHRControl_B.CANPack1.Data[4] = 0;
  PHRControl_B.CANPack1.Data[5] = 0;
  PHRControl_B.CANPack1.Data[6] = 0;
  PHRControl_B.CANPack1.Data[7] = 0;

  {
    (void) std::memcpy((PHRControl_B.CANPack1.Data),
                       &PHRControl_B.BytePacking_l[0],
                       8 * sizeof(uint8_T));
  }

  /* S-Function (sg_IO602_IO691_write_s): '<S6>/CAN Write1' */

  /* Level2 S-Function Block: '<S6>/CAN Write1' (sg_IO602_IO691_write_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[2];
    sfcnOutputs(rts,0);
  }

  /* End of Outputs for SubSystem: '<Root>/ID1 Write Hip Data' */

  /* Gain: '<Root>/Gain6' */
  PHRControl_B.Gain6 = PHRControl_cal->Gain6_Gain * PHRControl_B.theta2;

  /* Gain: '<Root>/Gain1' */
  PHRControl_B.Gain1 = PHRControl_cal->Gain1_Gain * PHRControl_B.T2;

  /* Outputs for Atomic SubSystem: '<Root>/ID2 Write Knee Data' */
  /* MATLAB Function: '<S7>/floats -> bytes' incorporates:
   *  Constant: '<S7>/Constant3'
   */
  PHRControl_floatsbytes(PHRControl_B.Gain6, PHRControl_cal->Constant3_Value_e,
    PHRControl_B.Kp2, PHRControl_B.Kd2, PHRControl_B.Gain1,
    &PHRControl_B.sf_floatsbytes_f);

  /* MultiPortSwitch: '<S7>/Multiport Switch' */
  switch (static_cast<int32_T>(PHRControl_B.c)) {
   case 1:
    /* MultiPortSwitch: '<S7>/Multiport Switch' incorporates:
     *  Constant: '<S7>/Constant'
     */
    for (s8_iter = 0; s8_iter < 8; s8_iter++) {
      PHRControl_B.MultiportSwitch_e[s8_iter] = PHRControl_cal->
        Constant_Value_k[s8_iter];
    }
    break;

   case 2:
    /* MultiPortSwitch: '<S7>/Multiport Switch' incorporates:
     *  Constant: '<S7>/Constant1'
     */
    for (s8_iter = 0; s8_iter < 8; s8_iter++) {
      PHRControl_B.MultiportSwitch_e[s8_iter] =
        PHRControl_cal->Constant1_Value_d[s8_iter];
    }
    break;

   case 3:
    /* MultiPortSwitch: '<S7>/Multiport Switch' */
    for (s8_iter = 0; s8_iter < 8; s8_iter++) {
      PHRControl_B.MultiportSwitch_e[s8_iter] =
        PHRControl_B.sf_floatsbytes_f.b[s8_iter];
    }
    break;

   default:
    /* MultiPortSwitch: '<S7>/Multiport Switch' incorporates:
     *  Constant: '<S7>/Constant2'
     */
    for (s8_iter = 0; s8_iter < 8; s8_iter++) {
      PHRControl_B.MultiportSwitch_e[s8_iter] =
        PHRControl_cal->Constant2_Value_of[s8_iter];
    }
    break;
  }

  /* End of MultiPortSwitch: '<S7>/Multiport Switch' */

  /* S-Function (slrealtimebytepacking): '<S7>/Byte Packing' */

  /* Byte Packing: <S7>/Byte Packing */
  (void)memcpy((uint8_T*)&PHRControl_B.BytePacking[0] + 0, (uint8_T*)
               &PHRControl_B.MultiportSwitch_e[0], 8);

  /* S-Function (scanpack): '<S7>/CAN Pack1' */
  /* S-Function (scanpack): '<S7>/CAN Pack1' */
  PHRControl_B.CANmsg2.ID = 2U;
  PHRControl_B.CANmsg2.Length = 8U;
  PHRControl_B.CANmsg2.Extended = 0U;
  PHRControl_B.CANmsg2.Remote = 0;
  PHRControl_B.CANmsg2.Data[0] = 0;
  PHRControl_B.CANmsg2.Data[1] = 0;
  PHRControl_B.CANmsg2.Data[2] = 0;
  PHRControl_B.CANmsg2.Data[3] = 0;
  PHRControl_B.CANmsg2.Data[4] = 0;
  PHRControl_B.CANmsg2.Data[5] = 0;
  PHRControl_B.CANmsg2.Data[6] = 0;
  PHRControl_B.CANmsg2.Data[7] = 0;

  {
    (void) std::memcpy((PHRControl_B.CANmsg2.Data), &PHRControl_B.BytePacking[0],
                       8 * sizeof(uint8_T));
  }

  /* S-Function (sg_IO602_IO691_write_s): '<S7>/CAN Write1' */

  /* Level2 S-Function Block: '<S7>/CAN Write1' (sg_IO602_IO691_write_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[3];
    sfcnOutputs(rts,0);
  }

  /* End of Outputs for SubSystem: '<Root>/ID2 Write Knee Data' */

  /* Stop: '<Root>/Stop Simulation' */
  if (PHRControl_B.stop != 0.0) {
    rtmSetStopRequested(PHRControl_M, 1);
  }

  /* End of Stop: '<Root>/Stop Simulation' */
  /* SignalConversion generated from: '<Root>/Delay1' */
  PHRControl_B.TmpSignalConversionAtDelay1Inport1[0] = PHRControl_B.Gain;
  PHRControl_B.TmpSignalConversionAtDelay1Inport1[1] = PHRControl_B.Gain2;

  /* S-Function (sg_IO602_IO691_status_s): '<Root>/CAN Status' */

  /* Level2 S-Function Block: '<Root>/CAN Status' (sg_IO602_IO691_status_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[6];
    sfcnOutputs(rts,0);
  }

  /* Update for Delay: '<Root>/Delay2' */
  PHRControl_DW.Delay2_DSTATE = PHRControl_B.ground;

  /* Update for UnitDelay: '<S4>/UD' */
  PHRControl_DW.UD_DSTATE = PHRControl_B.TSamp;

  /* Update for UnitDelay: '<S3>/UD' */
  PHRControl_DW.UD_DSTATE_n = PHRControl_B.TSamp_j;

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' incorporates:
   *  Constant: '<Root>/Constant2'
   */
  PHRControl_DW.DiscreteTimeIntegrator_DSTATE +=
    PHRControl_cal->DiscreteTimeIntegrator_gainval_f *
    PHRControl_cal->Constant2_Value_hf;
  if (PHRControl_B.isReady > 0.0) {
    PHRControl_DW.DiscreteTimeIntegrator_PrevResetState = 1;
  } else if (PHRControl_B.isReady < 0.0) {
    PHRControl_DW.DiscreteTimeIntegrator_PrevResetState = -1;
  } else if (PHRControl_B.isReady == 0.0) {
    PHRControl_DW.DiscreteTimeIntegrator_PrevResetState = 0;
  } else {
    PHRControl_DW.DiscreteTimeIntegrator_PrevResetState = 2;
  }

  /* End of Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */

  /* Update for Delay: '<Root>/Delay1' */
  PHRControl_DW.Delay1_DSTATE[0] =
    PHRControl_B.TmpSignalConversionAtDelay1Inport1[0];
  PHRControl_DW.Delay1_DSTATE[1] =
    PHRControl_B.TmpSignalConversionAtDelay1Inport1[1];

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++PHRControl_M->Timing.clockTick0)) {
    ++PHRControl_M->Timing.clockTickH0;
  }

  PHRControl_M->Timing.t[0] = PHRControl_M->Timing.clockTick0 *
    PHRControl_M->Timing.stepSize0 + PHRControl_M->Timing.clockTickH0 *
    PHRControl_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.005s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++PHRControl_M->Timing.clockTick1)) {
      ++PHRControl_M->Timing.clockTickH1;
    }

    PHRControl_M->Timing.t[1] = PHRControl_M->Timing.clockTick1 *
      PHRControl_M->Timing.stepSize1 + PHRControl_M->Timing.clockTickH1 *
      PHRControl_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Model initialize function */
void PHRControl_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&PHRControl_M->solverInfo,
                          &PHRControl_M->Timing.simTimeStep);
    rtsiSetTPtr(&PHRControl_M->solverInfo, &rtmGetTPtr(PHRControl_M));
    rtsiSetStepSizePtr(&PHRControl_M->solverInfo,
                       &PHRControl_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&PHRControl_M->solverInfo, (&rtmGetErrorStatus
      (PHRControl_M)));
    rtsiSetRTModelPtr(&PHRControl_M->solverInfo, PHRControl_M);
  }

  rtsiSetSimTimeStep(&PHRControl_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&PHRControl_M->solverInfo,"FixedStepDiscrete");
  PHRControl_M->solverInfoPtr = (&PHRControl_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = PHRControl_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;

    /* polyspace +2 MISRA2012:D4.1 [Justified:Low] "PHRControl_M points to
       static memory which is guaranteed to be non-NULL" */
    PHRControl_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    PHRControl_M->Timing.sampleTimes = (&PHRControl_M->Timing.sampleTimesArray[0]);
    PHRControl_M->Timing.offsetTimes = (&PHRControl_M->Timing.offsetTimesArray[0]);

    /* task periods */
    PHRControl_M->Timing.sampleTimes[0] = (0.0);
    PHRControl_M->Timing.sampleTimes[1] = (0.005);

    /* task offsets */
    PHRControl_M->Timing.offsetTimes[0] = (0.0);
    PHRControl_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(PHRControl_M, &PHRControl_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = PHRControl_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    PHRControl_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(PHRControl_M, -1);
  PHRControl_M->Timing.stepSize0 = 0.005;
  PHRControl_M->Timing.stepSize1 = 0.005;
  PHRControl_M->solverInfoPtr = (&PHRControl_M->solverInfo);
  PHRControl_M->Timing.stepSize = (0.005);
  rtsiSetFixedStepSize(&PHRControl_M->solverInfo, 0.005);
  rtsiSetSolverMode(&PHRControl_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  (void) std::memset((static_cast<void *>(&PHRControl_B)), 0,
                     sizeof(B_PHRControl_T));

  {
    PHRControl_B.CANRead_o2 = CAN_DATATYPE_GROUND;
    PHRControl_B.CANmsg2 = CAN_DATATYPE_GROUND;
    PHRControl_B.CANPack1 = CAN_DATATYPE_GROUND;
    PHRControl_B.CANPack1_j = CAN_DATATYPE_GROUND;
    PHRControl_B.CANPack2 = CAN_DATATYPE_GROUND;
  }

  /* states (dwork) */
  (void) std::memset(static_cast<void *>(&PHRControl_DW), 0,
                     sizeof(DW_PHRControl_T));

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo = &PHRControl_M->NonInlinedSFcns.sfcnInfo;
    PHRControl_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus(PHRControl_M)));
    PHRControl_M->Sizes.numSampTimes = (2);
    rtssSetNumRootSampTimesPtr(sfcnInfo, &PHRControl_M->Sizes.numSampTimes);
    PHRControl_M->NonInlinedSFcns.taskTimePtrs[0] = &(rtmGetTPtr(PHRControl_M)[0]);
    PHRControl_M->NonInlinedSFcns.taskTimePtrs[1] = &(rtmGetTPtr(PHRControl_M)[1]);
    rtssSetTPtrPtr(sfcnInfo,PHRControl_M->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(PHRControl_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(PHRControl_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput(PHRControl_M));
    rtssSetStepSizePtr(sfcnInfo, &PHRControl_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested(PHRControl_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo, &PHRControl_M->derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo, &PHRControl_M->zCCacheNeedsReset);
    rtssSetContTimeOutputInconsistentWithStateAtMajorStepPtr(sfcnInfo,
      &PHRControl_M->CTOutputIncnstWithState);
    rtssSetSampleHitsPtr(sfcnInfo, &PHRControl_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo,
      &PHRControl_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &PHRControl_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &PHRControl_M->solverInfoPtr);
  }

  PHRControl_M->Sizes.numSFcns = (7);

  /* register each child */
  {
    (void) std::memset(static_cast<void *>
                       (&PHRControl_M->NonInlinedSFcns.childSFunctions[0]), 0,
                       7*sizeof(SimStruct));
    PHRControl_M->childSfunctions =
      (&PHRControl_M->NonInlinedSFcns.childSFunctionPtrs[0]);

    {
      int_T i;
      for (i = 0; i < 7; i++) {
        PHRControl_M->childSfunctions[i] =
          (&PHRControl_M->NonInlinedSFcns.childSFunctions[i]);
      }
    }

    /* Level2 S-Function Block: PHRControl/<S5>/CAN Write1 (sg_IO602_IO691_write_s) */
    {
      SimStruct *rts = PHRControl_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod = PHRControl_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset = PHRControl_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap = PHRControl_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) std::memset(static_cast<void*>(sfcnPeriod), 0,
                         sizeof(time_T)*1);
      (void) std::memset(static_cast<void*>(sfcnOffset), 0,
                         sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &PHRControl_M->NonInlinedSFcns.blkInfo2[0]);
        ssSetBlkInfoSLSizePtr(rts, &PHRControl_M->NonInlinedSFcns.blkInfoSLSize
                              [0]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &PHRControl_M->NonInlinedSFcns.inputOutputPortInfo2[0]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, PHRControl_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &PHRControl_M->NonInlinedSFcns.methods2[0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &PHRControl_M->NonInlinedSFcns.methods3[0]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &PHRControl_M->NonInlinedSFcns.methods4[0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &PHRControl_M->NonInlinedSFcns.statesInfo2[0]);
        ssSetPeriodicStatesInfo(rts,
          &PHRControl_M->NonInlinedSFcns.periodicStatesInfo[0]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->inputs =
          &PHRControl_M->NonInlinedSFcns.Sfcn0.inputPortInfoSLSize[0];
        _ssSetPortInfo2ForInputUnits(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn0.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn0.inputPortCoSimAttribute[0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, &PHRControl_B.CANPack1_j);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidthAsInt(rts, 0, 1);
        }
      }

      /* path info */
      ssSetModelName(rts, "CAN Write1");
      ssSetPath(rts, "PHRControl/Enabled Subsystem/CAN Write1");
      ssSetRTModel(rts,PHRControl_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &PHRControl_M->NonInlinedSFcns.Sfcn0.params;
        ssSetSFcnParamsCount(rts, 1);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)PHRControl_cal->CANWrite1_P1_Size);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &PHRControl_DW.CANWrite1_PWORK_l);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn0.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn0.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &PHRControl_DW.CANWrite1_PWORK_l);
      }

      /* registration */
      sg_IO602_IO691_write_s(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.005);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: PHRControl/<S5>/CAN Write2 (sg_IO602_IO691_write_s) */
    {
      SimStruct *rts = PHRControl_M->childSfunctions[1];

      /* timing info */
      time_T *sfcnPeriod = PHRControl_M->NonInlinedSFcns.Sfcn1.sfcnPeriod;
      time_T *sfcnOffset = PHRControl_M->NonInlinedSFcns.Sfcn1.sfcnOffset;
      int_T *sfcnTsMap = PHRControl_M->NonInlinedSFcns.Sfcn1.sfcnTsMap;
      (void) std::memset(static_cast<void*>(sfcnPeriod), 0,
                         sizeof(time_T)*1);
      (void) std::memset(static_cast<void*>(sfcnOffset), 0,
                         sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &PHRControl_M->NonInlinedSFcns.blkInfo2[1]);
        ssSetBlkInfoSLSizePtr(rts, &PHRControl_M->NonInlinedSFcns.blkInfoSLSize
                              [1]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &PHRControl_M->NonInlinedSFcns.inputOutputPortInfo2[1]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, PHRControl_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &PHRControl_M->NonInlinedSFcns.methods2[1]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &PHRControl_M->NonInlinedSFcns.methods3[1]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &PHRControl_M->NonInlinedSFcns.methods4[1]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &PHRControl_M->NonInlinedSFcns.statesInfo2[1]);
        ssSetPeriodicStatesInfo(rts,
          &PHRControl_M->NonInlinedSFcns.periodicStatesInfo[1]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn1.inputPortInfo[0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->inputs =
          &PHRControl_M->NonInlinedSFcns.Sfcn1.inputPortInfoSLSize[0];
        _ssSetPortInfo2ForInputUnits(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn1.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn1.inputPortCoSimAttribute[0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, &PHRControl_B.CANPack2);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidthAsInt(rts, 0, 1);
        }
      }

      /* path info */
      ssSetModelName(rts, "CAN Write2");
      ssSetPath(rts, "PHRControl/Enabled Subsystem/CAN Write2");
      ssSetRTModel(rts,PHRControl_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &PHRControl_M->NonInlinedSFcns.Sfcn1.params;
        ssSetSFcnParamsCount(rts, 1);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)PHRControl_cal->CANWrite2_P1_Size);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &PHRControl_DW.CANWrite2_PWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn1.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn1.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &PHRControl_DW.CANWrite2_PWORK);
      }

      /* registration */
      sg_IO602_IO691_write_s(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.005);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: PHRControl/<S6>/CAN Write1 (sg_IO602_IO691_write_s) */
    {
      SimStruct *rts = PHRControl_M->childSfunctions[2];

      /* timing info */
      time_T *sfcnPeriod = PHRControl_M->NonInlinedSFcns.Sfcn2.sfcnPeriod;
      time_T *sfcnOffset = PHRControl_M->NonInlinedSFcns.Sfcn2.sfcnOffset;
      int_T *sfcnTsMap = PHRControl_M->NonInlinedSFcns.Sfcn2.sfcnTsMap;
      (void) std::memset(static_cast<void*>(sfcnPeriod), 0,
                         sizeof(time_T)*1);
      (void) std::memset(static_cast<void*>(sfcnOffset), 0,
                         sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &PHRControl_M->NonInlinedSFcns.blkInfo2[2]);
        ssSetBlkInfoSLSizePtr(rts, &PHRControl_M->NonInlinedSFcns.blkInfoSLSize
                              [2]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &PHRControl_M->NonInlinedSFcns.inputOutputPortInfo2[2]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, PHRControl_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &PHRControl_M->NonInlinedSFcns.methods2[2]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &PHRControl_M->NonInlinedSFcns.methods3[2]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &PHRControl_M->NonInlinedSFcns.methods4[2]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &PHRControl_M->NonInlinedSFcns.statesInfo2[2]);
        ssSetPeriodicStatesInfo(rts,
          &PHRControl_M->NonInlinedSFcns.periodicStatesInfo[2]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn2.inputPortInfo[0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->inputs =
          &PHRControl_M->NonInlinedSFcns.Sfcn2.inputPortInfoSLSize[0];
        _ssSetPortInfo2ForInputUnits(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn2.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn2.inputPortCoSimAttribute[0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, &PHRControl_B.CANPack1);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidthAsInt(rts, 0, 1);
        }
      }

      /* path info */
      ssSetModelName(rts, "CAN Write1");
      ssSetPath(rts, "PHRControl/ID1 Write Hip Data/CAN Write1");
      ssSetRTModel(rts,PHRControl_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &PHRControl_M->NonInlinedSFcns.Sfcn2.params;
        ssSetSFcnParamsCount(rts, 1);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)PHRControl_cal->CANWrite1_P1_Size_k);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &PHRControl_DW.CANWrite1_PWORK_a);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn2.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn2.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &PHRControl_DW.CANWrite1_PWORK_a);
      }

      /* registration */
      sg_IO602_IO691_write_s(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.005);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: PHRControl/<S7>/CAN Write1 (sg_IO602_IO691_write_s) */
    {
      SimStruct *rts = PHRControl_M->childSfunctions[3];

      /* timing info */
      time_T *sfcnPeriod = PHRControl_M->NonInlinedSFcns.Sfcn3.sfcnPeriod;
      time_T *sfcnOffset = PHRControl_M->NonInlinedSFcns.Sfcn3.sfcnOffset;
      int_T *sfcnTsMap = PHRControl_M->NonInlinedSFcns.Sfcn3.sfcnTsMap;
      (void) std::memset(static_cast<void*>(sfcnPeriod), 0,
                         sizeof(time_T)*1);
      (void) std::memset(static_cast<void*>(sfcnOffset), 0,
                         sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &PHRControl_M->NonInlinedSFcns.blkInfo2[3]);
        ssSetBlkInfoSLSizePtr(rts, &PHRControl_M->NonInlinedSFcns.blkInfoSLSize
                              [3]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &PHRControl_M->NonInlinedSFcns.inputOutputPortInfo2[3]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, PHRControl_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &PHRControl_M->NonInlinedSFcns.methods2[3]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &PHRControl_M->NonInlinedSFcns.methods3[3]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &PHRControl_M->NonInlinedSFcns.methods4[3]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &PHRControl_M->NonInlinedSFcns.statesInfo2[3]);
        ssSetPeriodicStatesInfo(rts,
          &PHRControl_M->NonInlinedSFcns.periodicStatesInfo[3]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 1);
        ssSetPortInfoForInputs(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn3.inputPortInfo[0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->inputs =
          &PHRControl_M->NonInlinedSFcns.Sfcn3.inputPortInfoSLSize[0];
        _ssSetPortInfo2ForInputUnits(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn3.inputPortUnits[0]);
        ssSetInputPortUnit(rts, 0, 0);
        _ssSetPortInfo2ForInputCoSimAttribute(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn3.inputPortCoSimAttribute[0]);
        ssSetInputPortIsContinuousQuantity(rts, 0, 0);

        /* port 0 */
        {
          ssSetInputPortRequiredContiguous(rts, 0, 1);
          ssSetInputPortSignal(rts, 0, &PHRControl_B.CANmsg2);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidthAsInt(rts, 0, 1);
        }
      }

      /* path info */
      ssSetModelName(rts, "CAN Write1");
      ssSetPath(rts, "PHRControl/ID2 Write Knee Data/CAN Write1");
      ssSetRTModel(rts,PHRControl_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &PHRControl_M->NonInlinedSFcns.Sfcn3.params;
        ssSetSFcnParamsCount(rts, 1);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)PHRControl_cal->CANWrite1_P1_Size_n);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &PHRControl_DW.CANWrite1_PWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn3.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn3.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &PHRControl_DW.CANWrite1_PWORK);
      }

      /* registration */
      sg_IO602_IO691_write_s(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.005);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
    }

    /* Level2 S-Function Block: PHRControl/<S8>/CAN Read (sg_IO602_IO691_read_s) */
    {
      SimStruct *rts = PHRControl_M->childSfunctions[4];

      /* timing info */
      time_T *sfcnPeriod = PHRControl_M->NonInlinedSFcns.Sfcn4.sfcnPeriod;
      time_T *sfcnOffset = PHRControl_M->NonInlinedSFcns.Sfcn4.sfcnOffset;
      int_T *sfcnTsMap = PHRControl_M->NonInlinedSFcns.Sfcn4.sfcnTsMap;
      (void) std::memset(static_cast<void*>(sfcnPeriod), 0,
                         sizeof(time_T)*1);
      (void) std::memset(static_cast<void*>(sfcnOffset), 0,
                         sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &PHRControl_M->NonInlinedSFcns.blkInfo2[4]);
        ssSetBlkInfoSLSizePtr(rts, &PHRControl_M->NonInlinedSFcns.blkInfoSLSize
                              [4]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &PHRControl_M->NonInlinedSFcns.inputOutputPortInfo2[4]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, PHRControl_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &PHRControl_M->NonInlinedSFcns.methods2[4]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &PHRControl_M->NonInlinedSFcns.methods3[4]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &PHRControl_M->NonInlinedSFcns.methods4[4]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &PHRControl_M->NonInlinedSFcns.statesInfo2[4]);
        ssSetPeriodicStatesInfo(rts,
          &PHRControl_M->NonInlinedSFcns.periodicStatesInfo[4]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn4.outputPortInfo[0]);
        rts->blkInfo.blkInfo2->blkInfoSLSize->outputs =
          &PHRControl_M->NonInlinedSFcns.Sfcn4.outputPortInfoSLSize[0];
        _ssSetNumOutputPorts(rts, 2);
        _ssSetPortInfo2ForOutputUnits(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn4.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &PHRControl_M->NonInlinedSFcns.Sfcn4.outputPortCoSimAttribute[0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidthAsInt(rts, 0, 1);
          ssSetOutputPortSignal(rts, 0, ((boolean_T *) &PHRControl_B.CANRead_o1));
        }

        /* port 1 */
        {
          _ssSetOutputPortNumDimensions(rts, 1, 1);
          ssSetOutputPortWidthAsInt(rts, 1, 1);
          ssSetOutputPortSignal(rts, 1, ((CAN_DATATYPE *)
            &PHRControl_B.CANRead_o2));
        }
      }

      /* path info */
      ssSetModelName(rts, "CAN Read");
      ssSetPath(rts, "PHRControl/Read All/CAN Read");
      ssSetRTModel(rts,PHRControl_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &PHRControl_M->NonInlinedSFcns.Sfcn4.params;
        ssSetSFcnParamsCount(rts, 1);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)PHRControl_cal->CANRead_P1_Size);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &PHRControl_DW.CANRead_PWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn4.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn4.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &PHRControl_DW.CANRead_PWORK);
      }

      /* registration */
      sg_IO602_IO691_read_s(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.005);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);

      /* Update the BufferDstPort flags for each input port */
    }

    /* Level2 S-Function Block: PHRControl/<Root>/CAN Setup  (sg_IO602_IO691_setup_s) */
    {
      SimStruct *rts = PHRControl_M->childSfunctions[5];

      /* timing info */
      time_T *sfcnPeriod = PHRControl_M->NonInlinedSFcns.Sfcn5.sfcnPeriod;
      time_T *sfcnOffset = PHRControl_M->NonInlinedSFcns.Sfcn5.sfcnOffset;
      int_T *sfcnTsMap = PHRControl_M->NonInlinedSFcns.Sfcn5.sfcnTsMap;
      (void) std::memset(static_cast<void*>(sfcnPeriod), 0,
                         sizeof(time_T)*1);
      (void) std::memset(static_cast<void*>(sfcnOffset), 0,
                         sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &PHRControl_M->NonInlinedSFcns.blkInfo2[5]);
        ssSetBlkInfoSLSizePtr(rts, &PHRControl_M->NonInlinedSFcns.blkInfoSLSize
                              [5]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &PHRControl_M->NonInlinedSFcns.inputOutputPortInfo2[5]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, PHRControl_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &PHRControl_M->NonInlinedSFcns.methods2[5]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &PHRControl_M->NonInlinedSFcns.methods3[5]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &PHRControl_M->NonInlinedSFcns.methods4[5]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &PHRControl_M->NonInlinedSFcns.statesInfo2[5]);
        ssSetPeriodicStatesInfo(rts,
          &PHRControl_M->NonInlinedSFcns.periodicStatesInfo[5]);
      }

      /* path info */
      ssSetModelName(rts, "CAN Setup ");
      ssSetPath(rts, "PHRControl/CAN Setup ");
      ssSetRTModel(rts,PHRControl_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &PHRControl_M->NonInlinedSFcns.Sfcn5.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)PHRControl_cal->CANSetup_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)PHRControl_cal->CANSetup_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)PHRControl_cal->CANSetup_P3_Size);
      }

      /* work vectors */
      ssSetPWork(rts, (void **) &PHRControl_DW.CANSetup_PWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn5.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn5.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* PWORK */
        ssSetDWorkWidthAsInt(rts, 0, 1);
        ssSetDWorkDataType(rts, 0,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &PHRControl_DW.CANSetup_PWORK);
      }

      /* registration */
      sg_IO602_IO691_setup_s(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.005);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      /* Update the BufferDstPort flags for each input port */
    }

    /* Level2 S-Function Block: PHRControl/<Root>/CAN Status (sg_IO602_IO691_status_s) */
    {
      SimStruct *rts = PHRControl_M->childSfunctions[6];

      /* timing info */
      time_T *sfcnPeriod = PHRControl_M->NonInlinedSFcns.Sfcn6.sfcnPeriod;
      time_T *sfcnOffset = PHRControl_M->NonInlinedSFcns.Sfcn6.sfcnOffset;
      int_T *sfcnTsMap = PHRControl_M->NonInlinedSFcns.Sfcn6.sfcnTsMap;
      (void) std::memset(static_cast<void*>(sfcnPeriod), 0,
                         sizeof(time_T)*1);
      (void) std::memset(static_cast<void*>(sfcnOffset), 0,
                         sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &PHRControl_M->NonInlinedSFcns.blkInfo2[6]);
        ssSetBlkInfoSLSizePtr(rts, &PHRControl_M->NonInlinedSFcns.blkInfoSLSize
                              [6]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &PHRControl_M->NonInlinedSFcns.inputOutputPortInfo2[6]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, PHRControl_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &PHRControl_M->NonInlinedSFcns.methods2[6]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &PHRControl_M->NonInlinedSFcns.methods3[6]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &PHRControl_M->NonInlinedSFcns.methods4[6]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &PHRControl_M->NonInlinedSFcns.statesInfo2[6]);
        ssSetPeriodicStatesInfo(rts,
          &PHRControl_M->NonInlinedSFcns.periodicStatesInfo[6]);
      }

      /* path info */
      ssSetModelName(rts, "CAN Status");
      ssSetPath(rts, "PHRControl/CAN Status");
      ssSetRTModel(rts,PHRControl_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &PHRControl_M->NonInlinedSFcns.Sfcn6.params;
        ssSetSFcnParamsCount(rts, 34);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)PHRControl_cal->CANStatus_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)PHRControl_cal->CANStatus_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)PHRControl_cal->CANStatus_P3_Size);
        ssSetSFcnParam(rts, 3, (mxArray*)PHRControl_cal->CANStatus_P4_Size);
        ssSetSFcnParam(rts, 4, (mxArray*)PHRControl_cal->CANStatus_P5_Size);
        ssSetSFcnParam(rts, 5, (mxArray*)PHRControl_cal->CANStatus_P6_Size);
        ssSetSFcnParam(rts, 6, (mxArray*)PHRControl_cal->CANStatus_P7_Size);
        ssSetSFcnParam(rts, 7, (mxArray*)PHRControl_cal->CANStatus_P8_Size);
        ssSetSFcnParam(rts, 8, (mxArray*)PHRControl_cal->CANStatus_P9_Size);
        ssSetSFcnParam(rts, 9, (mxArray*)PHRControl_cal->CANStatus_P10_Size);
        ssSetSFcnParam(rts, 10, (mxArray*)PHRControl_cal->CANStatus_P11_Size);
        ssSetSFcnParam(rts, 11, (mxArray*)PHRControl_cal->CANStatus_P12_Size);
        ssSetSFcnParam(rts, 12, (mxArray*)PHRControl_cal->CANStatus_P13_Size);
        ssSetSFcnParam(rts, 13, (mxArray*)PHRControl_cal->CANStatus_P14_Size);
        ssSetSFcnParam(rts, 14, (mxArray*)PHRControl_cal->CANStatus_P15_Size);
        ssSetSFcnParam(rts, 15, (mxArray*)PHRControl_cal->CANStatus_P16_Size);
        ssSetSFcnParam(rts, 16, (mxArray*)PHRControl_cal->CANStatus_P17_Size);
        ssSetSFcnParam(rts, 17, (mxArray*)PHRControl_cal->CANStatus_P18_Size);
        ssSetSFcnParam(rts, 18, (mxArray*)PHRControl_cal->CANStatus_P19_Size);
        ssSetSFcnParam(rts, 19, (mxArray*)PHRControl_cal->CANStatus_P20_Size);
        ssSetSFcnParam(rts, 20, (mxArray*)PHRControl_cal->CANStatus_P21_Size);
        ssSetSFcnParam(rts, 21, (mxArray*)PHRControl_cal->CANStatus_P22_Size);
        ssSetSFcnParam(rts, 22, (mxArray*)PHRControl_cal->CANStatus_P23_Size);
        ssSetSFcnParam(rts, 23, (mxArray*)PHRControl_cal->CANStatus_P24_Size);
        ssSetSFcnParam(rts, 24, (mxArray*)PHRControl_cal->CANStatus_P25_Size);
        ssSetSFcnParam(rts, 25, (mxArray*)PHRControl_cal->CANStatus_P26_Size);
        ssSetSFcnParam(rts, 26, (mxArray*)PHRControl_cal->CANStatus_P27_Size);
        ssSetSFcnParam(rts, 27, (mxArray*)PHRControl_cal->CANStatus_P28_Size);
        ssSetSFcnParam(rts, 28, (mxArray*)PHRControl_cal->CANStatus_P29_Size);
        ssSetSFcnParam(rts, 29, (mxArray*)PHRControl_cal->CANStatus_P30_Size);
        ssSetSFcnParam(rts, 30, (mxArray*)PHRControl_cal->CANStatus_P31_Size);
        ssSetSFcnParam(rts, 31, (mxArray*)PHRControl_cal->CANStatus_P32_Size);
        ssSetSFcnParam(rts, 32, (mxArray*)PHRControl_cal->CANStatus_P33_Size);
        ssSetSFcnParam(rts, 33, (mxArray*)PHRControl_cal->CANStatus_P34_Size);
      }

      /* work vectors */
      ssSetIWork(rts, (int_T *) &PHRControl_DW.CANStatus_IWORK[0]);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn6.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &PHRControl_M->NonInlinedSFcns.Sfcn6.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 1);

        /* IWORK */
        ssSetDWorkWidthAsInt(rts, 0, 34);
        ssSetDWorkDataType(rts, 0,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &PHRControl_DW.CANStatus_IWORK[0]);
      }

      /* registration */
      sg_IO602_IO691_status_s(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.005);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCsAsInt(rts, 0);

      /* Update connectivity flags for each port */
      /* Update the BufferDstPort flags for each input port */
    }
  }

  /* Start for S-Function (sg_IO602_IO691_setup_s): '<Root>/CAN Setup ' */
  /* Level2 S-Function Block: '<Root>/CAN Setup ' (sg_IO602_IO691_setup_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[5];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for Constant: '<Root>/isReady' */
  PHRControl_B.isReady = PHRControl_cal->isReady_Value;

  /* Start for S-Function (sg_IO602_IO691_status_s): '<Root>/CAN Status' */
  /* Level2 S-Function Block: '<Root>/CAN Status' (sg_IO602_IO691_status_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[6];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
  PHRControl_DW.Simscape_Transitions =
    PHRControl_cal->DataStoreMemory_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory1' */
  PHRControl_DW.Hip_Impact_Angle = PHRControl_cal->DataStoreMemory1_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory12' */
  PHRControl_DW.Knee_Impact_Angle =
    PHRControl_cal->DataStoreMemory12_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory13' */
  PHRControl_DW.hip_range = PHRControl_cal->DataStoreMemory13_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory14' */
  PHRControl_DW.knee_range = PHRControl_cal->DataStoreMemory14_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory15' */
  PHRControl_DW.hip_bend_factor = PHRControl_cal->DataStoreMemory15_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory16' */
  PHRControl_DW.knee_bend_factor =
    PHRControl_cal->DataStoreMemory16_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory2' */
  PHRControl_DW.foot_y_flight = PHRControl_cal->DataStoreMemory2_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory3' */
  PHRControl_DW.SLIP_impact_angle =
    PHRControl_cal->DataStoreMemory3_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory4' */
  PHRControl_DW.foot_x_flight = PHRControl_cal->DataStoreMemory4_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory5' */
  PHRControl_DW.contactTime = PHRControl_cal->DataStoreMemory5_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory6' */
  PHRControl_DW.simTime = PHRControl_cal->DataStoreMemory6_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory7' */
  PHRControl_DW.hasHopped = PHRControl_cal->DataStoreMemory7_InitialValue;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory8' */
  PHRControl_DW.run = PHRControl_cal->DataStoreMemory8_InitialValue;

  /* InitializeConditions for Delay: '<Root>/Delay2' */
  PHRControl_DW.Delay2_DSTATE = PHRControl_cal->Delay2_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S4>/UD' */
  PHRControl_DW.UD_DSTATE =
    PHRControl_cal->DiscreteDerivative1_ICPrevScaledInput;

  /* InitializeConditions for UnitDelay: '<S3>/UD' */
  PHRControl_DW.UD_DSTATE_n =
    PHRControl_cal->DiscreteDerivative_ICPrevScaledInput;

  /* InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  PHRControl_DW.DiscreteTimeIntegrator_DSTATE =
    PHRControl_cal->DiscreteTimeIntegrator_IC_m;
  PHRControl_DW.DiscreteTimeIntegrator_PrevResetState = 2;

  /* InitializeConditions for Delay: '<Root>/Delay1' */
  PHRControl_DW.Delay1_DSTATE[0] = PHRControl_cal->Delay1_InitialCondition[0];
  PHRControl_DW.Delay1_DSTATE[1] = PHRControl_cal->Delay1_InitialCondition[1];

  /* Start for S-Function (sg_IO602_IO691_read_s): '<S8>/CAN Read' */
  /* Level2 S-Function Block: '<S8>/CAN Read' (sg_IO602_IO691_read_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[4];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (scanunpack): '<S8>/CAN Unpack' */

  /*-----------S-Function Block: <S8>/CAN Unpack -----------------*/

  /* Start for S-Function (scanunpack): '<S8>/CAN Unpack1' */

  /*-----------S-Function Block: <S8>/CAN Unpack1 -----------------*/

  /* Start for S-Function (scanunpack): '<S8>/CAN Unpack2' */

  /*-----------S-Function Block: <S8>/CAN Unpack2 -----------------*/

  /* Start for S-Function (scanunpack): '<S8>/CAN Unpack3' */

  /*-----------S-Function Block: <S8>/CAN Unpack3 -----------------*/

  /* SystemInitialize for Outport: '<S8>/Z' */
  PHRControl_B.z = PHRControl_cal->Z_Y0;

  /* SystemInitialize for Outport: '<S8>/Xd' */
  PHRControl_B.xd = PHRControl_cal->Xd_Y0;

  /* SystemInitialize for Gain: '<S8>/Gain' incorporates:
   *  Outport: '<S8>/th1'
   */
  PHRControl_B.Gain = PHRControl_cal->th1_Y0;

  /* SystemInitialize for Gain: '<S8>/Gain2' incorporates:
   *  Outport: '<S8>/th2'
   */
  PHRControl_B.Gain2 = PHRControl_cal->th2_Y0;

  /* SystemInitialize for Outport: '<S8>/I1' */
  PHRControl_B.sf_bytesfloats.I_ff = PHRControl_cal->I1_Y0;

  /* SystemInitialize for Outport: '<S8>/I2' */
  PHRControl_B.sf_bytesfloats1.I_ff = PHRControl_cal->I2_Y0;

  /* SystemInitialize for Outport: '<S8>/X' */
  PHRControl_B.x = PHRControl_cal->X_Y0;

  /* SystemInitialize for Outport: '<S8>/Zd' */
  PHRControl_B.zd = PHRControl_cal->Zd_Y0;

  /* End of SystemInitialize for SubSystem: '<Root>/Read All' */

  /* SystemInitialize for Chart: '<Root>/State-Transition Diagram' */
  PHRControl_DW.sfEvent_e = -1;
  PHRControl_DW.temporalCounter_i1_m = 0U;
  PHRControl_DW.is_active_c4_PHRControl = 0U;
  PHRControl_DW.is_c4_PHRControl = PHRControl_IN_NO_ACTIVE_CHILD;
  PHRControl_DW.contactTime_l = 0.0;
  PHRControl_DW.landingTime_m = 0.0;
  PHRControl_DW.sfEvent_c = -1;
  PHRControl_DW.temporalCounter_i1_a = 0U;
  PHRControl_DW.is_active_c3_PHRControl = 0U;
  PHRControl_DW.is_c3_PHRControl = PHRControl_IN_NO_ACTIVE_CHILD;
  PHRControl_DW.landingTime = 0.0;
  PHRControl_DW.sfEvent_h = -1;
  PHRControl_DW.temporalCounter_i1_b = 0U;
  PHRControl_DW.is_active_c5_PHRControl = 0U;
  PHRControl_DW.is_c5_PHRControl = PHRControl_IN_NO_ACTIVE_CHILD;
  PHRControl_DW.landingTime_g = 0.0;
  PHRControl_DW.sfEvent = -1;
  PHRControl_DW.is_Command1 = PHRControl_IN_NO_ACTIVE_CHILD;
  PHRControl_DW.temporalCounter_i1 = 0U;
  PHRControl_DW.is_active_c8_PHRControl = 0U;
  PHRControl_DW.is_c8_PHRControl = PHRControl_IN_NO_ACTIVE_CHILD;

  /* SystemInitialize for Merge: '<S9>/ Merge 3' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   */
  PHRControl_B.n = 0.0;

  /* SystemInitialize for Merge: '<S9>/ Merge ' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   */
  PHRControl_B.MachineState = 0.0;

  /* SystemInitialize for Chart: '<Root>/State-Transition Diagram' */
  PHRControl_B.stop = 0.0;
  PHRControl_B.theta1 = 0.0;
  PHRControl_B.theta2 = 0.0;
  PHRControl_B.c = 0.0;

  /* SystemInitialize for Merge: '<S9>/ Merge 2' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   */
  PHRControl_B.T1 = 0.0;

  /* SystemInitialize for Merge: '<S9>/ Merge 1' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   */
  PHRControl_B.T2 = 0.0;

  /* SystemInitialize for Chart: '<Root>/State-Transition Diagram' */
  PHRControl_B.Kp1 = 5.0;
  PHRControl_B.Kp2 = 5.0;
  PHRControl_B.Kd1 = 0.4;
  PHRControl_B.Kd2 = 0.4;

  /* SystemInitialize for IfAction SubSystem: '<S17>/Flight' */
  /* InitializeConditions for DiscreteIntegrator: '<S403>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.Raibert'
   */
  PHRControl_DW.Filter_DSTATE_ai =
    PHRControl_cal->PIDController_InitialConditionForFilter_i;

  /* InitializeConditions for DiscreteIntegrator: '<S408>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.Raibert'
   */
  PHRControl_DW.Integrator_DSTATE_pp =
    PHRControl_cal->PIDController_InitialConditionForIntegrator_n;

  /* InitializeConditions for DiscreteIntegrator: '<S451>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.Raibert'
   */
  PHRControl_DW.Filter_DSTATE_el =
    PHRControl_cal->PIDController1_InitialConditionForFilter_b;

  /* InitializeConditions for DiscreteIntegrator: '<S456>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.Raibert'
   */
  PHRControl_DW.Integrator_DSTATE_b =
    PHRControl_cal->PIDController1_InitialConditionForIntegrator_d;

  /* End of SystemInitialize for SubSystem: '<S17>/Flight' */

  /* SystemInitialize for IfAction SubSystem: '<S17>/Idle' */

  /* SystemInitialize for Chart: '<S9>/Command1.Raibert' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   */
  PHRControl_Idle_Init(&PHRControl_DW.Idle_l,
                       &PHRControl_cal->PHRControl_Idle_l_cal);

  /* End of SystemInitialize for SubSystem: '<S17>/Idle' */

  /* SystemInitialize for IfAction SubSystem: '<S17>/Thrust' */
  /* InitializeConditions for DiscreteIntegrator: '<S598>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.Raibert'
   */
  PHRControl_DW.Filter_DSTATE_k =
    PHRControl_cal->PIDController2_InitialConditionForFilter_e;

  /* InitializeConditions for DiscreteIntegrator: '<S603>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.Raibert'
   */
  PHRControl_DW.Integrator_DSTATE_c =
    PHRControl_cal->PIDController2_InitialConditionForIntegrator_f;

  /* End of SystemInitialize for SubSystem: '<S17>/Thrust' */

  /* SystemInitialize for IfAction SubSystem: '<S17>/Compress' */
  /* InitializeConditions for DiscreteIntegrator: '<S353>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.Raibert'
   */
  PHRControl_DW.Filter_DSTATE_ak =
    PHRControl_cal->PIDController2_InitialConditionForFilter_j;

  /* InitializeConditions for DiscreteIntegrator: '<S358>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.Raibert'
   */
  PHRControl_DW.Integrator_DSTATE_oc =
    PHRControl_cal->PIDController2_InitialConditionForIntegrator_j;

  /* End of SystemInitialize for SubSystem: '<S17>/Compress' */

  /* SystemInitialize for IfAction SubSystem: '<S18>/Flight' */
  /* InitializeConditions for DiscreteIntegrator: '<S654>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.SLIP'
   */
  PHRControl_DW.Filter_DSTATE_e =
    PHRControl_cal->PIDController_InitialConditionForFilter_n;

  /* InitializeConditions for DiscreteIntegrator: '<S659>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.SLIP'
   */
  PHRControl_DW.Integrator_DSTATE_o =
    PHRControl_cal->PIDController_InitialConditionForIntegrator_f;

  /* InitializeConditions for DiscreteIntegrator: '<S702>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.SLIP'
   */
  PHRControl_DW.Filter_DSTATE_a =
    PHRControl_cal->PIDController1_InitialConditionForFilter_p;

  /* InitializeConditions for DiscreteIntegrator: '<S707>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.SLIP'
   */
  PHRControl_DW.Integrator_DSTATE_p =
    PHRControl_cal->PIDController1_InitialConditionForIntegrator_dr;

  /* End of SystemInitialize for SubSystem: '<S18>/Flight' */

  /* SystemInitialize for IfAction SubSystem: '<S18>/Idle' */
  /* InitializeConditions for DiscreteIntegrator: '<S752>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.SLIP'
   */
  PHRControl_DW.Filter_DSTATE =
    PHRControl_cal->PIDController_InitialConditionForFilter_j;

  /* InitializeConditions for DiscreteIntegrator: '<S757>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.SLIP'
   */
  PHRControl_DW.Integrator_DSTATE =
    PHRControl_cal->PIDController_InitialConditionForIntegrator_p;

  /* InitializeConditions for DiscreteIntegrator: '<S800>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.SLIP'
   */
  PHRControl_DW.Filter_DSTATE_c =
    PHRControl_cal->PIDController1_InitialConditionForFilter_h;

  /* InitializeConditions for DiscreteIntegrator: '<S805>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.SLIP'
   */
  PHRControl_DW.Integrator_DSTATE_j =
    PHRControl_cal->PIDController1_InitialConditionForIntegrator_o;

  /* End of SystemInitialize for SubSystem: '<S18>/Idle' */
  /* InitializeConditions for DiscreteIntegrator: '<S52>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.HSW_Controller'
   */
  PHRControl_DW.Filter_DSTATE_f =
    PHRControl_cal->PIDController_InitialConditionForFilter;

  /* InitializeConditions for DiscreteIntegrator: '<S57>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.HSW_Controller'
   */
  PHRControl_DW.Integrator_DSTATE_e =
    PHRControl_cal->PIDController_InitialConditionForIntegrator;

  /* InitializeConditions for DiscreteIntegrator: '<S100>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.HSW_Controller'
   */
  PHRControl_DW.Filter_DSTATE_fx =
    PHRControl_cal->PIDController1_InitialConditionForFilter;

  /* InitializeConditions for DiscreteIntegrator: '<S105>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.HSW_Controller'
   */
  PHRControl_DW.Integrator_DSTATE_a1 =
    PHRControl_cal->PIDController1_InitialConditionForIntegrator;

  /* End of SystemInitialize for SubSystem: '<S16>/Flight' */

  /* SystemInitialize for IfAction SubSystem: '<S16>/Idle' */

  /* SystemInitialize for Chart: '<S9>/Command1.HSW_Controller' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   */
  PHRControl_Idle_Init(&PHRControl_DW.Idle, &PHRControl_cal->PHRControl_Idle_cal);

  /* End of SystemInitialize for SubSystem: '<S16>/Idle' */

  /* SystemInitialize for IfAction SubSystem: '<S16>/HSW' */
  /* InitializeConditions for DiscreteIntegrator: '<S20>/Discrete-Time Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.HSW_Controller'
   */
  PHRControl_DW.DiscreteTimeIntegrator_DSTATE_c =
    PHRControl_cal->DiscreteTimeIntegrator_IC;
  PHRControl_DW.DiscreteTimeIntegrator_PrevResetState_c = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S152>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.HSW_Controller'
   */
  PHRControl_DW.Filter_DSTATE_ct =
    PHRControl_cal->PIDController2_InitialConditionForFilter;

  /* InitializeConditions for DiscreteIntegrator: '<S157>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.HSW_Controller'
   */
  PHRControl_DW.Integrator_DSTATE_l =
    PHRControl_cal->PIDController2_InitialConditionForIntegrator;

  /* InitializeConditions for DiscreteIntegrator: '<S200>/Filter' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.HSW_Controller'
   */
  PHRControl_DW.Filter_DSTATE_h =
    PHRControl_cal->PIDController3_InitialConditionForFilter;

  /* InitializeConditions for DiscreteIntegrator: '<S205>/Integrator' incorporates:
   *  Chart: '<Root>/State-Transition Diagram'
   *  Chart: '<S9>/Command1.HSW_Controller'
   */
  PHRControl_DW.Integrator_DSTATE_a =
    PHRControl_cal->PIDController3_InitialConditionForIntegrator;

  /* End of SystemInitialize for SubSystem: '<S16>/HSW' */

  /* SystemInitialize for Enabled SubSystem: '<Root>/Enabled Subsystem' */

  /* Start for S-Function (sg_IO602_IO691_write_s): '<S5>/CAN Write1' */
  /* Level2 S-Function Block: '<S5>/CAN Write1' (sg_IO602_IO691_write_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[0];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* Start for S-Function (sg_IO602_IO691_write_s): '<S5>/CAN Write2' */
  /* Level2 S-Function Block: '<S5>/CAN Write2' (sg_IO602_IO691_write_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[1];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* End of SystemInitialize for SubSystem: '<Root>/Enabled Subsystem' */

  /* SystemInitialize for Atomic SubSystem: '<Root>/ID1 Write Hip Data' */

  /* Start for S-Function (sg_IO602_IO691_write_s): '<S6>/CAN Write1' */
  /* Level2 S-Function Block: '<S6>/CAN Write1' (sg_IO602_IO691_write_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[2];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* End of SystemInitialize for SubSystem: '<Root>/ID1 Write Hip Data' */

  /* SystemInitialize for Atomic SubSystem: '<Root>/ID2 Write Knee Data' */

  /* Start for S-Function (sg_IO602_IO691_write_s): '<S7>/CAN Write1' */
  /* Level2 S-Function Block: '<S7>/CAN Write1' (sg_IO602_IO691_write_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[3];
    sfcnStart(rts);
    if (ssGetErrorStatus(rts) != (NULL))
      return;
  }

  /* End of SystemInitialize for SubSystem: '<Root>/ID2 Write Knee Data' */
}

/* Model terminate function */
void PHRControl_terminate(void)
{
  /* Terminate for S-Function (sg_IO602_IO691_setup_s): '<Root>/CAN Setup ' */
  /* Level2 S-Function Block: '<Root>/CAN Setup ' (sg_IO602_IO691_setup_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[5];
    sfcnTerminate(rts);
  }

  /* Terminate for Iterator SubSystem: '<Root>/Read All' */

  /* Terminate for S-Function (sg_IO602_IO691_read_s): '<S8>/CAN Read' */
  /* Level2 S-Function Block: '<S8>/CAN Read' (sg_IO602_IO691_read_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[4];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<Root>/Read All' */

  /* Terminate for Enabled SubSystem: '<Root>/Enabled Subsystem' */

  /* Terminate for S-Function (sg_IO602_IO691_write_s): '<S5>/CAN Write1' */
  /* Level2 S-Function Block: '<S5>/CAN Write1' (sg_IO602_IO691_write_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Terminate for S-Function (sg_IO602_IO691_write_s): '<S5>/CAN Write2' */
  /* Level2 S-Function Block: '<S5>/CAN Write2' (sg_IO602_IO691_write_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[1];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<Root>/Enabled Subsystem' */

  /* Terminate for Atomic SubSystem: '<Root>/ID1 Write Hip Data' */

  /* Terminate for S-Function (sg_IO602_IO691_write_s): '<S6>/CAN Write1' */
  /* Level2 S-Function Block: '<S6>/CAN Write1' (sg_IO602_IO691_write_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[2];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<Root>/ID1 Write Hip Data' */

  /* Terminate for Atomic SubSystem: '<Root>/ID2 Write Knee Data' */

  /* Terminate for S-Function (sg_IO602_IO691_write_s): '<S7>/CAN Write1' */
  /* Level2 S-Function Block: '<S7>/CAN Write1' (sg_IO602_IO691_write_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[3];
    sfcnTerminate(rts);
  }

  /* End of Terminate for SubSystem: '<Root>/ID2 Write Knee Data' */

  /* Terminate for S-Function (sg_IO602_IO691_status_s): '<Root>/CAN Status' */
  /* Level2 S-Function Block: '<Root>/CAN Status' (sg_IO602_IO691_status_s) */
  {
    SimStruct *rts = PHRControl_M->childSfunctions[6];
    sfcnTerminate(rts);
  }
}

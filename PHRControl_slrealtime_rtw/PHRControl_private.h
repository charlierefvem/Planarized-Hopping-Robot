/*
 * PHRControl_private.h
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

#ifndef RTW_HEADER_PHRControl_private_h_
#define RTW_HEADER_PHRControl_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "PHRControl.h"
#include "PHRControl_cal.h"
#include "can_message.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern CAN_DATATYPE CAN_DATATYPE_GROUND;
extern real_T rt_roundd_snf(real_T u);
extern void* slrtRegisterSignalToLoggingService(uintptr_t sigAddr);
extern "C" void sg_IO602_IO691_write_s(SimStruct *rts);
extern "C" void sg_IO602_IO691_read_s(SimStruct *rts);
extern "C" void sg_IO602_IO691_setup_s(SimStruct *rts);
extern "C" void sg_IO602_IO691_status_s(SimStruct *rts);
extern void PHRControl_floatsbytes(real_T rtu_position, real_T rtu_velocity,
  real_T rtu_K_p, real_T rtu_K_d, real_T rtu_T_ff, B_floatsbytes_PHRControl_T
  *localB);
extern void PHRControl_bytesfloats(B_bytesfloats_PHRControl_T *localB);
extern void PHRControl_Idle_Init(DW_Idle_PHRControl_T *localDW,
  PHRControl_Idle_cal_type *PHRControl_PageSwitching_arg);
extern void PHRControl_Idle(const real_T rtu_measLeg[2], real_T *rty_T1, real_T *
  rty_T2, real_T *rty_Machine_State, B_Idle_PHRControl_T *localB,
  DW_Idle_PHRControl_T *localDW, PHRControl_Idle_cal_type
  *PHRControl_PageSwitching_arg);
extern void PHRControl_Load(real_T *rty_T1, real_T *rty_T2, real_T
  *rty_Machine_State, PHRControl_Load_cal_type *PHRControl_PageSwitching_arg);
extern void PHRControl_Unload(real_T *rty_T1, real_T *rty_T2, real_T
  *rty_Machine_State, PHRControl_Unload_cal_type *PHRControl_PageSwitching_arg);

#endif                                 /* RTW_HEADER_PHRControl_private_h_ */

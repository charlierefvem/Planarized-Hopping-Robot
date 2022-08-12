/*
 * PHRControl.h
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

#ifndef RTW_HEADER_PHRControl_h_
#define RTW_HEADER_PHRControl_h_
#include <math.h>
#include <cstring>
#include <logsrv.h>
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "verify/verifyIntrf.h"
#include "PHRControl_types.h"
#include "can_message.h"
#include <stddef.h>
#include "PHRControl_cal.h"

extern "C" {

#include "rtGetInf.h"

}
  extern "C"
{

#include "rt_nonfinite.h"

}

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetSampleHitArray
#define rtmGetSampleHitArray(rtm)      ((rtm)->Timing.sampleHitArray)
#endif

#ifndef rtmGetStepSize
#define rtmGetStepSize(rtm)            ((rtm)->Timing.stepSize)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGet_TimeOfLastOutput
#define rtmGet_TimeOfLastOutput(rtm)   ((rtm)->Timing.timeOfLastOutput)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

#ifndef rtmGetTimeOfLastOutput
#define rtmGetTimeOfLastOutput(rtm)    ((rtm)->Timing.timeOfLastOutput)
#endif

/* Block signals for system '<S6>/floats -> bytes' */
struct B_floatsbytes_PHRControl_T {
  uint8_T b[8];                        /* '<S6>/floats -> bytes' */
};

/* Block signals for system '<S8>/bytes -> floats' */
struct B_bytesfloats_PHRControl_T {
  real_T position;                     /* '<S8>/bytes -> floats' */
  real_T velocity;                     /* '<S8>/bytes -> floats' */
  real_T I_ff;                         /* '<S8>/bytes -> floats' */
};

/* Block signals for system '<S16>/Idle' */
struct B_Idle_PHRControl_T {
  real_T Sum1;                         /* '<S21>/Sum1' */
  real_T DerivativeGain;               /* '<S249>/Derivative Gain' */
  real_T Filter;                       /* '<S250>/Filter' */
  real_T SumD;                         /* '<S250>/SumD' */
  real_T IntegralGain;                 /* '<S252>/Integral Gain' */
  real_T Integrator;                   /* '<S255>/Integrator' */
  real_T FilterCoefficient;            /* '<S258>/Filter Coefficient' */
  real_T ProportionalGain;             /* '<S260>/Proportional Gain' */
  real_T Sum;                          /* '<S264>/Sum' */
  real_T Sum6;                         /* '<S21>/Sum6' */
  real_T Sum2;                         /* '<S21>/Sum2' */
  real_T DerivativeGain_f;             /* '<S297>/Derivative Gain' */
  real_T Filter_k;                     /* '<S298>/Filter' */
  real_T SumD_l;                       /* '<S298>/SumD' */
  real_T IntegralGain_m;               /* '<S300>/Integral Gain' */
  real_T Integrator_j;                 /* '<S303>/Integrator' */
  real_T FilterCoefficient_i;          /* '<S306>/Filter Coefficient' */
  real_T ProportionalGain_h;           /* '<S308>/Proportional Gain' */
  real_T Sum_b;                        /* '<S312>/Sum' */
};

/* Block states (default storage) for system '<S16>/Idle' */
struct DW_Idle_PHRControl_T {
  real_T Filter_DSTATE;                /* '<S250>/Filter' */
  real_T Integrator_DSTATE;            /* '<S255>/Integrator' */
  real_T Filter_DSTATE_f;              /* '<S298>/Filter' */
  real_T Integrator_DSTATE_g;          /* '<S303>/Integrator' */
  int8_T Idle_SubsysRanBC;             /* '<S16>/Idle' */
};

/* Block states (default storage) for system '<S16>/Load' */
struct DW_Load_PHRControl_T {
  int8_T Load_SubsysRanBC;             /* '<S16>/Load' */
};

/* Block states (default storage) for system '<S17>/Unload' */
struct DW_Unload_PHRControl_T {
  int8_T Unload_SubsysRanBC;           /* '<S17>/Unload' */
};

/* Block signals (default storage) */
struct B_PHRControl_T {
  CAN_DATATYPE CANRead_o2;             /* '<S8>/CAN Read' */
  CAN_DATATYPE CANmsg2;                /* '<S7>/CAN Pack1' */
  CAN_DATATYPE CANPack1;               /* '<S6>/CAN Pack1' */
  CAN_DATATYPE CANPack1_j;             /* '<S5>/CAN Pack1' */
  CAN_DATATYPE CANPack2;               /* '<S5>/CAN Pack2' */
  real_T Clock;                        /* '<Root>/Clock' */
  real_T RateTransition;               /* '<Root>/Rate Transition' */
  real_T TSamp;                        /* '<S4>/TSamp' */
  real_T Uk1;                          /* '<S4>/UD' */
  real_T Diff;                         /* '<S4>/Diff' */
  real_T TSamp_j;                      /* '<S3>/TSamp' */
  real_T Uk1_d;                        /* '<S3>/UD' */
  real_T Diff_l;                       /* '<S3>/Diff' */
  real_T isReady;                      /* '<Root>/isReady' */
  real_T DiscreteTimeIntegrator;       /* '<Root>/Discrete-Time Integrator' */
  real_T Switch;                       /* '<Root>/Switch' */
  real_T Delay1[2];                    /* '<Root>/Delay1' */
  real_T MultiportSwitch;              /* '<Root>/Multiport Switch' */
  real_T Gain10;                       /* '<Root>/Gain10' */
  real_T Gain8;                        /* '<Root>/Gain8' */
  real_T Gain6;                        /* '<Root>/Gain6' */
  real_T Gain1;                        /* '<Root>/Gain1' */
  real_T TmpSignalConversionAtDelay1Inport1[2];
  real_T stop;                         /* '<Root>/State-Transition Diagram' */
  real_T theta1;                       /* '<Root>/State-Transition Diagram' */
  real_T theta2;                       /* '<Root>/State-Transition Diagram' */
  real_T c;                            /* '<Root>/State-Transition Diagram' */
  real_T Kp1;                          /* '<Root>/State-Transition Diagram' */
  real_T Kp2;                          /* '<Root>/State-Transition Diagram' */
  real_T Kd1;                          /* '<Root>/State-Transition Diagram' */
  real_T Kd2;                          /* '<Root>/State-Transition Diagram' */
  real_T MachineState;                 /* '<Root>/State-Transition Diagram' */
  real_T T2;                           /* '<Root>/State-Transition Diagram' */
  real_T T1;                           /* '<Root>/State-Transition Diagram' */
  real_T n;                            /* '<Root>/State-Transition Diagram' */
  real_T Cos;                          /* '<S822>/Cos' */
  real_T Sum6;                         /* '<S822>/Sum6' */
  real_T Sum5;                         /* '<S822>/Sum5' */
  real_T Sin3;                         /* '<S822>/Sin3' */
  real_T Product2;                     /* '<S822>/Product2' */
  real_T Sin3_e;                       /* '<S827>/Sin3' */
  real_T Product4;                     /* '<S827>/Product4' */
  real_T Sum;                          /* '<S827>/Sum' */
  real_T Sin2;                         /* '<S827>/Sin2' */
  real_T Product3;                     /* '<S827>/Product3' */
  real_T footxposition;                /* '<S827>/Sum6' */
  real_T Square2;                      /* '<S827>/Square2' */
  real_T Sin5;                         /* '<S827>/Sin5' */
  real_T Product6;                     /* '<S827>/Product6' */
  real_T Sin4;                         /* '<S827>/Sin4' */
  real_T Product5;                     /* '<S827>/Product5' */
  real_T footyposition;                /* '<S827>/Sum7' */
  real_T Square3;                      /* '<S827>/Square3' */
  real_T Sum8;                         /* '<S827>/Sum8' */
  real_T currentlength;                /* '<S827>/Square Root1' */
  real_T Sum2;                         /* '<S822>/Sum2' */
  real_T Sin6;                         /* '<S825>/Sin6' */
  real_T Product2_c;                   /* '<S825>/Product2' */
  real_T Sum_b;                        /* '<S825>/Sum' */
  real_T Sin1;                         /* '<S825>/Sin1' */
  real_T Product1;                     /* '<S825>/Product1' */
  real_T footxposition_h;              /* '<S825>/Sum1' */
  real_T Sin8;                         /* '<S825>/Sin8' */
  real_T Product8;                     /* '<S825>/Product8' */
  real_T Sin7;                         /* '<S825>/Sin7' */
  real_T Product7;                     /* '<S825>/Product7' */
  real_T footyposition_c;              /* '<S825>/Sum2' */
  real_T Divide;                       /* '<S825>/Divide' */
  real_T TrigonometricFunction;        /* '<S825>/Trigonometric Function' */
  real_T Sin1_p;                       /* '<S822>/Sin1' */
  real_T Product1_a;                   /* '<S822>/Product1' */
  real_T Divide_f;                     /* '<S822>/Divide' */
  real_T Sin3_j;                       /* '<S826>/Sin3' */
  real_T Product4_p;                   /* '<S826>/Product4' */
  real_T Sum_p;                        /* '<S826>/Sum' */
  real_T Sin2_f;                       /* '<S826>/Sin2' */
  real_T Product3_c;                   /* '<S826>/Product3' */
  real_T footxposition_a;              /* '<S826>/Sum6' */
  real_T Square2_e;                    /* '<S826>/Square2' */
  real_T Sin5_e;                       /* '<S826>/Sin5' */
  real_T Product6_i;                   /* '<S826>/Product6' */
  real_T Sin4_m;                       /* '<S826>/Sin4' */
  real_T Product5_c;                   /* '<S826>/Product5' */
  real_T footyposition_h;              /* '<S826>/Sum7' */
  real_T Square3_l;                    /* '<S826>/Square3' */
  real_T Sum8_b;                       /* '<S826>/Sum8' */
  real_T currentlength_g;              /* '<S826>/Square Root1' */
  real_T Sum_i;                        /* '<S822>/Sum' */
  real_T Sin6_p;                       /* '<S824>/Sin6' */
  real_T Product2_k;                   /* '<S824>/Product2' */
  real_T Sum_c;                        /* '<S824>/Sum' */
  real_T Sin1_g;                       /* '<S824>/Sin1' */
  real_T Product1_p;                   /* '<S824>/Product1' */
  real_T footxposition_he;             /* '<S824>/Sum1' */
  real_T Sin8_e;                       /* '<S824>/Sin8' */
  real_T Product8_m;                   /* '<S824>/Product8' */
  real_T Sin7_h;                       /* '<S824>/Sin7' */
  real_T Product7_a;                   /* '<S824>/Product7' */
  real_T footyposition_a;              /* '<S824>/Sum2' */
  real_T Divide_m;                     /* '<S824>/Divide' */
  real_T TrigonometricFunction_m;      /* '<S824>/Trigonometric Function' */
  real_T Sin;                          /* '<S822>/Sin' */
  real_T Product;                      /* '<S822>/Product' */
  real_T hiptorquenum;                 /* '<S822>/Sum1' */
  real_T mult2;                        /* '<S822>/Sin4' */
  real_T Sum4;                         /* '<S822>/Sum4' */
  real_T Sin5_a;                       /* '<S822>/Sin5' */
  real_T Product3_b;                   /* '<S822>/Product3' */
  real_T mult3;                        /* '<S822>/Sum3' */
  real_T hiptorquedenom;               /* '<S822>/Product4' */
  real_T hiptorque;                    /* '<S822>/Divide1' */
  real_T Minus;                        /* '<S822>/Minus' */
  real_T Sin6_b;                       /* '<S828>/Sin6' */
  real_T Product2_i;                   /* '<S828>/Product2' */
  real_T Sum7;                         /* '<S828>/Sum7' */
  real_T Sin1_k;                       /* '<S828>/Sin1' */
  real_T Product1_c;                   /* '<S828>/Product1' */
  real_T footxposition_i;              /* '<S828>/Sum1' */
  real_T Sin8_l;                       /* '<S828>/Sin8' */
  real_T Product8_a;                   /* '<S828>/Product8' */
  real_T Sin7_f;                       /* '<S828>/Sin7' */
  real_T Product7_ad;                  /* '<S828>/Product7' */
  real_T footyposition_n;              /* '<S828>/Sum2' */
  real_T Divide_j;                     /* '<S828>/Divide' */
  real_T TrigonometricFunction_g;      /* '<S828>/Trigonometric Function' */
  real_T Cos_f;                        /* '<S823>/Cos' */
  real_T Cos1;                         /* '<S823>/Cos1' */
  real_T Sum7_k;                       /* '<S823>/Sum7' */
  real_T Sum4_l;                       /* '<S823>/Sum4' */
  real_T Cos2;                         /* '<S823>/Cos2' */
  real_T Product1_i;                   /* '<S823>/Product1' */
  real_T Sin3_g;                       /* '<S829>/Sin3' */
  real_T Product4_n;                   /* '<S829>/Product4' */
  real_T Sum1;                         /* '<S829>/Sum1' */
  real_T Sin2_l;                       /* '<S829>/Sin2' */
  real_T Product3_j;                   /* '<S829>/Product3' */
  real_T footxposition_m;              /* '<S829>/Sum6' */
  real_T Square2_b;                    /* '<S829>/Square2' */
  real_T Sin5_g;                       /* '<S829>/Sin5' */
  real_T Product6_j;                   /* '<S829>/Product6' */
  real_T Sin4_f;                       /* '<S829>/Sin4' */
  real_T Product5_m;                   /* '<S829>/Product5' */
  real_T footyposition_g;              /* '<S829>/Sum7' */
  real_T Square3_j;                    /* '<S829>/Square3' */
  real_T Sum8_g;                       /* '<S829>/Sum8' */
  real_T currentlength_b;              /* '<S829>/Square Root1' */
  real_T Sum_ch;                       /* '<S823>/Sum' */
  real_T Product_l;                    /* '<S823>/Product' */
  real_T Sum1_h;                       /* '<S823>/Sum1' */
  real_T Product2_p;                   /* '<S823>/Product2' */
  real_T kneetorque;                   /* '<S823>/Divide' */
  real_T Sum1_a;                       /* '<S621>/Sum1' */
  real_T DerivativeGain;               /* '<S751>/Derivative Gain' */
  real_T Filter;                       /* '<S752>/Filter' */
  real_T SumD;                         /* '<S752>/SumD' */
  real_T IntegralGain;                 /* '<S754>/Integral Gain' */
  real_T Integrator;                   /* '<S757>/Integrator' */
  real_T FilterCoefficient;            /* '<S760>/Filter Coefficient' */
  real_T ProportionalGain;             /* '<S762>/Proportional Gain' */
  real_T Sum_ct;                       /* '<S766>/Sum' */
  real_T Sum6_i;                       /* '<S621>/Sum6' */
  real_T Sum2_k;                       /* '<S621>/Sum2' */
  real_T DerivativeGain_b;             /* '<S799>/Derivative Gain' */
  real_T Filter_k;                     /* '<S800>/Filter' */
  real_T SumD_f;                       /* '<S800>/SumD' */
  real_T IntegralGain_f;               /* '<S802>/Integral Gain' */
  real_T Integrator_i;                 /* '<S805>/Integrator' */
  real_T FilterCoefficient_p;          /* '<S808>/Filter Coefficient' */
  real_T ProportionalGain_f;           /* '<S810>/Proportional Gain' */
  real_T Sum_c1;                       /* '<S814>/Sum' */
  real_T Sin3_o;                       /* '<S627>/Sin3' */
  real_T Product4_i;                   /* '<S627>/Product4' */
  real_T Sum1_n;                       /* '<S627>/Sum1' */
  real_T Sin2_p;                       /* '<S627>/Sin2' */
  real_T Product3_bq;                  /* '<S627>/Product3' */
  real_T footxposition_j;              /* '<S627>/Sum6' */
  real_T Sin5_i;                       /* '<S627>/Sin5' */
  real_T Product6_je;                  /* '<S627>/Product6' */
  real_T Sin4_a;                       /* '<S627>/Sin4' */
  real_T Product5_l;                   /* '<S627>/Product5' */
  real_T footyposition_k;              /* '<S627>/Sum7' */
  real_T Square2_d;                    /* '<S627>/Square2' */
  real_T Square3_p;                    /* '<S627>/Square3' */
  real_T Sum8_gi;                      /* '<S627>/Sum8' */
  real_T currentlength_p;              /* '<S627>/Square Root1' */
  real_T DataStoreRead;                /* '<S620>/Data Store Read' */
  real_T Sum3;                         /* '<S620>/Sum3' */
  real_T NPOffset;                     /* '<S620>/Multiply' */
  real_T Sin_m;                        /* '<S620>/Sin' */
  real_T kneex;                        /* '<S620>/Product1' */
  real_T Sum7_f;                       /* '<S620>/Sum7' */
  real_T Sin1_a;                       /* '<S620>/Sin1' */
  real_T footx;                        /* '<S620>/Product2' */
  real_T x_actual;                     /* '<S620>/Sum2' */
  real_T NP;                           /* '<S620>/Product' */
  real_T X_desired;                    /* '<S620>/Sum4' */
  real_T error;                        /* '<S620>/Sum' */
  real_T DerivativeGain_o;             /* '<S653>/Derivative Gain' */
  real_T Filter_g;                     /* '<S654>/Filter' */
  real_T SumD_p;                       /* '<S654>/SumD' */
  real_T IntegralGain_i;               /* '<S656>/Integral Gain' */
  real_T Integrator_n;                 /* '<S659>/Integrator' */
  real_T FilterCoefficient_h;          /* '<S662>/Filter Coefficient' */
  real_T ProportionalGain_n;           /* '<S664>/Proportional Gain' */
  real_T Sum_o;                        /* '<S668>/Sum' */
  real_T Sum5_e;                       /* '<S620>/Sum5' */
  real_T DerivativeGain_d;             /* '<S701>/Derivative Gain' */
  real_T Filter_n;                     /* '<S702>/Filter' */
  real_T SumD_e;                       /* '<S702>/SumD' */
  real_T IntegralGain_p;               /* '<S704>/Integral Gain' */
  real_T Integrator_n0;                /* '<S707>/Integrator' */
  real_T FilterCoefficient_a;          /* '<S710>/Filter Coefficient' */
  real_T ProportionalGain_c;           /* '<S712>/Proportional Gain' */
  real_T Sum_h;                        /* '<S716>/Sum' */
  real_T impactangle;                  /* '<S324>/Data Store Read' */
  real_T Sum2_o;                       /* '<S324>/Sum2' */
  real_T desiredspeed;                 /* '<S324>/Gain' */
  real_T Sum_l;                        /* '<S324>/Sum' */
  real_T DerivativeGain_m;             /* '<S597>/Derivative Gain' */
  real_T Filter_c;                     /* '<S598>/Filter' */
  real_T SumD_m;                       /* '<S598>/SumD' */
  real_T IntegralGain_k;               /* '<S600>/Integral Gain' */
  real_T Integrator_k;                 /* '<S603>/Integrator' */
  real_T FilterCoefficient_l;          /* '<S606>/Filter Coefficient' */
  real_T ProportionalGain_m;           /* '<S608>/Proportional Gain' */
  real_T Sum_e;                        /* '<S612>/Sum' */
  real_T Sum3_b;                       /* '<S321>/Sum3' */
  real_T offsetfromneutralpointtocontrollocomotionspeed;/* '<S321>/Multiply' */
  real_T Sin_o;                        /* '<S321>/Sin' */
  real_T Product1_j;                   /* '<S321>/Product1' */
  real_T Sum6_d;                       /* '<S321>/Sum6' */
  real_T Sin1_b;                       /* '<S321>/Sin1' */
  real_T Product2_l;                   /* '<S321>/Product2' */
  real_T X_actual;                     /* '<S321>/Sum2' */
  real_T Product_h;                    /* '<S321>/Product' */
  real_T X_desired_k;                  /* '<S321>/Sum4' */
  real_T Sum_og;                       /* '<S321>/Sum' */
  real_T DerivativeGain_p;             /* '<S402>/Derivative Gain' */
  real_T Filter_kx;                    /* '<S403>/Filter' */
  real_T SumD_l;                       /* '<S403>/SumD' */
  real_T IntegralGain_g;               /* '<S405>/Integral Gain' */
  real_T Integrator_b;                 /* '<S408>/Integrator' */
  real_T FilterCoefficient_e;          /* '<S411>/Filter Coefficient' */
  real_T ProportionalGain_d;           /* '<S413>/Proportional Gain' */
  real_T Sum_od;                       /* '<S417>/Sum' */
  real_T Sum5_h;                       /* '<S321>/Sum5' */
  real_T DerivativeGain_e;             /* '<S450>/Derivative Gain' */
  real_T Filter_j;                     /* '<S451>/Filter' */
  real_T SumD_o;                       /* '<S451>/SumD' */
  real_T IntegralGain_c;               /* '<S453>/Integral Gain' */
  real_T Integrator_j;                 /* '<S456>/Integrator' */
  real_T FilterCoefficient_f;          /* '<S459>/Filter Coefficient' */
  real_T ProportionalGain_dm;          /* '<S461>/Proportional Gain' */
  real_T Sum_cr;                       /* '<S465>/Sum' */
  real_T DataStoreRead_e;              /* '<S320>/Data Store Read' */
  real_T Sum3_n;                       /* '<S320>/Sum3' */
  real_T Sum2_a;                       /* '<S320>/Sum2' */
  real_T DerivativeGain_h;             /* '<S352>/Derivative Gain' */
  real_T Filter_f;                     /* '<S353>/Filter' */
  real_T SumD_fv;                      /* '<S353>/SumD' */
  real_T IntegralGain_h;               /* '<S355>/Integral Gain' */
  real_T Integrator_o;                 /* '<S358>/Integrator' */
  real_T FilterCoefficient_c;          /* '<S361>/Filter Coefficient' */
  real_T ProportionalGain_mv;          /* '<S363>/Proportional Gain' */
  real_T Sum_g;                        /* '<S367>/Sum' */
  real_T DataStoreRead1;               /* '<S20>/Data Store Read1' */
  real_T Switch2;                      /* '<S20>/Switch2' */
  real_T DiscreteTimeIntegrator_f;     /* '<S20>/Discrete-Time Integrator' */
  real_T Sum_lf;                       /* '<S20>/Sum' */
  real_T DerivativeGain_f;             /* '<S151>/Derivative Gain' */
  real_T Filter_i;                     /* '<S152>/Filter' */
  real_T SumD_c;                       /* '<S152>/SumD' */
  real_T IntegralGain_a;               /* '<S154>/Integral Gain' */
  real_T Integrator_c;                 /* '<S157>/Integrator' */
  real_T FilterCoefficient_m;          /* '<S160>/Filter Coefficient' */
  real_T ProportionalGain_a;           /* '<S162>/Proportional Gain' */
  real_T Sum_hl;                       /* '<S166>/Sum' */
  real_T Sum2_p;                       /* '<S20>/Sum2' */
  real_T DerivativeGain_g;             /* '<S199>/Derivative Gain' */
  real_T Filter_d;                     /* '<S200>/Filter' */
  real_T SumD_en;                      /* '<S200>/SumD' */
  real_T IntegralGain_pj;              /* '<S202>/Integral Gain' */
  real_T Integrator_g;                 /* '<S205>/Integrator' */
  real_T FilterCoefficient_hx;         /* '<S208>/Filter Coefficient' */
  real_T ProportionalGain_p;           /* '<S210>/Proportional Gain' */
  real_T Sum_d;                        /* '<S214>/Sum' */
  real_T Sum4_f;                       /* '<S20>/Sum4' */
  real_T Sum6_dr;                      /* '<S20>/Sum6' */
  real_T T1_f;                         /* '<S20>/Torque Calculator' */
  real_T T2_e;                         /* '<S20>/Torque Calculator' */
  real_T HSW;                          /* '<S20>/Sine Generator' */
  real_T DataStoreRead_n;              /* '<S19>/Data Store Read' */
  real_T KneeJointAngle1;              /* '<S19>/Knee Joint Angle1' */
  real_T Sum3_h;                       /* '<S19>/Sum3' */
  real_T NPOffset_l;                   /* '<S19>/Multiply' */
  real_T Sin_k;                        /* '<S19>/Sin' */
  real_T kneex_e;                      /* '<S19>/Product1' */
  real_T Sum6_dy;                      /* '<S19>/Sum6' */
  real_T Sin1_bn;                      /* '<S19>/Sin1' */
  real_T footx_o;                      /* '<S19>/Product2' */
  real_T NP_h;                         /* '<S19>/Product' */
  real_T X_desired_e;                  /* '<S19>/Sum4' */
  real_T x_actual_m;                   /* '<S19>/Sum2' */
  real_T error_d;                      /* '<S19>/Sum' */
  real_T DerivativeGain_mb;            /* '<S51>/Derivative Gain' */
  real_T Filter_a;                     /* '<S52>/Filter' */
  real_T SumD_k;                       /* '<S52>/SumD' */
  real_T IntegralGain_m;               /* '<S54>/Integral Gain' */
  real_T Integrator_ob;                /* '<S57>/Integrator' */
  real_T FilterCoefficient_d;          /* '<S60>/Filter Coefficient' */
  real_T ProportionalGain_a5;          /* '<S62>/Proportional Gain' */
  real_T Sum_a;                        /* '<S66>/Sum' */
  real_T Sum5_c;                       /* '<S19>/Sum5' */
  real_T DerivativeGain_l;             /* '<S99>/Derivative Gain' */
  real_T Filter_jl;                    /* '<S100>/Filter' */
  real_T SumD_ct;                      /* '<S100>/SumD' */
  real_T IntegralGain_j;               /* '<S102>/Integral Gain' */
  real_T Integrator_p;                 /* '<S105>/Integrator' */
  real_T FilterCoefficient_hu;         /* '<S108>/Filter Coefficient' */
  real_T ProportionalGain_cc;          /* '<S110>/Proportional Gain' */
  real_T Sum_bj;                       /* '<S114>/Sum' */
  real_T CANUnpack_o1;                 /* '<S8>/CAN Unpack' */
  real_T CANUnpack_o2;                 /* '<S8>/CAN Unpack' */
  real_T CANUnpack_o3;                 /* '<S8>/CAN Unpack' */
  real_T ID;                           /* '<S8>/CAN Unpack' */
  real_T pos;                          /* '<S8>/CAN Unpack1' */
  real_T vel;                          /* '<S8>/CAN Unpack1' */
  real_T pos_h;                        /* '<S8>/CAN Unpack2' */
  real_T vel_e;                        /* '<S8>/CAN Unpack2' */
  real_T CANUnpack3_o1;                /* '<S8>/CAN Unpack3' */
  real_T CANUnpack3_o2;                /* '<S8>/CAN Unpack3' */
  real_T CANUnpack3_o3;                /* '<S8>/CAN Unpack3' */
  real_T ID_j;                         /* '<S8>/CAN Unpack3' */
  real_T Gain;                         /* '<S8>/Gain' */
  real_T Gain2;                        /* '<S8>/Gain2' */
  real_T x;                            /* '<S8>/Encoder 2 to Vertical Data' */
  real_T xd;                           /* '<S8>/Encoder 2 to Vertical Data' */
  real_T z;                            /* '<S8>/Encoder 1 to Vertical Data' */
  real_T zd;                           /* '<S8>/Encoder 1 to Vertical Data' */
  real_T danger;                       /* '<Root>/Danger Detector Function' */
  int32_T WhileIterator;               /* '<S8>/While Iterator' */
  uint8_T CANUnpack_o5;                /* '<S8>/CAN Unpack' */
  uint8_T CANUnpack1_o3;               /* '<S8>/CAN Unpack1' */
  uint8_T CANUnpack2_o3;               /* '<S8>/CAN Unpack2' */
  uint8_T CANUnpack3_o5;               /* '<S8>/CAN Unpack3' */
  uint8_T MultiportSwitch_e[8];        /* '<S7>/Multiport Switch' */
  uint8_T BytePacking[8];              /* '<S7>/Byte Packing' */
  uint8_T MultiportSwitch_l[8];        /* '<S6>/Multiport Switch' */
  uint8_T BytePacking_l[8];            /* '<S6>/Byte Packing' */
  boolean_T Delay2;                    /* '<Root>/Delay2' */
  boolean_T DataStoreRead_h;           /* '<S20>/Data Store Read' */
  boolean_T CANRead_o1;                /* '<S8>/CAN Read' */
  boolean_T ground;                    /* '<Root>/Collision Detector' */
  B_Idle_PHRControl_T Idle_l;          /* '<S17>/Idle' */
  B_Idle_PHRControl_T Idle;            /* '<S16>/Idle' */
  B_bytesfloats_PHRControl_T sf_bytesfloats1;/* '<S8>/bytes -> floats1' */
  B_bytesfloats_PHRControl_T sf_bytesfloats;/* '<S8>/bytes -> floats' */
  B_floatsbytes_PHRControl_T sf_floatsbytes_f;/* '<S7>/floats -> bytes' */
  B_floatsbytes_PHRControl_T sf_floatsbytes;/* '<S6>/floats -> bytes' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_PHRControl_T {
  real_T UD_DSTATE;                    /* '<S4>/UD' */
  real_T UD_DSTATE_n;                  /* '<S3>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<Root>/Discrete-Time Integrator' */
  real_T Delay1_DSTATE[2];             /* '<Root>/Delay1' */
  real_T Filter_DSTATE;                /* '<S752>/Filter' */
  real_T Integrator_DSTATE;            /* '<S757>/Integrator' */
  real_T Filter_DSTATE_c;              /* '<S800>/Filter' */
  real_T Integrator_DSTATE_j;          /* '<S805>/Integrator' */
  real_T Filter_DSTATE_e;              /* '<S654>/Filter' */
  real_T Integrator_DSTATE_o;          /* '<S659>/Integrator' */
  real_T Filter_DSTATE_a;              /* '<S702>/Filter' */
  real_T Integrator_DSTATE_p;          /* '<S707>/Integrator' */
  real_T Filter_DSTATE_k;              /* '<S598>/Filter' */
  real_T Integrator_DSTATE_c;          /* '<S603>/Integrator' */
  real_T Filter_DSTATE_ai;             /* '<S403>/Filter' */
  real_T Integrator_DSTATE_pp;         /* '<S408>/Integrator' */
  real_T Filter_DSTATE_el;             /* '<S451>/Filter' */
  real_T Integrator_DSTATE_b;          /* '<S456>/Integrator' */
  real_T Filter_DSTATE_ak;             /* '<S353>/Filter' */
  real_T Integrator_DSTATE_oc;         /* '<S358>/Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_c;/* '<S20>/Discrete-Time Integrator' */
  real_T Filter_DSTATE_ct;             /* '<S152>/Filter' */
  real_T Integrator_DSTATE_l;          /* '<S157>/Integrator' */
  real_T Filter_DSTATE_h;              /* '<S200>/Filter' */
  real_T Integrator_DSTATE_a;          /* '<S205>/Integrator' */
  real_T Filter_DSTATE_f;              /* '<S52>/Filter' */
  real_T Integrator_DSTATE_e;          /* '<S57>/Integrator' */
  real_T Filter_DSTATE_fx;             /* '<S100>/Filter' */
  real_T Integrator_DSTATE_a1;         /* '<S105>/Integrator' */
  real_T Hip_Impact_Angle;             /* '<Root>/Data Store Memory1' */
  real_T Knee_Impact_Angle;            /* '<Root>/Data Store Memory12' */
  real_T hip_range;                    /* '<Root>/Data Store Memory13' */
  real_T knee_range;                   /* '<Root>/Data Store Memory14' */
  real_T hip_bend_factor;              /* '<Root>/Data Store Memory15' */
  real_T knee_bend_factor;             /* '<Root>/Data Store Memory16' */
  real_T foot_y_flight;                /* '<Root>/Data Store Memory2' */
  real_T SLIP_impact_angle;            /* '<Root>/Data Store Memory3' */
  real_T foot_x_flight;                /* '<Root>/Data Store Memory4' */
  real_T contactTime;                  /* '<Root>/Data Store Memory5' */
  real_T simTime;                      /* '<Root>/Data Store Memory6' */
  real_T hasHopped;                    /* '<Root>/Data Store Memory7' */
  real_T run;                          /* '<Root>/Data Store Memory8' */
  real_T landingTime;                  /* '<S9>/Command1.SLIP' */
  real_T sf_internal_action_state_placeholder_data;/* '<S9>/Command1.SLIP' */
  real_T sf_internal_action_state_placeholder_data_f;/* '<S9>/Command1.SLIP' */
  real_T sf_internal_action_state_placeholder_data_n;/* '<S9>/Command1.SLIP' */
  real_T sf_internal_action_state_placeholder_data_j;/* '<S9>/Command1.SLIP' */
  real_T sf_internal_action_state_placeholder_data_e;/* '<S9>/Command1.SLIP' */
  real_T contactTime_l;                /* '<S9>/Command1.Raibert' */
  real_T landingTime_m;                /* '<S9>/Command1.Raibert' */
  real_T sf_internal_action_state_placeholder_data_h;/* '<S9>/Command1.Raibert' */
  real_T sf_internal_action_state_placeholder_data_p;/* '<S9>/Command1.Raibert' */
  real_T sf_internal_action_state_placeholder_data_g;/* '<S9>/Command1.Raibert' */
  real_T sf_internal_action_state_placeholder_data_b;/* '<S9>/Command1.Raibert' */
  real_T sf_internal_action_state_placeholder_data_i;/* '<S9>/Command1.Raibert' */
  real_T sf_internal_action_state_placeholder_data_k;/* '<S9>/Command1.Raibert' */
  real_T landingTime_g;                /* '<S9>/Command1.HSW_Controller' */
  real_T sf_internal_action_state_placeholder_data_l;/* '<S9>/Command1.HSW_Controller' */
  real_T sf_internal_action_state_placeholder_data_if;/* '<S9>/Command1.HSW_Controller' */
  real_T sf_internal_action_state_placeholder_data_gy;/* '<S9>/Command1.HSW_Controller' */
  real_T sf_internal_action_state_placeholder_data_n4;/* '<S9>/Command1.HSW_Controller' */
  real_T sf_internal_action_state_placeholder_data_a;/* '<S9>/Command1.HSW_Controller' */
  void *CANSetup_PWORK;                /* '<Root>/CAN Setup ' */
  struct {
    void *LoggedData;
  } ToWorkspace4_PWORK;                /* '<Root>/To Workspace4' */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_DangerDetectorFunction_at_outport_0_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_DiscreteDerivative1_at_outport_0_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_DiscreteDerivative_at_outport_0_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_StateTransitionDiagram_at_outport_10_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_StateTransitionDiagram_at_outport_3_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_StateTransitionDiagram_at_outport_4_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_StateTransitionDiagram_at_outport_5_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_StateTransitionDiagram_at_outport_6_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_StateTransitionDiagram_at_outport_8_PWORK;/* synthesized block */

  void* Flight_execBlock;              /* '<S9>/Command1.SLIP' */
  void* Unload_execBlock;              /* '<S9>/Command1.SLIP' */
  void* Load_execBlock;                /* '<S9>/Command1.SLIP' */
  void* Idle_execBlock;                /* '<S9>/Command1.SLIP' */
  void* Spring_execBlock;              /* '<S9>/Command1.SLIP' */
  void* Flight_execBlock_f;            /* '<S9>/Command1.Raibert' */
  void* Unload_execBlock_g;            /* '<S9>/Command1.Raibert' */
  void* Load_execBlock_o;              /* '<S9>/Command1.Raibert' */
  void* Idle_execBlock_d;              /* '<S9>/Command1.Raibert' */
  void* Thrust_execBlock;              /* '<S9>/Command1.Raibert' */
  void* Compress_execBlock;            /* '<S9>/Command1.Raibert' */
  void* Flight_execBlock_e;            /* '<S9>/Command1.HSW_Controller' */
  void* Unload_execBlock_f;            /* '<S9>/Command1.HSW_Controller' */
  void* Load_execBlock_m;              /* '<S9>/Command1.HSW_Controller' */
  void* Idle_execBlock_i;              /* '<S9>/Command1.HSW_Controller' */
  void* HSW_execBlock;                 /* '<S9>/Command1.HSW_Controller' */
  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_DataStoreRead_at_outport_0_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_KneeJointAngle1_at_outport_0_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Multiply_at_outport_0_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Product1_at_outport_0_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Product2_at_outport_0_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Product_at_outport_0_PWORK;/* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Sum4_at_outport_0_PWORK;/* synthesized block */

  void *CANRead_PWORK;                 /* '<S8>/CAN Read' */
  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_CANRead_at_outport_0_PWORK;/* synthesized block */

  void *CANWrite1_PWORK;               /* '<S7>/CAN Write1' */
  void *CANWrite1_PWORK_a;             /* '<S6>/CAN Write1' */
  void *CANWrite1_PWORK_l;             /* '<S5>/CAN Write1' */
  void *CANWrite2_PWORK;               /* '<S5>/CAN Write2' */
  int32_T sfEvent;                     /* '<Root>/State-Transition Diagram' */
  int32_T sfEvent_c;                   /* '<S9>/Command1.SLIP' */
  int32_T sfEvent_e;                   /* '<S9>/Command1.Raibert' */
  int32_T sfEvent_h;                   /* '<S9>/Command1.HSW_Controller' */
  uint32_T is_c8_PHRControl;           /* '<Root>/State-Transition Diagram' */
  uint32_T is_Command1;                /* '<Root>/State-Transition Diagram' */
  uint32_T is_c3_PHRControl;           /* '<S9>/Command1.SLIP' */
  uint32_T is_c4_PHRControl;           /* '<S9>/Command1.Raibert' */
  uint32_T is_c5_PHRControl;           /* '<S9>/Command1.HSW_Controller' */
  int_T CANStatus_IWORK[34];           /* '<Root>/CAN Status' */
  int_T CANUnpack_ModeSignalID;        /* '<S8>/CAN Unpack' */
  int_T CANUnpack_StatusPortID;        /* '<S8>/CAN Unpack' */
  int_T CANUnpack1_ModeSignalID;       /* '<S8>/CAN Unpack1' */
  int_T CANUnpack1_StatusPortID;       /* '<S8>/CAN Unpack1' */
  int_T CANUnpack2_ModeSignalID;       /* '<S8>/CAN Unpack2' */
  int_T CANUnpack2_StatusPortID;       /* '<S8>/CAN Unpack2' */
  int_T CANUnpack3_ModeSignalID;       /* '<S8>/CAN Unpack3' */
  int_T CANUnpack3_StatusPortID;       /* '<S8>/CAN Unpack3' */
  int_T BytePacking_IWORK[2];          /* '<S7>/Byte Packing' */
  int_T BytePacking_IWORK_i[2];        /* '<S6>/Byte Packing' */
  uint16_T temporalCounter_i1;         /* '<Root>/State-Transition Diagram' */
  boolean_T Delay2_DSTATE;             /* '<Root>/Delay2' */
  int8_T DiscreteTimeIntegrator_PrevResetState;/* '<Root>/Discrete-Time Integrator' */
  int8_T Spring_SubsysRanBC;           /* '<S18>/Spring' */
  int8_T Idle_SubsysRanBC;             /* '<S18>/Idle' */
  int8_T Flight_SubsysRanBC;           /* '<S18>/Flight' */
  int8_T Thrust_SubsysRanBC;           /* '<S17>/Thrust' */
  int8_T Flight_SubsysRanBC_m;         /* '<S17>/Flight' */
  int8_T Compress_SubsysRanBC;         /* '<S17>/Compress' */
  int8_T Unload_SubsysRanBC;           /* '<S16>/Unload' */
  int8_T HSW_SubsysRanBC;              /* '<S16>/HSW' */
  int8_T DiscreteTimeIntegrator_PrevResetState_c;/* '<S20>/Discrete-Time Integrator' */
  int8_T Flight_SubsysRanBC_f;         /* '<S16>/Flight' */
  int8_T EnabledSubsystem_SubsysRanBC; /* '<Root>/Enabled Subsystem' */
  uint8_T is_active_c8_PHRControl;     /* '<Root>/State-Transition Diagram' */
  uint8_T is_active_c3_PHRControl;     /* '<S9>/Command1.SLIP' */
  uint8_T temporalCounter_i1_a;        /* '<S9>/Command1.SLIP' */
  uint8_T is_active_c4_PHRControl;     /* '<S9>/Command1.Raibert' */
  uint8_T temporalCounter_i1_m;        /* '<S9>/Command1.Raibert' */
  uint8_T is_active_c5_PHRControl;     /* '<S9>/Command1.HSW_Controller' */
  uint8_T temporalCounter_i1_b;        /* '<S9>/Command1.HSW_Controller' */
  boolean_T Simscape_Transitions;      /* '<Root>/Data Store Memory' */
  DW_Unload_PHRControl_T Unload_le;    /* '<S18>/Unload' */
  DW_Load_PHRControl_T Load_k;         /* '<S18>/Load' */
  DW_Unload_PHRControl_T Unload_l;     /* '<S17>/Unload' */
  DW_Load_PHRControl_T Load_h;         /* '<S17>/Load' */
  DW_Idle_PHRControl_T Idle_l;         /* '<S17>/Idle' */
  DW_Load_PHRControl_T Load;           /* '<S16>/Load' */
  DW_Idle_PHRControl_T Idle;           /* '<S16>/Idle' */
};

/* Real-time Model Data Structure */
struct tag_RTM_PHRControl_T {
  struct SimStruct_tag * *childSfunctions;
  const char_T *errorStatus;
  SS_SimMode simMode;
  RTWSolverInfo solverInfo;
  RTWSolverInfo *solverInfoPtr;
  void *sfcnInfo;

  /*
   * NonInlinedSFcns:
   * The following substructure contains information regarding
   * non-inlined s-functions used in the model.
   */
  struct {
    RTWSfcnInfo sfcnInfo;
    time_T *taskTimePtrs[2];
    SimStruct childSFunctions[7];
    SimStruct *childSFunctionPtrs[7];
    struct _ssBlkInfo2 blkInfo2[7];
    struct _ssBlkInfoSLSize blkInfoSLSize[7];
    struct _ssSFcnModelMethods2 methods2[7];
    struct _ssSFcnModelMethods3 methods3[7];
    struct _ssSFcnModelMethods4 methods4[7];
    struct _ssStatesInfo2 statesInfo2[7];
    ssPeriodicStatesInfo periodicStatesInfo[7];
    struct _ssPortInfo2 inputOutputPortInfo2[7];
    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssPortInputsSLSize inputPortInfoSLSize[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      uint_T attribs[1];
      mxArray *params[1];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn0;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssPortInputsSLSize inputPortInfoSLSize[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      uint_T attribs[1];
      mxArray *params[1];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn1;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssPortInputsSLSize inputPortInfoSLSize[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      uint_T attribs[1];
      mxArray *params[1];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn2;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortInputs inputPortInfo[1];
      struct _ssPortInputsSLSize inputPortInfoSLSize[1];
      struct _ssInPortUnit inputPortUnits[1];
      struct _ssInPortCoSimAttribute inputPortCoSimAttribute[1];
      uint_T attribs[1];
      mxArray *params[1];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn3;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      struct _ssPortOutputs outputPortInfo[2];
      struct _ssPortOutputsSLSize outputPortInfoSLSize[2];
      struct _ssOutPortUnit outputPortUnits[2];
      struct _ssOutPortCoSimAttribute outputPortCoSimAttribute[2];
      uint_T attribs[1];
      mxArray *params[1];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn4;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      uint_T attribs[3];
      mxArray *params[3];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn5;

    struct {
      time_T sfcnPeriod[1];
      time_T sfcnOffset[1];
      int_T sfcnTsMap[1];
      uint_T attribs[34];
      mxArray *params[34];
      struct _ssDWorkRecord dWork[1];
      struct _ssDWorkAuxRecord dWorkAux[1];
    } Sfcn6;
  } NonInlinedSFcns;

  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T options;
    int_T numContStates;
    int_T numU;
    int_T numY;
    int_T numSampTimes;
    int_T numBlocks;
    int_T numBlockIO;
    int_T numBlockPrms;
    int_T numDwork;
    int_T numSFcnPrms;
    int_T numSFcns;
    int_T numIports;
    int_T numOports;
    int_T numNonSampZCs;
    int_T sysDirFeedThru;
    int_T rtwGenSfcn;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T stepSize;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T stepSize1;
    time_T tStart;
    time_T tFinal;
    time_T timeOfLastOutput;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *sampleTimes;
    time_T *offsetTimes;
    int_T *sampleTimeTaskIDPtr;
    int_T *sampleHits;
    int_T *perTaskSampleHits;
    time_T *t;
    time_T sampleTimesArray[2];
    time_T offsetTimesArray[2];
    int_T sampleTimeTaskIDArray[2];
    int_T sampleHitArray[2];
    int_T perTaskSampleHitsArray[4];
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern struct B_PHRControl_T PHRControl_B;

#ifdef __cplusplus

}
#endif

/* Block states (default storage) */
extern struct DW_PHRControl_T PHRControl_DW;

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void PHRControl_initialize(void);
  extern void PHRControl_step(void);
  extern void PHRControl_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_PHRControl_T *const PHRControl_M;

#ifdef __cplusplus

}
#endif

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'PHRControl'
 * '<S1>'   : 'PHRControl/Collision Detector'
 * '<S2>'   : 'PHRControl/Danger Detector Function'
 * '<S3>'   : 'PHRControl/Discrete Derivative'
 * '<S4>'   : 'PHRControl/Discrete Derivative1'
 * '<S5>'   : 'PHRControl/Enabled Subsystem'
 * '<S6>'   : 'PHRControl/ID1 Write Hip Data'
 * '<S7>'   : 'PHRControl/ID2 Write Knee Data'
 * '<S8>'   : 'PHRControl/Read All'
 * '<S9>'   : 'PHRControl/State-Transition Diagram'
 * '<S10>'  : 'PHRControl/ID1 Write Hip Data/floats -> bytes'
 * '<S11>'  : 'PHRControl/ID2 Write Knee Data/floats -> bytes'
 * '<S12>'  : 'PHRControl/Read All/Encoder 1 to Vertical Data'
 * '<S13>'  : 'PHRControl/Read All/Encoder 2 to Vertical Data'
 * '<S14>'  : 'PHRControl/Read All/bytes -> floats'
 * '<S15>'  : 'PHRControl/Read All/bytes -> floats1'
 * '<S16>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller'
 * '<S17>'  : 'PHRControl/State-Transition Diagram/Command1.Raibert'
 * '<S18>'  : 'PHRControl/State-Transition Diagram/Command1.SLIP'
 * '<S19>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight'
 * '<S20>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW'
 * '<S21>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle'
 * '<S22>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Load'
 * '<S23>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Unload'
 * '<S24>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller'
 * '<S25>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1'
 * '<S26>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Anti-windup'
 * '<S27>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/D Gain'
 * '<S28>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Filter'
 * '<S29>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Filter ICs'
 * '<S30>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/I Gain'
 * '<S31>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Ideal P Gain'
 * '<S32>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Ideal P Gain Fdbk'
 * '<S33>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Integrator'
 * '<S34>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Integrator ICs'
 * '<S35>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/N Copy'
 * '<S36>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/N Gain'
 * '<S37>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/P Copy'
 * '<S38>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Parallel P Gain'
 * '<S39>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Reset Signal'
 * '<S40>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Saturation'
 * '<S41>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Saturation Fdbk'
 * '<S42>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Sum'
 * '<S43>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Sum Fdbk'
 * '<S44>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Tracking Mode'
 * '<S45>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Tracking Mode Sum'
 * '<S46>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Tsamp - Integral'
 * '<S47>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Tsamp - Ngain'
 * '<S48>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/postSat Signal'
 * '<S49>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/preSat Signal'
 * '<S50>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Anti-windup/Passthrough'
 * '<S51>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/D Gain/Internal Parameters'
 * '<S52>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S53>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S54>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/I Gain/Internal Parameters'
 * '<S55>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Ideal P Gain/Passthrough'
 * '<S56>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S57>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Integrator/Discrete'
 * '<S58>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Integrator ICs/Internal IC'
 * '<S59>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/N Copy/Disabled'
 * '<S60>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/N Gain/Internal Parameters'
 * '<S61>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/P Copy/Disabled'
 * '<S62>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S63>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Reset Signal/Disabled'
 * '<S64>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Saturation/Passthrough'
 * '<S65>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Saturation Fdbk/Disabled'
 * '<S66>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Sum/Sum_PID'
 * '<S67>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Sum Fdbk/Disabled'
 * '<S68>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Tracking Mode/Disabled'
 * '<S69>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S70>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Tsamp - Integral/Passthrough'
 * '<S71>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S72>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/postSat Signal/Forward_Path'
 * '<S73>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller/preSat Signal/Forward_Path'
 * '<S74>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Anti-windup'
 * '<S75>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/D Gain'
 * '<S76>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Filter'
 * '<S77>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Filter ICs'
 * '<S78>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/I Gain'
 * '<S79>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Ideal P Gain'
 * '<S80>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Ideal P Gain Fdbk'
 * '<S81>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Integrator'
 * '<S82>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Integrator ICs'
 * '<S83>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/N Copy'
 * '<S84>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/N Gain'
 * '<S85>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/P Copy'
 * '<S86>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Parallel P Gain'
 * '<S87>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Reset Signal'
 * '<S88>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Saturation'
 * '<S89>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Saturation Fdbk'
 * '<S90>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Sum'
 * '<S91>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Sum Fdbk'
 * '<S92>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Tracking Mode'
 * '<S93>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Tracking Mode Sum'
 * '<S94>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Tsamp - Integral'
 * '<S95>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Tsamp - Ngain'
 * '<S96>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/postSat Signal'
 * '<S97>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/preSat Signal'
 * '<S98>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Anti-windup/Passthrough'
 * '<S99>'  : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/D Gain/Internal Parameters'
 * '<S100>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S101>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S102>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/I Gain/Internal Parameters'
 * '<S103>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Ideal P Gain/Passthrough'
 * '<S104>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S105>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Integrator/Discrete'
 * '<S106>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Integrator ICs/Internal IC'
 * '<S107>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/N Copy/Disabled'
 * '<S108>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/N Gain/Internal Parameters'
 * '<S109>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/P Copy/Disabled'
 * '<S110>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S111>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Reset Signal/Disabled'
 * '<S112>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Saturation/Passthrough'
 * '<S113>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Saturation Fdbk/Disabled'
 * '<S114>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Sum/Sum_PID'
 * '<S115>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Sum Fdbk/Disabled'
 * '<S116>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Tracking Mode/Disabled'
 * '<S117>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S118>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Tsamp - Integral/Passthrough'
 * '<S119>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S120>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/postSat Signal/Forward_Path'
 * '<S121>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Flight/PID Controller1/preSat Signal/Forward_Path'
 * '<S122>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2'
 * '<S123>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3'
 * '<S124>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/Sine Generator'
 * '<S125>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/Torque Calculator'
 * '<S126>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Anti-windup'
 * '<S127>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/D Gain'
 * '<S128>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Filter'
 * '<S129>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Filter ICs'
 * '<S130>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/I Gain'
 * '<S131>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Ideal P Gain'
 * '<S132>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Ideal P Gain Fdbk'
 * '<S133>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Integrator'
 * '<S134>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Integrator ICs'
 * '<S135>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/N Copy'
 * '<S136>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/N Gain'
 * '<S137>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/P Copy'
 * '<S138>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Parallel P Gain'
 * '<S139>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Reset Signal'
 * '<S140>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Saturation'
 * '<S141>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Saturation Fdbk'
 * '<S142>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Sum'
 * '<S143>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Sum Fdbk'
 * '<S144>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Tracking Mode'
 * '<S145>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Tracking Mode Sum'
 * '<S146>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Tsamp - Integral'
 * '<S147>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Tsamp - Ngain'
 * '<S148>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/postSat Signal'
 * '<S149>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/preSat Signal'
 * '<S150>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Anti-windup/Passthrough'
 * '<S151>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/D Gain/Internal Parameters'
 * '<S152>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Filter/Disc. Forward Euler Filter'
 * '<S153>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Filter ICs/Internal IC - Filter'
 * '<S154>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/I Gain/Internal Parameters'
 * '<S155>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Ideal P Gain/Passthrough'
 * '<S156>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S157>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Integrator/Discrete'
 * '<S158>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Integrator ICs/Internal IC'
 * '<S159>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/N Copy/Disabled'
 * '<S160>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/N Gain/Internal Parameters'
 * '<S161>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/P Copy/Disabled'
 * '<S162>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Parallel P Gain/Internal Parameters'
 * '<S163>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Reset Signal/Disabled'
 * '<S164>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Saturation/Passthrough'
 * '<S165>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Saturation Fdbk/Disabled'
 * '<S166>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Sum/Sum_PID'
 * '<S167>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Sum Fdbk/Disabled'
 * '<S168>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Tracking Mode/Disabled'
 * '<S169>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S170>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Tsamp - Integral/Passthrough'
 * '<S171>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S172>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/postSat Signal/Forward_Path'
 * '<S173>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller2/preSat Signal/Forward_Path'
 * '<S174>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Anti-windup'
 * '<S175>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/D Gain'
 * '<S176>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Filter'
 * '<S177>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Filter ICs'
 * '<S178>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/I Gain'
 * '<S179>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Ideal P Gain'
 * '<S180>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Ideal P Gain Fdbk'
 * '<S181>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Integrator'
 * '<S182>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Integrator ICs'
 * '<S183>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/N Copy'
 * '<S184>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/N Gain'
 * '<S185>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/P Copy'
 * '<S186>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Parallel P Gain'
 * '<S187>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Reset Signal'
 * '<S188>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Saturation'
 * '<S189>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Saturation Fdbk'
 * '<S190>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Sum'
 * '<S191>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Sum Fdbk'
 * '<S192>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Tracking Mode'
 * '<S193>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Tracking Mode Sum'
 * '<S194>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Tsamp - Integral'
 * '<S195>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Tsamp - Ngain'
 * '<S196>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/postSat Signal'
 * '<S197>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/preSat Signal'
 * '<S198>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Anti-windup/Passthrough'
 * '<S199>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/D Gain/Internal Parameters'
 * '<S200>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Filter/Disc. Forward Euler Filter'
 * '<S201>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Filter ICs/Internal IC - Filter'
 * '<S202>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/I Gain/Internal Parameters'
 * '<S203>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Ideal P Gain/Passthrough'
 * '<S204>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Ideal P Gain Fdbk/Disabled'
 * '<S205>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Integrator/Discrete'
 * '<S206>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Integrator ICs/Internal IC'
 * '<S207>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/N Copy/Disabled'
 * '<S208>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/N Gain/Internal Parameters'
 * '<S209>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/P Copy/Disabled'
 * '<S210>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Parallel P Gain/Internal Parameters'
 * '<S211>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Reset Signal/Disabled'
 * '<S212>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Saturation/Passthrough'
 * '<S213>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Saturation Fdbk/Disabled'
 * '<S214>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Sum/Sum_PID'
 * '<S215>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Sum Fdbk/Disabled'
 * '<S216>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Tracking Mode/Disabled'
 * '<S217>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Tracking Mode Sum/Passthrough'
 * '<S218>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Tsamp - Integral/Passthrough'
 * '<S219>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/Tsamp - Ngain/Passthrough'
 * '<S220>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/postSat Signal/Forward_Path'
 * '<S221>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/HSW/PID Controller3/preSat Signal/Forward_Path'
 * '<S222>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller'
 * '<S223>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1'
 * '<S224>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Anti-windup'
 * '<S225>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/D Gain'
 * '<S226>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Filter'
 * '<S227>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Filter ICs'
 * '<S228>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/I Gain'
 * '<S229>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Ideal P Gain'
 * '<S230>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Ideal P Gain Fdbk'
 * '<S231>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Integrator'
 * '<S232>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Integrator ICs'
 * '<S233>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/N Copy'
 * '<S234>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/N Gain'
 * '<S235>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/P Copy'
 * '<S236>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Parallel P Gain'
 * '<S237>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Reset Signal'
 * '<S238>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Saturation'
 * '<S239>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Saturation Fdbk'
 * '<S240>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Sum'
 * '<S241>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Sum Fdbk'
 * '<S242>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Tracking Mode'
 * '<S243>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Tracking Mode Sum'
 * '<S244>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Tsamp - Integral'
 * '<S245>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Tsamp - Ngain'
 * '<S246>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/postSat Signal'
 * '<S247>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/preSat Signal'
 * '<S248>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Anti-windup/Passthrough'
 * '<S249>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/D Gain/Internal Parameters'
 * '<S250>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S251>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S252>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/I Gain/Internal Parameters'
 * '<S253>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Ideal P Gain/Passthrough'
 * '<S254>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S255>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Integrator/Discrete'
 * '<S256>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Integrator ICs/Internal IC'
 * '<S257>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/N Copy/Disabled'
 * '<S258>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/N Gain/Internal Parameters'
 * '<S259>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/P Copy/Disabled'
 * '<S260>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S261>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Reset Signal/Disabled'
 * '<S262>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Saturation/Passthrough'
 * '<S263>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Saturation Fdbk/Disabled'
 * '<S264>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Sum/Sum_PID'
 * '<S265>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Sum Fdbk/Disabled'
 * '<S266>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Tracking Mode/Disabled'
 * '<S267>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S268>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Tsamp - Integral/Passthrough'
 * '<S269>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S270>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/postSat Signal/Forward_Path'
 * '<S271>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller/preSat Signal/Forward_Path'
 * '<S272>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Anti-windup'
 * '<S273>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/D Gain'
 * '<S274>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Filter'
 * '<S275>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Filter ICs'
 * '<S276>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/I Gain'
 * '<S277>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Ideal P Gain'
 * '<S278>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Ideal P Gain Fdbk'
 * '<S279>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Integrator'
 * '<S280>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Integrator ICs'
 * '<S281>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/N Copy'
 * '<S282>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/N Gain'
 * '<S283>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/P Copy'
 * '<S284>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Parallel P Gain'
 * '<S285>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Reset Signal'
 * '<S286>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Saturation'
 * '<S287>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Saturation Fdbk'
 * '<S288>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Sum'
 * '<S289>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Sum Fdbk'
 * '<S290>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Tracking Mode'
 * '<S291>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Tracking Mode Sum'
 * '<S292>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Tsamp - Integral'
 * '<S293>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Tsamp - Ngain'
 * '<S294>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/postSat Signal'
 * '<S295>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/preSat Signal'
 * '<S296>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Anti-windup/Passthrough'
 * '<S297>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/D Gain/Internal Parameters'
 * '<S298>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S299>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S300>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/I Gain/Internal Parameters'
 * '<S301>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Ideal P Gain/Passthrough'
 * '<S302>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S303>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Integrator/Discrete'
 * '<S304>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Integrator ICs/Internal IC'
 * '<S305>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/N Copy/Disabled'
 * '<S306>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/N Gain/Internal Parameters'
 * '<S307>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/P Copy/Disabled'
 * '<S308>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S309>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Reset Signal/Disabled'
 * '<S310>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Saturation/Passthrough'
 * '<S311>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Saturation Fdbk/Disabled'
 * '<S312>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Sum/Sum_PID'
 * '<S313>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Sum Fdbk/Disabled'
 * '<S314>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Tracking Mode/Disabled'
 * '<S315>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S316>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Tsamp - Integral/Passthrough'
 * '<S317>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S318>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/postSat Signal/Forward_Path'
 * '<S319>' : 'PHRControl/State-Transition Diagram/Command1.HSW_Controller/Idle/PID Controller1/preSat Signal/Forward_Path'
 * '<S320>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress'
 * '<S321>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight'
 * '<S322>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle'
 * '<S323>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Load'
 * '<S324>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust'
 * '<S325>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Unload'
 * '<S326>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2'
 * '<S327>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Anti-windup'
 * '<S328>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/D Gain'
 * '<S329>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Filter'
 * '<S330>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Filter ICs'
 * '<S331>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/I Gain'
 * '<S332>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Ideal P Gain'
 * '<S333>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Ideal P Gain Fdbk'
 * '<S334>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Integrator'
 * '<S335>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Integrator ICs'
 * '<S336>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/N Copy'
 * '<S337>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/N Gain'
 * '<S338>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/P Copy'
 * '<S339>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Parallel P Gain'
 * '<S340>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Reset Signal'
 * '<S341>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Saturation'
 * '<S342>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Saturation Fdbk'
 * '<S343>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Sum'
 * '<S344>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Sum Fdbk'
 * '<S345>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Tracking Mode'
 * '<S346>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Tracking Mode Sum'
 * '<S347>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Tsamp - Integral'
 * '<S348>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Tsamp - Ngain'
 * '<S349>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/postSat Signal'
 * '<S350>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/preSat Signal'
 * '<S351>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Anti-windup/Passthrough'
 * '<S352>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/D Gain/Internal Parameters'
 * '<S353>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Filter/Disc. Forward Euler Filter'
 * '<S354>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Filter ICs/Internal IC - Filter'
 * '<S355>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/I Gain/Internal Parameters'
 * '<S356>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Ideal P Gain/Passthrough'
 * '<S357>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S358>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Integrator/Discrete'
 * '<S359>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Integrator ICs/Internal IC'
 * '<S360>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/N Copy/Disabled'
 * '<S361>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/N Gain/Internal Parameters'
 * '<S362>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/P Copy/Disabled'
 * '<S363>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Parallel P Gain/Internal Parameters'
 * '<S364>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Reset Signal/Disabled'
 * '<S365>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Saturation/Passthrough'
 * '<S366>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Saturation Fdbk/Disabled'
 * '<S367>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Sum/Sum_PID'
 * '<S368>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Sum Fdbk/Disabled'
 * '<S369>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Tracking Mode/Disabled'
 * '<S370>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S371>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Tsamp - Integral/Passthrough'
 * '<S372>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S373>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/postSat Signal/Forward_Path'
 * '<S374>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Compress/PID Controller2/preSat Signal/Forward_Path'
 * '<S375>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller'
 * '<S376>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1'
 * '<S377>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Anti-windup'
 * '<S378>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/D Gain'
 * '<S379>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Filter'
 * '<S380>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Filter ICs'
 * '<S381>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/I Gain'
 * '<S382>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Ideal P Gain'
 * '<S383>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Ideal P Gain Fdbk'
 * '<S384>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Integrator'
 * '<S385>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Integrator ICs'
 * '<S386>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/N Copy'
 * '<S387>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/N Gain'
 * '<S388>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/P Copy'
 * '<S389>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Parallel P Gain'
 * '<S390>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Reset Signal'
 * '<S391>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Saturation'
 * '<S392>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Saturation Fdbk'
 * '<S393>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Sum'
 * '<S394>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Sum Fdbk'
 * '<S395>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Tracking Mode'
 * '<S396>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Tracking Mode Sum'
 * '<S397>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Tsamp - Integral'
 * '<S398>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Tsamp - Ngain'
 * '<S399>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/postSat Signal'
 * '<S400>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/preSat Signal'
 * '<S401>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Anti-windup/Passthrough'
 * '<S402>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/D Gain/Internal Parameters'
 * '<S403>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S404>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S405>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/I Gain/Internal Parameters'
 * '<S406>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Ideal P Gain/Passthrough'
 * '<S407>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S408>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Integrator/Discrete'
 * '<S409>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Integrator ICs/Internal IC'
 * '<S410>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/N Copy/Disabled'
 * '<S411>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/N Gain/Internal Parameters'
 * '<S412>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/P Copy/Disabled'
 * '<S413>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S414>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Reset Signal/Disabled'
 * '<S415>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Saturation/Passthrough'
 * '<S416>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Saturation Fdbk/Disabled'
 * '<S417>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Sum/Sum_PID'
 * '<S418>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Sum Fdbk/Disabled'
 * '<S419>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Tracking Mode/Disabled'
 * '<S420>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S421>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Tsamp - Integral/Passthrough'
 * '<S422>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S423>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/postSat Signal/Forward_Path'
 * '<S424>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller/preSat Signal/Forward_Path'
 * '<S425>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Anti-windup'
 * '<S426>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/D Gain'
 * '<S427>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Filter'
 * '<S428>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Filter ICs'
 * '<S429>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/I Gain'
 * '<S430>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Ideal P Gain'
 * '<S431>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Ideal P Gain Fdbk'
 * '<S432>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Integrator'
 * '<S433>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Integrator ICs'
 * '<S434>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/N Copy'
 * '<S435>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/N Gain'
 * '<S436>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/P Copy'
 * '<S437>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Parallel P Gain'
 * '<S438>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Reset Signal'
 * '<S439>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Saturation'
 * '<S440>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Saturation Fdbk'
 * '<S441>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Sum'
 * '<S442>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Sum Fdbk'
 * '<S443>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Tracking Mode'
 * '<S444>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Tracking Mode Sum'
 * '<S445>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Tsamp - Integral'
 * '<S446>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Tsamp - Ngain'
 * '<S447>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/postSat Signal'
 * '<S448>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/preSat Signal'
 * '<S449>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Anti-windup/Passthrough'
 * '<S450>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/D Gain/Internal Parameters'
 * '<S451>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S452>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S453>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/I Gain/Internal Parameters'
 * '<S454>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Ideal P Gain/Passthrough'
 * '<S455>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S456>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Integrator/Discrete'
 * '<S457>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Integrator ICs/Internal IC'
 * '<S458>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/N Copy/Disabled'
 * '<S459>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/N Gain/Internal Parameters'
 * '<S460>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/P Copy/Disabled'
 * '<S461>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S462>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Reset Signal/Disabled'
 * '<S463>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Saturation/Passthrough'
 * '<S464>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Saturation Fdbk/Disabled'
 * '<S465>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Sum/Sum_PID'
 * '<S466>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Sum Fdbk/Disabled'
 * '<S467>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Tracking Mode/Disabled'
 * '<S468>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S469>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Tsamp - Integral/Passthrough'
 * '<S470>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S471>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/postSat Signal/Forward_Path'
 * '<S472>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Flight/PID Controller1/preSat Signal/Forward_Path'
 * '<S473>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller'
 * '<S474>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1'
 * '<S475>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Anti-windup'
 * '<S476>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/D Gain'
 * '<S477>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Filter'
 * '<S478>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Filter ICs'
 * '<S479>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/I Gain'
 * '<S480>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Ideal P Gain'
 * '<S481>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Ideal P Gain Fdbk'
 * '<S482>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Integrator'
 * '<S483>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Integrator ICs'
 * '<S484>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/N Copy'
 * '<S485>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/N Gain'
 * '<S486>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/P Copy'
 * '<S487>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Parallel P Gain'
 * '<S488>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Reset Signal'
 * '<S489>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Saturation'
 * '<S490>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Saturation Fdbk'
 * '<S491>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Sum'
 * '<S492>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Sum Fdbk'
 * '<S493>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Tracking Mode'
 * '<S494>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Tracking Mode Sum'
 * '<S495>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Tsamp - Integral'
 * '<S496>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Tsamp - Ngain'
 * '<S497>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/postSat Signal'
 * '<S498>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/preSat Signal'
 * '<S499>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Anti-windup/Passthrough'
 * '<S500>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/D Gain/Internal Parameters'
 * '<S501>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S502>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S503>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/I Gain/Internal Parameters'
 * '<S504>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Ideal P Gain/Passthrough'
 * '<S505>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S506>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Integrator/Discrete'
 * '<S507>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Integrator ICs/Internal IC'
 * '<S508>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/N Copy/Disabled'
 * '<S509>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/N Gain/Internal Parameters'
 * '<S510>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/P Copy/Disabled'
 * '<S511>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S512>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Reset Signal/Disabled'
 * '<S513>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Saturation/Passthrough'
 * '<S514>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Saturation Fdbk/Disabled'
 * '<S515>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Sum/Sum_PID'
 * '<S516>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Sum Fdbk/Disabled'
 * '<S517>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Tracking Mode/Disabled'
 * '<S518>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S519>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Tsamp - Integral/Passthrough'
 * '<S520>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S521>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/postSat Signal/Forward_Path'
 * '<S522>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller/preSat Signal/Forward_Path'
 * '<S523>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Anti-windup'
 * '<S524>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/D Gain'
 * '<S525>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Filter'
 * '<S526>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Filter ICs'
 * '<S527>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/I Gain'
 * '<S528>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Ideal P Gain'
 * '<S529>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Ideal P Gain Fdbk'
 * '<S530>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Integrator'
 * '<S531>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Integrator ICs'
 * '<S532>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/N Copy'
 * '<S533>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/N Gain'
 * '<S534>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/P Copy'
 * '<S535>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Parallel P Gain'
 * '<S536>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Reset Signal'
 * '<S537>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Saturation'
 * '<S538>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Saturation Fdbk'
 * '<S539>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Sum'
 * '<S540>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Sum Fdbk'
 * '<S541>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Tracking Mode'
 * '<S542>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Tracking Mode Sum'
 * '<S543>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Tsamp - Integral'
 * '<S544>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Tsamp - Ngain'
 * '<S545>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/postSat Signal'
 * '<S546>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/preSat Signal'
 * '<S547>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Anti-windup/Passthrough'
 * '<S548>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/D Gain/Internal Parameters'
 * '<S549>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S550>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S551>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/I Gain/Internal Parameters'
 * '<S552>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Ideal P Gain/Passthrough'
 * '<S553>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S554>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Integrator/Discrete'
 * '<S555>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Integrator ICs/Internal IC'
 * '<S556>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/N Copy/Disabled'
 * '<S557>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/N Gain/Internal Parameters'
 * '<S558>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/P Copy/Disabled'
 * '<S559>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S560>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Reset Signal/Disabled'
 * '<S561>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Saturation/Passthrough'
 * '<S562>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Saturation Fdbk/Disabled'
 * '<S563>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Sum/Sum_PID'
 * '<S564>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Sum Fdbk/Disabled'
 * '<S565>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Tracking Mode/Disabled'
 * '<S566>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S567>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Tsamp - Integral/Passthrough'
 * '<S568>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S569>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/postSat Signal/Forward_Path'
 * '<S570>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Idle/PID Controller1/preSat Signal/Forward_Path'
 * '<S571>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2'
 * '<S572>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Anti-windup'
 * '<S573>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/D Gain'
 * '<S574>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Filter'
 * '<S575>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Filter ICs'
 * '<S576>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/I Gain'
 * '<S577>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Ideal P Gain'
 * '<S578>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Ideal P Gain Fdbk'
 * '<S579>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Integrator'
 * '<S580>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Integrator ICs'
 * '<S581>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/N Copy'
 * '<S582>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/N Gain'
 * '<S583>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/P Copy'
 * '<S584>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Parallel P Gain'
 * '<S585>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Reset Signal'
 * '<S586>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Saturation'
 * '<S587>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Saturation Fdbk'
 * '<S588>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Sum'
 * '<S589>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Sum Fdbk'
 * '<S590>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Tracking Mode'
 * '<S591>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Tracking Mode Sum'
 * '<S592>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Tsamp - Integral'
 * '<S593>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Tsamp - Ngain'
 * '<S594>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/postSat Signal'
 * '<S595>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/preSat Signal'
 * '<S596>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Anti-windup/Passthrough'
 * '<S597>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/D Gain/Internal Parameters'
 * '<S598>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Filter/Disc. Forward Euler Filter'
 * '<S599>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Filter ICs/Internal IC - Filter'
 * '<S600>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/I Gain/Internal Parameters'
 * '<S601>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Ideal P Gain/Passthrough'
 * '<S602>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S603>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Integrator/Discrete'
 * '<S604>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Integrator ICs/Internal IC'
 * '<S605>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/N Copy/Disabled'
 * '<S606>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/N Gain/Internal Parameters'
 * '<S607>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/P Copy/Disabled'
 * '<S608>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Parallel P Gain/Internal Parameters'
 * '<S609>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Reset Signal/Disabled'
 * '<S610>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Saturation/Passthrough'
 * '<S611>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Saturation Fdbk/Disabled'
 * '<S612>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Sum/Sum_PID'
 * '<S613>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Sum Fdbk/Disabled'
 * '<S614>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Tracking Mode/Disabled'
 * '<S615>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S616>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Tsamp - Integral/Passthrough'
 * '<S617>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S618>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/postSat Signal/Forward_Path'
 * '<S619>' : 'PHRControl/State-Transition Diagram/Command1.Raibert/Thrust/PID Controller2/preSat Signal/Forward_Path'
 * '<S620>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight'
 * '<S621>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle'
 * '<S622>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Load'
 * '<S623>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Spring'
 * '<S624>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Unload'
 * '<S625>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller'
 * '<S626>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1'
 * '<S627>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/SLIP Length1'
 * '<S628>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Anti-windup'
 * '<S629>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/D Gain'
 * '<S630>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Filter'
 * '<S631>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Filter ICs'
 * '<S632>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/I Gain'
 * '<S633>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Ideal P Gain'
 * '<S634>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Ideal P Gain Fdbk'
 * '<S635>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Integrator'
 * '<S636>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Integrator ICs'
 * '<S637>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/N Copy'
 * '<S638>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/N Gain'
 * '<S639>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/P Copy'
 * '<S640>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Parallel P Gain'
 * '<S641>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Reset Signal'
 * '<S642>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Saturation'
 * '<S643>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Saturation Fdbk'
 * '<S644>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Sum'
 * '<S645>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Sum Fdbk'
 * '<S646>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Tracking Mode'
 * '<S647>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Tracking Mode Sum'
 * '<S648>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Tsamp - Integral'
 * '<S649>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Tsamp - Ngain'
 * '<S650>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/postSat Signal'
 * '<S651>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/preSat Signal'
 * '<S652>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Anti-windup/Passthrough'
 * '<S653>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/D Gain/Internal Parameters'
 * '<S654>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S655>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S656>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/I Gain/Internal Parameters'
 * '<S657>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Ideal P Gain/Passthrough'
 * '<S658>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S659>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Integrator/Discrete'
 * '<S660>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Integrator ICs/Internal IC'
 * '<S661>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/N Copy/Disabled'
 * '<S662>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/N Gain/Internal Parameters'
 * '<S663>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/P Copy/Disabled'
 * '<S664>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S665>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Reset Signal/Disabled'
 * '<S666>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Saturation/Passthrough'
 * '<S667>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Saturation Fdbk/Disabled'
 * '<S668>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Sum/Sum_PID'
 * '<S669>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Sum Fdbk/Disabled'
 * '<S670>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Tracking Mode/Disabled'
 * '<S671>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S672>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Tsamp - Integral/Passthrough'
 * '<S673>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S674>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/postSat Signal/Forward_Path'
 * '<S675>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller/preSat Signal/Forward_Path'
 * '<S676>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Anti-windup'
 * '<S677>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/D Gain'
 * '<S678>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Filter'
 * '<S679>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Filter ICs'
 * '<S680>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/I Gain'
 * '<S681>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Ideal P Gain'
 * '<S682>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Ideal P Gain Fdbk'
 * '<S683>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Integrator'
 * '<S684>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Integrator ICs'
 * '<S685>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/N Copy'
 * '<S686>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/N Gain'
 * '<S687>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/P Copy'
 * '<S688>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Parallel P Gain'
 * '<S689>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Reset Signal'
 * '<S690>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Saturation'
 * '<S691>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Saturation Fdbk'
 * '<S692>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Sum'
 * '<S693>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Sum Fdbk'
 * '<S694>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Tracking Mode'
 * '<S695>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Tracking Mode Sum'
 * '<S696>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Tsamp - Integral'
 * '<S697>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Tsamp - Ngain'
 * '<S698>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/postSat Signal'
 * '<S699>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/preSat Signal'
 * '<S700>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Anti-windup/Passthrough'
 * '<S701>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/D Gain/Internal Parameters'
 * '<S702>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S703>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S704>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/I Gain/Internal Parameters'
 * '<S705>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Ideal P Gain/Passthrough'
 * '<S706>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S707>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Integrator/Discrete'
 * '<S708>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Integrator ICs/Internal IC'
 * '<S709>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/N Copy/Disabled'
 * '<S710>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/N Gain/Internal Parameters'
 * '<S711>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/P Copy/Disabled'
 * '<S712>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S713>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Reset Signal/Disabled'
 * '<S714>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Saturation/Passthrough'
 * '<S715>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Saturation Fdbk/Disabled'
 * '<S716>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Sum/Sum_PID'
 * '<S717>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Sum Fdbk/Disabled'
 * '<S718>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Tracking Mode/Disabled'
 * '<S719>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S720>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Tsamp - Integral/Passthrough'
 * '<S721>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S722>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/postSat Signal/Forward_Path'
 * '<S723>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Flight/PID Controller1/preSat Signal/Forward_Path'
 * '<S724>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller'
 * '<S725>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1'
 * '<S726>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Anti-windup'
 * '<S727>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/D Gain'
 * '<S728>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Filter'
 * '<S729>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Filter ICs'
 * '<S730>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/I Gain'
 * '<S731>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Ideal P Gain'
 * '<S732>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Ideal P Gain Fdbk'
 * '<S733>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Integrator'
 * '<S734>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Integrator ICs'
 * '<S735>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/N Copy'
 * '<S736>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/N Gain'
 * '<S737>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/P Copy'
 * '<S738>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Parallel P Gain'
 * '<S739>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Reset Signal'
 * '<S740>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Saturation'
 * '<S741>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Saturation Fdbk'
 * '<S742>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Sum'
 * '<S743>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Sum Fdbk'
 * '<S744>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Tracking Mode'
 * '<S745>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Tracking Mode Sum'
 * '<S746>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Tsamp - Integral'
 * '<S747>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Tsamp - Ngain'
 * '<S748>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/postSat Signal'
 * '<S749>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/preSat Signal'
 * '<S750>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Anti-windup/Passthrough'
 * '<S751>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/D Gain/Internal Parameters'
 * '<S752>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S753>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S754>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/I Gain/Internal Parameters'
 * '<S755>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Ideal P Gain/Passthrough'
 * '<S756>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S757>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Integrator/Discrete'
 * '<S758>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Integrator ICs/Internal IC'
 * '<S759>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/N Copy/Disabled'
 * '<S760>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/N Gain/Internal Parameters'
 * '<S761>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/P Copy/Disabled'
 * '<S762>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S763>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Reset Signal/Disabled'
 * '<S764>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Saturation/Passthrough'
 * '<S765>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Saturation Fdbk/Disabled'
 * '<S766>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Sum/Sum_PID'
 * '<S767>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Sum Fdbk/Disabled'
 * '<S768>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Tracking Mode/Disabled'
 * '<S769>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S770>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Tsamp - Integral/Passthrough'
 * '<S771>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S772>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/postSat Signal/Forward_Path'
 * '<S773>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller/preSat Signal/Forward_Path'
 * '<S774>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Anti-windup'
 * '<S775>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/D Gain'
 * '<S776>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Filter'
 * '<S777>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Filter ICs'
 * '<S778>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/I Gain'
 * '<S779>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Ideal P Gain'
 * '<S780>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Ideal P Gain Fdbk'
 * '<S781>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Integrator'
 * '<S782>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Integrator ICs'
 * '<S783>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/N Copy'
 * '<S784>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/N Gain'
 * '<S785>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/P Copy'
 * '<S786>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Parallel P Gain'
 * '<S787>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Reset Signal'
 * '<S788>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Saturation'
 * '<S789>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Saturation Fdbk'
 * '<S790>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Sum'
 * '<S791>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Sum Fdbk'
 * '<S792>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Tracking Mode'
 * '<S793>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Tracking Mode Sum'
 * '<S794>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Tsamp - Integral'
 * '<S795>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Tsamp - Ngain'
 * '<S796>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/postSat Signal'
 * '<S797>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/preSat Signal'
 * '<S798>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Anti-windup/Passthrough'
 * '<S799>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/D Gain/Internal Parameters'
 * '<S800>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S801>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S802>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/I Gain/Internal Parameters'
 * '<S803>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Ideal P Gain/Passthrough'
 * '<S804>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S805>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Integrator/Discrete'
 * '<S806>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Integrator ICs/Internal IC'
 * '<S807>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/N Copy/Disabled'
 * '<S808>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/N Gain/Internal Parameters'
 * '<S809>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/P Copy/Disabled'
 * '<S810>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S811>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Reset Signal/Disabled'
 * '<S812>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Saturation/Passthrough'
 * '<S813>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Saturation Fdbk/Disabled'
 * '<S814>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Sum/Sum_PID'
 * '<S815>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Sum Fdbk/Disabled'
 * '<S816>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Tracking Mode/Disabled'
 * '<S817>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S818>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Tsamp - Integral/Passthrough'
 * '<S819>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S820>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/postSat Signal/Forward_Path'
 * '<S821>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Idle/PID Controller1/preSat Signal/Forward_Path'
 * '<S822>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Spring/Hip Torque Calculation'
 * '<S823>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Spring/Knee Torque Calculation'
 * '<S824>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Spring/Hip Torque Calculation/SLIP Angle'
 * '<S825>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Spring/Hip Torque Calculation/SLIP Angle1'
 * '<S826>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Spring/Hip Torque Calculation/SLIP Length'
 * '<S827>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Spring/Hip Torque Calculation/SLIP Length1'
 * '<S828>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Spring/Knee Torque Calculation/SLIP Angle1'
 * '<S829>' : 'PHRControl/State-Transition Diagram/Command1.SLIP/Spring/Knee Torque Calculation/SLIP Length1'
 */
#endif                                 /* RTW_HEADER_PHRControl_h_ */

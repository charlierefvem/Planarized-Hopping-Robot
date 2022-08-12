#ifndef RTW_HEADER_PHRControl_cal_h_
#define RTW_HEADER_PHRControl_cal_h_
#include "rtwtypes.h"

/* Storage class 'PageSwitching', for system '<S16>/Idle' */
struct PHRControl_Idle_cal_type {
  real_T PIDController_D;              /* Mask Parameter: PIDController_D
                                        * Referenced by: '<S249>/Derivative Gain'
                                        */
  real_T PIDController1_D;             /* Mask Parameter: PIDController1_D
                                        * Referenced by: '<S297>/Derivative Gain'
                                        */
  real_T PIDController_I;              /* Mask Parameter: PIDController_I
                                        * Referenced by: '<S252>/Integral Gain'
                                        */
  real_T PIDController1_I;             /* Mask Parameter: PIDController1_I
                                        * Referenced by: '<S300>/Integral Gain'
                                        */
  real_T PIDController_InitialConditionForFilter;
                      /* Mask Parameter: PIDController_InitialConditionForFilter
                       * Referenced by: '<S250>/Filter'
                       */
  real_T PIDController1_InitialConditionForFilter;
                     /* Mask Parameter: PIDController1_InitialConditionForFilter
                      * Referenced by: '<S298>/Filter'
                      */
  real_T PIDController_InitialConditionForIntegrator;
                  /* Mask Parameter: PIDController_InitialConditionForIntegrator
                   * Referenced by: '<S255>/Integrator'
                   */
  real_T PIDController1_InitialConditionForIntegrator;
                 /* Mask Parameter: PIDController1_InitialConditionForIntegrator
                  * Referenced by: '<S303>/Integrator'
                  */
  real_T PIDController_N;              /* Mask Parameter: PIDController_N
                                        * Referenced by: '<S258>/Filter Coefficient'
                                        */
  real_T PIDController1_N;             /* Mask Parameter: PIDController1_N
                                        * Referenced by: '<S306>/Filter Coefficient'
                                        */
  real_T PIDController_P;              /* Mask Parameter: PIDController_P
                                        * Referenced by: '<S260>/Proportional Gain'
                                        */
  real_T PIDController1_P;             /* Mask Parameter: PIDController1_P
                                        * Referenced by: '<S308>/Proportional Gain'
                                        */
  real_T Filter_gainval;               /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S250>/Filter'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S255>/Integrator'
                                        */
  real_T Filter_gainval_o;             /* Computed Parameter: Filter_gainval_o
                                        * Referenced by: '<S298>/Filter'
                                        */
  real_T Integrator_gainval_b;       /* Computed Parameter: Integrator_gainval_b
                                      * Referenced by: '<S303>/Integrator'
                                      */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S21>/Constant'
                                        */
};

/* Storage class 'PageSwitching', for system '<S16>/Load' */
struct PHRControl_Load_cal_type {
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S22>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S22>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: 2
                                        * Referenced by: '<S22>/Constant2'
                                        */
};

/* Storage class 'PageSwitching', for system '<S17>/Unload' */
struct PHRControl_Unload_cal_type {
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S325>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S325>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: 5
                                        * Referenced by: '<S325>/Constant2'
                                        */
};

/* Storage class 'PageSwitching', for system '<Root>' */
struct PHRControl_cal_type {
  real_T CPR;                          /* Variable: CPR
                                        * Referenced by:
                                        *   '<S8>/Encoder 1 to Vertical Data'
                                        *   '<S8>/Encoder 2 to Vertical Data'
                                        */
  real_T amplitude;                    /* Variable: amplitude
                                        * Referenced by: '<S20>/Constant'
                                        */
  real_T contactTime;                  /* Variable: contactTime
                                        * Referenced by: '<S321>/Constant'
                                        */
  real_T desired_speed;                /* Variable: desired_speed
                                        * Referenced by:
                                        *   '<S321>/Constant1'
                                        *   '<S324>/Constant'
                                        *   '<S620>/Constant1'
                                        */
  real_T equilibrium_length;           /* Variable: equilibrium_length
                                        * Referenced by:
                                        *   '<S620>/Knee Joint Angle1'
                                        *   '<S822>/Constant1'
                                        *   '<S822>/Constant4'
                                        *   '<S823>/Constant'
                                        */
  real_T h;                            /* Variable: h
                                        * Referenced by: '<S8>/Encoder 1 to Vertical Data'
                                        */
  real_T hip_bend_factor;              /* Variable: hip_bend_factor
                                        * Referenced by:
                                        *   '<Root>/Constant8'
                                        *   '<S320>/Constant4'
                                        */
  real_T hip_fuzz;                     /* Variable: hip_fuzz
                                        * Referenced by: '<Root>/Constant7'
                                        */
  real_T initHip;                      /* Variable: initHip
                                        * Referenced by:
                                        *   '<Root>/State-Transition Diagram'
                                        *   '<S21>/Constant2'
                                        *   '<S322>/Constant2'
                                        *   '<S621>/Constant2'
                                        */
  real_T initKnee;                     /* Variable: initKnee
                                        * Referenced by:
                                        *   '<Root>/State-Transition Diagram'
                                        *   '<S19>/Knee Joint Angle1'
                                        *   '<S21>/Constant1'
                                        *   '<S321>/Constant5'
                                        *   '<S322>/Constant1'
                                        *   '<S621>/Constant1'
                                        */
  real_T k_landing_angle;              /* Variable: k_landing_angle
                                        * Referenced by:
                                        *   '<S19>/Multiply'
                                        *   '<S321>/Multiply'
                                        *   '<S620>/Multiply'
                                        */
  real_T k_slip;                       /* Variable: k_slip
                                        * Referenced by:
                                        *   '<S822>/Constant'
                                        *   '<S822>/Constant3'
                                        *   '<S823>/Constant1'
                                        */
  real_T knee_bend_factor;             /* Variable: knee_bend_factor
                                        * Referenced by: '<Root>/Constant10'
                                        */
  real_T knee_fuzz;                    /* Variable: knee_fuzz
                                        * Referenced by: '<Root>/Constant9'
                                        */
  real_T lower_leg_length;             /* Variable: lower_leg_length
                                        * Referenced by:
                                        *   '<S19>/Constant3'
                                        *   '<S20>/Torque Calculator'
                                        *   '<S321>/Constant3'
                                        *   '<S620>/Constant3'
                                        *   '<S627>/Constant5'
                                        *   '<S627>/Constant7'
                                        *   '<S822>/Constant10'
                                        *   '<S822>/Constant6'
                                        *   '<S823>/Constant4'
                                        *   '<S824>/Constant3'
                                        *   '<S824>/Constant9'
                                        *   '<S825>/Constant3'
                                        *   '<S825>/Constant9'
                                        *   '<S826>/Constant5'
                                        *   '<S826>/Constant7'
                                        *   '<S827>/Constant5'
                                        *   '<S827>/Constant7'
                                        *   '<S828>/Constant3'
                                        *   '<S828>/Constant9'
                                        *   '<S829>/Constant5'
                                        *   '<S829>/Constant7'
                                        */
  real_T max_torque;                   /* Variable: max_torque
                                        * Referenced by:
                                        *   '<S19>/Saturation2'
                                        *   '<S19>/Saturation3'
                                        *   '<S20>/Saturation'
                                        *   '<S20>/Saturation1'
                                        *   '<S21>/Saturation2'
                                        *   '<S21>/Saturation3'
                                        *   '<S321>/Saturation2'
                                        *   '<S321>/Saturation3'
                                        *   '<S322>/Saturation2'
                                        *   '<S322>/Saturation3'
                                        *   '<S324>/Constant1'
                                        *   '<S324>/Saturation1'
                                        *   '<S620>/Saturation2'
                                        *   '<S620>/Saturation3'
                                        *   '<S621>/Saturation2'
                                        *   '<S621>/Saturation3'
                                        *   '<S623>/Saturation'
                                        *   '<S623>/Saturation1'
                                        */
  real_T r;                            /* Variable: r
                                        * Referenced by:
                                        *   '<S8>/Encoder 1 to Vertical Data'
                                        *   '<S8>/Encoder 2 to Vertical Data'
                                        */
  real_T upper_leg_length;             /* Variable: upper_leg_length
                                        * Referenced by:
                                        *   '<S19>/Constant4'
                                        *   '<S20>/Torque Calculator'
                                        *   '<S321>/Constant4'
                                        *   '<S620>/Constant4'
                                        *   '<S627>/Constant6'
                                        *   '<S627>/Constant8'
                                        *   '<S822>/Constant7'
                                        *   '<S822>/Constant9'
                                        *   '<S823>/Constant3'
                                        *   '<S824>/Constant10'
                                        *   '<S824>/Constant4'
                                        *   '<S825>/Constant10'
                                        *   '<S825>/Constant4'
                                        *   '<S826>/Constant6'
                                        *   '<S826>/Constant8'
                                        *   '<S827>/Constant6'
                                        *   '<S827>/Constant8'
                                        *   '<S828>/Constant10'
                                        *   '<S828>/Constant4'
                                        *   '<S829>/Constant6'
                                        *   '<S829>/Constant8'
                                        */
  real_T v_des;                        /* Variable: v_des
                                        * Referenced by:
                                        *   '<S19>/Constant1'
                                        *   '<S20>/Constant10'
                                        */
  real_T z_des;                        /* Variable: z_des
                                        * Referenced by: '<S20>/Constant5'
                                        */
  real_T PIDController_D;              /* Mask Parameter: PIDController_D
                                        * Referenced by: '<S51>/Derivative Gain'
                                        */
  real_T PIDController1_D;             /* Mask Parameter: PIDController1_D
                                        * Referenced by: '<S99>/Derivative Gain'
                                        */
  real_T PIDController2_D;             /* Mask Parameter: PIDController2_D
                                        * Referenced by: '<S151>/Derivative Gain'
                                        */
  real_T PIDController3_D;             /* Mask Parameter: PIDController3_D
                                        * Referenced by: '<S199>/Derivative Gain'
                                        */
  real_T PIDController2_D_l;           /* Mask Parameter: PIDController2_D_l
                                        * Referenced by: '<S352>/Derivative Gain'
                                        */
  real_T PIDController_D_b;            /* Mask Parameter: PIDController_D_b
                                        * Referenced by: '<S402>/Derivative Gain'
                                        */
  real_T PIDController1_D_j;           /* Mask Parameter: PIDController1_D_j
                                        * Referenced by: '<S450>/Derivative Gain'
                                        */
  real_T PIDController2_D_m;           /* Mask Parameter: PIDController2_D_m
                                        * Referenced by: '<S597>/Derivative Gain'
                                        */
  real_T PIDController_D_l;            /* Mask Parameter: PIDController_D_l
                                        * Referenced by: '<S653>/Derivative Gain'
                                        */
  real_T PIDController1_D_o;           /* Mask Parameter: PIDController1_D_o
                                        * Referenced by: '<S701>/Derivative Gain'
                                        */
  real_T PIDController_D_f;            /* Mask Parameter: PIDController_D_f
                                        * Referenced by: '<S751>/Derivative Gain'
                                        */
  real_T PIDController1_D_l;           /* Mask Parameter: PIDController1_D_l
                                        * Referenced by: '<S799>/Derivative Gain'
                                        */
  real_T PIDController_I;              /* Mask Parameter: PIDController_I
                                        * Referenced by: '<S54>/Integral Gain'
                                        */
  real_T PIDController1_I;             /* Mask Parameter: PIDController1_I
                                        * Referenced by: '<S102>/Integral Gain'
                                        */
  real_T PIDController2_I;             /* Mask Parameter: PIDController2_I
                                        * Referenced by: '<S154>/Integral Gain'
                                        */
  real_T PIDController3_I;             /* Mask Parameter: PIDController3_I
                                        * Referenced by: '<S202>/Integral Gain'
                                        */
  real_T PIDController2_I_i;           /* Mask Parameter: PIDController2_I_i
                                        * Referenced by: '<S355>/Integral Gain'
                                        */
  real_T PIDController_I_k;            /* Mask Parameter: PIDController_I_k
                                        * Referenced by: '<S405>/Integral Gain'
                                        */
  real_T PIDController1_I_k;           /* Mask Parameter: PIDController1_I_k
                                        * Referenced by: '<S453>/Integral Gain'
                                        */
  real_T PIDController2_I_c;           /* Mask Parameter: PIDController2_I_c
                                        * Referenced by: '<S600>/Integral Gain'
                                        */
  real_T PIDController_I_m;            /* Mask Parameter: PIDController_I_m
                                        * Referenced by: '<S656>/Integral Gain'
                                        */
  real_T PIDController1_I_i;           /* Mask Parameter: PIDController1_I_i
                                        * Referenced by: '<S704>/Integral Gain'
                                        */
  real_T PIDController_I_e;            /* Mask Parameter: PIDController_I_e
                                        * Referenced by: '<S754>/Integral Gain'
                                        */
  real_T PIDController1_I_b;           /* Mask Parameter: PIDController1_I_b
                                        * Referenced by: '<S802>/Integral Gain'
                                        */
  real_T DiscreteDerivative1_ICPrevScaledInput;
                        /* Mask Parameter: DiscreteDerivative1_ICPrevScaledInput
                         * Referenced by: '<S4>/UD'
                         */
  real_T DiscreteDerivative_ICPrevScaledInput;
                         /* Mask Parameter: DiscreteDerivative_ICPrevScaledInput
                          * Referenced by: '<S3>/UD'
                          */
  real_T PIDController_InitialConditionForFilter;
                      /* Mask Parameter: PIDController_InitialConditionForFilter
                       * Referenced by: '<S52>/Filter'
                       */
  real_T PIDController1_InitialConditionForFilter;
                     /* Mask Parameter: PIDController1_InitialConditionForFilter
                      * Referenced by: '<S100>/Filter'
                      */
  real_T PIDController2_InitialConditionForFilter;
                     /* Mask Parameter: PIDController2_InitialConditionForFilter
                      * Referenced by: '<S152>/Filter'
                      */
  real_T PIDController3_InitialConditionForFilter;
                     /* Mask Parameter: PIDController3_InitialConditionForFilter
                      * Referenced by: '<S200>/Filter'
                      */
  real_T PIDController2_InitialConditionForFilter_j;
                   /* Mask Parameter: PIDController2_InitialConditionForFilter_j
                    * Referenced by: '<S353>/Filter'
                    */
  real_T PIDController_InitialConditionForFilter_i;
                    /* Mask Parameter: PIDController_InitialConditionForFilter_i
                     * Referenced by: '<S403>/Filter'
                     */
  real_T PIDController1_InitialConditionForFilter_b;
                   /* Mask Parameter: PIDController1_InitialConditionForFilter_b
                    * Referenced by: '<S451>/Filter'
                    */
  real_T PIDController2_InitialConditionForFilter_e;
                   /* Mask Parameter: PIDController2_InitialConditionForFilter_e
                    * Referenced by: '<S598>/Filter'
                    */
  real_T PIDController_InitialConditionForFilter_n;
                    /* Mask Parameter: PIDController_InitialConditionForFilter_n
                     * Referenced by: '<S654>/Filter'
                     */
  real_T PIDController1_InitialConditionForFilter_p;
                   /* Mask Parameter: PIDController1_InitialConditionForFilter_p
                    * Referenced by: '<S702>/Filter'
                    */
  real_T PIDController_InitialConditionForFilter_j;
                    /* Mask Parameter: PIDController_InitialConditionForFilter_j
                     * Referenced by: '<S752>/Filter'
                     */
  real_T PIDController1_InitialConditionForFilter_h;
                   /* Mask Parameter: PIDController1_InitialConditionForFilter_h
                    * Referenced by: '<S800>/Filter'
                    */
  real_T PIDController_InitialConditionForIntegrator;
                  /* Mask Parameter: PIDController_InitialConditionForIntegrator
                   * Referenced by: '<S57>/Integrator'
                   */
  real_T PIDController1_InitialConditionForIntegrator;
                 /* Mask Parameter: PIDController1_InitialConditionForIntegrator
                  * Referenced by: '<S105>/Integrator'
                  */
  real_T PIDController2_InitialConditionForIntegrator;
                 /* Mask Parameter: PIDController2_InitialConditionForIntegrator
                  * Referenced by: '<S157>/Integrator'
                  */
  real_T PIDController3_InitialConditionForIntegrator;
                 /* Mask Parameter: PIDController3_InitialConditionForIntegrator
                  * Referenced by: '<S205>/Integrator'
                  */
  real_T PIDController2_InitialConditionForIntegrator_j;
               /* Mask Parameter: PIDController2_InitialConditionForIntegrator_j
                * Referenced by: '<S358>/Integrator'
                */
  real_T PIDController_InitialConditionForIntegrator_n;
                /* Mask Parameter: PIDController_InitialConditionForIntegrator_n
                 * Referenced by: '<S408>/Integrator'
                 */
  real_T PIDController1_InitialConditionForIntegrator_d;
               /* Mask Parameter: PIDController1_InitialConditionForIntegrator_d
                * Referenced by: '<S456>/Integrator'
                */
  real_T PIDController2_InitialConditionForIntegrator_f;
               /* Mask Parameter: PIDController2_InitialConditionForIntegrator_f
                * Referenced by: '<S603>/Integrator'
                */
  real_T PIDController_InitialConditionForIntegrator_f;
                /* Mask Parameter: PIDController_InitialConditionForIntegrator_f
                 * Referenced by: '<S659>/Integrator'
                 */
  real_T PIDController1_InitialConditionForIntegrator_dr;
              /* Mask Parameter: PIDController1_InitialConditionForIntegrator_dr
               * Referenced by: '<S707>/Integrator'
               */
  real_T PIDController_InitialConditionForIntegrator_p;
                /* Mask Parameter: PIDController_InitialConditionForIntegrator_p
                 * Referenced by: '<S757>/Integrator'
                 */
  real_T PIDController1_InitialConditionForIntegrator_o;
               /* Mask Parameter: PIDController1_InitialConditionForIntegrator_o
                * Referenced by: '<S805>/Integrator'
                */
  real_T PIDController_N;              /* Mask Parameter: PIDController_N
                                        * Referenced by: '<S60>/Filter Coefficient'
                                        */
  real_T PIDController1_N;             /* Mask Parameter: PIDController1_N
                                        * Referenced by: '<S108>/Filter Coefficient'
                                        */
  real_T PIDController2_N;             /* Mask Parameter: PIDController2_N
                                        * Referenced by: '<S160>/Filter Coefficient'
                                        */
  real_T PIDController3_N;             /* Mask Parameter: PIDController3_N
                                        * Referenced by: '<S208>/Filter Coefficient'
                                        */
  real_T PIDController2_N_l;           /* Mask Parameter: PIDController2_N_l
                                        * Referenced by: '<S361>/Filter Coefficient'
                                        */
  real_T PIDController_N_l;            /* Mask Parameter: PIDController_N_l
                                        * Referenced by: '<S411>/Filter Coefficient'
                                        */
  real_T PIDController1_N_o;           /* Mask Parameter: PIDController1_N_o
                                        * Referenced by: '<S459>/Filter Coefficient'
                                        */
  real_T PIDController2_N_j;           /* Mask Parameter: PIDController2_N_j
                                        * Referenced by: '<S606>/Filter Coefficient'
                                        */
  real_T PIDController_N_b;            /* Mask Parameter: PIDController_N_b
                                        * Referenced by: '<S662>/Filter Coefficient'
                                        */
  real_T PIDController1_N_n;           /* Mask Parameter: PIDController1_N_n
                                        * Referenced by: '<S710>/Filter Coefficient'
                                        */
  real_T PIDController_N_n;            /* Mask Parameter: PIDController_N_n
                                        * Referenced by: '<S760>/Filter Coefficient'
                                        */
  real_T PIDController1_N_j;           /* Mask Parameter: PIDController1_N_j
                                        * Referenced by: '<S808>/Filter Coefficient'
                                        */
  real_T PIDController_P;              /* Mask Parameter: PIDController_P
                                        * Referenced by: '<S62>/Proportional Gain'
                                        */
  real_T PIDController1_P;             /* Mask Parameter: PIDController1_P
                                        * Referenced by: '<S110>/Proportional Gain'
                                        */
  real_T PIDController2_P;             /* Mask Parameter: PIDController2_P
                                        * Referenced by: '<S162>/Proportional Gain'
                                        */
  real_T PIDController3_P;             /* Mask Parameter: PIDController3_P
                                        * Referenced by: '<S210>/Proportional Gain'
                                        */
  real_T PIDController2_P_i;           /* Mask Parameter: PIDController2_P_i
                                        * Referenced by: '<S363>/Proportional Gain'
                                        */
  real_T PIDController_P_n;            /* Mask Parameter: PIDController_P_n
                                        * Referenced by: '<S413>/Proportional Gain'
                                        */
  real_T PIDController1_P_e;           /* Mask Parameter: PIDController1_P_e
                                        * Referenced by: '<S461>/Proportional Gain'
                                        */
  real_T PIDController2_P_o;           /* Mask Parameter: PIDController2_P_o
                                        * Referenced by: '<S608>/Proportional Gain'
                                        */
  real_T PIDController_P_j;            /* Mask Parameter: PIDController_P_j
                                        * Referenced by: '<S664>/Proportional Gain'
                                        */
  real_T PIDController1_P_j;           /* Mask Parameter: PIDController1_P_j
                                        * Referenced by: '<S712>/Proportional Gain'
                                        */
  real_T PIDController_P_i;            /* Mask Parameter: PIDController_P_i
                                        * Referenced by: '<S762>/Proportional Gain'
                                        */
  real_T PIDController1_P_ex;          /* Mask Parameter: PIDController1_P_ex
                                        * Referenced by: '<S810>/Proportional Gain'
                                        */
  real_T CANWrite1_P1_Size[2];         /* Computed Parameter: CANWrite1_P1_Size
                                        * Referenced by: '<S5>/CAN Write1'
                                        */
  real_T CANWrite1_P1[8];
  /* Expression: [initValues(1:4) messageType initValues(6) enableStatusPort BusInput]
   * Referenced by: '<S5>/CAN Write1'
   */
  real_T CANWrite2_P1_Size[2];         /* Computed Parameter: CANWrite2_P1_Size
                                        * Referenced by: '<S5>/CAN Write2'
                                        */
  real_T CANWrite2_P1[8];
  /* Expression: [initValues(1:4) messageType initValues(6) enableStatusPort BusInput]
   * Referenced by: '<S5>/CAN Write2'
   */
  real_T Constant3_Value;              /* Expression: 0
                                        * Referenced by: '<S6>/Constant3'
                                        */
  real_T CANWrite1_P1_Size_k[2];      /* Computed Parameter: CANWrite1_P1_Size_k
                                       * Referenced by: '<S6>/CAN Write1'
                                       */
  real_T CANWrite1_P1_o[8];
  /* Expression: [initValues(1:4) messageType initValues(6) enableStatusPort BusInput]
   * Referenced by: '<S6>/CAN Write1'
   */
  real_T Constant3_Value_e;            /* Expression: 0
                                        * Referenced by: '<S7>/Constant3'
                                        */
  real_T CANWrite1_P1_Size_n[2];      /* Computed Parameter: CANWrite1_P1_Size_n
                                       * Referenced by: '<S7>/CAN Write1'
                                       */
  real_T CANWrite1_P1_l[8];
  /* Expression: [initValues(1:4) messageType initValues(6) enableStatusPort BusInput]
   * Referenced by: '<S7>/CAN Write1'
   */
  real_T Z_Y0;                         /* Computed Parameter: Z_Y0
                                        * Referenced by: '<S8>/Z'
                                        */
  real_T Xd_Y0;                        /* Computed Parameter: Xd_Y0
                                        * Referenced by: '<S8>/Xd'
                                        */
  real_T th1_Y0;                       /* Computed Parameter: th1_Y0
                                        * Referenced by: '<S8>/th1'
                                        */
  real_T th2_Y0;                       /* Computed Parameter: th2_Y0
                                        * Referenced by: '<S8>/th2'
                                        */
  real_T I1_Y0;                        /* Computed Parameter: I1_Y0
                                        * Referenced by: '<S8>/I1'
                                        */
  real_T I2_Y0;                        /* Computed Parameter: I2_Y0
                                        * Referenced by: '<S8>/I2'
                                        */
  real_T X_Y0;                         /* Computed Parameter: X_Y0
                                        * Referenced by: '<S8>/X'
                                        */
  real_T Zd_Y0;                        /* Computed Parameter: Zd_Y0
                                        * Referenced by: '<S8>/Zd'
                                        */
  real_T CANRead_P1_Size[2];           /* Computed Parameter: CANRead_P1_Size
                                        * Referenced by: '<S8>/CAN Read'
                                        */
  real_T CANRead_P1[7];
            /* Expression: [initValues(1:4) messageType initValues(6) BusOutput]
             * Referenced by: '<S8>/CAN Read'
             */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S8>/Gain'
                                        */
  real_T Gain2_Gain;                   /* Expression: -1
                                        * Referenced by: '<S8>/Gain2'
                                        */
  real_T Filter_gainval;               /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S52>/Filter'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S57>/Integrator'
                                        */
  real_T Filter_gainval_e;             /* Computed Parameter: Filter_gainval_e
                                        * Referenced by: '<S100>/Filter'
                                        */
  real_T Integrator_gainval_l;       /* Computed Parameter: Integrator_gainval_l
                                      * Referenced by: '<S105>/Integrator'
                                      */
  real_T Constant2_Value;              /* Expression: 1
                                        * Referenced by: '<S19>/Constant2'
                                        */
  real_T Constant8_Value;              /* Expression: 1
                                        * Referenced by: '<S20>/Constant8'
                                        */
  real_T Constant9_Value;              /* Expression: 0
                                        * Referenced by: '<S20>/Constant9'
                                        */
  real_T Constant7_Value;              /* Expression: 1
                                        * Referenced by: '<S20>/Constant7'
                                        */
  real_T DiscreteTimeIntegrator_gainval;
                           /* Computed Parameter: DiscreteTimeIntegrator_gainval
                            * Referenced by: '<S20>/Discrete-Time Integrator'
                            */
  real_T DiscreteTimeIntegrator_IC;    /* Expression: 0
                                        * Referenced by: '<S20>/Discrete-Time Integrator'
                                        */
  real_T Filter_gainval_f;             /* Computed Parameter: Filter_gainval_f
                                        * Referenced by: '<S152>/Filter'
                                        */
  real_T Integrator_gainval_b;       /* Computed Parameter: Integrator_gainval_b
                                      * Referenced by: '<S157>/Integrator'
                                      */
  real_T Filter_gainval_b;             /* Computed Parameter: Filter_gainval_b
                                        * Referenced by: '<S200>/Filter'
                                        */
  real_T Integrator_gainval_a;       /* Computed Parameter: Integrator_gainval_a
                                      * Referenced by: '<S205>/Integrator'
                                      */
  real_T Constant2_Value_h;            /* Expression: 3
                                        * Referenced by: '<S20>/Constant2'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S23>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S23>/Constant1'
                                        */
  real_T Constant2_Value_n;            /* Expression: 5
                                        * Referenced by: '<S23>/Constant2'
                                        */
  real_T Filter_gainval_c;             /* Computed Parameter: Filter_gainval_c
                                        * Referenced by: '<S353>/Filter'
                                        */
  real_T Integrator_gainval_g;       /* Computed Parameter: Integrator_gainval_g
                                      * Referenced by: '<S358>/Integrator'
                                      */
  real_T Saturation1_UpperSat;         /* Expression: 8
                                        * Referenced by: '<S320>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: -8
                                        * Referenced by: '<S320>/Saturation1'
                                        */
  real_T Constant_Value_f;             /* Expression: 1
                                        * Referenced by: '<S320>/Constant'
                                        */
  real_T Constant2_Value_e;            /* Expression: 3
                                        * Referenced by: '<S320>/Constant2'
                                        */
  real_T Filter_gainval_c2;            /* Computed Parameter: Filter_gainval_c2
                                        * Referenced by: '<S403>/Filter'
                                        */
  real_T Integrator_gainval_gd;     /* Computed Parameter: Integrator_gainval_gd
                                     * Referenced by: '<S408>/Integrator'
                                     */
  real_T Filter_gainval_g;             /* Computed Parameter: Filter_gainval_g
                                        * Referenced by: '<S451>/Filter'
                                        */
  real_T Integrator_gainval_d;       /* Computed Parameter: Integrator_gainval_d
                                      * Referenced by: '<S456>/Integrator'
                                      */
  real_T Constant2_Value_j;            /* Expression: 1
                                        * Referenced by: '<S321>/Constant2'
                                        */
  real_T Gain_Gain_k;                  /* Expression: .25
                                        * Referenced by: '<S324>/Gain'
                                        */
  real_T Filter_gainval_ew;            /* Computed Parameter: Filter_gainval_ew
                                        * Referenced by: '<S598>/Filter'
                                        */
  real_T Integrator_gainval_e;       /* Computed Parameter: Integrator_gainval_e
                                      * Referenced by: '<S603>/Integrator'
                                      */
  real_T Constant2_Value_p;            /* Expression: 4
                                        * Referenced by: '<S324>/Constant2'
                                        */
  real_T Filter_gainval_d;             /* Computed Parameter: Filter_gainval_d
                                        * Referenced by: '<S654>/Filter'
                                        */
  real_T Integrator_gainval_p;       /* Computed Parameter: Integrator_gainval_p
                                      * Referenced by: '<S659>/Integrator'
                                      */
  real_T Filter_gainval_l;             /* Computed Parameter: Filter_gainval_l
                                        * Referenced by: '<S702>/Filter'
                                        */
  real_T Integrator_gainval_o;       /* Computed Parameter: Integrator_gainval_o
                                      * Referenced by: '<S707>/Integrator'
                                      */
  real_T Constant2_Value_o;            /* Expression: 1
                                        * Referenced by: '<S620>/Constant2'
                                        */
  real_T Filter_gainval_m;             /* Computed Parameter: Filter_gainval_m
                                        * Referenced by: '<S752>/Filter'
                                        */
  real_T Integrator_gainval_eb;     /* Computed Parameter: Integrator_gainval_eb
                                     * Referenced by: '<S757>/Integrator'
                                     */
  real_T Filter_gainval_gw;            /* Computed Parameter: Filter_gainval_gw
                                        * Referenced by: '<S800>/Filter'
                                        */
  real_T Integrator_gainval_oy;     /* Computed Parameter: Integrator_gainval_oy
                                     * Referenced by: '<S805>/Integrator'
                                     */
  real_T Constant_Value_l;             /* Expression: 0
                                        * Referenced by: '<S621>/Constant'
                                        */
  real_T Constant2_Value_ea;           /* Expression: 0
                                        * Referenced by: '<S822>/Constant2'
                                        */
  real_T Constant8_Value_d;            /* Expression: 1
                                        * Referenced by: '<S822>/Constant8'
                                        */
  real_T Constant2_Value_f;            /* Expression: 3
                                        * Referenced by: '<S623>/Constant2'
                                        */
  real_T Constant_Value_c;             /* Expression: 0
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T Constant3_Value_m;            /* Expression: 0
                                        * Referenced by: '<Root>/Constant3'
                                        */
  real_T Constant1_Value_p;            /* Expression: 0
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T Constant4_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant4'
                                        */
  real_T Constant5_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant5'
                                        */
  real_T Constant6_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant6'
                                        */
  real_T CANSetup_P1_Size[2];          /* Computed Parameter: CANSetup_P1_Size
                                        * Referenced by: '<Root>/CAN Setup '
                                        */
  real_T CANSetup_P1[40];
  /* Expression: [moduleInitValues, chn1, ArbitrationManbdrChn1, FdManbdrChn1, chn2, ArbitrationManbdrChn2, FdManbdrChn2, chn3, ArbitrationManbdrChn3, FdManbdrChn3, chn4, ArbitrationManbdrChn4, FdManbdrChn4]
   * Referenced by: '<Root>/CAN Setup '
   */
  real_T CANSetup_P2_Size[2];          /* Computed Parameter: CANSetup_P2_Size
                                        * Referenced by: '<Root>/CAN Setup '
                                        */
  real_T CANSetup_P2;                  /* Expression: initStruct
                                        * Referenced by: '<Root>/CAN Setup '
                                        */
  real_T CANSetup_P3_Size[2];          /* Computed Parameter: CANSetup_P3_Size
                                        * Referenced by: '<Root>/CAN Setup '
                                        */
  real_T CANSetup_P3;                  /* Expression: termStruct
                                        * Referenced by: '<Root>/CAN Setup '
                                        */
  real_T TSamp_WtEt;                   /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S4>/TSamp'
                                        */
  real_T TSamp_WtEt_n;                 /* Computed Parameter: TSamp_WtEt_n
                                        * Referenced by: '<S3>/TSamp'
                                        */
  real_T isReady_Value;                /* Expression: 4
                                        * Referenced by: '<Root>/isReady'
                                        */
  real_T DiscreteTimeIntegrator_gainval_f;
                         /* Computed Parameter: DiscreteTimeIntegrator_gainval_f
                          * Referenced by: '<Root>/Discrete-Time Integrator'
                          */
  real_T DiscreteTimeIntegrator_IC_m;  /* Expression: 0
                                        * Referenced by: '<Root>/Discrete-Time Integrator'
                                        */
  real_T Switch_Threshold;             /* Expression: 1
                                        * Referenced by: '<Root>/Switch'
                                        */
  real_T Delay1_InitialCondition[2];   /* Expression: [0 0]
                                        * Referenced by: '<Root>/Delay1'
                                        */
  real_T Gain10_Gain;                  /* Expression: -1
                                        * Referenced by: '<Root>/Gain10'
                                        */
  real_T Gain8_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain8'
                                        */
  real_T Gain6_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain6'
                                        */
  real_T Gain1_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T Constant2_Value_hf;           /* Expression: 1
                                        * Referenced by: '<Root>/Constant2'
                                        */
  real_T CANStatus_P1_Size[2];         /* Computed Parameter: CANStatus_P1_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P1;                 /* Expression: moduleId
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P2_Size[2];         /* Computed Parameter: CANStatus_P2_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P2;                 /* Expression: sampleTime
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P3_Size[2];         /* Computed Parameter: CANStatus_P3_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P3;                 /* Expression: channel
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P4_Size[2];         /* Computed Parameter: CANStatus_P4_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P4;                 /* Expression: busRecovery
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P5_Size[2];         /* Computed Parameter: CANStatus_P5_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P5;                 /* Expression: avgBusLoad
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P6_Size[2];         /* Computed Parameter: CANStatus_P6_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P6;                 /* Expression: opMode
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P7_Size[2];         /* Computed Parameter: CANStatus_P7_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P7;                 /* Expression: brp
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P8_Size[2];         /* Computed Parameter: CANStatus_P8_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P8;                 /* Expression: timeSegment1
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P9_Size[2];         /* Computed Parameter: CANStatus_P9_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P9;                 /* Expression: timeSegment2
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P10_Size[2];        /* Computed Parameter: CANStatus_P10_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P10;                /* Expression: synchronisationJumpWidth
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P11_Size[2];        /* Computed Parameter: CANStatus_P11_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P11;                /* Expression: dataTSEG1
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P12_Size[2];        /* Computed Parameter: CANStatus_P12_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P12;                /* Expression: dataTSEG2
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P13_Size[2];        /* Computed Parameter: CANStatus_P13_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P13;                /* Expression: dataSJW
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P14_Size[2];        /* Computed Parameter: CANStatus_P14_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P14;                /* Expression: txPending
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P15_Size[2];        /* Computed Parameter: CANStatus_P15_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P15;                /* Expression: dataOverrunTx
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P16_Size[2];        /* Computed Parameter: CANStatus_P16_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P16;                /* Expression: receiving
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P17_Size[2];        /* Computed Parameter: CANStatus_P17_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P17;                /* Expression: RxQueueEmpty
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P18_Size[2];        /* Computed Parameter: CANStatus_P18_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P18;                /* Expression: dataOverrunRcv
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P19_Size[2];        /* Computed Parameter: CANStatus_P19_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P19;                /* Expression: errWarnLimit
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P20_Size[2];        /* Computed Parameter: CANStatus_P20_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P20;                /* Expression: errPassLimit
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P21_Size[2];        /* Computed Parameter: CANStatus_P21_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P21;                /* Expression: errBusOff
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P22_Size[2];        /* Computed Parameter: CANStatus_P22_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P22;                /* Expression: busRecoveryCounter
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P23_Size[2];        /* Computed Parameter: CANStatus_P23_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P23;                /* Expression: initModeAct
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P24_Size[2];        /* Computed Parameter: CANStatus_P24_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P24;                /* Expression: busCouplingErr
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P25_Size[2];        /* Computed Parameter: CANStatus_P25_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P25;                /* Expression: transceiverErr
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P26_Size[2];        /* Computed Parameter: CANStatus_P26_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P26;                /* Expression: controllerCpuLoad
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P27_Size[2];        /* Computed Parameter: CANStatus_P27_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P27;                /* Expression: controllerLiveCount
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P28_Size[2];        /* Computed Parameter: CANStatus_P28_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P28;                /* Expression: rxBufferLevel
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P29_Size[2];        /* Computed Parameter: CANStatus_P29_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P29;                /* Expression: txBufferLevel
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P30_Size[2];        /* Computed Parameter: CANStatus_P30_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P30;                /* Expression: arrayOutput
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P31_Size[2];        /* Computed Parameter: CANStatus_P31_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P31;                /* Expression: moduleType
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P32_Size[2];        /* Computed Parameter: CANStatus_P32_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P32;                /* Expression: qtyStatBlk
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P33_Size[2];        /* Computed Parameter: CANStatus_P33_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P33;                /* Expression: ptIdx
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P34_Size[2];        /* Computed Parameter: CANStatus_P34_Size
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T CANStatus_P34;                /* Expression: isFDMod
                                        * Referenced by: '<Root>/CAN Status'
                                        */
  real_T DataStoreMemory1_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory1'
                                        */
  real_T DataStoreMemory12_InitialValue;/* Expression: 0
                                         * Referenced by: '<Root>/Data Store Memory12'
                                         */
  real_T DataStoreMemory13_InitialValue;/* Expression: 0
                                         * Referenced by: '<Root>/Data Store Memory13'
                                         */
  real_T DataStoreMemory14_InitialValue;/* Expression: 0
                                         * Referenced by: '<Root>/Data Store Memory14'
                                         */
  real_T DataStoreMemory15_InitialValue;/* Expression: 0
                                         * Referenced by: '<Root>/Data Store Memory15'
                                         */
  real_T DataStoreMemory16_InitialValue;/* Expression: 0
                                         * Referenced by: '<Root>/Data Store Memory16'
                                         */
  real_T DataStoreMemory2_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory2'
                                        */
  real_T DataStoreMemory3_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory3'
                                        */
  real_T DataStoreMemory4_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory4'
                                        */
  real_T DataStoreMemory5_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory5'
                                        */
  real_T DataStoreMemory6_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory6'
                                        */
  real_T DataStoreMemory7_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory7'
                                        */
  real_T DataStoreMemory8_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory8'
                                        */
  boolean_T Delay2_InitialCondition;   /* Expression: true
                                        * Referenced by: '<Root>/Delay2'
                                        */
  boolean_T DataStoreMemory_InitialValue;
                             /* Computed Parameter: DataStoreMemory_InitialValue
                              * Referenced by: '<Root>/Data Store Memory'
                              */
  uint8_T Constant_Value_g;            /* Expression: uint8(1)
                                        * Referenced by: '<S5>/Constant'
                                        */
  uint8_T Constant_Value_fg[8];
                 /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
                  * Referenced by: '<S6>/Constant'
                  */
  uint8_T Constant1_Value_n[8];
                 /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
                  * Referenced by: '<S6>/Constant1'
                  */
  uint8_T Constant2_Value_g[8];
                 /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
                  * Referenced by: '<S6>/Constant2'
                  */
  uint8_T Constant_Value_k[8];
                 /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
                  * Referenced by: '<S7>/Constant'
                  */
  uint8_T Constant1_Value_d[8];
                 /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
                  * Referenced by: '<S7>/Constant1'
                  */
  uint8_T Constant2_Value_of[8];
                 /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
                  * Referenced by: '<S7>/Constant2'
                  */
  PHRControl_Unload_cal_type PHRControl_Unload_le_cal;/* '<S18>/Unload' */
  PHRControl_Load_cal_type PHRControl_Load_k_cal;/* '<S18>/Load' */
  PHRControl_Unload_cal_type PHRControl_Unload_l_cal;/* '<S17>/Unload' */
  PHRControl_Load_cal_type PHRControl_Load_h_cal;/* '<S17>/Load' */
  PHRControl_Idle_cal_type PHRControl_Idle_l_cal;/* '<S17>/Idle' */
  PHRControl_Load_cal_type PHRControl_Load_cal;/* '<S16>/Load' */
  PHRControl_Idle_cal_type PHRControl_Idle_cal;/* '<S16>/Idle' */
};

/* Storage class 'PageSwitching' */
extern PHRControl_cal_type PHRControl_cal_impl;
extern PHRControl_cal_type *PHRControl_cal;

#endif                                 /* RTW_HEADER_PHRControl_cal_h_ */

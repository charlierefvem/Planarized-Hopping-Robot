#include "PHRControl_cal.h"
#include "PHRControl.h"

/* Storage class 'PageSwitching' */
PHRControl_cal_type PHRControl_cal_impl = {
  /* Variable: CPR
   * Referenced by:
   *   '<S8>/Encoder 1 to Vertical Data'
   *   '<S8>/Encoder 2 to Vertical Data'
   */
  10000.0,

  /* Variable: amplitude
   * Referenced by: '<S20>/Constant'
   */
  0.0,

  /* Variable: contactTime
   * Referenced by: '<S321>/Constant'
   */
  0.0,

  /* Variable: desired_speed
   * Referenced by:
   *   '<S321>/Constant1'
   *   '<S324>/Constant'
   *   '<S620>/Constant1'
   */
  0.0,

  /* Variable: equilibrium_length
   * Referenced by:
   *   '<S620>/Knee Joint Angle1'
   *   '<S822>/Constant1'
   *   '<S822>/Constant4'
   *   '<S823>/Constant'
   */
  0.0,

  /* Variable: h
   * Referenced by: '<S8>/Encoder 1 to Vertical Data'
   */
  0.35,

  /* Variable: hip_bend_factor
   * Referenced by:
   *   '<Root>/Constant8'
   *   '<S320>/Constant4'
   */
  0.0,

  /* Variable: hip_fuzz
   * Referenced by: '<Root>/Constant7'
   */
  0.0,

  /* Variable: initHip
   * Referenced by:
   *   '<Root>/State-Transition Diagram'
   *   '<S21>/Constant2'
   *   '<S322>/Constant2'
   *   '<S621>/Constant2'
   */
  0.52359877559829882,

  /* Variable: initKnee
   * Referenced by:
   *   '<Root>/State-Transition Diagram'
   *   '<S19>/Knee Joint Angle1'
   *   '<S21>/Constant1'
   *   '<S321>/Constant5'
   *   '<S322>/Constant1'
   *   '<S621>/Constant1'
   */
  -1.0471975511965976,

  /* Variable: k_landing_angle
   * Referenced by:
   *   '<S19>/Multiply'
   *   '<S321>/Multiply'
   *   '<S620>/Multiply'
   */
  0.0,

  /* Variable: k_slip
   * Referenced by:
   *   '<S822>/Constant'
   *   '<S822>/Constant3'
   *   '<S823>/Constant1'
   */
  0.0,

  /* Variable: knee_bend_factor
   * Referenced by: '<Root>/Constant10'
   */
  0.0,

  /* Variable: knee_fuzz
   * Referenced by: '<Root>/Constant9'
   */
  0.0,

  /* Variable: lower_leg_length
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
  0.15,

  /* Variable: max_torque
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
  8.0,

  /* Variable: r
   * Referenced by:
   *   '<S8>/Encoder 1 to Vertical Data'
   *   '<S8>/Encoder 2 to Vertical Data'
   */
  1.35,

  /* Variable: upper_leg_length
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
  0.15,

  /* Variable: v_des
   * Referenced by:
   *   '<S19>/Constant1'
   *   '<S20>/Constant10'
   */
  0.0,

  /* Variable: z_des
   * Referenced by: '<S20>/Constant5'
   */
  0.0,

  /* Mask Parameter: PIDController_D
   * Referenced by: '<S51>/Derivative Gain'
   */
  10.0,

  /* Mask Parameter: PIDController1_D
   * Referenced by: '<S99>/Derivative Gain'
   */
  5.0,

  /* Mask Parameter: PIDController2_D
   * Referenced by: '<S151>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController3_D
   * Referenced by: '<S199>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController2_D_l
   * Referenced by: '<S352>/Derivative Gain'
   */
  10.0,

  /* Mask Parameter: PIDController_D_b
   * Referenced by: '<S402>/Derivative Gain'
   */
  10.0,

  /* Mask Parameter: PIDController1_D_j
   * Referenced by: '<S450>/Derivative Gain'
   */
  5.0,

  /* Mask Parameter: PIDController2_D_m
   * Referenced by: '<S597>/Derivative Gain'
   */
  10.0,

  /* Mask Parameter: PIDController_D_l
   * Referenced by: '<S653>/Derivative Gain'
   */
  10.0,

  /* Mask Parameter: PIDController1_D_o
   * Referenced by: '<S701>/Derivative Gain'
   */
  5.0,

  /* Mask Parameter: PIDController_D_f
   * Referenced by: '<S751>/Derivative Gain'
   */
  10.0,

  /* Mask Parameter: PIDController1_D_l
   * Referenced by: '<S799>/Derivative Gain'
   */
  5.0,

  /* Mask Parameter: PIDController_I
   * Referenced by: '<S54>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController1_I
   * Referenced by: '<S102>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController2_I
   * Referenced by: '<S154>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController3_I
   * Referenced by: '<S202>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController2_I_i
   * Referenced by: '<S355>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController_I_k
   * Referenced by: '<S405>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController1_I_k
   * Referenced by: '<S453>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController2_I_c
   * Referenced by: '<S600>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController_I_m
   * Referenced by: '<S656>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController1_I_i
   * Referenced by: '<S704>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController_I_e
   * Referenced by: '<S754>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: PIDController1_I_b
   * Referenced by: '<S802>/Integral Gain'
   */
  0.0,

  /* Mask Parameter: DiscreteDerivative1_ICPrevScaledInput
   * Referenced by: '<S4>/UD'
   */
  0.0,

  /* Mask Parameter: DiscreteDerivative_ICPrevScaledInput
   * Referenced by: '<S3>/UD'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionForFilter
   * Referenced by: '<S52>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditionForFilter
   * Referenced by: '<S100>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController2_InitialConditionForFilter
   * Referenced by: '<S152>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController3_InitialConditionForFilter
   * Referenced by: '<S200>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController2_InitialConditionForFilter_j
   * Referenced by: '<S353>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionForFilter_i
   * Referenced by: '<S403>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditionForFilter_b
   * Referenced by: '<S451>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController2_InitialConditionForFilter_e
   * Referenced by: '<S598>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionForFilter_n
   * Referenced by: '<S654>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditionForFilter_p
   * Referenced by: '<S702>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionForFilter_j
   * Referenced by: '<S752>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditionForFilter_h
   * Referenced by: '<S800>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionForIntegrator
   * Referenced by: '<S57>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditionForIntegrator
   * Referenced by: '<S105>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController2_InitialConditionForIntegrator
   * Referenced by: '<S157>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController3_InitialConditionForIntegrator
   * Referenced by: '<S205>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController2_InitialConditionForIntegrator_j
   * Referenced by: '<S358>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionForIntegrator_n
   * Referenced by: '<S408>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditionForIntegrator_d
   * Referenced by: '<S456>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController2_InitialConditionForIntegrator_f
   * Referenced by: '<S603>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionForIntegrator_f
   * Referenced by: '<S659>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditionForIntegrator_dr
   * Referenced by: '<S707>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditionForIntegrator_p
   * Referenced by: '<S757>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController1_InitialConditionForIntegrator_o
   * Referenced by: '<S805>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController_N
   * Referenced by: '<S60>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController1_N
   * Referenced by: '<S108>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController2_N
   * Referenced by: '<S160>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController3_N
   * Referenced by: '<S208>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController2_N_l
   * Referenced by: '<S361>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController_N_l
   * Referenced by: '<S411>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController1_N_o
   * Referenced by: '<S459>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController2_N_j
   * Referenced by: '<S606>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController_N_b
   * Referenced by: '<S662>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController1_N_n
   * Referenced by: '<S710>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController_N_n
   * Referenced by: '<S760>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController1_N_j
   * Referenced by: '<S808>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController_P
   * Referenced by: '<S62>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: PIDController1_P
   * Referenced by: '<S110>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: PIDController2_P
   * Referenced by: '<S162>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: PIDController3_P
   * Referenced by: '<S210>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: PIDController2_P_i
   * Referenced by: '<S363>/Proportional Gain'
   */
  100.0,

  /* Mask Parameter: PIDController_P_n
   * Referenced by: '<S413>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: PIDController1_P_e
   * Referenced by: '<S461>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: PIDController2_P_o
   * Referenced by: '<S608>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: PIDController_P_j
   * Referenced by: '<S664>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: PIDController1_P_j
   * Referenced by: '<S712>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: PIDController_P_i
   * Referenced by: '<S762>/Proportional Gain'
   */
  50.0,

  /* Mask Parameter: PIDController1_P_ex
   * Referenced by: '<S810>/Proportional Gain'
   */
  50.0,

  /* Computed Parameter: CANWrite1_P1_Size
   * Referenced by: '<S5>/CAN Write1'
   */
  { 1.0, 8.0 },

  /* Expression: [initValues(1:4) messageType initValues(6) enableStatusPort BusInput]
   * Referenced by: '<S5>/CAN Write1'
   */
  { 691.0, 1.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0 },

  /* Computed Parameter: CANWrite2_P1_Size
   * Referenced by: '<S5>/CAN Write2'
   */
  { 1.0, 8.0 },

  /* Expression: [initValues(1:4) messageType initValues(6) enableStatusPort BusInput]
   * Referenced by: '<S5>/CAN Write2'
   */
  { 691.0, 1.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0 },

  /* Expression: 0
   * Referenced by: '<S6>/Constant3'
   */
  0.0,

  /* Computed Parameter: CANWrite1_P1_Size_k
   * Referenced by: '<S6>/CAN Write1'
   */
  { 1.0, 8.0 },

  /* Expression: [initValues(1:4) messageType initValues(6) enableStatusPort BusInput]
   * Referenced by: '<S6>/CAN Write1'
   */
  { 691.0, 1.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0 },

  /* Expression: 0
   * Referenced by: '<S7>/Constant3'
   */
  0.0,

  /* Computed Parameter: CANWrite1_P1_Size_n
   * Referenced by: '<S7>/CAN Write1'
   */
  { 1.0, 8.0 },

  /* Expression: [initValues(1:4) messageType initValues(6) enableStatusPort BusInput]
   * Referenced by: '<S7>/CAN Write1'
   */
  { 691.0, 1.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0 },

  /* Computed Parameter: Z_Y0
   * Referenced by: '<S8>/Z'
   */
  0.0,

  /* Computed Parameter: Xd_Y0
   * Referenced by: '<S8>/Xd'
   */
  0.0,

  /* Computed Parameter: th1_Y0
   * Referenced by: '<S8>/th1'
   */
  0.0,

  /* Computed Parameter: th2_Y0
   * Referenced by: '<S8>/th2'
   */
  0.0,

  /* Computed Parameter: I1_Y0
   * Referenced by: '<S8>/I1'
   */
  0.0,

  /* Computed Parameter: I2_Y0
   * Referenced by: '<S8>/I2'
   */
  0.0,

  /* Computed Parameter: X_Y0
   * Referenced by: '<S8>/X'
   */
  0.0,

  /* Computed Parameter: Zd_Y0
   * Referenced by: '<S8>/Zd'
   */
  0.0,

  /* Computed Parameter: CANRead_P1_Size
   * Referenced by: '<S8>/CAN Read'
   */
  { 1.0, 7.0 },

  /* Expression: [initValues(1:4) messageType initValues(6) BusOutput]
   * Referenced by: '<S8>/CAN Read'
   */
  { 691.0, 1.0, -1.0, 0.0, 1.0, 1.0, 0.0 },

  /* Expression: -1
   * Referenced by: '<S8>/Gain'
   */
  -1.0,

  /* Expression: -1
   * Referenced by: '<S8>/Gain2'
   */
  -1.0,

  /* Computed Parameter: Filter_gainval
   * Referenced by: '<S52>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S57>/Integrator'
   */
  0.005,

  /* Computed Parameter: Filter_gainval_e
   * Referenced by: '<S100>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_l
   * Referenced by: '<S105>/Integrator'
   */
  0.005,

  /* Expression: 1
   * Referenced by: '<S19>/Constant2'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S20>/Constant8'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S20>/Constant9'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S20>/Constant7'
   */
  1.0,

  /* Computed Parameter: DiscreteTimeIntegrator_gainval
   * Referenced by: '<S20>/Discrete-Time Integrator'
   */
  0.005,

  /* Expression: 0
   * Referenced by: '<S20>/Discrete-Time Integrator'
   */
  0.0,

  /* Computed Parameter: Filter_gainval_f
   * Referenced by: '<S152>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_b
   * Referenced by: '<S157>/Integrator'
   */
  0.005,

  /* Computed Parameter: Filter_gainval_b
   * Referenced by: '<S200>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_a
   * Referenced by: '<S205>/Integrator'
   */
  0.005,

  /* Expression: 3
   * Referenced by: '<S20>/Constant2'
   */
  3.0,

  /* Expression: 0
   * Referenced by: '<S23>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S23>/Constant1'
   */
  0.0,

  /* Expression: 5
   * Referenced by: '<S23>/Constant2'
   */
  5.0,

  /* Computed Parameter: Filter_gainval_c
   * Referenced by: '<S353>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_g
   * Referenced by: '<S358>/Integrator'
   */
  0.005,

  /* Expression: 8
   * Referenced by: '<S320>/Saturation1'
   */
  8.0,

  /* Expression: -8
   * Referenced by: '<S320>/Saturation1'
   */
  -8.0,

  /* Expression: 1
   * Referenced by: '<S320>/Constant'
   */
  1.0,

  /* Expression: 3
   * Referenced by: '<S320>/Constant2'
   */
  3.0,

  /* Computed Parameter: Filter_gainval_c2
   * Referenced by: '<S403>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_gd
   * Referenced by: '<S408>/Integrator'
   */
  0.005,

  /* Computed Parameter: Filter_gainval_g
   * Referenced by: '<S451>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_d
   * Referenced by: '<S456>/Integrator'
   */
  0.005,

  /* Expression: 1
   * Referenced by: '<S321>/Constant2'
   */
  1.0,

  /* Expression: .25
   * Referenced by: '<S324>/Gain'
   */
  0.25,

  /* Computed Parameter: Filter_gainval_ew
   * Referenced by: '<S598>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_e
   * Referenced by: '<S603>/Integrator'
   */
  0.005,

  /* Expression: 4
   * Referenced by: '<S324>/Constant2'
   */
  4.0,

  /* Computed Parameter: Filter_gainval_d
   * Referenced by: '<S654>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_p
   * Referenced by: '<S659>/Integrator'
   */
  0.005,

  /* Computed Parameter: Filter_gainval_l
   * Referenced by: '<S702>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_o
   * Referenced by: '<S707>/Integrator'
   */
  0.005,

  /* Expression: 1
   * Referenced by: '<S620>/Constant2'
   */
  1.0,

  /* Computed Parameter: Filter_gainval_m
   * Referenced by: '<S752>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_eb
   * Referenced by: '<S757>/Integrator'
   */
  0.005,

  /* Computed Parameter: Filter_gainval_gw
   * Referenced by: '<S800>/Filter'
   */
  0.005,

  /* Computed Parameter: Integrator_gainval_oy
   * Referenced by: '<S805>/Integrator'
   */
  0.005,

  /* Expression: 0
   * Referenced by: '<S621>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S822>/Constant2'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S822>/Constant8'
   */
  1.0,

  /* Expression: 3
   * Referenced by: '<S623>/Constant2'
   */
  3.0,

  /* Expression: 0
   * Referenced by: '<Root>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Constant3'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Constant1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Constant4'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Constant5'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Constant6'
   */
  0.0,

  /* Computed Parameter: CANSetup_P1_Size
   * Referenced by: '<Root>/CAN Setup '
   */
  { 1.0, 40.0 },

  /* Expression: [moduleInitValues, chn1, ArbitrationManbdrChn1, FdManbdrChn1, chn2, ArbitrationManbdrChn2, FdManbdrChn2, chn3, ArbitrationManbdrChn3, FdManbdrChn3, chn4, ArbitrationManbdrChn4, FdManbdrChn4]
   * Referenced by: '<Root>/CAN Setup '
   */
  { 691.0, 1.0, -1.0, -1.0, 2.0, 2.0, 8.0, 31.0, 8.0, 2.0, 2.0, 5.0, 2.0, 2.0,
    2.0, 8.0, 31.0, 8.0, 2.0, 2.0, 5.0, 2.0, 1.0, 2.0, 8.0, 31.0, 8.0, 2.0, 2.0,
    5.0, 2.0, 1.0, 2.0, 8.0, 31.0, 8.0, 2.0, 2.0, 5.0, 2.0 },

  /* Computed Parameter: CANSetup_P2_Size
   * Referenced by: '<Root>/CAN Setup '
   */
  { 1.0, 1.0 },

  /* Expression: initStruct
   * Referenced by: '<Root>/CAN Setup '
   */
  0.0,

  /* Computed Parameter: CANSetup_P3_Size
   * Referenced by: '<Root>/CAN Setup '
   */
  { 1.0, 1.0 },

  /* Expression: termStruct
   * Referenced by: '<Root>/CAN Setup '
   */
  0.0,

  /* Computed Parameter: TSamp_WtEt
   * Referenced by: '<S4>/TSamp'
   */
  200.0,

  /* Computed Parameter: TSamp_WtEt_n
   * Referenced by: '<S3>/TSamp'
   */
  200.0,

  /* Expression: 4
   * Referenced by: '<Root>/isReady'
   */
  4.0,

  /* Computed Parameter: DiscreteTimeIntegrator_gainval_f
   * Referenced by: '<Root>/Discrete-Time Integrator'
   */
  0.005,

  /* Expression: 0
   * Referenced by: '<Root>/Discrete-Time Integrator'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Switch'
   */
  1.0,

  /* Expression: [0 0]
   * Referenced by: '<Root>/Delay1'
   */
  { 0.0, 0.0 },

  /* Expression: -1
   * Referenced by: '<Root>/Gain10'
   */
  -1.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain8'
   */
  -1.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain6'
   */
  -1.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain1'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Constant2'
   */
  1.0,

  /* Computed Parameter: CANStatus_P1_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: moduleId
   * Referenced by: '<Root>/CAN Status'
   */
  1.0,

  /* Computed Parameter: CANStatus_P2_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: sampleTime
   * Referenced by: '<Root>/CAN Status'
   */
  -1.0,

  /* Computed Parameter: CANStatus_P3_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: channel
   * Referenced by: '<Root>/CAN Status'
   */
  1.0,

  /* Computed Parameter: CANStatus_P4_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: busRecovery
   * Referenced by: '<Root>/CAN Status'
   */
  1.0,

  /* Computed Parameter: CANStatus_P5_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: avgBusLoad
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P6_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: opMode
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P7_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: brp
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P8_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: timeSegment1
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P9_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: timeSegment2
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P10_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: synchronisationJumpWidth
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P11_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: dataTSEG1
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P12_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: dataTSEG2
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P13_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: dataSJW
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P14_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: txPending
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P15_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: dataOverrunTx
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P16_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: receiving
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P17_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: RxQueueEmpty
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P18_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: dataOverrunRcv
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P19_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: errWarnLimit
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P20_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: errPassLimit
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P21_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: errBusOff
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P22_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: busRecoveryCounter
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P23_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: initModeAct
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P24_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: busCouplingErr
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P25_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: transceiverErr
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P26_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: controllerCpuLoad
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P27_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: controllerLiveCount
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P28_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: rxBufferLevel
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P29_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: txBufferLevel
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P30_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: arrayOutput
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P31_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: moduleType
   * Referenced by: '<Root>/CAN Status'
   */
  691.0,

  /* Computed Parameter: CANStatus_P32_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: qtyStatBlk
   * Referenced by: '<Root>/CAN Status'
   */
  1.0,

  /* Computed Parameter: CANStatus_P33_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: ptIdx
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Computed Parameter: CANStatus_P34_Size
   * Referenced by: '<Root>/CAN Status'
   */
  { 1.0, 1.0 },

  /* Expression: isFDMod
   * Referenced by: '<Root>/CAN Status'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory12'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory13'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory14'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory15'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory16'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory2'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory3'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory4'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory5'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory6'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory7'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Data Store Memory8'
   */
  0.0,

  /* Expression: true
   * Referenced by: '<Root>/Delay2'
   */
  true,

  /* Computed Parameter: DataStoreMemory_InitialValue
   * Referenced by: '<Root>/Data Store Memory'
   */
  false,

  /* Expression: uint8(1)
   * Referenced by: '<S5>/Constant'
   */
  1U,

  /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
   * Referenced by: '<S6>/Constant'
   */
  { 255U, 255U, 255U, 255U, 255U, 255U, 255U, 254U },

  /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
   * Referenced by: '<S6>/Constant1'
   */
  { 255U, 255U, 255U, 255U, 255U, 255U, 255U, 252U },

  /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
   * Referenced by: '<S6>/Constant2'
   */
  { 255U, 255U, 255U, 255U, 255U, 255U, 255U, 253U },

  /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
   * Referenced by: '<S7>/Constant'
   */
  { 255U, 255U, 255U, 255U, 255U, 255U, 255U, 254U },

  /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
   * Referenced by: '<S7>/Constant1'
   */
  { 255U, 255U, 255U, 255U, 255U, 255U, 255U, 252U },

  /* Expression: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
   * Referenced by: '<S7>/Constant2'
   */
  { 255U, 255U, 255U, 255U, 255U, 255U, 255U, 253U },

  /* Start of '<S18>/Unload' */
  {
    /* Expression: 0
     * Referenced by: '<S624>/Constant'
     */
    0.0,

    /* Expression: 0
     * Referenced by: '<S624>/Constant1'
     */
    0.0,

    /* Expression: 5
     * Referenced by: '<S624>/Constant2'
     */
    5.0
  }
  ,

  /* End of '<S18>/Unload' */

  /* Start of '<S18>/Load' */
  {
    /* Expression: 0
     * Referenced by: '<S622>/Constant'
     */
    0.0,

    /* Expression: 0
     * Referenced by: '<S622>/Constant1'
     */
    0.0,

    /* Expression: 2
     * Referenced by: '<S622>/Constant2'
     */
    2.0
  }
  ,

  /* End of '<S18>/Load' */

  /* Start of '<S17>/Unload' */
  {
    /* Expression: 0
     * Referenced by: '<S325>/Constant'
     */
    0.0,

    /* Expression: 0
     * Referenced by: '<S325>/Constant1'
     */
    0.0,

    /* Expression: 5
     * Referenced by: '<S325>/Constant2'
     */
    5.0
  }
  ,

  /* End of '<S17>/Unload' */

  /* Start of '<S17>/Load' */
  {
    /* Expression: 0
     * Referenced by: '<S323>/Constant'
     */
    0.0,

    /* Expression: 0
     * Referenced by: '<S323>/Constant1'
     */
    0.0,

    /* Expression: 2
     * Referenced by: '<S323>/Constant2'
     */
    2.0
  }
  ,

  /* End of '<S17>/Load' */

  /* Start of '<S17>/Idle' */
  {
    /* Mask Parameter: PIDController_D
     * Referenced by: '<S500>/Derivative Gain'
     */
    5.0,

    /* Mask Parameter: PIDController1_D
     * Referenced by: '<S548>/Derivative Gain'
     */
    5.0,

    /* Mask Parameter: PIDController_I
     * Referenced by: '<S503>/Integral Gain'
     */
    0.0,

    /* Mask Parameter: PIDController1_I
     * Referenced by: '<S551>/Integral Gain'
     */
    0.0,

    /* Mask Parameter: PIDController_InitialConditionForFilter
     * Referenced by: '<S501>/Filter'
     */
    0.0,

    /* Mask Parameter: PIDController1_InitialConditionForFilter
     * Referenced by: '<S549>/Filter'
     */
    0.0,

    /* Mask Parameter: PIDController_InitialConditionForIntegrator
     * Referenced by: '<S506>/Integrator'
     */
    0.0,

    /* Mask Parameter: PIDController1_InitialConditionForIntegrator
     * Referenced by: '<S554>/Integrator'
     */
    0.0,

    /* Mask Parameter: PIDController_N
     * Referenced by: '<S509>/Filter Coefficient'
     */
    100.0,

    /* Mask Parameter: PIDController1_N
     * Referenced by: '<S557>/Filter Coefficient'
     */
    100.0,

    /* Mask Parameter: PIDController_P
     * Referenced by: '<S511>/Proportional Gain'
     */
    50.0,

    /* Mask Parameter: PIDController1_P
     * Referenced by: '<S559>/Proportional Gain'
     */
    50.0,

    /* Computed Parameter: Filter_gainval
     * Referenced by: '<S501>/Filter'
     */
    0.005,

    /* Computed Parameter: Integrator_gainval
     * Referenced by: '<S506>/Integrator'
     */
    0.005,

    /* Computed Parameter: Filter_gainval_o
     * Referenced by: '<S549>/Filter'
     */
    0.005,

    /* Computed Parameter: Integrator_gainval_b
     * Referenced by: '<S554>/Integrator'
     */
    0.005,

    /* Expression: 0
     * Referenced by: '<S322>/Constant'
     */
    0.0
  }
  ,

  /* End of '<S17>/Idle' */

  /* Start of '<S16>/Load' */
  {
    /* Expression: 0
     * Referenced by: '<S22>/Constant'
     */
    0.0,

    /* Expression: 0
     * Referenced by: '<S22>/Constant1'
     */
    0.0,

    /* Expression: 2
     * Referenced by: '<S22>/Constant2'
     */
    2.0
  }
  ,

  /* End of '<S16>/Load' */

  /* Start of '<S16>/Idle' */
  {
    /* Mask Parameter: PIDController_D
     * Referenced by: '<S249>/Derivative Gain'
     */
    10.0,

    /* Mask Parameter: PIDController1_D
     * Referenced by: '<S297>/Derivative Gain'
     */
    5.0,

    /* Mask Parameter: PIDController_I
     * Referenced by: '<S252>/Integral Gain'
     */
    0.0,

    /* Mask Parameter: PIDController1_I
     * Referenced by: '<S300>/Integral Gain'
     */
    0.0,

    /* Mask Parameter: PIDController_InitialConditionForFilter
     * Referenced by: '<S250>/Filter'
     */
    0.0,

    /* Mask Parameter: PIDController1_InitialConditionForFilter
     * Referenced by: '<S298>/Filter'
     */
    0.0,

    /* Mask Parameter: PIDController_InitialConditionForIntegrator
     * Referenced by: '<S255>/Integrator'
     */
    0.0,

    /* Mask Parameter: PIDController1_InitialConditionForIntegrator
     * Referenced by: '<S303>/Integrator'
     */
    0.0,

    /* Mask Parameter: PIDController_N
     * Referenced by: '<S258>/Filter Coefficient'
     */
    100.0,

    /* Mask Parameter: PIDController1_N
     * Referenced by: '<S306>/Filter Coefficient'
     */
    100.0,

    /* Mask Parameter: PIDController_P
     * Referenced by: '<S260>/Proportional Gain'
     */
    50.0,

    /* Mask Parameter: PIDController1_P
     * Referenced by: '<S308>/Proportional Gain'
     */
    50.0,

    /* Computed Parameter: Filter_gainval
     * Referenced by: '<S250>/Filter'
     */
    0.005,

    /* Computed Parameter: Integrator_gainval
     * Referenced by: '<S255>/Integrator'
     */
    0.005,

    /* Computed Parameter: Filter_gainval_o
     * Referenced by: '<S298>/Filter'
     */
    0.005,

    /* Computed Parameter: Integrator_gainval_b
     * Referenced by: '<S303>/Integrator'
     */
    0.005,

    /* Expression: 0
     * Referenced by: '<S21>/Constant'
     */
    0.0
  }
  /* End of '<S16>/Idle' */
};

PHRControl_cal_type *PHRControl_cal = &PHRControl_cal_impl;

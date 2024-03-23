//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: my_sim.h
//
// Code generated for Simulink model 'my_sim'.
//
// Model version                  : 1.3
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Thu Feb  8 20:32:14 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_my_sim_h_
#define RTW_HEADER_my_sim_h_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "dt_info.h"
#include "ext_work.h"
#include "slros_initialize.h"
#include "my_sim_types.h"

extern "C"
{

#include "rtGetNaN.h"

}

#include <float.h>
#include <string.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

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

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
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

// Block signals (default storage)
struct B_my_sim_T {
  real_T D[169];
  real_T b_H[169];
  real_T U[169];
  real_T RLinv[169];
  real_T TL[169];
  real_T R[169];
  real_T b_A[169];
  real_T rseq[80];
  real_T a__1[48];
  real_T dv[48];
  real_T cTol[48];
  SL_Bus_my_sim_mavros_msgs_PositionTarget BusAssignment1;// '<Root>/Bus Assignment1' 
  SL_Bus_my_sim_geometry_msgs_PoseStamped BusAssignment;// '<Root>/Bus Assignment' 
  real_T Opt[26];
  real_T Rhs[26];
  int32_T iC[48];
  real_T zopt[13];
  real_T f[13];
  real_T r[13];
  real_T z[13];
  real_T tau[13];
  real_T work[13];
  real_T measuredoutput[4];            // '<Root>/State-Space'
  real_T xk1[4];                       // '<S26>/optimizer'
  real_T u[4];                         // '<S26>/optimizer'
  boolean_T iAout[48];                 // '<S26>/optimizer'
  real_T y_innov[4];
  real_T old_u[4];
  real_T TmpSignalConversionAtSFunct[4];// '<S26>/optimizer'
  real_T xest[4];
  real_T x_ref;                        // '<Root>/Sine Wave'
  real_T y_ref;                        // '<Root>/Constant1'
  real_T z_ref;                        // '<Root>/Constant2'
  real_T yaw_ref;                      // '<Root>/Constant3'
  real_T umin_scale1[4];               // '<S6>/umin_scale1'
  real_T old_u_m;
  real_T old_u_c;
  real_T old_u_k;
  real_T old_u_cx;
  real_T rtb_TmpSignalConversionAtSFun_b;
  real_T rtb_TmpSignalConversionAtSFun_p;
  real_T rtb_TmpSignalConversionAtSFun_c;
  real_T rtb_TmpSignalConversionAtSFun_f;
  real_T rMin;
  real_T Xnorm0;
  real_T cMin;
  real_T cVal;
  real_T zTa;
  real_T atmp;
  real_T beta1;
  real_T b_A_g;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T temp;
  real_T c;
  real32_T yaw_pos;                    // '<Root>/Data Type Conversion'
  int32_T i;
  int32_T rseq_tmp;
  int32_T nA;
  int32_T tmp;
  int32_T kDrop;
  uint32_T ri;
};

// Block states (default storage) for system '<Root>'
struct DW_my_sim_T {
  ros_slroscpp_internal_block_P_T obj; // '<S5>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_e;// '<S4>/SinkBlock'
  real_T last_mv_DSTATE[4];            // '<S6>/last_mv'
  real_T last_x_PreviousInput[4];      // '<S6>/last_x'
  struct {
    void *LoggedData[4];
  } Velocities_PWORK;                  // '<Root>/Velocities'

  struct {
    void *LoggedData[8];
  } Positions_PWORK;                   // '<Root>/Positions'

  uint8_T is_active_c3_mpclib;         // '<S26>/optimizer'
  boolean_T Memory_PreviousInput[48];  // '<S6>/Memory'
};

// Continuous states (default storage)
struct X_my_sim_T {
  real_T position[4];                  // '<Root>/State-Space'
};

// State derivatives (default storage)
struct XDot_my_sim_T {
  real_T position[4];                  // '<Root>/State-Space'
};

// State disabled
struct XDis_my_sim_T {
  boolean_T position[4];               // '<Root>/State-Space'
};

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
struct ODE3_IntgData {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
};

#endif

// Parameters (default storage)
struct P_my_sim_T_ {
  SL_Bus_my_sim_mavros_msgs_PositionTarget Constant_Value;// Computed Parameter: Constant_Value
                                                             //  Referenced by: '<S2>/Constant'

  SL_Bus_my_sim_geometry_msgs_PoseStamped Constant_Value_h;// Computed Parameter: Constant_Value_h
                                                              //  Referenced by: '<S1>/Constant'

  real_T StateSpace_B_pr[4];           // Computed Parameter: StateSpace_B_pr
                                          //  Referenced by: '<Root>/State-Space'

  real_T StateSpace_C_pr[4];           // Computed Parameter: StateSpace_C_pr
                                          //  Referenced by: '<Root>/State-Space'

  real_T StateSpace_InitialCondition[4];// Expression: [0, 0, 2, 1.57]
                                           //  Referenced by: '<Root>/State-Space'

  real_T SineWave_Amp;                 // Expression: 1.2
                                          //  Referenced by: '<Root>/Sine Wave'

  real_T SineWave_Bias;                // Expression: 0
                                          //  Referenced by: '<Root>/Sine Wave'

  real_T SineWave_Freq;                // Expression: 1
                                          //  Referenced by: '<Root>/Sine Wave'

  real_T SineWave_Phase;               // Expression: 0
                                          //  Referenced by: '<Root>/Sine Wave'

  real_T Constant1_Value;              // Expression: 0
                                          //  Referenced by: '<Root>/Constant1'

  real_T Constant2_Value;              // Expression: 2
                                          //  Referenced by: '<Root>/Constant2'

  real_T Constant3_Value;              // Expression: 1.57
                                          //  Referenced by: '<Root>/Constant3'

  real_T last_x_InitialCondition[4];   // Expression: lastx+xoff
                                          //  Referenced by: '<S6>/last_x'

  real_T last_mv_InitialCondition[4];  // Expression: lastu+uoff
                                          //  Referenced by: '<S6>/last_mv'

  real_T md_zero_Value;                // Expression: zeros(1,1)
                                          //  Referenced by: '<S3>/md_zero'

  real_T umin_zero_Value[4];           // Expression: zeros(4,1)
                                          //  Referenced by: '<S3>/umin_zero'

  real_T umax_zero_Value[4];           // Expression: zeros(4,1)
                                          //  Referenced by: '<S3>/umax_zero'

  real_T ymin_zero_Value[4];           // Expression: zeros(4,1)
                                          //  Referenced by: '<S3>/ymin_zero'

  real_T ymax_zero_Value[4];           // Expression: zeros(4,1)
                                          //  Referenced by: '<S3>/ymax_zero'

  real_T E_zero_Value[4];              // Expression: zeros(1,4)
                                          //  Referenced by: '<S3>/E_zero'

  real_T umin_scale4_Gain[4];      // Expression: MVscale(:,ones(1,max(nCC,1)))'
                                      //  Referenced by: '<S6>/umin_scale4'

  real_T F_zero_Value[4];              // Expression: zeros(1,4)
                                          //  Referenced by: '<S3>/F_zero'

  real_T ymin_scale1_Gain[4];       // Expression: Yscale(:,ones(1,max(nCC,1)))'
                                       //  Referenced by: '<S6>/ymin_scale1'

  real_T G_zero_Value;                 // Expression: zeros(1,1)
                                          //  Referenced by: '<S3>/G_zero'

  real_T S_zero_Value;                 // Expression: zeros(1,1)
                                          //  Referenced by: '<S3>/S_zero'

  real_T ymin_scale2_Gain;         // Expression: MDscale(:,ones(1,max(nCC,1)))'
                                      //  Referenced by: '<S6>/ymin_scale2'

  real_T switch_zero_Value;            // Expression: zeros(1,1)
                                          //  Referenced by: '<S3>/switch_zero'

  real_T extmv_zero_Value[4];          // Expression: zeros(4,1)
                                          //  Referenced by: '<S3>/ext.mv_zero'

  real_T extmv_scale_Gain[4];          // Expression: RMVscale
                                          //  Referenced by: '<S6>/ext.mv_scale'

  real_T mvtarget_zero_Value[4];       // Expression: zeros(4,1)
                                          //  Referenced by: '<S3>/mv.target_zero'

  real_T extmv_scale1_Gain[4];         // Expression: RMVscale
                                          //  Referenced by: '<S6>/ext.mv_scale1'

  real_T ywt_zero_Value[4];            // Expression: zeros(4,1)
                                          //  Referenced by: '<S3>/y.wt_zero'

  real_T uwt_zero_Value[4];            // Expression: zeros(4,1)
                                          //  Referenced by: '<S3>/u.wt_zero'

  real_T duwt_zero_Value[4];           // Expression: zeros(4,1)
                                          //  Referenced by: '<S3>/du.wt_zero'

  real_T ecrwt_zero_Value;             // Expression: zeros(1,1)
                                          //  Referenced by: '<S3>/ecr.wt_zero'

  real_T umin_scale1_Gain[4];          // Expression: MVscale
                                          //  Referenced by: '<S6>/umin_scale1'

  real_T SimulationPace_P1;            // Expression: SimulationPace
                                          //  Referenced by: '<Root>/Simulation Pace'

  real_T SimulationPace_P2;            // Expression: 2
                                          //  Referenced by: '<Root>/Simulation Pace'

  real_T SimulationPace_P3;            // Expression: OutputPaceError
                                          //  Referenced by: '<Root>/Simulation Pace'

  real_T SimulationPace_P4;            // Expression: SampleTime
                                          //  Referenced by: '<Root>/Simulation Pace'

  uint32_T StateSpace_B_ir[4];         // Computed Parameter: StateSpace_B_ir
                                          //  Referenced by: '<Root>/State-Space'

  uint32_T StateSpace_B_jc[5];         // Computed Parameter: StateSpace_B_jc
                                          //  Referenced by: '<Root>/State-Space'

  uint32_T StateSpace_C_ir[4];         // Computed Parameter: StateSpace_C_ir
                                          //  Referenced by: '<Root>/State-Space'

  uint32_T StateSpace_C_jc[5];         // Computed Parameter: StateSpace_C_jc
                                          //  Referenced by: '<Root>/State-Space'

  uint16_T Constant4_Value;            // Computed Parameter: Constant4_Value
                                          //  Referenced by: '<Root>/Constant4'

  boolean_T Memory_InitialCondition[48];// Expression: iA
                                           //  Referenced by: '<S6>/Memory'

  uint8_T Constant_Value_d;            // Computed Parameter: Constant_Value_d
                                          //  Referenced by: '<Root>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_my_sim_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;
  RTWSolverInfo solverInfo;
  X_my_sim_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_my_sim_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[4];
  real_T odeF[3][4];
  ODE3_IntgData intgData;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    uint32_T checksums[4];
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  //
  //  SpecialInfo:
  //  The following substructure contains special information
  //  related to other components that are dependent on RTW.

  struct {
    const void *mappingInfo;
  } SpecialInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTick2;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    time_T tStart;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern P_my_sim_T my_sim_P;

#ifdef __cplusplus

}

#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_my_sim_T my_sim_B;

#ifdef __cplusplus

}

#endif

// Continuous states (default storage)
extern X_my_sim_T my_sim_X;

// Disabled states (default storage)
extern XDis_my_sim_T my_sim_XDis;

// Block states (default storage)
extern struct DW_my_sim_T my_sim_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  // Model entry point functions
  extern void my_sim_initialize(void);
  extern void my_sim_step(void);
  extern void my_sim_terminate(void);

#ifdef __cplusplus

}

#endif

// Real-time Model object
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_my_sim_T *const my_sim_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S6>/Constant' : Unused code path elimination
//  Block '<S6>/Floor' : Unused code path elimination
//  Block '<S6>/Floor1' : Unused code path elimination
//  Block '<S7>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S8>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S9>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S10>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S11>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S12>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S13>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S14>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S15>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S16>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S17>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S18>/Matrix Dimension Check' : Unused code path elimination
//  Block '<S19>/Vector Dimension Check' : Unused code path elimination
//  Block '<S20>/Vector Dimension Check' : Unused code path elimination
//  Block '<S21>/Vector Dimension Check' : Unused code path elimination
//  Block '<S22>/Vector Dimension Check' : Unused code path elimination
//  Block '<S23>/Vector Dimension Check' : Unused code path elimination
//  Block '<S24>/Vector Dimension Check' : Unused code path elimination
//  Block '<S6>/Min' : Unused code path elimination
//  Block '<S6>/constant' : Unused code path elimination
//  Block '<S25>/Vector Dimension Check' : Unused code path elimination
//  Block '<S6>/umin_scale2' : Unused code path elimination
//  Block '<S6>/umin_scale3' : Unused code path elimination
//  Block '<S6>/umin_scale5' : Unused code path elimination
//  Block '<S6>/ym_zero' : Unused code path elimination
//  Block '<S3>/m_zero' : Unused code path elimination
//  Block '<S3>/p_zero' : Unused code path elimination
//  Block '<S6>/Reshape' : Reshape block reduction
//  Block '<S6>/Reshape1' : Reshape block reduction
//  Block '<S6>/Reshape2' : Reshape block reduction
//  Block '<S6>/Reshape3' : Reshape block reduction
//  Block '<S6>/Reshape4' : Reshape block reduction
//  Block '<S6>/Reshape5' : Reshape block reduction


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'my_sim'
//  '<S1>'   : 'my_sim/Blank Message'
//  '<S2>'   : 'my_sim/Blank Message1'
//  '<S3>'   : 'my_sim/MPC Controller'
//  '<S4>'   : 'my_sim/Publish'
//  '<S5>'   : 'my_sim/Publish1'
//  '<S6>'   : 'my_sim/MPC Controller/MPC'
//  '<S7>'   : 'my_sim/MPC Controller/MPC/MPC Matrix Signal Check'
//  '<S8>'   : 'my_sim/MPC Controller/MPC/MPC Matrix Signal Check1'
//  '<S9>'   : 'my_sim/MPC Controller/MPC/MPC Matrix Signal Check2'
//  '<S10>'  : 'my_sim/MPC Controller/MPC/MPC Preview Signal Check'
//  '<S11>'  : 'my_sim/MPC Controller/MPC/MPC Preview Signal Check1'
//  '<S12>'  : 'my_sim/MPC Controller/MPC/MPC Preview Signal Check2'
//  '<S13>'  : 'my_sim/MPC Controller/MPC/MPC Preview Signal Check3'
//  '<S14>'  : 'my_sim/MPC Controller/MPC/MPC Preview Signal Check4'
//  '<S15>'  : 'my_sim/MPC Controller/MPC/MPC Preview Signal Check5'
//  '<S16>'  : 'my_sim/MPC Controller/MPC/MPC Preview Signal Check6'
//  '<S17>'  : 'my_sim/MPC Controller/MPC/MPC Preview Signal Check7'
//  '<S18>'  : 'my_sim/MPC Controller/MPC/MPC Preview Signal Check8'
//  '<S19>'  : 'my_sim/MPC Controller/MPC/MPC Scalar Signal Check'
//  '<S20>'  : 'my_sim/MPC Controller/MPC/MPC Scalar Signal Check1'
//  '<S21>'  : 'my_sim/MPC Controller/MPC/MPC Scalar Signal Check2'
//  '<S22>'  : 'my_sim/MPC Controller/MPC/MPC Vector Signal Check'
//  '<S23>'  : 'my_sim/MPC Controller/MPC/MPC Vector Signal Check1'
//  '<S24>'  : 'my_sim/MPC Controller/MPC/MPC Vector Signal Check6'
//  '<S25>'  : 'my_sim/MPC Controller/MPC/moorx'
//  '<S26>'  : 'my_sim/MPC Controller/MPC/optimizer'
//  '<S27>'  : 'my_sim/MPC Controller/MPC/optimizer/optimizer'

#endif                                 // RTW_HEADER_my_sim_h_

//
// File trailer for generated code.
//
// [EOF]
//

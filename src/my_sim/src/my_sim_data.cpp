//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: my_sim_data.cpp
//
// Code generated for Simulink model 'my_sim'.
//
// Model version                  : 1.6
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Thu Feb  8 23:42:43 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "my_sim.h"

// Block parameters (default storage)
P_my_sim_T my_sim_P = {
  // Computed Parameter: Constant_Value
  //  Referenced by: '<S1>/Constant'

  {
    {
      0U,                              // Seq

      {
        0.0,                           // Sec
        0.0                            // Nsec
      },                               // Stamp

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // FrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      }                                // FrameId_SL_Info
    },                                 // Header
    0U,                                // CoordinateFrame
    0U,                                // TypeMask

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // Position

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // Velocity

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // AccelerationOrForce
    0.0F,                              // Yaw
    0.0F                               // YawRate
  },

  // Computed Parameter: StateSpace_B_pr
  //  Referenced by: '<Root>/State-Space'

  { 1.0, 1.0, 1.0, 1.0 },

  // Computed Parameter: StateSpace_C_pr
  //  Referenced by: '<Root>/State-Space'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: [0, 0, 2, 1.57]
  //  Referenced by: '<Root>/State-Space'

  { 0.0, 0.0, 2.0, 1.57 },

  // Expression: 1.2
  //  Referenced by: '<Root>/Sine Wave'

  1.2,

  // Expression: 0
  //  Referenced by: '<Root>/Sine Wave'

  0.0,

  // Expression: 1
  //  Referenced by: '<Root>/Sine Wave'

  1.0,

  // Expression: 0
  //  Referenced by: '<Root>/Sine Wave'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Constant1'

  0.0,

  // Expression: 2
  //  Referenced by: '<Root>/Constant2'

  2.0,

  // Expression: 1.57
  //  Referenced by: '<Root>/Constant3'

  1.57,

  // Expression: lastx+xoff
  //  Referenced by: '<S4>/last_x'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: lastu+uoff
  //  Referenced by: '<S4>/last_mv'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(1,1)
  //  Referenced by: '<S2>/md_zero'

  0.0,

  // Expression: zeros(4,1)
  //  Referenced by: '<S2>/umin_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S2>/umax_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S2>/ymin_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S2>/ymax_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(1,4)
  //  Referenced by: '<S2>/E_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: MVscale(:,ones(1,max(nCC,1)))'
  //  Referenced by: '<S4>/umin_scale4'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: zeros(1,4)
  //  Referenced by: '<S2>/F_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: Yscale(:,ones(1,max(nCC,1)))'
  //  Referenced by: '<S4>/ymin_scale1'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: zeros(1,1)
  //  Referenced by: '<S2>/G_zero'

  0.0,

  // Expression: zeros(1,1)
  //  Referenced by: '<S2>/S_zero'

  0.0,

  // Expression: MDscale(:,ones(1,max(nCC,1)))'
  //  Referenced by: '<S4>/ymin_scale2'

  1.0,

  // Expression: zeros(1,1)
  //  Referenced by: '<S2>/switch_zero'

  0.0,

  // Expression: zeros(4,1)
  //  Referenced by: '<S2>/ext.mv_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: RMVscale
  //  Referenced by: '<S4>/ext.mv_scale'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S2>/mv.target_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: RMVscale
  //  Referenced by: '<S4>/ext.mv_scale1'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S2>/y.wt_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S2>/u.wt_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S2>/du.wt_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(1,1)
  //  Referenced by: '<S2>/ecr.wt_zero'

  0.0,

  // Expression: MVscale
  //  Referenced by: '<S4>/umin_scale1'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: SimulationPace
  //  Referenced by: '<Root>/Simulation Pace'

  1.0,

  // Expression: 2
  //  Referenced by: '<Root>/Simulation Pace'

  2.0,

  // Expression: OutputPaceError
  //  Referenced by: '<Root>/Simulation Pace'

  0.0,

  // Expression: SampleTime
  //  Referenced by: '<Root>/Simulation Pace'

  0.033333333333333333,

  // Computed Parameter: StateSpace_B_ir
  //  Referenced by: '<Root>/State-Space'

  { 0U, 1U, 2U, 3U },

  // Computed Parameter: StateSpace_B_jc
  //  Referenced by: '<Root>/State-Space'

  { 0U, 1U, 2U, 3U, 4U },

  // Computed Parameter: StateSpace_C_ir
  //  Referenced by: '<Root>/State-Space'

  { 0U, 1U, 2U, 3U },

  // Computed Parameter: StateSpace_C_jc
  //  Referenced by: '<Root>/State-Space'

  { 0U, 1U, 2U, 3U, 4U },

  // Computed Parameter: Constant4_Value
  //  Referenced by: '<Root>/Constant4'

  1987U,

  // Expression: iA
  //  Referenced by: '<S4>/Memory'

  { false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false },

  // Computed Parameter: Constant_Value_d
  //  Referenced by: '<Root>/Constant'

  8U
};

//
// File trailer for generated code.
//
// [EOF]
//

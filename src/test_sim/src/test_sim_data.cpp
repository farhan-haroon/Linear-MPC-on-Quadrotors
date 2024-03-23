//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: test_sim_data.cpp
//
// Code generated for Simulink model 'test_sim'.
//
// Model version                  : 1.11
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Tue Feb  6 19:41:08 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "test_sim.h"

// Block parameters (default storage)
P_test_sim_T test_sim_P = {
  // Computed Parameter: Constant_Value
  //  Referenced by: '<S5>/Constant'

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

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S30>/Out1'

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

    {
      {
        0.0,                           // X
        0.0,                           // Y
        0.0                            // Z
      },                               // Position

      {
        0.0,                           // X
        0.0,                           // Y
        0.0,                           // Z
        0.0                            // W
      }                                // Orientation
    }                                  // Pose
  },

  // Computed Parameter: Constant_Value_f
  //  Referenced by: '<S4>/Constant'

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

    {
      {
        0.0,                           // X
        0.0,                           // Y
        0.0                            // Z
      },                               // Position

      {
        0.0,                           // X
        0.0,                           // Y
        0.0,                           // Z
        0.0                            // W
      }                                // Orientation
    }                                  // Pose
  },

  // Computed Parameter: mv_vel_x_Y0
  //  Referenced by: '<S3>/mv_vel_x'

  0.0,

  // Computed Parameter: mv_vel_y_Y0
  //  Referenced by: '<S3>/mv_vel_y'

  0.0,

  // Computed Parameter: mv_vel_z_Y0
  //  Referenced by: '<S3>/mv_vel_z'

  0.0,

  // Computed Parameter: mv_vel_yaw_Y0
  //  Referenced by: '<S3>/mv_vel_yaw'

  0.0,

  // Expression: zeros(1,1)
  //  Referenced by: '<S7>/G_zero'

  0.0,

  // Expression: zeros(4,1)
  //  Referenced by: '<S7>/y.wt_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S7>/u.wt_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S7>/du.wt_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S7>/ext.mv_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: RMVscale
  //  Referenced by: '<S8>/ext.mv_scale'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S7>/mv.target_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: RMVscale
  //  Referenced by: '<S8>/ext.mv_scale1'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: lastu+uoff
  //  Referenced by: '<S8>/last_mv'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: lastx+xoff
  //  Referenced by: '<S8>/last_x'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(1,1)
  //  Referenced by: '<S7>/md_zero'

  0.0,

  // Expression: zeros(4,1)
  //  Referenced by: '<S7>/umin_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S7>/umax_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S7>/ymin_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(4,1)
  //  Referenced by: '<S7>/ymax_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(1,4)
  //  Referenced by: '<S7>/E_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: MVscale(:,ones(1,max(nCC,1)))'
  //  Referenced by: '<S8>/umin_scale4'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: zeros(1,4)
  //  Referenced by: '<S7>/F_zero'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: Yscale(:,ones(1,max(nCC,1)))'
  //  Referenced by: '<S8>/ymin_scale1'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: zeros(1,1)
  //  Referenced by: '<S7>/S_zero'

  0.0,

  // Expression: MDscale(:,ones(1,max(nCC,1)))'
  //  Referenced by: '<S8>/ymin_scale2'

  1.0,

  // Expression: zeros(1,1)
  //  Referenced by: '<S7>/switch_zero'

  0.0,

  // Expression: zeros(1,1)
  //  Referenced by: '<S7>/ecr.wt_zero'

  0.0,

  // Expression: MVscale
  //  Referenced by: '<S8>/umin_scale1'

  { 1.0, 1.0, 1.0, 1.0 },

  // Computed Parameter: PlantStateSpace_B_pr
  //  Referenced by: '<S3>/Plant (State-Space)'

  { 1.0, 1.0, 1.0, 1.0 },

  // Computed Parameter: PlantStateSpace_C_pr
  //  Referenced by: '<S3>/Plant (State-Space)'

  { 1.0, 1.0, 1.0, 1.0 },

  // Expression: [0]
  //  Referenced by: '<S3>/Plant (State-Space)'

  0.0,

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

  // Computed Parameter: PlantStateSpace_B_ir
  //  Referenced by: '<S3>/Plant (State-Space)'

  { 0U, 1U, 2U, 3U },

  // Computed Parameter: PlantStateSpace_B_jc
  //  Referenced by: '<S3>/Plant (State-Space)'

  { 0U, 1U, 2U, 3U, 4U },

  // Computed Parameter: PlantStateSpace_C_ir
  //  Referenced by: '<S3>/Plant (State-Space)'

  { 0U, 1U, 2U, 3U },

  // Computed Parameter: PlantStateSpace_C_jc
  //  Referenced by: '<S3>/Plant (State-Space)'

  { 0U, 1U, 2U, 3U, 4U },

  // Computed Parameter: TypeMask_Value
  //  Referenced by: '<S1>/Type Mask'

  1987U,

  // Expression: iA
  //  Referenced by: '<S8>/Memory'

  { false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false, false,
    false, false, false, false },

  // Computed Parameter: CoordinateFrame_Value
  //  Referenced by: '<S1>/Coordinate Frame'

  8U
};

//
// File trailer for generated code.
//
// [EOF]
//

//
//  my_sim_dt.h
//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  Code generation for model "my_sim".
//
//  Model version              : 1.6
//  Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
//  C++ source code generated on : Thu Feb  8 23:42:43 2024
//
//  Target selection: ert.tlc
//  Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
//  Code generation objectives: Unspecified
//  Validation result: Not run


#include "ext_types.h"

// data type size table
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(int32_T),
  sizeof(SL_Bus_my_sim_ros_time_Time),
  sizeof(SL_Bus_ROSVariableLengthArrayInfo),
  sizeof(SL_Bus_my_sim_std_msgs_Header),
  sizeof(SL_Bus_my_sim_geometry_msgs_Point),
  sizeof(SL_Bus_my_sim_geometry_msgs_Vector3),
  sizeof(SL_Bus_my_sim_mavros_msgs_PositionTarget),
  sizeof(int32_T),
  sizeof(int32_T),
  sizeof(int32_T),
  sizeof(ros_slroscpp_internal_block_P_T),
  sizeof(uint_T),
  sizeof(char_T),
  sizeof(uchar_T),
  sizeof(time_T)
};

// data type name table
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "physical_connection",
  "SL_Bus_my_sim_ros_time_Time",
  "SL_Bus_ROSVariableLengthArrayInfo",
  "SL_Bus_my_sim_std_msgs_Header",
  "SL_Bus_my_sim_geometry_msgs_Point",
  "SL_Bus_my_sim_geometry_msgs_Vector3",
  "SL_Bus_my_sim_mavros_msgs_PositionTarget",
  "struct_WTmPWsEMvOzNnnAVv5fQNC",
  "struct_WHjMt45Sk148iktWsfFxl",
  "struct_lnQ9KXdSZFplhcBp5LBCc",
  "ros_slroscpp_internal_block_P_T",
  "uint_T",
  "char_T",
  "uchar_T",
  "time_T"
};

// data type transitions for block I/O structure
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&my_sim_B.measuredoutput[0]), 0, 0, 8 },

  { (char_T *)(&my_sim_B.x_ref), 0, 0, 12 },

  { (char_T *)(&my_sim_B.yaw_pos), 1, 0, 1 },

  { (char_T *)(&my_sim_B.iAout[0]), 8, 0, 48 }
  ,

  { (char_T *)(&my_sim_DW.obj), 24, 0, 1 },

  { (char_T *)(&my_sim_DW.last_mv_DSTATE[0]), 0, 0, 8 },

  { (char_T *)(&my_sim_DW.Velocities_PWORK.LoggedData[0]), 11, 0, 12 },

  { (char_T *)(&my_sim_DW.is_active_c3_mpclib), 3, 0, 1 },

  { (char_T *)(&my_sim_DW.Memory_PreviousInput[0]), 8, 0, 48 }
};

// data type transition table for block I/O structure
static DataTypeTransitionTable rtBTransTable = {
  9U,
  rtBTransitions
};

// data type transitions for Parameters structure
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&my_sim_P.Constant_Value), 20, 0, 1 },

  { (char_T *)(&my_sim_P.StateSpace_B_pr[0]), 0, 0, 101 },

  { (char_T *)(&my_sim_P.StateSpace_B_ir[0]), 7, 0, 18 },

  { (char_T *)(&my_sim_P.Constant4_Value), 5, 0, 1 },

  { (char_T *)(&my_sim_P.Memory_InitialCondition[0]), 8, 0, 48 },

  { (char_T *)(&my_sim_P.Constant_Value_d), 3, 0, 1 }
};

// data type transition table for Parameters structure
static DataTypeTransitionTable rtPTransTable = {
  6U,
  rtPTransitions
};

// [EOF] my_sim_dt.h

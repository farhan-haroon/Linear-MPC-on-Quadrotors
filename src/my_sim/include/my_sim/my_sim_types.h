//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: my_sim_types.h
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
#ifndef RTW_HEADER_my_sim_types_h_
#define RTW_HEADER_my_sim_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_my_sim_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_my_sim_ros_time_Time_

// MsgType=ros_time/Time
struct SL_Bus_my_sim_ros_time_Time
{
  real_T Sec;
  real_T Nsec;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_my_sim_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_my_sim_std_msgs_Header_

// MsgType=std_msgs/Header
struct SL_Bus_my_sim_std_msgs_Header
{
  uint32_T Seq;

  // MsgType=ros_time/Time
  SL_Bus_my_sim_ros_time_Time Stamp;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_my_sim_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_my_sim_geometry_msgs_Point_

// MsgType=geometry_msgs/Point
struct SL_Bus_my_sim_geometry_msgs_Point
{
  real_T X;
  real_T Y;
  real_T Z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_my_sim_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_my_sim_geometry_msgs_Vector3_

// MsgType=geometry_msgs/Vector3
struct SL_Bus_my_sim_geometry_msgs_Vector3
{
  real_T X;
  real_T Y;
  real_T Z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_my_sim_mavros_msgs_PositionTarget_
#define DEFINED_TYPEDEF_FOR_SL_Bus_my_sim_mavros_msgs_PositionTarget_

// MsgType=mavros_msgs/PositionTarget
struct SL_Bus_my_sim_mavros_msgs_PositionTarget
{
  // MsgType=std_msgs/Header
  SL_Bus_my_sim_std_msgs_Header Header;
  uint8_T CoordinateFrame;
  uint16_T TypeMask;

  // MsgType=geometry_msgs/Point
  SL_Bus_my_sim_geometry_msgs_Point Position;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_my_sim_geometry_msgs_Vector3 Velocity;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_my_sim_geometry_msgs_Vector3 AccelerationOrForce;
  real32_T Yaw;
  real32_T YawRate;
};

#endif

#ifndef struct_ros_slroscpp_internal_block_P_T
#define struct_ros_slroscpp_internal_block_P_T

struct ros_slroscpp_internal_block_P_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                // struct_ros_slroscpp_internal_block_P_T

// Parameters (default storage)
typedef struct P_my_sim_T_ P_my_sim_T;

// Forward declaration for rtModel
typedef struct tag_RTM_my_sim_T RT_MODEL_my_sim_T;

#endif                                 // RTW_HEADER_my_sim_types_h_

//
// File trailer for generated code.
//
// [EOF]
//

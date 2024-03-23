#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_my_sim_geometry_msgs_Point and geometry_msgs::Point

void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_my_sim_geometry_msgs_Point const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_my_sim_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_my_sim_geometry_msgs_Vector3 and geometry_msgs::Vector3

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_my_sim_geometry_msgs_Vector3 const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_my_sim_geometry_msgs_Vector3* busPtr, geometry_msgs::Vector3 const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_my_sim_mavros_msgs_PositionTarget and mavros_msgs::PositionTarget

void convertFromBus(mavros_msgs::PositionTarget* msgPtr, SL_Bus_my_sim_mavros_msgs_PositionTarget const* busPtr)
{
  const std::string rosMessageType("mavros_msgs/PositionTarget");

  convertFromBus(&msgPtr->acceleration_or_force, &busPtr->AccelerationOrForce);
  msgPtr->coordinate_frame =  busPtr->CoordinateFrame;
  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBus(&msgPtr->position, &busPtr->Position);
  msgPtr->type_mask =  busPtr->TypeMask;
  convertFromBus(&msgPtr->velocity, &busPtr->Velocity);
  msgPtr->yaw =  busPtr->Yaw;
  msgPtr->yaw_rate =  busPtr->YawRate;
}

void convertToBus(SL_Bus_my_sim_mavros_msgs_PositionTarget* busPtr, mavros_msgs::PositionTarget const* msgPtr)
{
  const std::string rosMessageType("mavros_msgs/PositionTarget");

  convertToBus(&busPtr->AccelerationOrForce, &msgPtr->acceleration_or_force);
  busPtr->CoordinateFrame =  msgPtr->coordinate_frame;
  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBus(&busPtr->Position, &msgPtr->position);
  busPtr->TypeMask =  msgPtr->type_mask;
  convertToBus(&busPtr->Velocity, &msgPtr->velocity);
  busPtr->Yaw =  msgPtr->yaw;
  busPtr->YawRate =  msgPtr->yaw_rate;
}


// Conversions between SL_Bus_my_sim_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_my_sim_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->nsec =  busPtr->Nsec;
  msgPtr->sec =  busPtr->Sec;
}

void convertToBus(SL_Bus_my_sim_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Nsec =  msgPtr->nsec;
  busPtr->Sec =  msgPtr->sec;
}


// Conversions between SL_Bus_my_sim_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_my_sim_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_my_sim_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}


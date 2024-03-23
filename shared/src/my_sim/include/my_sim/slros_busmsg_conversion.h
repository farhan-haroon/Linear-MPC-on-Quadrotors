#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include "my_sim_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_my_sim_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_my_sim_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(geometry_msgs::Pose* msgPtr, SL_Bus_my_sim_geometry_msgs_Pose const* busPtr);
void convertToBus(SL_Bus_my_sim_geometry_msgs_Pose* busPtr, geometry_msgs::Pose const* msgPtr);

void convertFromBus(geometry_msgs::PoseStamped* msgPtr, SL_Bus_my_sim_geometry_msgs_PoseStamped const* busPtr);
void convertToBus(SL_Bus_my_sim_geometry_msgs_PoseStamped* busPtr, geometry_msgs::PoseStamped const* msgPtr);

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_my_sim_geometry_msgs_Quaternion const* busPtr);
void convertToBus(SL_Bus_my_sim_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr);

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_my_sim_geometry_msgs_Vector3 const* busPtr);
void convertToBus(SL_Bus_my_sim_geometry_msgs_Vector3* busPtr, geometry_msgs::Vector3 const* msgPtr);

void convertFromBus(mavros_msgs::PositionTarget* msgPtr, SL_Bus_my_sim_mavros_msgs_PositionTarget const* busPtr);
void convertToBus(SL_Bus_my_sim_mavros_msgs_PositionTarget* busPtr, mavros_msgs::PositionTarget const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_my_sim_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_my_sim_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_my_sim_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_my_sim_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);


#endif

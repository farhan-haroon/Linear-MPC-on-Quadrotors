#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include "untitled_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_untitled_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_untitled_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_untitled_geometry_msgs_Vector3 const* busPtr);
void convertToBus(SL_Bus_untitled_geometry_msgs_Vector3* busPtr, geometry_msgs::Vector3 const* msgPtr);

void convertFromBus(mavros_msgs::PositionTarget* msgPtr, SL_Bus_untitled_mavros_msgs_PositionTarget const* busPtr);
void convertToBus(SL_Bus_untitled_mavros_msgs_PositionTarget* busPtr, mavros_msgs::PositionTarget const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_untitled_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_untitled_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_untitled_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_untitled_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);


#endif

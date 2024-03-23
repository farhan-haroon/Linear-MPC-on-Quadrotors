#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "rough";

// For Block rough/Command Velocity Publisher/Publish2
SimulinkPublisher<mavros_msgs::PositionTarget, SL_Bus_rough_mavros_msgs_PositionTarget> Pub_rough_12;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}


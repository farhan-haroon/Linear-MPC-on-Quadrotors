#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "untitled";

// For Block untitled/Command Velocity Publisher/Publish2
SimulinkPublisher<mavros_msgs::PositionTarget, SL_Bus_untitled_mavros_msgs_PositionTarget> Pub_untitled_12;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}


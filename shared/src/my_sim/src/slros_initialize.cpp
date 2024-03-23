#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "my_sim";

// For Block my_sim/Publish
SimulinkPublisher<geometry_msgs::PoseStamped, SL_Bus_my_sim_geometry_msgs_PoseStamped> Pub_my_sim_1;

// For Block my_sim/Publish1
SimulinkPublisher<mavros_msgs::PositionTarget, SL_Bus_my_sim_mavros_msgs_PositionTarget> Pub_my_sim_41;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}


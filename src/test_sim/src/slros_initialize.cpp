#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "test_sim";

// For Block test_sim/Subscribe
SimulinkSubscriber<geometry_msgs::PoseStamped, SL_Bus_test_sim_geometry_msgs_PoseStamped> Sub_test_sim_41;

// For Block test_sim/Command Velocity Publisher/Publish2
SimulinkPublisher<mavros_msgs::PositionTarget, SL_Bus_test_sim_mavros_msgs_PositionTarget> Pub_test_sim_8;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}


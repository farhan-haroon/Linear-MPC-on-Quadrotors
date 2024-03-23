#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "test_sim_types.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block test_sim/Subscribe
extern SimulinkSubscriber<geometry_msgs::PoseStamped, SL_Bus_test_sim_geometry_msgs_PoseStamped> Sub_test_sim_41;

// For Block test_sim/Command Velocity Publisher/Publish2
extern SimulinkPublisher<mavros_msgs::PositionTarget, SL_Bus_test_sim_mavros_msgs_PositionTarget> Pub_test_sim_8;

void slros_node_init(int argc, char** argv);

#endif

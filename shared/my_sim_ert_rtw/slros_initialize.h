#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "my_sim_types.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block my_sim/Publish
extern SimulinkPublisher<geometry_msgs::PoseStamped, SL_Bus_my_sim_geometry_msgs_PoseStamped> Pub_my_sim_1;

// For Block my_sim/Publish1
extern SimulinkPublisher<mavros_msgs::PositionTarget, SL_Bus_my_sim_mavros_msgs_PositionTarget> Pub_my_sim_41;

void slros_node_init(int argc, char** argv);

#endif

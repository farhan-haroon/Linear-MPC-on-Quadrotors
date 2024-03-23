#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "untitled_types.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block untitled/Command Velocity Publisher/Publish2
extern SimulinkPublisher<mavros_msgs::PositionTarget, SL_Bus_untitled_mavros_msgs_PositionTarget> Pub_untitled_12;

void slros_node_init(int argc, char** argv);

#endif

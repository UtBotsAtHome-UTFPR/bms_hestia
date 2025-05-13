/*
 *  ros.h
 *  Created on: nov 30, 2023
 *  Author: Luis Serafim
 *  This is needed to change the amount of Nodes published on the ROS.
 */

#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{
  // default is 25, 25, 512, 512
  typedef NodeHandle_<ArduinoHardware, 40, 10, 512, 512> NodeHandle;

  // This is legal too and will use the default 25, 25, 512, 512
  // typedef NodeHandle_<ArduinoHardware> NodeHandle;
}

#endif

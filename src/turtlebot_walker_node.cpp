/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file turtlebot_walker_node.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Apr 17, 2017 - Creation
 *
 * @brief <brief description>
 *
 * <details>
 *
 * *
 * * BSD 3-Clause License
 *
 * Copyright (c) 2017, Mark Jenkins
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include "ros/ros.h"
#include "Tbot.hpp"



int main(int argc, char **argv) {
  /**
   * Call ros::init() with argc and argv so that it can perform any ROS arguments and
   * name remapping that were provided at the command line.  The third argument to
   * init() is the name of the node.
   */
  ros::init(argc, argv, "turtlebot_walker_node");

  /**
   * Create ROS node handle so that we can do neat ROS things as a node
   */
  ros::NodeHandle nh;

  // Let user know we are running
  ROS_INFO_STREAM("turtlebot_walker_node has started");

  // Create my turtlebot object
  Tbot tbot;

  tbot.initialize(nh);

  ros::Rate rate(tbot.getCommandFrequency());
  while (ros::ok()) {
    tbot.behaviorWalkerUniturn();
    ros::spinOnce();
    rate.sleep();
  }


  // Tell user we are exiting
  ROS_INFO_STREAM("turtlebot_walker_node is exiting");

    return 0;
  }



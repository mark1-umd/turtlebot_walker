/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file Tbot.hpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Apr 17, 2017 - Creation
 *
 * @brief Tbot represents certain aspects of a Turtlebot for the purposes of sensing and control
 *
 * The Tbot class represents certain aspects of a Turtlebot for the purposes of receiving
 * LaserScan data (processed from a 3D camera), storing the Turtlebot state with respect
 * to local obstacles, and behaving in certain ways based on the stored Turtlebot state.
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
#ifndef TURTLEBOT_WALKER_SRC_TBOT_HPP_
#define TURTLEBOT_WALKER_SRC_TBOT_HPP_

#include <vector>
#include <algorithm>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

/** @brief Tbot represents certain aspects of a Turtlebot for the purposes of sensing and control
 */

class Tbot {
 public:
  Tbot();
  virtual ~Tbot();
  double getCommandFrequency();
  void scanMessageReceived(const sensor_msgs::LaserScan::ConstPtr& scan);
  void initialize(ros::NodeHandle &nh);
  void behaviorWalkerBiturn();
  void behaviorWalkerUniturn();

 private:
  double commandFrequency;
  double maxLinearVelocity;
  double maxAngularVelocity;
  double minObstacleClearance;
  bool minObstacleDistanceValid;
  double minObstacleDistance;
  int numberOfRanges;
  int leftMinRangeIndex;
  int leftMaxRangeIndex;
  bool leftMinObstacleDistanceValid;
  double leftMinObstacleDistance;
  int rightMinRangeIndex;
  int rightMaxRangeIndex;
  bool rightMinObstacleDistanceValid;
  double rightMinObstacleDistance;
  ros::Subscriber scanSubscriber;
  ros::Publisher cmdVelPublisher;

};

#endif /* TURTLEBOT_WALKER_SRC_TBOT_HPP_ */

/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file Tbot.cpp
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
#include "Tbot.hpp"

Tbot::Tbot()
    : commandFrequency(10.0),
      maxLinearVelocity(0.2),
      maxAngularVelocity(0.2),
      minObstacleClearance(0.75),
      minObstacleDistanceValid(false),
      minObstacleDistance(0),
      leftMinObstacleDistanceValid(false),
      leftMinObstacleDistance(0),
      rightMinObstacleDistanceValid(false),
      rightMinObstacleDistance(0) {
}

Tbot::~Tbot() {
  // TODO(Mark Jenkins): Auto-generated destructor stub
}
void Tbot::initialize(ros::NodeHandle &nh) {
  // Advertise as a command velocity publisher
  cmdVelPublisher = nh.advertise<geometry_msgs::Twist>(
      "cmd_vel_mux/input/teleop", 1000);

  // Establish callback for subscription to scan messages
  scanSubscriber = nh.subscribe("scan", 1000, &Tbot::scanMessageReceived, this);
}

double Tbot::getCommandFrequency() {
  return commandFrequency;
}

void Tbot::scanMessageReceived(const sensor_msgs::LaserScan::ConstPtr& scan) {
  numberOfRanges = scan->ranges.size();
  ROS_DEBUG_STREAM(
      "Received scan with " << numberOfRanges << " points from " << scan->angle_min << " to " << scan->angle_max);
  leftMinRangeIndex = 0;
  leftMaxRangeIndex = numberOfRanges / 2 - 1;
  rightMinRangeIndex = leftMaxRangeIndex;
  rightMaxRangeIndex = numberOfRanges - 1;

  // Find right minimum obstacle distance
  rightMinObstacleDistanceValid = false;
  for (int i = rightMinRangeIndex; i <= rightMaxRangeIndex; i++) {
    double obstacleDistance = scan->ranges[i];
    if (scan->range_min <= obstacleDistance
        && obstacleDistance <= scan->range_max) {
      if (!rightMinObstacleDistanceValid) {
        rightMinObstacleDistanceValid = true;
        rightMinObstacleDistance = obstacleDistance;
      } else {
        if (obstacleDistance < rightMinObstacleDistance)
          rightMinObstacleDistance = obstacleDistance;
      }
    }
  }

  // Find left minimum obstacle distance
  leftMinObstacleDistanceValid = false;
  for (int i = leftMinRangeIndex; i <= leftMaxRangeIndex; i++) {
    double obstacleDistance = scan->ranges[i];
    if (scan->range_min <= obstacleDistance
        && obstacleDistance <= scan->range_max) {
      if (!leftMinObstacleDistanceValid) {
        leftMinObstacleDistanceValid = true;
        leftMinObstacleDistance = obstacleDistance;
      } else {
        if (obstacleDistance < leftMinObstacleDistance)
          leftMinObstacleDistance = obstacleDistance;
      }
    }
  }

  // Find minimum obstacle distance
  minObstacleDistanceValid = false;
  if (rightMinObstacleDistanceValid && leftMinObstacleDistanceValid) {
    minObstacleDistanceValid = true;
    minObstacleDistance = std::min(rightMinObstacleDistance,
                                   leftMinObstacleDistance);
  } else if (rightMinObstacleDistanceValid) {
    minObstacleDistanceValid = true;
    minObstacleDistance = rightMinObstacleDistance;
  } else if (leftMinObstacleDistanceValid) {
    minObstacleDistanceValid = true;
    minObstacleDistance = leftMinObstacleDistance;
  }
  return;
}

void Tbot::behaviorWalkerBiturn() {
  // Prepare an appropriate command velocity message
  geometry_msgs::Twist msg;
  if (!minObstacleDistanceValid || minObstacleDistance > minObstacleClearance) {
    // We aren't near anything, so move forward
    ROS_DEBUG_STREAM("Not near anything - moving forward");
    msg.linear.x = maxLinearVelocity;
    msg.angular.z = 0;
  } else {
    // We are too close to something and need to turn
    if (rightMinObstacleDistanceValid && leftMinObstacleDistanceValid) {
      // We have two valid distances, so turn towards the greater distance
      if (rightMinObstacleDistance > leftMinObstacleDistance) {
        ROS_DEBUG_STREAM("Right clearance is greater - turning right");
        msg.linear.x = 0;
        msg.angular.z = -1 * maxAngularVelocity;
      } else {
        ROS_DEBUG_STREAM("Left clearance is greater - turning left");
        msg.linear.x = 0;
        msg.angular.z = maxAngularVelocity;
      }

    } else {
      // Assume the invalid distance is due to too great a distance, and turn that way
      if (!rightMinObstacleDistanceValid) {
        ROS_DEBUG_STREAM("Invalid distance on right - turning right");
        msg.linear.x = 0;
        msg.angular.z = -1 * maxAngularVelocity;
      } else {
        ROS_DEBUG_STREAM("Invalid distance on left - turning left");
        msg.linear.x = 0;
        msg.angular.z = maxAngularVelocity;
      }
    }
  }
  // Now publish the velocity command
  ROS_DEBUG_STREAM(
      "Linear velocity " << msg.linear.x << ", angular velocity " << msg.angular.z);
  cmdVelPublisher.publish(msg);
  return;
}

void Tbot::behaviorWalkerUniturn() {
  // Prepare an appropriate command velocity message
  geometry_msgs::Twist msg;
  if (!minObstacleDistanceValid || minObstacleDistance > minObstacleClearance) {
    // We aren't near anything, so move forward
    ROS_DEBUG_STREAM("Not near anything - moving forward");
    msg.linear.x = maxLinearVelocity;
    msg.angular.z = 0;
  } else {
    ROS_DEBUG_STREAM("Obstacle too close - turning right");
        msg.linear.x = 0;
        msg.angular.z = -1 * maxAngularVelocity;
  }

  // Now publish the velocity command
  ROS_DEBUG_STREAM(
      "Linear velocity " << msg.linear.x << ", angular velocity " << msg.angular.z);
  cmdVelPublisher.publish(msg);
  return;
}

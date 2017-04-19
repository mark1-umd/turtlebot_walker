/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file Tbot.cpp
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
#include "Tbot.hpp"

Tbot::Tbot()
    : commandFrequency(30.0),
      maxLinearVelocity(0.3),
      maxAngularVelocity(0.4),
      minObstacleClearance(0.75),
      minObstacleDistanceValid(false),
      minObstacleDistance(0),
      numberOfRanges(0),
      leftMinRangeIndex(0),
      leftMaxRangeIndex(0),
      rightMinRangeIndex(0),
      rightMaxRangeIndex(0),
      leftMinObstacleDistanceValid(false),
      leftMinObstacleDistance(0),
      rightMinObstacleDistanceValid(false),
      rightMinObstacleDistance(0) {
}

Tbot::~Tbot() {
  // TODO(Mark Jenkins): Auto-generated destructor stub
}

/**
 * @brief Initialize the Tbot object for inter-operation with the ROS system
 * @param [in] nh is the node handle used for the ROS node this object is part of
 */
void Tbot::initialize(ros::NodeHandle &nh) {
  // Advertise as a command velocity publisher; don't want too big a queue
  // because unprocessed commands will stink like old fish
  cmdVelPublisher = nh.advertise<geometry_msgs::Twist>(
      "cmd_vel_mux/input/teleop", 2);

  // Establish callback for subscription to scan messages; don't want
  // too big a queue because old scan data is just not worthwhile
  scanSubscriber = nh.subscribe("scan", 2, &Tbot::scanMessageReceived, this);
  return;
}

/**
 * @brief Return the value of the private attribute commandFrequency
 * @return double commandFrequency
 */
double Tbot::getCommandFrequency() {
  return commandFrequency;
}

/**
 * @brief A ROS subscription callback to process LaserScan messages on the /scan topic
 * @param [in] scan - a ROS sensor_msgs/LaserScan message
 */
void Tbot::scanMessageReceived(const sensor_msgs::LaserScan::ConstPtr& scan) {
  // Determine size (width) of scan from the message
  numberOfRanges = scan->ranges.size();
  ROS_DEBUG_STREAM(
      "Received scan with " << numberOfRanges << " points from " << scan->angle_min << " to " << scan->angle_max);

  // Divide the scan region into a right half and a left half
  leftMinRangeIndex = 0;
  leftMaxRangeIndex = numberOfRanges / 2 - 1;
  rightMinRangeIndex = leftMaxRangeIndex;
  rightMaxRangeIndex = numberOfRanges - 1;

  // Processing scan data is complicated by the fact that range values smaller than the
  // "range_min" or greater than the "range_max" are not valid data.  For a simulated
  // asus_xtion_pro 3D camera, for instance, the range_min value is about .5 meters, and
  // the range_max value is about 10 meters.  The simple mechanisms below store the lowest
  // valid value as the obstacle distance; this will misrepresent reality if an invalid
  // range or ranges is present because the 3D camera is too close to an obstacle

  // Find minimum obstacle distance present in the right half of the scan
  // Until we find valid data, the minimum obstacle distance is invalid
  rightMinObstacleDistanceValid = false;
  for (int i = rightMinRangeIndex; i <= rightMaxRangeIndex; i++) {
    // Take a range value from the scan
    double obstacleDistance = scan->ranges[i];
    // if the range value is valid data
    if (scan->range_min <= obstacleDistance
        && obstacleDistance <= scan->range_max) {
      // Check to see if it is the first valid data
      if (!rightMinObstacleDistanceValid) {
        rightMinObstacleDistanceValid = true;
        rightMinObstacleDistance = obstacleDistance;
      } else {
        // Check to see if it is smaller than any previous valid data
        if (obstacleDistance < rightMinObstacleDistance)
          rightMinObstacleDistance = obstacleDistance;
      }
    }
  }

  // Find minimum obstacle distance in the left half of the scan
  // Until we find valid data, the minimum obstacle distance is invalid
  leftMinObstacleDistanceValid = false;
  for (int i = leftMinRangeIndex; i <= leftMaxRangeIndex; i++) {
    // Take a range value from the scan
    double obstacleDistance = scan->ranges[i];
    // if the range value is valid data
    if (scan->range_min <= obstacleDistance
        && obstacleDistance <= scan->range_max) {
      // Check to see if it is the first valid data
      if (!leftMinObstacleDistanceValid) {
        leftMinObstacleDistanceValid = true;
        leftMinObstacleDistance = obstacleDistance;
      } else {
        // Check to see if it is smaller than any previous valid data
        if (obstacleDistance < leftMinObstacleDistance)
          leftMinObstacleDistance = obstacleDistance;
      }
    }
  }

  // Find minimum obstacle distance
  minObstacleDistanceValid = false;
  // If both the right and left minimum distances are valid
  if (rightMinObstacleDistanceValid && leftMinObstacleDistanceValid) {
    // We have a valid overall minimum as the smallest of the right and left distances
    minObstacleDistanceValid = true;
    minObstacleDistance = std::min(rightMinObstacleDistance,
                                   leftMinObstacleDistance);
  } else if (rightMinObstacleDistanceValid) {
    // Use the right minimum distance if it is valid (the left must not be valid)
    minObstacleDistanceValid = true;
    minObstacleDistance = rightMinObstacleDistance;
  } else {
    // Use the left minimum distance (because the right must not be valid)
    minObstacleDistanceValid = true;
    minObstacleDistance = leftMinObstacleDistance;
  }
  return;
}

/**
 * @brief Walker behavior with right/left turn based on side with greater clearance
 */
void Tbot::behaviorWalkerBiturn() {
  // This behavior is malfunctional; Turtlebot stops, then wiggles back and forth
  // instead of smoothly turning left or right.  If the obstacle is moved slightly,
  // Turtlebot turns towards the object instead of away from the object.

  // Prepare an appropriate command velocity message
  geometry_msgs::Twist msg;

  ROS_DEBUG_STREAM(
      "Right distance " << rightMinObstacleDistanceValid << " at " << rightMinObstacleDistance << ", left distance " << leftMinObstacleDistanceValid << " at " << leftMinObstacleDistance);
  // Determine if there is anything close enough to worry about
  if (!minObstacleDistanceValid || minObstacleDistance > minObstacleClearance) {
    // We aren't near anything, so move forward
    ROS_DEBUG_STREAM("Not near anything - moving forward");
    msg.linear.x = maxLinearVelocity;
    msg.angular.z = 0;
  } else {
    // We are too close to something and need to turn; check to see
    // if we have two valid distances or not
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
      // We have at least one invalid distance; assume that the invalid distance
      // is the greater distance than the valid distance, and just turn right
      // if they are both invalid
      if (!rightMinObstacleDistanceValid) {
        // Right distance is invalid
        ROS_DEBUG_STREAM("Invalid distance on right - turning right");
        msg.linear.x = 0;
        msg.angular.z = -1 * maxAngularVelocity;
      } else {
        // Right distance was valid, so left must be invalid
        ROS_DEBUG_STREAM("Invalid distance on left - turning left");
        msg.linear.x = 0;
        msg.angular.z = maxAngularVelocity;
      }
    }
  }
  // Publish the velocity command and return
  cmdVelPublisher.publish(msg);
  return;
}

/**
 * @brief Walker behavior with one direction of turning (right)
 */
void Tbot::behaviorWalkerUniturn() {
  // Prepare an appropriate command velocity message
  geometry_msgs::Twist msg;

  // Determine if there is anything close enough to worry about
  if (!minObstacleDistanceValid || minObstacleDistance > minObstacleClearance) {
    // We aren't near anything, so move forward
    ROS_DEBUG_STREAM("Not near anything - moving forward");
    msg.linear.x = maxLinearVelocity;
    msg.angular.z = 0;
  } else {
    // Always turn right when we encounter an obstacle
    ROS_DEBUG_STREAM("Obstacle too close - turning right");
        msg.linear.x = 0;
        msg.angular.z = -1 * maxAngularVelocity;
  }

  // Publish the velocity command and return
  cmdVelPublisher.publish(msg);
  return;
}

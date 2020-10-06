#ifndef ROBOTMOTION_H
#define ROBOTMOTION_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#include <iostream>

#include "utils.h"

class RobotMotion
{
  private:
    //The node handle we'll be using
    ros::NodeHandle nh_;
    //We will be publishing to the "/base_controller/command" topic to issue commands
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Twist base_cmd;
    //Variables de alineacion
    float desired_yaw;
    bool aligned, onGoal;

  public:
    RobotMotion();
    ~RobotMotion();

    bool goToPoint(float x1, float y1, float x2, float y2, float yaw);
    bool align(float x1, float y1, float x2, float y2, float yaw);
    float normalizeAngle(float angle);
    double euclideanDistance(float x1, float y1, float x2, float y2);
};

#endif // ROBOTMOTION_H

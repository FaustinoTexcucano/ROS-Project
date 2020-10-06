#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include "ros/ros.h"
#include "GoToPoint.cpp"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "vector"
#include "tf/tf.h"
#include "../../bug_algorithms/src/utils.h"
#include "graph.h"
#include "visualization_msgs/Marker.h"



class RobotMotion{

  RobotMotion(){
  }


  void moveTo(float desPosX, float desPosY,float cX, float cY){
    Node *temp = new Node(cX, cY);
    if(GoToPoint::calculateCandidateN(temp, desPosX,desPosY)<=.05) {
      baseCmd.linear.x = 0;
      baseCmd.angular.z = 0;
      cmdVelPub.publish(baseCmd);
      state = CandReached;

    }

    else {
      //cout<<"aquí me muevo"<<endl;
      if (!obstacle)
      {
        desired_yaw = atan2(desPosY - cY,desPosX - cX);
        float error = normalizeAngle(desired_yaw - yaw_);

        cos(error) < 0.1 ? vel = 0 : vel = cos(error);

        baseCmd.linear.x = .17*vel;/*cout<<"aquí me muevo vdd"<<endl;*/}
      else {
        {
          baseCmd.linear.x = .0;
        }
      }

      cmdVelPub.publish(baseCmd);

    }
  }

  void align(float desPosX, float desPosY,float cX, float cY){

      desired_yaw = atan2(desPosY - cY,desPosX - cX);
      float error = normalizeAngle(desired_yaw - yaw_);

      if (!obstacle){
        if(error >= 2.61799 && error <= 3.66519)
        {
          //error > 3 ? angular = .2 : angular = sin(error);
          baseCmd.angular.z = .2;
        }
        else
        {
          error > 3 ? angular = .2 : angular = sin(error);
          baseCmd.angular.z = .7*angular;
        //error > 0 ? baseCmd.angular.z = .2 : baseCmd.angular.z = -.2;
        }

      }
      else {
          baseCmd.angular.z = .6*sin(error);
      }
      cmdVelPub.publish(baseCmd);



  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "RobotMotion");
  ros::NodeHandle nh;
}

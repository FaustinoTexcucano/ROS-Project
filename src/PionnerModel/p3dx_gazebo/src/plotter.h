#ifndef PLOTTER_H
#define PLOTTER_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "visualization_msgs/Marker.h"

#include <iostream>

class Plotter
{
  private:
    //The node handle we'll be using
    ros::NodeHandle nh_;
    //Publicador para RVIZ
    ros::Publisher vis_pub;
    //indice se la figura pintada en rviz
    int idMarker = 0;

  public:
    Plotter();
    ~Plotter();

    void markerPoints(float x, float y);
    void markerLines(geometry_msgs::Point pointA, geometry_msgs::Point pointB);
};

#endif // PLOTTER_H

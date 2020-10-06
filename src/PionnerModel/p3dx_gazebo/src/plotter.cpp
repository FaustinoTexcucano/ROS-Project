#include "plotter.h"

Plotter::Plotter()
{
  vis_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 1);
}

Plotter::~Plotter(){}


void Plotter::markerPoints(float x, float y)
{
  idMarker ++;
  visualization_msgs::Marker marker;
  //marker.header.frame_id = "base_link";
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = idMarker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  vis_pub.publish( marker );
}

void Plotter::markerLines(geometry_msgs::Point pointA, geometry_msgs::Point pointB)
{
  idMarker ++;
  visualization_msgs::Marker marker;
  //marker.header.frame_id = "base_link";
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = idMarker;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.points.push_back(pointA);
  marker.points.push_back(pointB);
  vis_pub.publish( marker );
}

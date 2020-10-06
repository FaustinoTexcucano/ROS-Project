#include "robotmotion.h"

RobotMotion::RobotMotion()
{
  //set up the publisher for the cmd_vel topic
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //Publicador cuando se usa el robot real
  //cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
}

RobotMotion::~RobotMotion(){}


//funcion para alinear al robot en direccion al punto destino desde el punto actual
bool RobotMotion::align(float x1, float y1, float x2, float y2, float yaw)
{
  desired_yaw = atan2(y2 - y1, x2 - x1);
  float error = normalizeAngle(desired_yaw - yaw);
  aligned = false;

  if (abs(error) > PI/180 && abs(error) < PI/170 && !aligned)
  {
    base_cmd.angular.z = 0;
    cmd_vel_pub_.publish(base_cmd);
    aligned = true;
  }
  else
  {
    if(error > 0 && error <= 0.7)
      base_cmd.angular.z = sin(error);
    else
      base_cmd.angular.z = 0.7 * sin(error);
  }
  cmd_vel_pub_.publish(base_cmd);
  return aligned;
}

float RobotMotion::normalizeAngle(float angle)
{
  if(abs(angle) > PI)
    angle = angle - (2*PI*angle) / (abs(angle));

  return angle;
}

//Metodo para dirigir al robot a su punto destino
bool RobotMotion::goToPoint(float x1, float y1, float x2, float y2, float yaw)
{
  onGoal = false;
  desired_yaw = atan2(y2 - y1, x2 - x1);
  float error = normalizeAngle(desired_yaw - yaw);
  //Si la distancia al nodo candidato es grande sigue avanzando
  if(euclideanDistance(x1, y1, x2, y2) >= 0.05)
    if(error < 1 && error > -1)
      base_cmd.linear.x = 0.15 * cos(error);
    else
       base_cmd.linear.x = 0.01;
  else
  {
    onGoal = true;
  }
  cmd_vel_pub_.publish(base_cmd);
  return onGoal;
}

//Metodo para obtener la distancia entre 2 puntos
double RobotMotion::euclideanDistance(float x1, float y1, float x2, float y2)
{
  return sqrt((pow(x2 - x1, 2))+(pow(y2 - y1, 2)));
}

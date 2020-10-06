#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include "graph.h"

#define PI 3.14159265

using namespace std;

class RobotDriver
{
    private:
        //The node handle we'll be using
        ros::NodeHandle nh_;
        //We will be publishing to the "/base_controller/command" topic to issue commands
        ros::Publisher cmd_vel_pub_;
        geometry_msgs::Twist base_cmd;
        ros::Subscriber sub;

        //Nodo suscriptor para la odometria
        ros::Subscriber sub_Odom;

        float x;
        float y;
        float z;

        float x_meta = 5.0;
        float y_meta = 0.0;
        bool samarripa = true;

        Graph *graph;
        Node *node;

        //estados para validar si ha llegado a la meta
        enum class states{isStop, onPoint, isMoving, isObstacle, isFinish};
        states currentState;

    public:
        //ROS node initialization
        RobotDriver(ros::NodeHandle &nh) {
            nh_ = nh;
            //set up the publisher for the cmd_vel topic
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

            node = new Node(0.0, 0.0);
            graph = new Graph();
            graph->insertNode(*node);

            currentState = states::isStop;

        }

        void setX(float x)
        {
          this->x = x;
        }
        float getX()
        {
          return x;
        }

        void setY(float y)
        {
          this->y = y;
        }
        float getY()
        {
          return y;
        }

        void setZ(float z)
        {
          this->z = z;
        }
        float getZ()
        {
          return z;
        }

        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
          //El robot se para cuando encuentra un obstaculo a 20 cm de distancia
          if(msg->ranges[364] <= 0.2 && samarripa == true)
          {
              base_cmd.angular.z = 0;
              base_cmd.linear.x = 0;
              cmd_vel_pub_.publish(base_cmd);

              graph->showList();
              ROS_INFO("StopStop");
              samarripa = false;
          }

          if(currentState == states::isStop)
          {
            if(msg->ranges[364] >= 1.0)
            {
              currentState = states::isMoving;
              ROS_INFO("Robot Moving");

              ROS_INFO("New Node");
              node = new Node(getX()+1.0, getY());
              graph->insertNode(*node);

              base_cmd.angular.z = 0;
              base_cmd.linear.x = 0.3;
              cmd_vel_pub_.publish(base_cmd);
            }
          }
        }

        void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
          setX(msg->pose.pose.position.x);
          setY(msg->pose.pose.position.z);

          float x = graph->list_Nodes.back().getPos_X();
          float y = graph->list_Nodes.back().getPos_Y();

          if(distancia_Euclidiana(getX(), getY(), x, y) <= 0.01)
          {
            currentState = states::isStop;
            ROS_INFO_STREAM("STOP");
            /*base_cmd.angular.z = 0;
            base_cmd.linear.x = 0;
            cmd_vel_pub_.publish(base_cmd);*/
          }

        }

        //Metodo para obtener la distancia entre 2 puntos
        double distancia_Euclidiana(float x1, float y1, float x2, float y2)
        {
          return sqrt((pow(x2 - x1, 2))+(pow(y2 - y1, 2)));
        }

        //Funcion que determina cuantos grados debe girar respecto a 2 puntos (punto actual y punto destino)
        void anguloRotacion(float x1, float y1, float x2, float y2)
        {
          double angulo = atan2(y2 - y1, x2 - x1) * 180 / PI;
          ROS_INFO_STREAM("El angulo de rotacion es: "<<angulo);
          ROS_INFO_STREAM("Mi Solecito");

          //rotarAngulo(angulo);

         /* base_cmd.linear.x = 0.0;
          base_cmd.angular.z = 0.5;
          cmd_vel_pub_.publish(base_cmd);
          */
        }

        void rotarAngulo(double angulo)
        {
          double velocidad = 0.5;
          double velocidadAngular = velocidad * 2 * PI / 360;
          double anguloRelativo = angulo * 2 * PI /360;
          //double anguloRelativo = angulo;

          ROS_INFO_STREAM("La velocidad Angular es: "<<velocidadAngular);
          ROS_INFO_STREAM("El Angulo Relativo es: "<<anguloRelativo);

          //double anguloActual = 0.0;
          double anguloActual = ros::Time::now().toSec();

          //Variable de tiempo inicial
          double t0 = ros::Time::now().toSec();
          ROS_INFO_STREAM("Tiempo Inicial "<<t0);


          while(anguloActual <= anguloRelativo)
          {
            base_cmd.angular.z = velocidadAngular;
            cmd_vel_pub_.publish(base_cmd);
            double t1 = ros::Time::now().toSec();
            ROS_INFO_STREAM("Tiempo 1: "<<t1);
            anguloActual = velocidadAngular * (t1 - t0);
            //ROS_INFO_STREAM("Angulo Actual "<<anguloActual);
          }
          ROS_INFO_STREAM("Publica ");
          //Se establece la velocidad angular a 0 y el robot se detiene
           base_cmd.angular.z = 0;
           base_cmd.linear.x = 0;
            cmd_vel_pub_.publish(base_cmd);
            ROS_INFO_STREAM("Termina ");

        }

        /*void tiempo()
        {
          double secs =ros::Time::now().toSec();

              ros::Duration d(0.5);
              secs = d.toSec();
              ROS_INFO_STREAM("Tiempo "<<secs*2);

        }*/

        //Loop forever while sending drive commands based on keyboard input
        bool driveKeyboard() {
             std::cout << "Type a command and then press enter. " <<
                   "Use '+' to move forward, 'l' to turn left, " << "'r' to turn right, '.' to exit.\n";
             //we will be sending commands of type "twist"
             //geometry_msgs::Twist base_cmd;
             char cmd[50];
             //while(nh_.ok()) {
                 std::cin.getline(cmd, 50);
                 if(cmd[0]!='w' && cmd[0]!='a' && cmd[0]!='d' && cmd[0]!='s' && cmd[0]!='.') {
                     std::cout << "unknown command:" << cmd << "\n";
                     //continue;
                 }
                 //base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
                 //move forward
                 if(cmd[0]=='w') {
                     base_cmd.angular.z = 0;
                     base_cmd.linear.x += 0.25;
                 }
                 //turn left (yaw) and drive forward at the same time
                 else if(cmd[0]=='a') {
                     base_cmd.angular.z += 0.75;
                     base_cmd.linear.x = 0.25;
                 }
                 //turn right (yaw) and drive forward at the same time
                 else if(cmd[0]=='d') {
                     base_cmd.angular.z -= 0.75;
                     base_cmd.linear.x = 0.25;
                 }
                 else if(cmd[0]=='s') {
                     base_cmd.angular.z = 0;
                     base_cmd.linear.x -= 0.25;
                 }
                 //quit
                 else if(cmd[0]=='.') {
                     //break;
                 }
                 //publish the assembled command
                 cmd_vel_pub_.publish(base_cmd);
            //}
            return true;
        }
};

int main(int argc, char** argv) {
    //init the ROS node
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    RobotDriver *driver= new RobotDriver(nh);
    ros::Subscriber sub = nh.subscribe("p3dx/laser/scan", 100, &RobotDriver::laserCallback,driver);
    ros::Subscriber sub2 = nh.subscribe("/odom", 100, &RobotDriver::odometryCallback,driver);
    //driver->driveKeyboard();
    //driver->anguloRotacion(0.0, 0.0, 2.0, 2.0);
    //driver->map();
    ros::spin();
}


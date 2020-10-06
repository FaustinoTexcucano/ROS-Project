#include <iostream>
#include <math.h>
#include <cstdlib>
#include <queue>
#include <time.h>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"

#include "graph.h"
#include "plotter.h"
#include "robotmotion.h"
#include "utils.h"

#define PI 3.14159265
//VAriables de region segura de
#define distanceMin 1.5//0.7//1.1//0.5           //Distancia minima para lanzar un rayo 1.5    0.5    //Robot Real 0.7
#define distanceMax 3.3//0.7//1.1//2.6           //Distancia maxima 3.0                               //Robot Real 0.7
#define distanceLimit 0.35//0.35         //Limite de region segura respecto al obstaculo 0.5   0.5    //Robot Real 0.3
#define distanceIncrement 0.1//0.3     //Variable que aumenta la distancia permitida 0.3              //Robot Real 0.1

using namespace std;

class RobotDriver
{

    private:
        //The node handle we'll be using
        ros::NodeHandle nh_;

        //Nodo suscriptor para la odometria
        ros::Subscriber sub_Odom;

        Graph *graph;
        Node *currentNode;
        Node *nodeFather;

        //Variable del Ploter
        Plotter *plotter;

        //Variable RobotMotion
        RobotMotion *robotMotion;

        //variables de SRT
        geometry_msgs::Point currentPoint = geometry_msgs::Point();         //Variable de la posicion global del robot
        //Puntos auxiliares para poibles candidatos
        geometry_msgs::Point childrenPoints[5];
        geometry_msgs::Point goalPoint = geometry_msgs::Point();
        //Variables de BackTracking
        int childSelected;
        queue<geometry_msgs::Point> listPoints;
        bool isBack=false, firstBack=false, backTracking=false;

        //Variables de alineacion
        float yaw_, desired_yaw;
        bool aligned, onGoal;

        //estados para validar si ha llegado a la meta
        enum class states{isStop, onPoint, readyToMove, isBackTracking, isFinish, goToGoal};
        states currentState;

        //Regiones Para el laser
        map<string, float> regions = {{"right",0},{"front_right",0},{"front",0},{"front_left",0},{"left",0},{"sideRight",0},{"sideLeft",0}};
        bool isLaserReady = false;
        bool isOdomReady = false;


    public:

        //ROS node initialization
        RobotDriver(ros::NodeHandle &nh) {
            nh_ = nh;

            //Creamos un nuevo Grafo
            graph = new Graph();
            //Creamos un nuevo Node
            currentNode = new Node();
            nodeFather = new Node();

            //Creamos un nuevo Ploter
            plotter = new Plotter();

            //Creamos un nuevo RobotMotion
            robotMotion = new RobotMotion();

            //Inicializamos los estados y del navegador
            currentState = states::isStop;
            aligned = false;
            onGoal = false;

            //Inicializamos el punta actual como 0.0
            currentPoint.x = 0;
            currentPoint.y = 0;

            cout<<"GOAL X= "<<X_goal<<" Y= "<<Y_goal<<endl;
            plotter->markerPoints(X_goal, Y_goal);
        }

        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
          //Regiones del laser
          //ros::Duration(1.0).sleep();
          regions["right"] = *min_element(begin(msg->ranges), begin(msg->ranges)+200, myfn);
          regions["front_right"] = *min_element(begin(msg->ranges)+200, begin(msg->ranges)+310, myfn);
          regions["front"] = *min_element(begin(msg->ranges)+310, begin(msg->ranges)+420, myfn);
          regions["front_left"] = *min_element(begin(msg->ranges)+420, begin(msg->ranges)+530, myfn);
          regions["left"] = *min_element(begin(msg->ranges)+530, end(msg->ranges), myfn);

          //regions["sideRight"] = *min_element(begin(msg->ranges), begin(msg->ranges)+364);
          //regions["sideLeft"] = *min_element(begin(msg->ranges)+365, end(msg->ranges));
          isLaserReady = true;
          //ALGORITMO SRT Arboles Aleatorios Basados en Sensores
          //Algoritmo para la exploracion del ambiente desconocido

          if((currentState == states::onPoint || currentState == states::isStop || currentState == states::isBackTracking) && isLaserReady)
          {

            //Obtener Hijos candidatos
            if (isOdomReady)
              getChildren(msg->angle_min, msg->angle_increment);
          }
        }

        //Nuevos Hijos
        void getChildren(float angle_min, float angle_increment)
        {
          //Se comprueba si el estado actual es backtracking
          if(currentState == states::isBackTracking || backTracking)
          {
            if(firstBack)
            {
              cout<<"Funcion bactracking PADRE"<<endl;
              graph->backtracking();
              //Se actualiza el Nodo Actual con el Nodo actual despues de hacer el backTracking
              currentNode = graph->getCurrentNode();
              //markerPoints(currentNode->getX(), currentNode->getY());
              firstBack = false;
            }
            else
            {
              if(!graph->listPoints.empty())
              {
                cout<<"Funcion bactracking hERMANOS"<<endl;
                isBack = true;
                backtracking();
              }
              else
              {
                isBack = false;
                backTracking = false;
                currentState = states::readyToMove;
              }
            }
          }
          //Si el estado es un estado normal de exploracion
          else if(!isBack)
          {
            firstBack = true;
            aligned = false;
            onGoal = false;

            int rays[5];
            float distances[5], localX, localY, tempX, tempY;
            float globalAngle = yaw_;
            float angle;
            //Arreglos de las posiciones del robot que se van a insertar los que son validos
            float x[5], y[5];
            int numChild = 0;

            //Elegir alatoriamente un rayo por region
            /*rays[0] = 0 + (rand() % 191); //0 a 200
            rays[1] = 210 + (rand() % 101); //200 a 310
            rays[2] = 320 + (rand() % 101); //310 a 420
            rays[3] = 430 + (rand() % 101); //420 a 530
            rays[4] = 540 + (rand() % 198); //530 a 727*/

            //Elegir los rayos de manera determinista
            rays[0] = 122;
            rays[1] = 243;
            rays[2] = 364;
            rays[3] = 485;
            rays[4] = 606;

            for(int i = 0; i < 5; i++)
            {
              ros::Duration(0.1).sleep();
              //Elegir la distancia maxima para cada region, el valor inicial es distance min
              distances[i] = getMaxDistance(i,distanceMin);

              //Comparamos si la distancia pertenece a una region segura libre de colision
              if(distances[i] != -1)
              {
                //Determinamos el angulo del rayo elegido
                angle = angle_min + (rays[i] * angle_increment);
                //Determinamos sus coordenadas Locales
                localX = convertPolarToCartesianX(distances[i], angle);
                localY = convertPolarToCartesianY(distances[i], angle);
                //Ajustamos las coordenadas locales tomando en cuenta el desajuste del laser
                localX = localX + round(0.12*cos(yaw_));
                localY = localY + round(0.12*sin(yaw_));
                //Determinamos sus coordenadas Globales
                tempX = ((localX * cos(globalAngle)) - (localY * sin(globalAngle))) + currentPoint.x;
                tempY = ((localX * sin(globalAngle)) + (localY * cos(globalAngle))) + currentPoint.y;

                //Se pintan todos los puntos sin excepcion
                //goalPoint.x = tempX;
                //goalPoint.y = tempY;
                //markerLines2(currentPoint, goalPoint);
                //markerPoints(tempX, tempY);

                //Comparamos si el candidato esta fuera de la regiosn segura de otro nodo tio
                //if(currentNode->safeRegion(tempX, tempY))
                if(outSafeRegion(tempX, tempY))
                {
                  numChild++;
                  x[i] = tempX;
                  y[i] = tempY;
                  //Pintar puntos validos
                  goalPoint.x = x[i];
                  goalPoint.y = y[i];
                  plotter->markerPoints(x[i], y[i]);
                  plotter->markerLines(currentPoint, goalPoint);
                }
                else
                {
                  //Posicion no valida o punto inaccesible
                  x[i] = 9999;
                  y[i] = 9999;
                }
              }
              else
              {
                //Posicion no valida o punto inaccesible
                x[i] = 9999;
                y[i] = 9999;
              }
              //ros::Duration(0.1).sleep();
            }
            //Si existen nodos candidatos
            if(numChild != 0)
            {
              numChild = 0;
              //Fijamos las coordenadas del siguiente Punto
              currentNode = graph->insertNode(x, y);
              goalPoint.x = currentNode->getX();
              goalPoint.y = currentNode->getY();
              //markerLines(currentPoint, goalPoint);
              currentState = states::readyToMove;
            }
            else
            {
              //cout<<"Debe hacer BacTracking"<<endl;
              currentState = states::isBackTracking;
              backTracking = true;
            }
          }
        }

        //Funcion para verificar si el nodo candidato pertenece a la region segura de los nodos tios del nodo actual o los nodos ya visitados
        bool outSafeRegion(float x, float y)
        {
          //Se hace la comparacion con los nodos tios
          /*if(currentNode->safeRegion(x, y))
            //Se hace la comparacion con los nodos que ya han sido visitados
            //return graph->safeRegionPath(x, y);
            return true;
          else
            return false;*/

          //Se hace la comparacion con todas las poiciones
          return graph->safeRegionPath(x, y);
        }

        //Funcion que determina la distancia maxima posible para la region seleccionada
        float getMaxDistance(int region, float distanceRo)
        {
          int i=0;
          if(processRegion(region, distanceRo))
          {
            //Comparamos si la distancia aun esta libre de obstaculo
            while(processRegion(region, distanceRo) && distanceRo <= distanceMax)
            {
               distanceRo += distanceIncrement;    //Se aumentan n cm de distancia en cada iteracion
               i++;
            }
            if(i == 0)
              return distanceRo;
            else
               return distanceRo - distanceLimit;     //Restamos la distancia limite para contemplar una zona segura respecto al obstaculo
          }
          else
            return -1;      //Retorna -1 por que el obstaculo esta demasiado cerca
        }

        //Funcion que procesa la region seleccionada para determinar si la distancia del rayo candidato esta libre de obstaculos
        bool processRegion(int region, float distanceRo)
        {
          //Se maneja una region segura de n cm
          switch(region)
          {
            case 0:
              return distanceRo + distanceLimit <= regions["right"] ? true : false;
              break;
            case 1:
              return distanceRo + distanceLimit <= regions["front_right"] ? true : false;
              break;
            case 2:
              return distanceRo + distanceLimit <= regions["front"] ? true : false;
              break;
            case 3:
              return distanceRo + distanceLimit <= regions["front_left"] ? true : false;
              break;
            case 4:
              return distanceRo + distanceLimit <= regions["left"] ? true : false;
              break;
            default:
              break;
          }
        }

        //Funcion que sirve para hacer el backtracking tomando la cola de puntos a seguir
        void backtracking()
        {
            //Al punto meta le asignamos el primer elemento de la lista de puntos
            goalPoint = graph->listPoints.front();
            //cout<<"Punto X = "<<goalPoint.x<<"    Y = "<<goalPoint.y<<endl;
            //Eliminamos el primer elemento de la lista
            graph->listPoints.pop();

            aligned = false;
            onGoal = false;
            currentState = states::readyToMove;
        }

        //este metodo convierte X de coordenadas Polares a Cartesianas
        float convertPolarToCartesianX(float distanceRo, float angleTheta)
        {
          return distanceRo * cos(angleTheta);
        }

        //este metodo convierte Y de coordenadas Polares a Cartesianas
        float convertPolarToCartesianY(float distanceRo, float angleTheta)
        {
          return distanceRo * sin(angleTheta);
        }

        void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
          tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);

          tf::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
          yaw_ = yaw;

          currentPoint = msg->pose.pose.position;
          isOdomReady = true;

          //Parar al robot de emergencia por un obtaculo no contemplado en un ambiente dinamico
          if(isLaserReady && regions["front"] <= distanceLimit)
          {
            firstBack = true;
            cout<<"Obstaculo imprevisto"<<endl;
            currentState = states::isBackTracking;
            backTracking = true;
          }

          //SRT META
          //Si ya esta cerca de la meta se dirije directamente a la meta
          /*if(euclideanDistance(msg->pose.pose.position.x, msg->pose.pose.position.y, X_goal, Y_goal) <= 0.5)
          {
            currentState = states::goToGoal;
            goToPoint(msg->pose.pose.position.x, msg->pose.pose.position.y, X_goal, Y_goal);
            //if(!onGoal)
              align(msg->pose.pose.position.x, msg->pose.pose.position.y, X_goal, Y_goal);
            if(onGoal)
            {
              base_cmd.linear.x = 0;
              base_cmd.angular.z = 0;
              cmd_vel_pub_.publish(base_cmd);
              cout<<"Llego a su punto destino"<<endl;
            }
          }
          else if(currentState == states::readyToMove)
          {
            goToPoint(msg->pose.pose.position.x, msg->pose.pose.position.y, goalPoint.x, goalPoint.y);
            //if(!onGoal)
              align(msg->pose.pose.position.x, msg->pose.pose.position.y, goalPoint.x, goalPoint.y);

            if(onGoal)     //if(aligned && onGoal)
              currentState = states::onPoint;
          }*/



          //SRT SLAM
          //Si ya ha generado sus nodos hijos, procede a alinearse y a dirigirse al punto
          if(currentState == states::readyToMove)
          {
            //cout<<"Moviendo"<<endl;
            onGoal = robotMotion->goToPoint(msg->pose.pose.position.x, msg->pose.pose.position.y, goalPoint.x, goalPoint.y, yaw);
            aligned = robotMotion->align(msg->pose.pose.position.x, msg->pose.pose.position.y, goalPoint.x, goalPoint.y, yaw);
            //goToPoint(msg->pose.pose.position.x, msg->pose.pose.position.y, goalPoint.x, goalPoint.y);
            //align(msg->pose.pose.position.x, msg->pose.pose.position.y, goalPoint.x, goalPoint.y);

            if(onGoal)     //if(aligned && onGoal)
              currentState = states::onPoint;
          }
        }
};

int main(int argc, char** argv)
{
    //init the ROS node
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    RobotDriver *driver= new RobotDriver(nh);
    //ros::Subscriber sub = nh.subscribe("scan", 100, &RobotDriver::laserCallback,driver);
    //ros::Subscriber sub2 = nh.subscribe("RosAria/pose", 100, &RobotDriver::odometryCallback,driver);
    ros::Subscriber sub = nh.subscribe("/p3dx/laser/scan", 100, &RobotDriver::laserCallback,driver);
    ros::Subscriber sub2 = nh.subscribe("/odom", 100, &RobotDriver::odometryCallback,driver);
    ros::spin();
  return 0;

}


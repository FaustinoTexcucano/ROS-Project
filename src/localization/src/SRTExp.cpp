#include <iostream>
#include <stdlib.h>
#include <time.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "vector"
#include "tf/tf.h"
#include "../../bug_algorithms/src/utils.h"
#include "graph.h"
#include "visualization_msgs/Marker.h"

#define PI 3.14159265358979323846
//3.14159265358979323846
class SRTExp
{
  private:
    ros::NodeHandle n;
    ros::Publisher cmdVelPub;
    ros::Publisher marker;
    geometry_msgs::Twist baseCmd;
    visualization_msgs::Marker mark;
    //geometry_msgs::Point p;
    int side,reg,regionD, idMarker; //side of wall ,
    float yaw_, desired_yaw, desPosX, desPosY, vel, error,cX,cY;
    bool laserReady,obstacle,fNode;
    enum States {Initial,Goal, CandReached,ToCand,ToFather};
    Graph *graph;
    Node *nodeInit, *cand;

    int state;

    std::map<string, float> regions_ = {
      {"right",0},
      {"front_right",0},
      {"front",0},
      {"front_left",0},
      {"left",0}
    };

  public:

    SRTExp(ros::NodeHandle &nh) {
      n = nh;
      cmdVelPub = n.advertise<geometry_msgs::Twist>(/*"RosAria/cmd_vel"*/ "cmd_vel",2);
      marker = n.advertise<visualization_msgs::Marker>("visualization_marker", 30);

      laserReady = false;
      nodeInit = new Node(0,0);
      cand = new Node(0,0);
      idMarker=1;
      graph = new Graph(nodeInit);
      desPosX = -10;
      desPosY = 3;
      obstacle = false;
      fNode = true;
      state = Initial;
    }

    void region(float desPosX, float desPosY, float currentX, float currentY)
    {
      if(/*(desPosX >= 0 && desPosY >=0) || */(currentX<=desPosX && currentY<=desPosY) )
      {
        reg = 1;
      }
      else if(/*(desPosX < 0 && desPosY >= 0) ||*/ (currentX>desPosX && currentY<desPosY))
      {
        reg = 2;
      }
      else if(/*(desPosX < 0 && desPosY < 0) || */(currentX>desPosX && currentY>desPosY))
      {
        reg = 3;
      }
      else if(/*(desPosX >= 0 && desPosY < 0) || */(currentX<desPosX && currentY>desPosY))
      {
        reg = 4;
      }
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      regions_["right"] = *min_element(begin(msg->ranges), begin(msg->ranges)+125, myfn);
      //regions_["right"] = processRay(regions_["right"], max_laser_range);
      regions_["front_right"] = *min_element(begin(msg->ranges)+200, begin(msg->ranges)+310, myfn);
      //regions_["front_right"] = processRay(regions_["front_right"], max_laser_range);
      regions_["front"] = *min_element(begin(msg->ranges)+310, begin(msg->ranges)+420, myfn);
      //regions_["front"] = processRay(regions_["front"], max_laser_range);
      regions_["front_left"] = *min_element(begin(msg->ranges)+420, begin(msg->ranges)+530, myfn);
      //regions_["front_left"] = processRay(regions_["front_left"], max_laser_range);
      regions_["left"] = *min_element(begin(msg->ranges)+600, end(msg->ranges)+725, myfn);
      //regions_["left"] = processRay(regions_["left"], max_laser_range);

      if(regions_["front"]<=0.35
         || ( (regions_["front_right"]  <= 0.35)
         || (regions_["front_left"] <= 0.35) )) {

       obstacle = true;
      }
      else {
        obstacle = false;
      }
      laserReady = true;
    }

    void moveTo(float desPosX, float desPosY, float aPosX, float aPosY){
      //cout<<"Región "<<reg<<endl;
        if(reg == 1 && ((aPosY > desPosY)||(aPosY > desPosY-.05)) &&
           ((aPosX > desPosX)||(aPosX > desPosX-.05)) )
        {
          baseCmd.linear.x = 0;
          baseCmd.angular.z = 0;
          cmdVelPub.publish(baseCmd);
          state = CandReached;
          cout<<"Candidato alcanzado"<<endl;
        }
        else if(reg == 2 && ((aPosY > desPosY)||(aPosY > desPosY-.05)) &&
                ((aPosX < desPosX)||(aPosX < desPosX+.05)) )
        {
          baseCmd.linear.x = 0;
          baseCmd.angular.z = 0;
          cmdVelPub.publish(baseCmd);
          state = CandReached;
        }
        else if(reg ==3 && ((aPosY < desPosY)||(aPosY < desPosY+0.05)) &&
                ((aPosX < desPosX)||(aPosX < desPosX+0.05)) )
        {
          baseCmd.linear.x = 0;
          baseCmd.angular.z = 0;
          cmdVelPub.publish(baseCmd);
          state = CandReached;
        }
        else if(reg == 4 && ((aPosY < desPosY)||(aPosY < desPosY+0.05)) &&
                ((aPosX > desPosX)||(aPosX > desPosX-0.05)) )
        {
          baseCmd.linear.x = 0;
          baseCmd.angular.z = 0;
          cmdVelPub.publish(baseCmd);
          state = CandReached;
        }
        else {
          //cout<<"aquí me muevo"<<endl;
          if (!obstacle)
          {
            desired_yaw = atan2(desPosY - aPosY,desPosX - aPosX);
            float error = normalizeAngle(desired_yaw - yaw_);

            cos(error) < 0 ? vel = 0 : vel = cos(error);

            baseCmd.linear.x = .2*vel;/*cout<<"aquí me muevo vdd"<<endl;*/}
          else {
            {//baseCmd.linear.x = .0;cout<<"aquí me muevo mtr"<<endl;
            }
          }

          cmdVelPub.publish(baseCmd);

        }
    }

    void align(float desPosX, float desPosY,float aPosX, float aPosY){

        desired_yaw = atan2(desPosY - aPosY,desPosX - aPosX);
        float error = normalizeAngle(desired_yaw - yaw_);

        if (!obstacle){
          /*if (abs(error) > PI/60 && abs(error) < PI/60*6 && !obstacle){
            baseCmd.angular.z = 0;
            //baseCmd.linear.x = 0;
            cmdVelPub.publish(baseCmd);
            aligned = true;
        }
          else*/ if(error > 0)
          {
            baseCmd.angular.z = .6*sin(error);
          }
          else
          {
            baseCmd.angular.z = .6*sin(error);
          //error > 0 ? baseCmd.angular.z = .2 : baseCmd.angular.z = -.2;
          }

        }
        else {
            //baseCmd.angular.z = 0.0;
        }
        cmdVelPub.publish(baseCmd);



    }

    void checkFreeState(Node *state, int cone)
    {
      switch (cone)
      {
        case 1: if(regions_["right"]<=1.55)
                {
                  state->setPos_X(100);
                  state->setPos_Y(100);
                  state->setIsFree(1);
                }
                else {
                  state->setIsFree(0);
                }
                break;
        case 2: if(regions_["front_right"]<=1.55)
                {
                  state->setPos_X(100);
                  state->setPos_Y(100);
                  state->setIsFree(2);
                }
                else {
                  state->setIsFree(0);
                }
                break;
        case 3: if(regions_["front"]<=1.55)
                {
                  state->setPos_X(100);
                  state->setPos_Y(100);
                  state->setIsFree(3);
                }
                else {
                  state->setIsFree(0);
                }
                break;
        case 4: if(regions_["front_left"]<=1.55)
                {
                  state->setPos_X(100);
                  state->setPos_Y(100);
                  state->setIsFree(4);
                }
                else {
                  state->setIsFree(0);
                }
                break;
        case 5: if(regions_["left"]<=1.55)
                {
                  state->setPos_X(100);
                  state->setPos_Y(100);
                  state->setIsFree(5);
                }
                else {
                  state->setIsFree(0);
                }
                break;
        default:
                break;
      }
    }

    void checkBackTracking(Node *c,Node *r,Node *fR, Node *f, Node *fL, Node *l)
    {
      if(r->getPos_X()==100 && fR->getPos_X()==100 &&
         f->getPos_X()==100 && fL->getPos_X()==100 && l->getPos_X()==100)
      {
        //c->setPos_X(100);
        //c->setPos_Y(100);
        state = ToFather;
      }
      else
        state = ToCand;
    }

    float RandomFloat(float a, float b) {
        srand (time(NULL));
        float random = ((float) rand()) / (float) RAND_MAX;
        float diff = b - a;
        float r = random * diff;
        return a + r;
    }

    void checkOutSR(Node *state){
      Node *aux = new Node();
      *aux = state->getFather();
      while(aux->getPos_X()!=0){
        //cout<<"H2"<<endl;
        if(sqrt((pow(state->getPos_X() - aux->getPos_X(),2)) + (pow(state->getPos_Y() - aux->getPos_Y(),2)))<.69){
          cout<<"H3"<<endl;
          state->setPos_X(100);
          state->setPos_Y(100);
          state->setIsFree(-1);
          cout<<"Break While"<<endl;
          break;
        }
        else {
          cout<<"H4"<<endl;
          *aux = aux->getFather();
        }
      };
      aux->~Node();
    }

    Node calculateStates(Node *current) //calculate 5 posible state from current state
    {


      if ((state == Initial || state==CandReached) && laserReady == true)
      {

        if(current->getHasChildren()==true)
        {
          Node *right  = new Node();
          Node *fRight = new Node();
          Node *front  = new Node();
          Node *fLeft  = new Node();
          Node *left   = new Node();

          *right  = current->getRight();
          *fRight = current->getFRight();
          *front  = current->getFront();
          *fLeft  = current->getFLeft();
          *left   = current->getLeft();

          int i=0, select;
          srand (time(NULL));
          do
          {
            select = rand() % 5 + 1;
          switch(select)
          {
            case 1:
              if(right->getIsFree()==0 && right->getVisited()==false)
              {
                markerCand(right);

                state = ToCand;

                region(right->getPos_X(),right->getPos_Y(),cX,cY);
                right->setVisited(true);

                return *right;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 2:
              if(fRight->getIsFree()==0 && fRight->getVisited()==false)
              {
                markerCand(fRight);

                state = ToCand;

                region(fRight->getPos_X(),fRight->getPos_Y(),cX,cY);
                fRight->setVisited(true);

                return *fRight;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 3:
              if(front->getIsFree()==0 && front->getVisited()==false)
              {
                markerCand(front);

                state = ToCand;

                region(front->getPos_X(),front->getPos_Y(),cX,cY);
                front->setVisited(true);

                return *front;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 4:
              if(fLeft->getIsFree()==0 && fLeft->getVisited()==false)
              {
                markerCand(fLeft);

                state = ToCand;

                region(fLeft->getPos_X(),fLeft->getPos_Y(),cX,cY);
                fLeft->setVisited(true);

                return *fLeft;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 5:
              if(left->getIsFree()==0 && left->getVisited()==false)
              {
                markerCand(left);

                state = ToCand;

                region(left->getPos_X(),left->getPos_Y(),cX,cY);
                left->setVisited(true);

                return *left;
              }
              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;
          }
          /*i++;
          cout<<"I: "<<i<<endl;*/
          }while(i<15);

          if(i==15)
          {
            Node *father = new Node();
            *father = current->getFather();
            markerCand(father);
            state = ToCand;

            region(father->getPos_X(),father->getPos_Y(),cX,cY);
            return *father;
          }
        }


        else {
          state=ToCand;
          float randR,randFR,randF,randFL,randL;

          randR  = RandomFloat(1.06465,2.26893);
          randFR = RandomFloat(0.366519,1.0472);
          randF  = RandomFloat(-0.349066,0.349066);
          randFL = RandomFloat(0.366519,1.0472);
          randL  = RandomFloat(1.06465,2.26893);

          Node *right  = new Node(current->getPos_X() + .7*cos(yaw_ - randR), current->getPos_Y()+ .7*sin(yaw_ - randR));
          Node *fRight = new Node(current->getPos_X() + .7*cos(yaw_ - randFR), current->getPos_Y()+ .7*sin(yaw_ - randFR));
          Node *front  = new Node(current->getPos_X() + .7*cos(yaw_ + randF), current->getPos_Y()+ .7*sin(yaw_ + randF));
          Node *fLeft  = new Node(current->getPos_X() + .7*cos(yaw_ + randFL), current->getPos_Y()+ .7*sin(yaw_ + randFL));
          Node *left   = new Node(current->getPos_X() + .7*cos(yaw_ + randL), current->getPos_Y()+ .7*sin(yaw_ + randL));

          int select;

          cout<<"************"<<endl;
          cout<<"X3: "<< current->getPos_X() + 1*cos(yaw_)<<", Y3: "<< current->getPos_Y()+ 1*sin(yaw_)<<endl;
          cout<<"X1: "<< current->getPos_X() + 1*cos(yaw_ - 1.95477)<<", Y1: "<< current->getPos_Y()+ 1*sin(yaw_ - 1.95477)<<endl;
          cout<<"X2: "<< current->getPos_X() + 1*cos(yaw_ - 0.785398)<<", Y2: "<< current->getPos_Y()+ 1*sin(yaw_ - 0.785398)<<endl;
          cout<<"X4: "<< current->getPos_X() + 1*cos(yaw_ + 0.785398)<<", Y4: "<< current->getPos_Y()+ 1*sin(yaw_ + 0.785398)<<endl;
          cout<<"X5: "<< current->getPos_X() + 1*cos(yaw_ + 1.95477)<<", Y5: "<< current->getPos_Y()+ 1*sin(yaw_ + 1.95477)<<endl;
          cout<<"************"<<endl;

          current->setHasChildren(true);


          graph->insertRight(current,right);
          graph->insertFRight(current,fRight);
          graph->insertFront(current,front);
          graph->insertFLeft(current,fLeft);
          graph->insertLeft(current,left);

          checkFreeState(right,1);
          if(right->getPos_X()!=100)
            checkOutSR(right);

          checkFreeState(fRight,2);
          if(fRight->getPos_X()!=100)
            checkOutSR(fRight);

          checkFreeState(front,3);
          if(front->getPos_X()!=100)
            checkOutSR(front);

          checkFreeState(fLeft,4);
          if(fLeft->getPos_X()!=100)
            checkOutSR(fLeft);

          checkFreeState(left,5);
          if(left->getPos_X()!=100)
            checkOutSR(left);


          checkBackTracking(current,right,fRight,front,fLeft, left);

          float *r = new float, fR, *f = new float, fL, l;
          int i=0;
          srand (time(NULL));
          do
          {
            select = rand() % 5 + 1;
          switch(select)
          {
            case 1:
              if(right->getIsFree()==0)
              {
                markerLines(current,right);
                markerLines(current,fRight);
                markerLines(current,front);
                markerLines(current,fLeft);
                markerLines(current,left);

                markerNodes(right);
                markerNodes(fRight);
                markerNodes(front);
                markerNodes(fLeft);
                markerNodes(left);

                markerCand(right);

                state = ToCand;

                region(right->getPos_X(),right->getPos_Y(),cX,cY);
                right->setVisited(true);

                return *right;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 2:
              if(fRight->getIsFree()==0)
              {
                markerLines(current,right);
                markerLines(current,fRight);
                markerLines(current,front);
                markerLines(current,fLeft);
                markerLines(current,left);

                markerNodes(right);
                markerNodes(fRight);
                markerNodes(front);
                markerNodes(fLeft);
                markerNodes(left);

                markerCand(fRight);

                state = ToCand;

                region(fRight->getPos_X(),fRight->getPos_Y(),cX,cY);
                fRight->setVisited(true);

                return *fRight;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 3:
              if(front->getIsFree()==0)
              {
                markerLines(current,right);
                markerLines(current,fRight);
                markerLines(current,front);
                markerLines(current,fLeft);
                markerLines(current,left);

                markerNodes(right);
                markerNodes(fRight);
                markerNodes(front);
                markerNodes(fLeft);
                markerNodes(left);

                markerCand(front);

                state = ToCand;

                region(front->getPos_X(),front->getPos_Y(),cX,cY);
                front->setVisited(true);

                return *front;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 4:
              if(fLeft->getIsFree()==0)
              {
                markerLines(current,right);
                markerLines(current,fRight);
                markerLines(current,front);
                markerLines(current,fLeft);
                markerLines(current,left);

                markerNodes(right);
                markerNodes(fRight);
                markerNodes(front);
                markerNodes(fLeft);
                markerNodes(left);

                markerCand(fLeft);

                state = ToCand;

                region(fLeft->getPos_X(),fLeft->getPos_Y(),cX,cY);
                fLeft->setVisited(true);

                return *fLeft;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 5:
              if(left->getIsFree()==0)
              {
                markerLines(current,right);
                markerLines(current,fRight);
                markerLines(current,front);
                markerLines(current,fLeft);
                markerLines(current,left);

                markerNodes(right);
                markerNodes(fRight);
                markerNodes(front);
                markerNodes(fLeft);
                markerNodes(left);

                markerCand(left);

                state = ToCand;

                region(left->getPos_X(),left->getPos_Y(),cX,cY);
                left->setVisited(true);

                return *left;
              }
              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;
          }
          /*i++;
          cout<<"I: "<<i<<endl;*/
          }while(i<15);

          if(i==15)
          {
            Node *father = new Node();
            *father = current->getFather();

            markerCand(father);
            state = ToCand;

            region(father->getPos_X(),father->getPos_Y(),cX,cY);
            return *father;
          }



          /*cout<<"R: "<<*r<<", FR: "<<fR<<", F: "<<*f<<", FL: "<<fL<<", L: "<<l<<endl;
          delete r;
          cout<<"R Eliminado: "<<*r<<endl;*/
          state = ToCand;
        }
      }

    }

    void markerCand(Node *node)
    {
      //idMarker ++;
      visualization_msgs::Marker mark;
      mark.header.frame_id = "odom";
      mark.header.stamp = ros::Time();
      mark.ns = "my_namespace";
      mark.id = 0;
      mark.type = visualization_msgs::Marker::CUBE;
      mark.action = visualization_msgs::Marker::ADD;
      mark.pose.position.x = node->getPos_X();
      mark.pose.position.y = node->getPos_Y();
      mark.pose.position.z = 1;
      mark.pose.orientation.x = 0.0;
      mark.pose.orientation.y = 0.0;
      mark.pose.orientation.z = 0.0;
      mark.pose.orientation.w = 0.0;
      mark.scale.x = 0.12;
      mark.scale.y = 0.12;
      mark.scale.z = 0.12;
      mark.color.a = 1.0; // Don't forget to set the alpha!
      mark.color.r = 0.0;
      mark.color.g = 1.0;
      mark.color.b = 0.0;
      marker.publish( mark );
    }

    void markerNodes(Node *node)
    {
      idMarker ++;
      visualization_msgs::Marker mark;
      mark.header.frame_id = "odom";
      mark.header.stamp = ros::Time();
      mark.ns = "my_namespace";
      mark.id = idMarker;
      mark.type = visualization_msgs::Marker::CUBE;
      mark.action = visualization_msgs::Marker::ADD;
      mark.pose.position.x = node->getPos_X();
      mark.pose.position.y = node->getPos_Y();
      mark.pose.position.z = 1;
      mark.pose.orientation.x = 0.0;
      mark.pose.orientation.y = 0.0;
      mark.pose.orientation.z = 0.0;
      mark.pose.orientation.w = 0.0;
      mark.scale.x = 0.1;
      mark.scale.y = 0.1;
      mark.scale.z = 0.1;
      mark.color.a = 1.0; // Don't forget to set the alpha!
      mark.color.r = 0.0;
      mark.color.g = 0.0;
      mark.color.b = 1.0;
      marker.publish( mark );
    }

    void markerLines(Node *current, Node *destiny)
    {
      if(destiny->getPos_X() == 100 && destiny->getPos_Y() == 100)
      {}
      else
      {
        geometry_msgs::Point origin;
        geometry_msgs::Point end;

        origin.x = current->getPos_X();
        origin.y = current->getPos_Y();
        origin.z = 1;

        end.x = destiny->getPos_X();
        end.y = destiny->getPos_Y();
        end.z = 1;

        idMarker ++;

        mark.header.frame_id = "odom";
        mark.header.stamp = ros::Time();
        mark.ns = "my_namespace";
        mark.id = idMarker;
        mark.type = visualization_msgs::Marker::LINE_LIST;
        mark.action = visualization_msgs::Marker::ADD;
        mark.scale.x = 0.05;
        mark.color.a = 1.0; // Don't forget to set the alpha!
        mark.color.r = 0.0;
        mark.color.g = 0.0;
        mark.color.b = 0.0;
        mark.points.push_back(origin);
        mark.points.push_back(end);
        marker.publish( mark );
      }
    }

    float calculateCandidate(Node *node)
    {
      return sqrt((pow(desPosX - node->getPos_X(),2)) + (pow(desPosY - node->getPos_Y(),2)));
    }

    void rrt(Node *initial, double currentX, double currentY)
    {
      //Node *cand = new Node();
      //cout<<"calculó región"<<endl;
      if(fNode && laserReady)
      {
        //graph->insertNode(*nodeInit);
        Node *f = new Node (-1,-1);
        initial->setFather(f);
        *cand=calculateStates(initial);
        graph->setCNode(cand);
        //cout<<"calculó estados"<<endl;
        cout<<"Cand: "<<cand->getPos_X()<<", "<<cand->getPos_Y()<<endl;
        //graph->insertRight(nodeInit,nodeInit);
        fNode=false;
        //calculateStates(nodeInit);
        state=ToCand;
        align(cand->getPos_X(), cand->getPos_Y(), currentX, currentY);
        moveTo(cand->getPos_X(), cand->getPos_Y(), currentX, currentY);
      }
      else if(laserReady)
      {
        if(state!=Goal)
        {

          if(state==ToCand)
          {
            align(cand->getPos_X(), cand->getPos_Y(), currentX, currentY); // aquí está el eeror
            moveTo(cand->getPos_X(), cand->getPos_Y(), currentX, currentY);
          }
          //cout<<"Entró "<<endl;

          Node *p = new Node ();
          Node *cu = new Node ();
          *cu = graph->getCNode();

          if(state== CandReached)
          {


            cout<<"Región: "<<reg<<endl;
            cout<<"calcula nuevo cand"<<endl;
            //Node * state = new Node (cand->getPos_X(), cand->getPos_Y());

            //bool v=true;
            cu->setVisited(true);
            *cand=calculateStates(cu);
            cout<<"Cand: "<<cand->getPos_X()<<", "<<cand->getPos_Y()<<endl;
            graph->setCNode(cand);
            *p = cand->getFather();
            //*cu = graph->getCNode();
            cout<<"PadreCand: "<<p->getPos_X()<<", "<<p->getPos_Y()<<endl;
            cout<<"Current: "<<cu->getPos_X()<<", "<<cu->getPos_Y()<<endl;
            //state = ToCand;

          }

          /*else if(state==ToFather){

            *cand=calculateStates(cu);
            cout<<"Cand: "<<cand->getPos_X()<<", "<<cand->getPos_Y()<<endl;
            graph->setCNode(cand);
            *p = cand->getFather();
            cout<<"PadreCand: "<<p->getPos_X()<<", "<<p->getPos_Y()<<endl;
            cout<<"Current: "<<cu->getPos_X()<<", "<<cu->getPos_Y()<<endl;
            cout<<"Entró padre"<<endl;
            //state = ToCand;

          }*/
        }
      }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      //if(!aligned) {

        tf::Quaternion q(
              msg->pose.pose.orientation.x,
              msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z,
              msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        yaw_ = yaw;

        msg->pose.pose.orientation.z;

        cX=msg->pose.pose.position.x;
        cY=msg->pose.pose.position.y;

        rrt(nodeInit, msg->pose.pose.position.x, msg->pose.pose.position.y);
        //cout<<cand->getPos_X()<<endl;
        //align(cand->getPos_X(), cand->getPos_Y(), msg->pose.pose.position.x,msg->pose.pose.position.y);
        //calculateStates(nodeInit);
      //}
      //else {
        //moveTo(cand->getPos_X(), cand->getPos_Y(), msg->pose.pose.position.x,msg->pose.pose.position.y);
      //}
    }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SRTExp");
  ros::NodeHandle nh;
  SRTExp *driver= new SRTExp(nh);
  ros::Subscriber sub = nh.subscribe(/*"scan"*/ "p3dx/laser/scan", 10, &SRTExp::laserCallback,driver);
  ros::Subscriber sub1 = nh.subscribe(/*"RosAria/pose"*/ "odom", 10, &SRTExp::odomCallback,driver);
  //ros::Subscriber sub2 = nh.subscribe("/robot_pose_ekf/odom_combined", 10, &SRTExp::odomCallback,driver);
  ros::spin();
}

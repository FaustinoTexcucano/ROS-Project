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
#include "utils.h"
#include "graph.h"
#include "visualization_msgs/Marker.h"
#include <list>
#include <iterator>
#include <ctime>

#define PI 3.14159265358979323846
#define ND .5 //distance between nodes
//3.14159265358979323846
class SRTExp
{
  public:
    bool isFinished;
  private:
    ros::NodeHandle n;
    ros::Publisher cmdVelPub;
    ros::Publisher marker;
    geometry_msgs::Twist baseCmd;
    visualization_msgs::Marker mark;
    //geometry_msgs::Point p;
    int idMarker, createdNodes; //side of wall ,
    float yaw_, desired_yaw, desPosX, desPosY, vel, error,cX,cY,angular;
    bool laserReady, odomReady, obstacle, fNode;
    enum States {Initial,Goal, CandReached,ToCand,ToFather};
    Graph *graph;
    Node *nodeInit, *cand;
    list<Node> existentNodes;
    float max_laser_range = 4.0;

    unsigned t0, t1;

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
      //cmdVelPub = n.advertise<geometry_msgs::Twist>("cmd_vel",2);
      cmdVelPub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel" ,2);
      marker = n.advertise<visualization_msgs::Marker>("visualization_marker", 30);

      odomReady = false;
      laserReady = false;
      nodeInit = new Node(0,0);
      cand = new Node(0,0);
      idMarker=1;
      createdNodes = 0;
      graph = new Graph(nodeInit);
      desPosX = 5;
      desPosY = 1;
      obstacle = false;
      fNode = true;
      state = Initial;
      isFinished = false;
      t0=clock();
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      regions_["right"] = *min_element(begin(msg->ranges), begin(msg->ranges)+170, myfn); //145
      regions_["right"] = processRay(regions_["right"], max_laser_range);
      regions_["front_right"] = *min_element(begin(msg->ranges)+120, begin(msg->ranges)+315, myfn); //145 - 290
      regions_["front_right"] = processRay(regions_["front_right"], max_laser_range);
      regions_["front"] = *min_element(begin(msg->ranges)+265, begin(msg->ranges)+460, myfn);  //290 - 435
      regions_["front"] = processRay(regions_["front"], max_laser_range);
      regions_["front_left"] = *min_element(begin(msg->ranges)+415, begin(msg->ranges)+605, myfn);  //435 - 580
      regions_["front_left"] = processRay(regions_["front_left"], max_laser_range);
      regions_["left"] = *min_element(begin(msg->ranges)+555, end(msg->ranges)+725, myfn);  //580 - 725
      regions_["left"] = processRay(regions_["left"], max_laser_range);

      if(regions_["front"]<=0.29
         || ( (regions_["front_right"]  <= 0.29)
         || (regions_["front_left"] <= 0.29) )) {

       obstacle = true;
      }
      else {
        obstacle = false;
      }
      laserReady = true;
    }

    void moveTo(float desPosX, float desPosY){
      Node *temp = new Node(cX, cY);
      if(calculateCandidateN(temp, desPosX,desPosY)<=.05) {
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

          baseCmd.linear.x = .13*vel;/*cout<<"aquí me muevo vdd"<<endl;*/}
        else {
          {
            baseCmd.linear.x = .0;
          }
        }

        cmdVelPub.publish(baseCmd);

      }
    }

    void align(float desPosX, float desPosY){

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

    void goalReached(float aPosX, float aPosY)
    {
      Node *temp = new Node(aPosX, aPosY);
      if(calculateCandidate(temp)<=.05) {
        baseCmd.linear.x = 0;
        baseCmd.angular.z = 0;
        //cout<<"Meta 1"<<endl;
        t1=clock();
        double time = (double(t1-t0)/1000);
        cout<< "Execution Time: " << time << endl;
        cout<< "Nodes: " << createdNodes << endl;
        state = Goal;
      }
      cmdVelPub.publish(baseCmd);

    }

    void checkFreeState(Node *state, int cone)
    {
      switch (cone)
      {
        case 1: if(regions_["right"]<=ND+.35)
                {
                  state->setPos_X(100);
                  state->setPos_Y(100);
                  state->setIsFree(1);
                }
                else {
                  state->setIsFree(0);
                }
                break;
        case 2: if(regions_["front_right"]<=ND+.35)
                {
                  state->setPos_X(100);
                  state->setPos_Y(100);
                  state->setIsFree(2);
                }
                else {
                  state->setIsFree(0);
                }
                break;
        case 3: if(regions_["front"]<=ND+.35)
                {
                  state->setPos_X(100);
                  state->setPos_Y(100);
                  state->setIsFree(3);
                }
                else {
                  state->setIsFree(0);
                }
                break;
        case 4: if(regions_["front_left"]<=ND+.35)
                {
                  state->setPos_X(100);
                  state->setPos_Y(100);
                  state->setIsFree(4);
                }
                else {
                  state->setIsFree(0);
                }
                break;
        case 5: if(regions_["left"]<=ND+.35)
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

    /*void checkOutSR(Node *state){
      Node *aux = new Node();
      *aux = state->getFather();
      while(aux->getPos_X()!=-1){
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
          //cout<<"H4"<<endl;
          *aux = aux->getFather();
        }
      };
      aux->~Node();
    }*/

    void checkOutSR(Node *state){
      /*Node *aux = new Node();
      *aux = state->getFather();*/
      cout<<"Start checkOutSR"<<endl;
      list<Node>::iterator aux = existentNodes.end();
      bool firstIteration = true;
      while(aux != existentNodes.begin()){
        //cout<<"AUX x "<< aux->getPos_X()<<", AUX y "<<aux->getPos_Y()<<endl;
        //cout<<"H2"<<endl;
        if(sqrt((pow(state->getPos_X() - aux->getPos_X(),2)) + (pow(state->getPos_Y() - aux->getPos_Y(),2)))<ND-.01){
          cout<<"H3"<<endl;
          state->setPos_X(100);
          state->setPos_Y(100);
          state->setIsFree(-1);
          cout<<"Break While"<<endl;
          break;
        }
        else {
          //cout<<"H4"<<endl;
          aux--;
        }
      };
      //aux->~Node();
    }

    Node *calculateStates(Node *current) //calculate 5 posible state from current state
    {


      if ((state == Initial || state==CandReached) && laserReady == true)
      {

        if(current->getHasChildren()==true)
        {
          cout<<"Start states with children"<<endl;
          Node *right  = new Node();
          Node *fRight = new Node();
          Node *front  = new Node();
          Node *fLeft  = new Node();
          Node *left   = new Node();

          right  = current->getRight();
          fRight = current->getFRight();
          front  = current->getFront();
          fLeft  = current->getFLeft();
          left   = current->getLeft();

          int i=0, select;
          srand (time(NULL));
          do
          {

            select = rand() % 5 + 1;

          switch(select)
          {
            case 1:
            cout<<"Nodo R en estado "<<right->getVisited()<<endl;
              if(right->getIsFree()==0 && right->getVisited() == 0)
              {
                markerCand(right);

                state = ToCand;

                //region(right->getPos_X(),right->getPos_Y(),cX,cY);
                right->setVisited(1);

                //existentNodes.push_back(*right);

                return right;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 2:
              cout<<"Nodo FR en estado "<<fRight->getVisited()<<endl;
              if(fRight->getIsFree()==0 && fRight->getVisited() == 0)
              {
                markerCand(fRight);

                state = ToCand;

                //region(fRight->getPos_X(),fRight->getPos_Y(),cX,cY);
                fRight->setVisited(1);

                //existentNodes.push_back(*fRight);

                return fRight;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 3:
              cout<<"Nodo F en estado "<<front->getVisited()<<endl;
              if(front->getIsFree()==0 && front->getVisited() == 0)
              {
                markerCand(front);

                state = ToCand;

                //region(front->getPos_X(),front->getPos_Y(),cX,cY);
                front->setVisited(1);

                //existentNodes.push_back(*front);

                return front;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 4:
              cout<<"Nodo FL en estado "<<fLeft->getVisited()<<endl;
              if(fLeft->getIsFree()==0 && fLeft->getVisited() == 0)
              {
                markerCand(fLeft);

                state = ToCand;

                //region(fLeft->getPos_X(),fLeft->getPos_Y(),cX,cY);
                fLeft->setVisited(1);

                //existentNodes.push_back(*fLeft);

                return fLeft;
              }

              else {
                i++;
                cout<<"I: "<<i<<endl;
              }
              break;

            case 5:
              cout<<"Nodo L en estado "<<left->getVisited()<<endl;
              if(left->getIsFree()==0 && left->getVisited() == 0)
              {
                markerCand(left);

                state = ToCand;

                //region(left->getPos_X(),left->getPos_Y(),cX,cY);
                left->setVisited(1);

                //existentNodes.push_back(*left);

                return left;
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
            cout<<"Antes del padre"<<endl;
            father = current->getFather();
            if(father == NULL){
              cout<<"padre nulo"<<endl;
              return father;
            }
            cout<<"después del padre"<<endl;
            markerCand(father);
            state = ToCand;

            //region(father->getPos_X(),father->getPos_Y(),cX,cY);
            return father;
          }
        }


        else {
          cout<<"Start states without children"<<endl;
          state=ToCand;
          float randR,randFR,randF,randFL,randL;

          /*randR  = RandomFloat(1.3439,2.00713); //77 <-> 115
          randFR = RandomFloat(0.506145,1.16937); // 29 <-> 67
          randF  = RandomFloat(-0.331613,0.331613); //-19<-> 19
          randFL = RandomFloat(0.506145,1.16937);  //-29 <-> -67
          randL  = RandomFloat(1.3439,2.00713); // -77 <-> -115*/

          randR  = 1.71042;  // 96
          randFR = 0.837758; // 48
          randF  = 0;
          randFL = 0.837758; // -48
          randL  = 1.71042;  // -96


          Node *right  = new Node(current->getPos_X() + ND*cos(yaw_ - randR), current->getPos_Y() +  ND*sin(yaw_ - randR));
          Node *fRight = new Node(current->getPos_X() + ND*cos(yaw_ - randFR), current->getPos_Y()+  ND*sin(yaw_ - randFR));
          Node *front  = new Node(current->getPos_X() + ND*cos(yaw_ + randF), current->getPos_Y() +  ND*sin(yaw_ + randF));
          Node *fLeft  = new Node(current->getPos_X() + ND*cos(yaw_ + randFL), current->getPos_Y()+  ND*sin(yaw_ + randFL));
          Node *left   = new Node(current->getPos_X() + ND*cos(yaw_ + randL), current->getPos_Y() +  ND*sin(yaw_ + randL));

          int select;

          cout<<"************"<<endl;
          cout<<"X3: "<< current->getPos_X() + ND*cos(yaw_)<<", Y3: "<< current->getPos_Y()           + ND*sin(yaw_)<<endl;
          cout<<"X1: "<< current->getPos_X() + ND*cos(yaw_ - 1.95477)<<", Y1: "<< current->getPos_Y() + ND*sin(yaw_ - 1.95477)<<endl;
          cout<<"X2: "<< current->getPos_X() + ND*cos(yaw_ - 0.785398)<<", Y2: "<< current->getPos_Y()+ ND*sin(yaw_ - 0.785398)<<endl;
          cout<<"X4: "<< current->getPos_X() + ND*cos(yaw_ + 0.785398)<<", Y4: "<< current->getPos_Y()+ ND*sin(yaw_ + 0.785398)<<endl;
          cout<<"X5: "<< current->getPos_X() + ND*cos(yaw_ + 1.95477)<<", Y5: "<< current->getPos_Y() + ND*sin(yaw_ + 1.95477)<<endl;
          cout<<"************"<<endl;


          graph->insertRight(current,right);
          graph->insertFRight(current,fRight);
          graph->insertFront(current,front);
          graph->insertFLeft(current,fLeft);
          graph->insertLeft(current,left);

          current->setHasChildren(true);

          checkFreeState(right,1);
          if(right->getPos_X()!=100){

            checkOutSR(right);
          }

          checkFreeState(fRight,2);
          if(fRight->getPos_X()!=100){

            checkOutSR(fRight);
          }

          checkFreeState(front,3);
          if(front->getPos_X()!=100){

            checkOutSR(front);
          }

          checkFreeState(fLeft,4);
          if(fLeft->getPos_X()!=100){

            checkOutSR(fLeft);
          }

          checkFreeState(left,5);
          if(left->getPos_X()!=100){

            checkOutSR(left);
          }


          float *r = new float, fR, *f = new float, fL, l;

          int i=0;
          srand (time(NULL));
          do
          {

            select = rand() % 5 + 1;

            if(calculateCandidate(right)<=ND+.05)
            {
              cout<<"Cambio de destino"<<endl;
              right->setPos_X(desPosX);
              right->setPos_Y(desPosY);
              select = 1;
            }
            if(calculateCandidate(fRight)<=ND+.05)
            {
              fRight->setPos_X(desPosX);
              fRight->setPos_Y(desPosY);
              select = 2;
            }
            if(calculateCandidate(front)<=ND+.05)
            {
              front->setPos_X(desPosX);
              front->setPos_Y(desPosY);
              select = 3;
            }
            if(calculateCandidate(fLeft)<=ND+.05)
            {
              fLeft->setPos_X(desPosX);
              fLeft->setPos_Y(desPosY);
              select = 4;
            }
            if(calculateCandidate(left)<=ND+.05)
            {
              left->setPos_X(desPosX);
              left->setPos_Y(desPosY);
              select = 5;
            }


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

                //region(right->getPos_X(),right->getPos_Y(),cX,cY);
                right->setVisited(1);

                if(right->getPos_X()!=100)existentNodes.push_back(*right);
                if(front->getPos_X()!=100)existentNodes.push_back(*front);
                if(left->getPos_X()!=100)existentNodes.push_back(*left);
                if(fRight->getPos_X()!=100)existentNodes.push_back(*fRight);
                if(fLeft->getPos_X()!=100)existentNodes.push_back(*fLeft);

                return right;
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

                //region(fRight->getPos_X(),fRight->getPos_Y(),cX,cY);
                fRight->setVisited(1);

                if(fRight->getPos_X()!=100)existentNodes.push_back(*fRight);
                if(left->getPos_X()!=100)existentNodes.push_back(*left);
                if(fLeft->getPos_X()!=100)existentNodes.push_back(*fLeft);
                if(front->getPos_X()!=100)existentNodes.push_back(*front);
                if(right->getPos_X()!=100)existentNodes.push_back(*right);

                return fRight;
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

                //region(front->getPos_X(),front->getPos_Y(),cX,cY);
                front->setVisited(1);

                //existentNodes.push_back(*front);
                if(front->getPos_X()!=100)existentNodes.push_back(*front);
                if(right->getPos_X()!=100)existentNodes.push_back(*right);
                if(left->getPos_X()!=100)existentNodes.push_back(*left);
                if(fRight->getPos_X()!=100)existentNodes.push_back(*fRight);
                if(fLeft->getPos_X()!=0)existentNodes.push_back(*fLeft);

                return front;
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

                //region(fLeft->getPos_X(),fLeft->getPos_Y(),cX,cY);
                fLeft->setVisited(1);

                if(fLeft->getPos_X()!=100)existentNodes.push_back(*fLeft);
                if(right->getPos_X()!=100)existentNodes.push_back(*right);
                if(fRight->getPos_X()!=100)existentNodes.push_back(*fRight);
                if(front->getPos_X()!=100)existentNodes.push_back(*front);
                if(left->getPos_X()!=100)existentNodes.push_back(*left);

                //existentNodes.push_back(*fLeft);

                return fLeft;
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

                //region(left->getPos_X(),left->getPos_Y(),cX,cY);
                left->setVisited(1);

                existentNodes.push_back(*left);
                existentNodes.push_back(*front);
                existentNodes.push_back(*right);
                //existentNodes.push_back(*left);
                existentNodes.push_back(*fRight);
                existentNodes.push_back(*fLeft);

                return left;
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
            cout<<"Antes del padre"<<endl;
            father = current->getFather();
            if(father == nullptr){
              cout<<"padre nulo"<<endl;
              return father;
            }
            cout<<"después del padre"<<endl;
            markerCand(father);
            state = ToCand;

            //region(father->getPos_X(),father->getPos_Y(),cX,cY);
            return father;
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
        createdNodes++;
        marker.publish( mark );
      }
    }

    float calculateCandidate(Node *node)
    {
      return sqrt((pow(desPosX - node->getPos_X(),2)) + (pow(desPosY - node->getPos_Y(),2)));
    }

    float calculateCandidateN(Node *node, float x, float y)
    {
      return sqrt((pow(x - node->getPos_X(),2)) + (pow(y - node->getPos_Y(),2)));
    }

    void rrt()
    {
      //Node *cand = new Node();
      //cout<<"calculó región"<<endl;
      if(fNode && laserReady && odomReady)
      {
        //graph->insertNode(*nodeInit);
        Node *f = new Node (-1,-1);
        //initial->setFather(f);
        cand=calculateStates(nodeInit);
        if(cand == NULL) {
          isFinished = true;
          cout<<"Cambio a true"<<endl;
        }
        else {
          graph->setCNode(cand);
          //cout<<"calculó estados"<<endl;
          cout<<"Cand: "<<cand->getPos_X()<<", "<<cand->getPos_Y()<<endl;
          //graph->insertRight(nodeInit,nodeInit);
          fNode=false;
          //calculateStates(nodeInit);
          state=ToCand;
        }
      }
      else if(laserReady && odomReady)
      {
        goalReached(cX,cY);
        if(state!=Goal)
        {

          if(state==ToCand)
          {
            if(cand == NULL){
              cout<<"candidato nulo"<<endl;
            }

            else{
              align(cand->getPos_X(), cand->getPos_Y());
              moveTo(cand->getPos_X(), cand->getPos_Y());
            }
          }

          //cout<<"Entró "<<endl;

          if(state== CandReached)
          {
            Node *p = new Node ();
            Node *cu = new Node ();
            cu = graph->getCNode();


            cout<<"calcula nuevo cand"<<endl;
            //Node * state = new Node (cand->getPos_X(), cand->getPos_Y());

            //bool v=true;
            //cu->setVisited(1);
            cand=calculateStates(cu);
            if(cand == NULL) {
              isFinished = true;
              cout<<"Cambio a true"<<endl;
            }
            else{
              cout<<"Cand: "<<cand->getPos_X()<<", "<<cand->getPos_Y()<<endl;
              graph->setCNode(cand);
              p = cand->getFather();
              //*cu = graph->getCNode();
              if(p != NULL){
                cout<<"PadreCand: "<<p->getPos_X()<<", "<<p->getPos_Y()<<endl;
                cout<<"Current: "<<cu->getPos_X()<<", "<<cu->getPos_Y()<<endl;
                //odomReady=false;
              }
              //state = ToCand;
            }

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
        else {
          isFinished = true;
        }
      }
    }

    void shutDownSubscribers(ros::Subscriber sub, ros::Subscriber sub1)
    {
      sub.shutdown();
      sub1.shutdown();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      //if(!aligned) {
        //cout<<"Start odometry"<<endl;
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
        odomReady = true;


        //cout<<"Before rrt"<<endl;
        rrt();
        //cout<<"After rrt"<<endl;
        //cout<<"Finish Odometry"<<endl;
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
  //ros::Subscriber sub  = nh.subscribe(/*"scan"*/ "p3dx/laser/scan", 30, &SRTExp::laserCallback,driver);
  //ros::Subscriber sub1 = nh.subscribe(/*"RosAria/pose"*/ "odom", 30, &SRTExp::odomCallback,driver);
  ros::Subscriber sub  = nh.subscribe("scan", 30, &SRTExp::laserCallback,driver);
  ros::Subscriber sub1 = nh.subscribe("RosAria/pose", 1, &SRTExp::odomCallback,driver);
  //ros::Subscriber sub2 = nh.subscribe("/robot_pose_ekf/odom_combined", 10, &SRTExp::odomCallback,driver);
  //ros::Timer timer2 = nh.createTimer(ros::Duration(1),&SRTExp::rrt,driver);
  //ros::spin();
  ros::Rate loop_rate(1.5); // 5Hz

    while (ros::ok())
    {

      //driver->rrt();
      if(driver->isFinished){
        driver->shutDownSubscribers(sub,sub1);
        ros::shutdown();
      }

      loop_rate.sleep();
      ros::spinOnce();
    }
    cout<<"Finish"<<endl;
}

#ifndef NODE_H
#define NODE_H

#include "ros/ros.h"
#include <iostream>

using namespace std;

class Node
{
  private:

    float pos_X;
    float pos_Y;

    bool hasChildren;
    int isFree;
    Node *father;
    Node *right;
    Node *fRight;
    Node *front;
    Node *fLeft;
    Node *left;

  public:

    int visited;
    Node();
    Node(float x, float y);
    ~Node();
    void  setPos_X(float x);
    float getPos_X();
    void  setPos_Y(float y);
    float getPos_Y();
    void  setVisited(int v);
    int  getVisited();
    void  setHasChildren(bool hC);
    bool  getHasChildren();
    void  setIsFree(int iFree);
    int   getIsFree();
    Node  *getFather();
    void  setFather(Node *father);
    void  setRight(Node *right);
    Node  *getRight();
    void  setFRight(Node *fRight);
    Node  *getFRight();
    void  setFront(Node *front);
    Node  *getFront();
    void  setFLeft(Node *fLeft);
    Node  *getFLeft();
    void  setLeft(Node *left);
    Node  *getLeft();
};

/*
 * Esta clase definira los nodos de las posiciones de los caminos recoridos
 * */

#endif // NODE_H

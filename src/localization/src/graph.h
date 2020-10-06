#ifndef GRAPH_H
#define GRAPH_H

#include "ros/ros.h"

#include <iostream>
#include <list>
#include <iterator>
#include "node.h"

using namespace std;

class Graph
{
  private:
      Node *root;
      Node *cNode;
      //list <Node> list_Nodes;

  public:

      list <Node> list_Nodes;
      Graph();
      Graph(Node *root);
      ~Graph();
      Node *getCNode();
      Node getRoot();
      void setCNode(Node *nCurrent);
      void insertNode(Node node);
      void insertRight(Node *current, Node *right);
      void insertFRight(Node *current, Node *fRight);
      void insertFront(Node *current, Node *front);
      void insertFLeft(Node *current, Node *fLeft);
      void insertLeft(Node *current, Node *left);
      void showList();

};

/*
 * Esta clase contiene el grafo de los nodos recorridos por el robot
 */


#endif // GRAPH_H

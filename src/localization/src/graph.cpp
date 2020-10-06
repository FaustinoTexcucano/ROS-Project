#include "graph.h"

Graph::Graph(){}

Graph::Graph(Node *root)
{
  this->root = root;
  this->cNode = root;
}

Graph::~Graph(){}

Node *Graph::getCNode()
{
  return cNode;
}

Node Graph::getRoot()
{
  return *root;
}

void Graph::setCNode(Node *nCurrent)
{
  this->cNode=nCurrent;
}

void Graph::insertNode(Node node)
{
  list_Nodes.push_back(node);
}

void Graph::insertRight(Node *current, Node *right)
{
  current->setRight(right);
  right->setFather(current);
}

void Graph::insertFRight(Node *current, Node *fRight)
{
  current->setFRight(fRight);
  fRight->setFather(current);
}

void Graph::insertFront(Node *current, Node *front)
{
  current->setFront(front);
  front->setFather(current);
}

void Graph::insertFLeft(Node *current, Node *fLeft)
{
  current->setFLeft(fLeft);
  fLeft->setFather(current);
}

void Graph::insertLeft(Node *current, Node *left)
{
  current->setLeft(left);
  left->setFather(current);
}

void Graph::showList()
{
  int i= 0;

  list<Node>::iterator it = list_Nodes.begin();
  while( it != list_Nodes.end() )
  {
    cout<<"Posicion: "<<it->getPos_X()<<endl;
    cout<<"Posicion: "<<it->getPos_Y()<<endl;
    cout<<" "<<i<<endl;
    it++;
    i++;
  }
}

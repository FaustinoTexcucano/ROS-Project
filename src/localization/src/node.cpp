#include "node.h"

Node::Node(){

  this->visited = 0;
  this->hasChildren=false;
  this->isFree=-1;

  father = NULL;
  right = NULL;
  fRight = NULL;
  front = NULL;
  fLeft = NULL;
  left = NULL;
}

Node::Node(float x, float y)
{
  this->pos_X = x;
  this->pos_Y = y;

  this->visited=0;
  this->hasChildren=false;
  this->isFree=-1;


  father = NULL;
  right = NULL;
  fRight = NULL;
  front = NULL;
  fLeft = NULL;
  left = NULL;
}

Node::~Node(){}

void Node::setPos_X(float x)
{
  this->pos_X = x;
}

float Node::getPos_X()
{
  return pos_X;
}

void Node::setPos_Y(float y)
{
  this->pos_Y = y;
}

float Node::getPos_Y()
{
  return pos_Y;
}

void Node::setVisited(int v)
{
  this->visited = v;
}

int Node::getVisited()
{
  return visited;
}

void Node::setHasChildren(bool hC)
{
  this->hasChildren = hC;
}

bool Node::getHasChildren()
{
  return hasChildren;
}

void Node::setIsFree(int iFree)
{
  this->isFree = iFree;
}

int Node::getIsFree()
{
  return isFree;
}

Node *Node::getFather()
{
  if(father==NULL)
    cout<<"NULL Node"<<endl;
  return father;
}

void Node::setFather(Node *father)
{
  this->father = father;
}

void Node::setRight(Node *right)
{
  this->right = right;
}

Node *Node::getRight()
{
  if(right==NULL)
    cout<<"NULL"<<endl;
  return right;
}

void Node::setFRight(Node *fRight)
{
  this->fRight = fRight;
}

Node *Node::getFRight()
{
  return fRight;
}

void Node::setFront(Node *front)
{
  this->front = front;
}

Node *Node::getFront()
{
  return front;
}

void Node::setFLeft(Node *fLeft)
{
  this->fLeft = fLeft;
}

Node *Node::getFLeft()
{
  return fLeft;
}

void Node::setLeft(Node *left)
{
  this->left = left;
}

Node *Node::getLeft()
{
  return left;
}

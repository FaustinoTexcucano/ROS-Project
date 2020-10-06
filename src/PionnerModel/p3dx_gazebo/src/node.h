#ifndef NODE_H
#define NODE_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "tf/tf.h"
#include <iostream>

#define k 5
#define safeDistanceUncles 1.0       //Distancia segura de cada nodo 1.5

using namespace std;

class Node
{
  private:
    //Punto Actual (contiene la informacion de la posicion x,y)
    float x, y;
    //Apuntador de arreglo de objetos de los Puntos candidatos o Nodos Hijos
    Node *children;

  public:
    Node();
    ~Node();

    //Referencia al Nodo padre
    Node *father;
    //Boolean que dice si el nodo ya ha sido visitado
    bool visited = false;
    //Boolean para saber si tiene hijos
    bool hasChildren = false;
    //Boolean que determina si el nodo es accecible o no
    bool accessible = false;
    //Indice o Numero de hijo que es
    int numberChild;

    float getX();
    float getY();
    void setX(float px);
    void setY(float py);
    void showInfo();
    void setChildren(float px[], float py[]);
    bool safeRegion(float x, float y);
    float getEuclideanDistance(float x, float y);
    Node *getChild(int i);
    Node *getFather();


};

/*
 * Esta clase definira los nodos de las posiciones de los caminos recoridos
 * */

#endif // NODE_H

#ifndef GRAPH_H
#define GRAPH_H

#include "ros/ros.h"

#include <iostream>
#include<cstdlib>
#include<time.h>
#include <queue>
#include <vector>

#include "node.h"

//Variables de las coordenadas Meta del algoritmo SRT
#define X_goal 5.0//-6.0//2.5
#define Y_goal 1.0//1.0//2.0

//Region segura para cada nodo ya visitado distancia safe 1.4
#define safeDistanceNodes 1.4//0.4//1.0//0.25//1.0     //1.0     1.4 para distancias grandes
//Para robot real
//0.4 de region segura
//0.7 min Distancia
//0.7 max distancia

using namespace std;

class Graph
{
  private:
    //Nodo Raiz   o Nodo Padre Actual
    Node *rootNode;
    //Nodo hijo Actual (Nodo candidato)
    Node *currentNode;
    //Indice del nodo seleccionado
    int childSelected;

  public:
    //Primer Nodo del grafo (0.0)
    Node *firstNode;

    //Cola de puntos que debe seguir para hacer el backtraking
    queue <geometry_msgs::Point> listPoints;

    //Lista de todos los Nodos del Arbol
    vector <Node*> listNodes;

    Graph();
    ~Graph();
    Node *insertNode(float x[], float y[]);
    Node *selectChild();
    Node *getCurrentNode();
    void backtracking();
    int checkChildren();
    bool safeRegionPath(float x, float y);
    void showGraph(Node *node);
    void showPath();
};

/*
 * Esta clase contiene el grafo de los nodos recorridos por el robot
 */


#endif // GRAPH_H

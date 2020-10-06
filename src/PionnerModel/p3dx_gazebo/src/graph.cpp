#include "graph.h"

Graph::Graph()
{
  rootNode = NULL;
  currentNode = NULL;
  firstNode = NULL;
}

Graph::~Graph(){}

//Metodo que inserta los hijos candidatos y devuelve un hijo el cual es el nodo actual
Node *Graph::insertNode(float x[], float y[])
{
  //Se planta la semilla para elegir el aleatorio
  srand(time(NULL));
  //cout<<"Numero de Nodes "<<listNodes.size()<<endl;

  //Si no existe una raiz se crea una nueva
  if(rootNode == NULL)
  {
    Node *newNode;
    newNode = new Node();
    newNode->setX(0.0);
    newNode->setY(0.0);
    //  [newNodo] -> Null       Nodo Raiz
    newNode->father = NULL;    //Apunta a Null por que no tiene padre
    //Insertamos el Nodo de inicio (0,0)
    listNodes.push_back(newNode);
    Node *aux;
    //Se insertan todos los nodos a la lista de nodos   lo unico que importa son las posiciones
    for(int i=0; i<k; i++)
      if(x[i] != 9999 && y[i] != 9999)
      {
        aux = new Node();
        aux->setX(x[i]);
        aux->setY(y[i]);
        listNodes.push_back(aux);
      }

    newNode->setChildren(x, y);
    //Se marca que ya tiene hijos
    newNode->hasChildren = true;
    newNode->visited = true;
//Insertamos el Nodo de inicio (0,0)
//listNodes.push_back(newNode);
    //Asignamos el newNodo a FirstNode para saber cual es el nodo creador de todo
    firstNode = newNode;
    //Seconvierte en padre
    rootNode = newNode;
    // [currentNode] -> [newNode].getChild
    //Se convierte en el nodo padre de los siguientes hijos
    //Se envia como parametro un hijo seleccionado
    currentNode = selectChild();
    //Se le asigna el que es su Nodo Padre el cual contiene los nodos tios
    currentNode->father = rootNode;
//Insertamos el nodo candidato a la lista
//listNodes.push_back(currentNode);

    //Se marca como visitado
    currentNode->visited = true;

    return currentNode;
  }
  else
  {
    Node *aux;
    //Se insertan todos los nodos a la lista de nodos   lo unico que importa son las posiciones
    for(int i=0; i<k; i++)
      if(x[i] != 9999 && y[i] != 9999)
      {
        aux = new Node();
        aux->setX(x[i]);
        aux->setY(y[i]);
        listNodes.push_back(aux);
      }

    //Nodo Actual
    //Se le asigna el que es su Nodo Padre
    currentNode->father = rootNode;
    //Se asignan los nodos hijos del nodo actual
    currentNode->setChildren(x, y);
    //Se marca que ya tiene hijos
    currentNode->hasChildren = true;
    //Raiz pasa a ser el nodo acual o Nuevo Padre
    rootNode = currentNode;
    //Se actualiza el nodo actual al hijo seleccionado (candidato seleccionado)
    //Se envia como parametro un hijo seleccionado
    currentNode = selectChild();
    //Se le asigna el que es su Nodo Padre el cual contiene los nodos tios
    currentNode->father = rootNode;
//Insertamos el nodo a la lista
//listNodes.push_back(currentNode);

    //Se marca como visitado
    currentNode->visited = true;

    return currentNode;
  }

}

Node *Graph::selectChild()
{
  //Metodo para elegir el nodo mas cercano a la meta
  /*float distance = 9999;
  float disAux =0;
  int numNode;

  for(int i=0; i<5; i++)
  {
    if(rootNode->getChild(i)->accessible)
    {
      disAux = rootNode->getChild(i)->getEuclideanDistance(X_goal, Y_goal);
      if(disAux < distance)
      {
        distance = disAux;
        numNode = i;
      }
    }
  }

  return rootNode->getChild(numNode);*/

  //Metodo para elegir un nodo aleatoriamente
  int i=0;

  while(i < 15)
  {
    //Se elije un nodo hijo aleatoriamente
    childSelected = 0 + (rand() % 5);
    //Si el hijo seleccionado es accecible se retorna
    if(rootNode->getChild(childSelected)->accessible)
      return rootNode->getChild(childSelected);
    i++;
  }

  //Si no, se itera hasta que encuentre un nodo accecible
  for(int i=0; i<5; i++)
    if(rootNode->getChild(i)->accessible)
      return rootNode->getChild(i);
}

Node *Graph::getCurrentNode()
{
  return currentNode;
}

void Graph::backtracking()
{
  int numChild = checkChildren();
  //Si todos los nodos hijos aun no estan visitados
  if(numChild != -1)
  {
    //Agrego a la lista el Punto padre
    geometry_msgs::Point father = geometry_msgs::Point();
    father.x = rootNode->getX();
    father.y = rootNode->getY();
    listPoints.push(father);

    //El nodo actual se convierte en un nodo hermano
    currentNode = rootNode->getChild(numChild);
    //Actualizamos el padre del nodo Actual
    currentNode->father = rootNode;
    geometry_msgs::Point brother = geometry_msgs::Point();
    brother.x = currentNode->getX();
    brother.y = currentNode->getY();
    //Agrego a la lista el punto hermano
    listPoints.push(brother);
//Agrego el punto destino a la lista de los visitados
//listNodes.push_back(currentNode);
    //Se marca como visitado
    currentNode->visited = true;

    //return listPoints;
  }
  else
  {
    geometry_msgs::Point grandFather = geometry_msgs::Point();
    grandFather.x = rootNode->getX();
    grandFather.y = rootNode->getY();
    //cout<<"X= "<<rootNode->getX()<<" Y= "<<rootNode->getY()<<endl;
    //Agrego a la lista el punto padre
    listPoints.push(grandFather);
    //El nodo actual se convierte en su padre
    currentNode = rootNode;

    if(currentNode->father != NULL)
    {
      //La raiz se convierte en el nodo padre del nodo aactual
      rootNode = currentNode->father;
      //Se realiza el mismo proceso
      backtracking();
    }
    else
      cout<<"Hola Mundo"<<endl;
  }
}

int Graph::checkChildren()
{
  //Verificamos si existen nodos hermanos que aun no han sido visitados y que sean accesibles
  for(int i=0; i<5; i++)
    if(rootNode->getChild(i)->visited == false && rootNode->getChild(i)->accessible == true)
      return i;
  return -1;
}

//Metodo que verifica la si el nodo candidato pertenece a la zona segura de un nodo ya visitado
bool Graph::safeRegionPath(float x, float y)
{
  vector<Node*>::const_iterator node;
  if(!listNodes.empty())
    for (node = listNodes.begin(); node != listNodes.end(); node++)
      if((*node)->getEuclideanDistance(x, y) <= safeDistanceNodes)
        return false;
  return true;
  //cout <<"Node X= "<<(*node)->getX()<<" Y= "<<(*node)->getY()<<endl;
}

void Graph::showGraph(Node *node)
{
  if(node->hasChildren)
    for(int i=0; i<5; i++)
    {
      node->getChild(i)->showInfo();
      showGraph(node->getChild(i));
    }
}

void Graph::showPath()
{
  Node *path = rootNode;
  cout<<"\nYo soy el ultimo nodo"<<endl;
  cout<<"X = "<<currentNode->getX()<<"  Y = "<<currentNode->getY()<<endl;
  cout<<"\nListado de los Elementos del Arbol\n"<<endl;
  while(path != NULL)
  {
    path->showInfo();
    path = path->father;
  }
}

#include "node.h"

Node::Node()
{
  father = NULL;
}

//Metodos get and set de la clase Node
float Node::getX()
{
  return x;
}

float Node::getY()
{
  return y;
}

void Node::setX(float px)
{
   x = px;
}

void Node::setY(float py)
{
  y = py;
}

//Metodo que actualiza el valor de los hijos del nodo y determina si son nodos accesibles o no
void Node::setChildren(float px[], float py[])
{
  //Se crea el arreglo de tipo Nodo de Tamaño K
  children = new Node[k];
  for(int i=0; i<k; i++)
  {//Si no tiene colision con un obstaculo
    if(px[i] != 9999 && py[i] != 9999)
    {
      //Como children es apuntador se puede hacer operaciones con el le asignamos sus coordendas, lo volvemos accecible y le asignamos que numero de hijo es
      (children+i)->setX(px[i]);
      (children+i)->setY(py[i]);
      (children+i)->accessible = true;
      (children+i)->numberChild = i;
    }
  }
}

//Metodo que verifica si el nodo hijo candidato esta dentro de la zona segura de su abuelo Tomando como region segura 1.5 m
bool Node::safeRegion(float x, float y)
{
  /*Node *auxFather;
  float range;
  if(father != NULL)
  {
    //Asignamos el nodo abuelo
    auxFather = getFather();
    //verificamos si hay tios
    if(auxFather->hasChildren)
    {
      for(int i=0; i<5; i++)
      {
        //Revisamos si el tio es accesible y el numero de hijo es distinto al del nodo actual
        if(auxFather->getChild(i)->accessible)
        //if(auxFather->getChild(i)->accessible && i!=numberChild)
        {
            //Comparamos con la region segura de los tios
            range = sqrt((pow(auxFather->getChild(i)->getX() - x, 2))+(pow(auxFather->getChild(i)->getY() - y, 2)));
            if(range < safeDistanceUncles)
              return false;
         }
       }
       return true;
     }
  }
  else
    return true;*/

  //Comparacion con la lista de todos los Nodos
  /*vector<Node*>::const_iterator node;
  if(!listNodes.empty())
    for (node = listNodes.begin(); node != listNodes.end(); node++)
      if((*node)->getEuclideanDistance(x, y) <= 1.1)
        return false;*/
  return true;
}

//Metodo que determina la dstancia entre 2 puntos
float Node::getEuclideanDistance(float x, float y)
{
  return sqrt((pow(x - getX(), 2))+(pow(y - getY(), 2)));
}

//Este metodo retorna un apuntador te tipo nodo
//Retorna uno de los hijos del nodo
Node *Node::getChild(int i)
{
  return (children+i);
}

//Metodo que retorna el nodo padre
Node *Node::getFather()
{
  return father;
}

//Metodo que muestra la informacion del nodo
void Node::showInfo()
{
  cout<<"\n\nYo soy el Nodo \n x = "<<getX()<<"  y = "<<getY()<<"\n"<<endl;
  if(father != NULL)
    cout<<"Mi padre es X = "<<father->getX()<<"   Y = "<<father->getY()<<endl;
  else
    cout<<"Yo soy huerfano"<<endl;
  if(hasChildren)
  {
    cout<<"Mis Nodos Hijos son"<<endl;
    for(int i=0; i<k; i++)
      cout<<"Hijo "<<i<<" x = "<<(children+i)->getX()<<"  y = "<<(children+i)->getY()<<endl;
  }
  else
    cout<<"No tengo bendiciones"<<endl;
}


Node::~Node(){}

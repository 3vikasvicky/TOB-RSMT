#ifndef TEMP_H //Avoid duplicate definition
#define TEMP_H

#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;


class Event
{
public:
	Event(int X,int Y,int Layer,bool LeftmostOfEdge,SGEdge* refEdge) : x(X) , y(Y) , layer(Layer) ,State(LeftmostOfEdge) , E(refEdge)  {}
	int getX() const {return x;} //Read only
	int getY() const {return y;}
	int getLayer() const {return layer;}
	void setX(int val) { x=val; } 
	void setY(int val) { y=val;}
	void setLayer(int val) { layer=val;}
	
	bool LeftmostOfEdge() {return State;}

	SGEdge* getEdge() {return E;}
	
private:
	int x;
	int y;
	int layer;
	bool State; //true for beginning of edge, false for beginning of edge //true for leftmost point of edge
	SGEdge* E;
};

class Event_XLessThan //Using function object instead of function pointer because of inlining effect
{	public:
		bool operator() (Event* E1,Event* E2)
		{	
			return E1->getX() < E2->getX();
		}
};

#endif
#include <stdio.h>
#include <iostream>
#include <string>
#include "basic.h"
#include "database.h"

using namespace std;

void Database::AddPin(const X& x,const Y& y,const Layer& layer,double req_time,double sink_loading) 
{
	
	//--------------------------------------------------------
	//traverse the database to avoid adding duplicated node
	//because test case in C tree has this situation .....
	//--------------------------------------------------------
	for(int i=0;i<this->vPin.size();i++)
	{	Pin* P=this->GetPin(i);
		
		if(P->getX()==x.val && P->getY()==y.val)
		{	cout<<"catch wrong node ("<<x.val<<","<<y.val<<")\n";
			exit(0);
		}
	}
	


	vPin.push_back(new Pin(x,y,layer,req_time,sink_loading));
}

size_t Database::GetPinSize() 
{ 
	return vPin.size(); 
}

Pin* Database::GetPin(size_t index) 
{ 
	if(index >= vPin.size()) 
	{ 
		cout<<"GetPin's index exceeds the size of vPin !\n"; 
		exit(0); 
	}
	else 
	{ 
		return vPin[index]; 
	}
}

void Database::AddObstacle(const X1& x1,const Y1& y1,const X2& x2,const Y2& y2,const Layer& layer) 
{
	vObstacle.push_back(new Obstacle(x1,y1,x2,y2,layer));
}

size_t Database::GetObstacleSize() 
{ 
	return vObstacle.size(); 
}

Obstacle* Database::GetObstacle(size_t index) 
{ 
	if(index >= vObstacle.size()) 
	{ 
		cout<<"GetObstacle's index exceeds the size of vObstacle !\n"; exit(0); 
	}
	else 
	{ 
		return vObstacle[index]; 
	}
}

void Database::SortPin_ByX()	//encapsulation !
{	sort(vPin.begin(),vPin.end(),PinLessThan());
}

void Database::SortObstacle_ByX1() //encapsulation !
{	sort(vObstacle.begin(),vObstacle.end(),ObstacleLessThan());
}

//use for doing Prim's algorithm
void Database::build_ObstacleRTree()
{
	//Adding Obstacle
	for(size_t i=0;i<GetObstacleSize();i++)
	{	
		Obstacle* O = GetObstacle(i);
		//Adding SGNode into Region Tree
		RTreeInsertObstacle(O);	
	}
}
#ifndef DATABASE_H //Avoid duplicate definition
#define DATABASE_H

#include <vector>
#include <stdio.h>
#include <algorithm>
#include "basic.h"
#include "RTree.h"
#include "RunTime.h"
#include "definition.h"


using namespace std;

class Database;
class OARSteinerTree;
class elmoreDelay;

void readfile(string filename,Database& database,elmoreDelay* elmoreModel);
#ifdef BUF_DRIVEN
void readBufFile(char* filename,Database& database,elmoreDelay* elmoreModel);
#endif
void set_driver_arr_0(Database& database,elmoreDelay* elmoreModel);

class Database {

public:
	void AddPin(const X& x,const Y& y,const Layer& layer,double req_time,double sink_loading);
	size_t GetPinSize();
	Pin* GetPin(size_t index);

	Pin* getDriver() { return DrivingNode; }
	void setDriver(Pin* Driving) { DrivingNode=Driving; }

	void AddObstacle(const X1& x1,const Y1& y1,const X2& x2,const Y2& y2,const Layer& layer);
	size_t GetObstacleSize();
	Obstacle* GetObstacle(size_t index);

	void SortPin_ByX();
	void SortObstacle_ByX1();

	Database() : minX(0),minY(0),maxX(0),maxY(0),topLayer(_UNSET),buttomLayer(_UNSET),numRTreeObstacles(0),DrivingNode(NULL) { 
	  RunTimeSet = new RunTime(); 
#ifdef PRIORITY_BASED
	  tempMaxSlack=_NINFINITE;
	  tempMinSlack=_INFINITE;
#endif
	}
#ifdef PRIORITY_BASED
	void setTempMaxSlack(double s) { tempMaxSlack=s; }
	double getTempMaxSlack() { return tempMaxSlack; }
	void setTempMinSlack(double s) { tempMinSlack=s; }
	double getTempMinSlack() { return tempMinSlack; }
#endif

#ifdef BUF_DRIVEN
	void setCapUpperBound(double ub) { capUpperBound=ub; }
	double getCapUpperBound() { return capUpperBound; }
#endif

	//use for doing Prim's algorithm
	void build_ObstacleRTree();
	void RTreeInsertObstacle(Obstacle* obstacle);
	//int obstacleRTreeSearch(const Point *P11,const Point *P22,bool __cdecl a_resultCallback(int a_data, void* a_context));
	int obstacleRTreeSearch(const Point *P11,const Point *P22,bool a_resultCallback(int a_data, void* a_context));

	int getMinX() { return minX; }
	int getMinY() { return minY; }
	int getMaxX() { return maxX; }
	int getMaxY() { return maxY; }
	int getTopLayerNum() { return topLayer; }
	int getButtomLayerNum() { return buttomLayer; }

	int getSecXLength() { return secXLength; }
	int getSecYLength() { return secYLength; }

	int getObstacleNum() { return numObstacle; }
	int getPinNum() { return numPin; }


	void setObstacleNum(int val) { numObstacle=val; }
	void setPinNum(int val) { numPin=val; }

	void setMinX(int val) { minX=val; }
	void setMinY(int val) { minY=val; }
	void setMaxX(int val) { maxX=val; }
	void setMaxY(int val) { maxY=val; }
	void setTopLayerNum(int val) { topLayer=val; }
	void setButtomLayerNum(int val) { buttomLayer=val; }

	void setSecXLength(int val) { secXLength=val; }
	void setSecYLength(int val) { secYLength=val; }

	RunTime* getRunTimeSet() { return RunTimeSet; }

private:
	vector<Pin*> vPin;	//Save input data in this container (user will never know we store it as a pointer : encapsulation !)
	vector<Obstacle*> vObstacle; //Save input data in this container
	Pin* DrivingNode; 

	RunTime* RunTimeSet;

	RTree<int,int,3,float> obstacleRTree; //The purpose of seperating R-Tree is that we want to speed up the time of querying obstacle
	int numRTreeObstacles; //stands for the number of nodes in R Tree

	int minX,minY,maxX,maxY,topLayer,buttomLayer; //
	int numPin,numObstacle;

	int secXLength,secYLength; //for drawing grid on display window

#ifdef PRIORITY_BASED
	double tempMaxSlack, tempMinSlack;
#endif

#ifdef BUF_DRIVEN
	double capUpperBound;
#endif
	
//-----------------------------------------------------------------------------
//database.h
//-----------------------------------------------------------------------------
	class PinLessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const Pin* P1,const Pin* P2)
			{	return P1->getX() < P2->getX();
			}
	};

	class ObstacleLessThan
	{	public:
			bool operator() (const Obstacle* O1,const Obstacle* O2)
			{	return O1->getX1() < O2->getX1();
			}
	};
};


#endif

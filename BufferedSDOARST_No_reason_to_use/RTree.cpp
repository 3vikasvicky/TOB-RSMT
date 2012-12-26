#include "OARST.h"
#include "OASG.h"

struct Rect
{
  Rect()  {}

  Rect(int a_minX, int a_minY, int a_maxX, int a_maxY,int Z)
  {
    min[0] = a_minX;
    min[1] = a_minY;
	min[2] = Z;

    max[0] = a_maxX;
    max[1] = a_maxY;
	max[2] = Z;
  }


  int min[3];
  int max[3];
};



void OASpanningGraph::RTreeInsertNode(SGNode* node)
{
  int min[3],max[3];
  
  min[0] = node->getX();
  min[1] = node->getY();
  min[2] = node->getLayer();
  max[0] = node->getX();
  max[1] = node->getY();
  max[2] = node->getLayer();
  
  
  nodeRTree.Insert(min,max,numRTreeNodes); 

  numRTreeNodes++; //increment the number of saved nodes
}

void Database::RTreeInsertObstacle(Obstacle* obstacle)
{
  int min[3],max[3];
  
  min[0] = obstacle->getX1();
  min[1] = obstacle->getY1();
  min[2] = obstacle->getLayer();
  max[0] = obstacle->getX2();
  max[1] = obstacle->getY2();
  max[2] = obstacle->getLayer();

#ifdef DEBUG_RTREE_INSERT_OBSTACLE
  cout<<"Add "<<numRTreeObstacles<<" obstacle, ("<<obstacle->getX1()<<","<<obstacle->getY1()<<") ("<<obstacle->getX2()<<","<<obstacle->getY2()<<") L:"<<obstacle->getLayer()<<endl;
#endif

  Rect R(obstacle->getX1(),obstacle->getY1(),obstacle->getX2(),obstacle->getY2(),obstacle->getLayer());
  
  //obstacleRTree.Insert(min,max,numRTreeObstacles); 
  obstacleRTree.Insert(R.min,R.max,numRTreeObstacles); 

  numRTreeObstacles++; //increment the number of saved nodes
}

//int OASpanningGraph::nodeRTreeSearch(const Point *P11,const Point *P22, bool __cdecl a_resultCallback(int a_data, void* a_context))
int OASpanningGraph::nodeRTreeSearch(const Point *P11,const Point *P22, bool a_resultCallback(int a_data, void* a_context))
{
  int min[3],max[3];
  
  min[0] = P11->getX();
  min[1] = P11->getY();
  min[2] = P11->getLayer();
  max[0] = P22->getX();
  max[1] = P22->getY();
  max[2] = P22->getLayer();

  // NOTE: May want to return search result another way, perhaps returning the number of found elements here.

  int nhits = nodeRTree.Search(min,max,a_resultCallback,NULL);

  return nhits;
}

//int Database::obstacleRTreeSearch(const Point *P11,const Point *P22, bool __cdecl a_resultCallback(int a_data, void* a_context))
int Database::obstacleRTreeSearch(const Point *P11,const Point *P22, bool a_resultCallback(int a_data, void* a_context))
{
  int min[3],max[3];
  
  min[0] = P11->getX();
  min[1] = P11->getY();
  min[2] = P11->getLayer();
  max[0] = P22->getX();
  max[1] = P22->getY();
  max[2] = P22->getLayer();

  // NOTE: May want to return search result another way, perhaps returning the number of found elements here.

  Rect search_rect(min[0],min[1],max[0],max[1],P11->getLayer());

#ifdef DEBUG_RTREE_SEARCH_OBSTACLE
  cout<<"\nNow search region ("<<search_rect.min[0]<<","<<search_rect.min[1]<<","<<search_rect.max[0]<<","<<search_rect.max[1]<<") Layer="<<search_rect.min[2]<<"\n";
#endif

  int nhits = obstacleRTree.Search(search_rect.min,search_rect.max,a_resultCallback,NULL);

#ifdef DEBUG_RTREE_SEARCH_OBSTACLE
  printf("Search resulted in %d hits\n", nhits);
#endif

  return nhits;
}
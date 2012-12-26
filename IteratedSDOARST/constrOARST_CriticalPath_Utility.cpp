#include "OARST.h"

void OARSteinerTree::GrowOneCriticalPath(SGNode* Sink)
{
  if(Sink->getType()!=_PIN)
  {	
    cout<<"Grow critical path must start from a pin\n";
    exit(0);
  }

  SGNode* N = Sink;		

#ifdef PRIORITY_BASED
  double MAXPRI = this->getDelayModel()->getBestPrioritySinkInOASG()->getPriority() ;
  double REFPRI = Sink->getPriority() ;
#endif

  while(N != this->getDrivingNode())
  {
    //select next internode
    SGNode* nextN = NULL;
    SGEdge* connectEdge = NULL;

    for(int j=0;j<N->getNumEdge();j++)
    {
      if(N->getEdge(j)->checkRouted())
      {
	if(N->getEdge(j)->getToNode()->getDistFromDriving() < N->getDistFromDriving())
	{
	  if(nextN == NULL)
	  {	
	    nextN = N->getEdge(j)->getToNode();
	    connectEdge = N->getEdge(j);
	  }
	  else
	  {
	    cout<<"\n--[Grow One Critical Path]\n";					
	    cout<<"Error occured in OARSteinerTree::GrowOneCriticalPath()\n";
	    exit(0);
	  }
	}
      }
    }

    if(nextN==NULL)
    {	
      cout<<"\n--[Grow One Critical Path]\n";					
      cout<<"Error occured in OARSteinerTree::GrowOneCriticalPath()\n";
      cout<<"nextN cannot be NULL\n";
      exit(0);
    }

    //process passing edge and node
    N->setInCriticalPath(true);
#ifdef PRIORITY_BASED
    //N->setDPF( ((REFPRI)/MAXPRI)*(N->getReferenceRadius()/this->maxRadiusInOASG)) ;
    N->setDPF(N->getReferenceRadius()/this->maxRadiusInOASG);
#else
    N->setDPF(N->getReferenceRadius()/this->maxRadiusInOASG);
#endif
    connectEdge->setInCriticalPath(true);
    connectEdge->getDualEdge()->setInCriticalPath(true);					

    N = nextN;
  }
}

void OARSteinerTree::Restore_exceptCriticalPath()
{
	//cout<<"--[re-initialize sink's members]\n";
	for(int i=0;i<this->OASG->getNumNode();i++)
	{
		SGNode* N = this->OASG->getNode(i);

		if(N->checkInCriticalPath()==false)
		{
			//restore routing information
			N->setTreeState(false); //set node's tree state
			N->setDistFromDriving(_INFINITE); //set node's distance from driving node
			N->setDPF(0);
			N->setSID(0); //for new cost function, 6/28/2008
			              //other not in-tree sink its SID set for 0
		}

		N->setTempDistFromSource(_INFINITE);	//init for routing use
		N->setTempProCost(_INFINITE);			//init for routing use
		N->setTempSource(NULL);				    //init for routing use
		N->setTempPrevNode(NULL);				//init for routing use
		N->setTempPrevEdge(NULL);				//init for routing use
		N->setTempDistFromTree(_INFINITE);		//init for prim's algorithms
	}

	//cout<<"--[un-route routed edges (except edges in critical path)]\n"; //(except edges in critical path)
	for(int i=0;i<this->getOASG()->getNumEdge();i++)
	{	SGEdge* E = getOASG()->getEdge(i);
		if(E->checkRouted() && E->checkInCriticalPath()==false)
		{	this->UnRouteEdge(E);
		}
	}

	// Homer: Do we really need the following block?
	
	//cout<<"--[re-initialize edge's members (except edges in critical path)]\n";
	for(int i=0;i<this->OASG->getNumEdge();i++)
	{
		SGEdge* E = this->OASG->getEdge(i);
		if(E->checkInCriticalPath()==false)
		{	//restore routing information
			E->setEnable(true);
			E->setRoutingState(false);
		}
	}

	//reset driving node
	this->getDrivingNode()->setDistFromDriving(0);
	this->getDrivingNode()->setTreeState(true);
}

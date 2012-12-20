#include "OARST.h"

void elmoreDelay::Restore_Delay_Info_OASG()
{
	for(int i=0;i<this->OASG->getNumNode();i++)
	{
		SGNode* N = this->OASG->getNode(i);

		//restore delay information
		N->setDowntreamCap(0);
		N->setDelay(0);
		N->setDelay_by_Driving_Node(0);
		N->setDelay_by_Itself(0);
		N->setDelay_by_Pi_Model(0);
		N->setDelay_by_Sub_Tree(0);
	}
}

double elmoreDelay::Compute_OASG_Delay(Database &data)
{
	//restore downstream cap & others
	Restore_Delay_Info_OASG();

	//check sink's loading !=0 except driver
	//-- can be removed to speed up
	//OASG->Check_Sink_Loading();   

	//calculate downstream cap
	Compute_OASG_DownstreamCap();
	double delay_sum = 0;

	OASG->getDrivingNode()->setDelay(0); // Homer: for general cases?

	for(int i=0;i<OASG->getNumNode();i++)
	{	
		SGNode* N = OASG->getNode(i);		

		if(N->getType()==_PIN && N!=OASG->getDrivingNode())
		{			
			Compute_Sink_Delay(N);
			delay_sum = delay_sum + N->getDelay();
		}
	}

	double average_sink_delay( 0 ) ;
#ifndef PRIORITY_BASED
	SGNode* worseDelayNode = NULL;
	for(int i=0;i<OASG->getNumNode();i++)
	{	
		SGNode* N = OASG->getNode(i);		
		if(N->getType() == _PIN && N!=OASG->getDriving()&& worseDelayNode == NULL || worseDelayNode->getDelay() < N->getDelay())
		{	
		  worseDelayNode = N;
		}
	}

	average_sink_delay = (double)delay_sum/(data.getPinNum()-1);
	cout<<"\nAverage sink delay (in OASG)= "<<average_sink_delay<<endl;
	cout<<"Worse sink delay = "<<worseDelayNode->getDelay()<<" (ps)"<<endl;
	//cout<<"Worse sink delay node : leaf ? "<<worseDelayNode->checkIsLeaf()<<endl;
	//cout<<" (1) Delay_by_Pi_Model     = "<<worseDelayNode->getDelay_by_Pi_Model()<<endl;
	//cout<<" (2) Delay_by_Itself       = "<<worseDelayNode->getDelay_by_Itself()<<endl;
	//cout<<" (3) Delay_by_Sub_Tree     = "<<worseDelayNode->getDelay_by_Sub_Tree()<<endl;
	//cout<<" (4) Delay_by_Driving_Node = "<<worseDelayNode->getDelay_by_Driving_Node()<<endl;
	//PrintNode(worseDelayNode);
	setWorseDelaySinkInOASG(worseDelayNode);
#endif
	return average_sink_delay;
}

#ifdef PRIORITY_BASED
double elmoreDelay::Compute_OASG_Priority(Database& data)
{
  double WORSTPRIORITY=_INFINITE;
  double BESTPRIORITY = _NINFINITE;
  SGNode* worstPriorityNode = NULL;
  SGNode* bestPriorityNode = NULL;
  double PRIORITYSUM=0;

  // Compute the initial priority
  for(int i=0;i < OASG->getNumNode();i++)
  {
    SGNode* N = OASG->getNode(i);
    if(N->getType() == _PIN && N !=  OASG->getDrivingNode())
    //if(N->getSlack()!=_INFINITE)
    {
      assert(N->getSlack()!=_INFINITE);
      double priority = N->getSlack() - N->getDelay();
      PRIORITYSUM += priority;
      N->setPriority( priority );
      if(priority < WORSTPRIORITY)
      {
	WORSTPRIORITY=priority;
	worstPriorityNode = N;
      }
      if(priority > BESTPRIORITY)
      {
	BESTPRIORITY = priority;
	bestPriorityNode = N;
      }
    }
  }

  // Nullify priority
  /*
  for(int i = 0; i < OASG->getNumNode(); i++)
  {
    SGNode* N = OASG->getNode(i);
    if(N->getType() == _PIN && N !=  OASG->getDrivingNode())
    {
      N->setPriority( N->getPriority() - WORSTPRIORITY);
    }
  }
  */

  setWorstPrioritySinkInOASG(worstPriorityNode);
  setBestPrioritySinkInOASG(bestPriorityNode);
  setOrgWorstPriority( WORSTPRIORITY ) ;
  setOrgBestPriority( BESTPRIORITY ) ;

  double averagePriority = (double) (PRIORITYSUM / (data.getPinNum() - 1));// - WORSTPRIORITY;

  return averagePriority;
}
#endif


void elmoreDelay::Compute_Sink_Delay(SGNode* Sink)
{

	double Delay_by_Pi_Model = 0;
	double Delay_by_Itself = 0;
	double Delay_by_Sub_Tree = 0;
	double Delay_by_Driving_Node = 0;


#ifdef DEBUG_ELMORE_SINK_DELAY
	cout<<"\nNow Compute Sink Delay, Node :"<<endl;

	PrintRGNode(Sink);
#endif

	//[0] apply Pi-model formula
	double Rwire = Sink->getDistFromDriving() * this->unit_r / 1000; //(ohms/um)  //turn ohms to kilo-ohms
	double Cwire = Sink->getDistFromDriving() * this->unit_c; //(Ff/um)

	Delay_by_Pi_Model = Rwire * Cwire / 2;

	//[1] delay cause by its own loading
	//Delay_by_Itself = Rwire * this->Cloading;
	Delay_by_Itself = Rwire * Sink->getLoading();

	//--------------------------------------------------------------
	//	consider sub-tree : (to leaf or internal node, the way to compute sink delay is deffirent)
	//--------------------------------------------------------------

	//[2] Internal Node ?
	if(Sink->checkIsLeaf() == false)
	{	
		//in the beginning, consider one sub-tree rooted on itself
		//sink its own loading is considered by Delay_by_Itself (Rwire * this->Cloading)
		//so now the capacitance need to minus this->Cloading (Sink->getDowntreamCap() - this->Cloading)
		//Delay_by_Sub_Tree = Rwire * (Sink->getDowntreamCap()-this->Cloading);	
		Delay_by_Sub_Tree = Rwire * (Sink->getDowntreamCap() - Sink->getLoading());	
	}

	//[3] Trace back to Driving Node
	SGNode* N = Sink;
	while(N != OASG->getDrivingNode())
	{
		
		SGNode* nextN = NULL;
		SGEdge* selectEdge = NULL;
		
		//[3-1] select next internode
		for(int j=0;j<N->getNumEdge();j++)
		{
			if(N->getEdge(j)->getToNode()->getDistFromDriving() < N->getDistFromDriving())
			{
				nextN = N->getEdge(j)->getToNode();
				selectEdge = N->getEdge(j);
			}
		}

		//[3-2] compute delay caused by subtrees that are rooted on nextN
		if(nextN != OASG->getDrivingNode())
		{	
			double subtree_cap = nextN->getDowntreamCap() - N->getDowntreamCap() - (selectEdge->getLength() * unit_c);
			
			Delay_by_Sub_Tree = Delay_by_Sub_Tree + subtree_cap * (nextN->getDistFromDriving() * unit_r / 1000);		
		}

		//[3-3] keep back-tracing
		N = nextN;
	}


	//[4] Delay caused by driving node (const value)
	Delay_by_Driving_Node = this->Rd / 1000 * OASG->getDrivingNode()->getDowntreamCap();


	//[5] summarize
	Sink->setDelay(Delay_by_Pi_Model + Delay_by_Itself + Delay_by_Sub_Tree + Delay_by_Driving_Node);

#ifdef DEBUG_ELMORE_SINK_DELAY
	cout<<"Delay_by_Pi_Model = "<<Delay_by_Pi_Model<<endl;	
	cout<<"Delay_by_Itself = "<<Delay_by_Itself<<endl;
	cout<<"Delay_by_Sub_Tree = "<<Delay_by_Sub_Tree<<endl;
	cout<<"Delay_by_Driving_Node = "<<Delay_by_Driving_Node<<endl;
	cout<<"Sink Delay = "<<Sink->getDelay()<<endl;
#endif

	Sink->setDelay_by_Pi_Model(Delay_by_Pi_Model);
	Sink->setDelay_by_Itself(Delay_by_Itself);
	Sink->setDelay_by_Sub_Tree(Delay_by_Sub_Tree);
	Sink->setDelay_by_Driving_Node(Delay_by_Driving_Node);
}




void elmoreDelay::Compute_OASG_DownstreamCap()
{
	for(int i=0;i<OASG->getNumNode();i++)
	{	
						
		SGNode* N = OASG->getNode(i);		

		if(N->getType()==_PIN && N->checkInTree()==true)
		{


			if(N->getType()==_PIN && N!=OASG->getDrivingNode())
			{	//N->setDowntreamCap(N->getDowntreamCap()+this->Cloading);
				N->setDowntreamCap(N->getDowntreamCap()+N->getLoading());
			}

			double cap = 0;
			if(N->checkIsLeaf()) //case 1 : leaf node
			{	cap = N->getDowntreamCap();
			}
			else if(N->getType()==_PIN && N!=OASG->getDrivingNode()) //case 2 : internal node 
			{	//cap = this->Cloading; //avoid recompute cap of this internal node
				cap = N->getLoading(); //avoid recompute cap of this internal node
			}

			SGEdge* selectEdge = NULL;
			bool traceOneEdge = false;

			while(N != OASG->getDrivingNode())
			{
				//select next internode
				SGNode* nextN = NULL;

				for(int j=0;j<N->getNumEdge();j++)
				{
					if(N->getEdge(j)->checkRouted())
					{
						if(N->getEdge(j)->getToNode()->getDistFromDriving() < N->getDistFromDriving())
						{
							if(nextN == NULL)
							{	
								selectEdge = N->getEdge(j);
								nextN = N->getEdge(j)->getToNode();
							}
							else
							{	
								cout<<"Error occured in elmoreDelay::Compute_OASG_DownstreamCap()\n";
								exit(0);
							}
						}
					}
				}
				

				if(nextN==NULL)
				{	cout<<"Error occured in elmoreDelay::Compute_OASG_DownstreamCap()\n";
					exit(0);
				}

				if(!traceOneEdge)
				{	traceOneEdge = true;
					cap = cap + selectEdge->getLength()*this->unit_c;
				}

				nextN->setDowntreamCap(nextN->getDowntreamCap() + cap);

				N = nextN;
			}
		}
	}
}
#ifdef PRIORITY_BASED
void elmoreDelay::computeHalfRC() { half_RC= 0.0005*unit_r*unit_c; }
double elmoreDelay::getHalfRC() { return half_RC; }
#endif
#ifdef BUF_DRIVEN
/*
void elmoreDelay::Compute_OASG_DownstreamCap_BUF()
{
	for(int i=0;i<OASG->getNumNode();i++)
	{	
						
		SGNode* N = OASG->getNode(i);		

		if(N->getType()==_PIN && N->checkInTree()==true)
		{


			if(N->getType()==_PIN && N!=OASG->getDrivingNode())
			{	//N->setDowntreamCap(N->getDowntreamCap()+this->Cloading);
				N->setDowntreamCap(N->getDowntreamCap()+N->getLoading());
			}

			double cap = 0;
			if(N->checkIsLeaf()) //case 1 : leaf node
			{	cap = N->getDowntreamCap();
			}
			else if(N->getType()==_PIN && N!=OASG->getDrivingNode()) //case 2 : internal node 
			{	//cap = this->Cloading; //avoid recompute cap of this internal node
				cap = N->getLoading(); //avoid recompute cap of this internal node
			}

			SGEdge* selectEdge = NULL;
			bool traceOneEdge = false;

			while(N != OASG->getDrivingNode())
			{
				//select next internode
				SGNode* nextN = NULL;

				for(int j=0;j<N->getNumEdge();j++)
				{
					if(N->getEdge(j)->checkRouted())
					{
						if(N->getEdge(j)->getToNode()->getDistFromDriving() < N->getDistFromDriving())
						{
							if(nextN == NULL)
							{	
								selectEdge = N->getEdge(j);
								nextN = N->getEdge(j)->getToNode();
							}
							else
							{	
								cout<<"Error occured in elmoreDelay::Compute_OASG_DownstreamCap()\n";
								exit(0);
							}
						}
					}
				}
				

				if(nextN==NULL)
				{	cout<<"Error occured in elmoreDelay::Compute_OASG_DownstreamCap()\n";
					exit(0);
				}

				if(!traceOneEdge)
				{	traceOneEdge = true;
					cap = cap + selectEdge->getLength()*this->unit_c;
				}

				nextN->setDowntreamCap(nextN->getDowntreamCap() + cap);

				N = nextN;
			}
		}
	}
}
*/
#endif

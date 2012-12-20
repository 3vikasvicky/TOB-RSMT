#include "OARST.h"


void elmoreDelay::Restore_Delay_Info_OARG()
{
	cout<<"--[re-initialize sink's members]\n";
	for(int i=0;i<this->OARG->getNumNode();i++)
	{
		RGNode* N = this->OARG->getNode(i);

		//restore delay information
		N->setDowntreamCap(0);
		N->setDelay(0);
		N->setDelay_by_Driving_Node(0);
		N->setDelay_by_Itself(0);
		N->setDelay_by_Pi_Model(0);
		N->setDelay_by_Sub_Tree(0);
	}

}

double elmoreDelay::Compute_OARG_Delay(Database &data)
{

	//restore downstream cap & others
	Restore_Delay_Info_OARG();

	//calculate downstream cap
	Compute_OARG_DownstreamCap();
	double delay_sum = 0;

	for(int i=0;i<OARG->getNumNode();i++)
	{	
		RGNode* N = OARG->getNode(i);		

		Compute_Sink_Delay(N);

		if(N->getType()==_PIN && N!=OARG->getDrivingNode())
		{			
			//Compute_Sink_Delay(N);
			delay_sum = delay_sum + N->getDelay();
		}
	}


	RGNode* worseDelayNode = NULL;
	for(int i=0;i<OARG->getNumNode();i++)
	{	
		RGNode* N = OARG->getNode(i);		
		if(N->getType() == _PIN && worseDelayNode == NULL || worseDelayNode->getDelay() < N->getDelay())
		{	worseDelayNode = N;
		}
	}

	double average_sink_delay = (double)delay_sum/(data.getPinNum()-1);
	cout<<"\nAverage sink delay = "<<average_sink_delay<<endl;
	OARG->setWorstDelaySink(worseDelayNode);
	cout<<"Worst sink delay = "<<worseDelayNode->getDelay()<<" (ps)"<<endl;
#ifdef DEBUG_DELAY_INFO
	cout<<"It's required arrival time = "<<worseDelayNode->getRequiredTime()<<endl;
	cout<<"Worst sink delay node : leaf ? "<<worseDelayNode->checkIsLeaf()<<endl;
	cout<<" (1) Delay_by_Pi_Model     = "<<worseDelayNode->getDelay_by_Pi_Model()<<endl;
	cout<<" (2) Delay_by_Itself       = "<<worseDelayNode->getDelay_by_Itself()<<endl;
	cout<<" (3) Delay_by_Sub_Tree     = "<<worseDelayNode->getDelay_by_Sub_Tree()<<endl;
	cout<<" (4) Delay_by_Driving_Node = "<<worseDelayNode->getDelay_by_Driving_Node()<<endl;
	cout<<"Worst Delay Sink : "; PrintRGNode(worseDelayNode);	
#endif
	setWorseDelaySinkInOARG(worseDelayNode); //elmore delay model class

	return average_sink_delay;
}


void elmoreDelay::Compute_Sink_Delay(RGNode* Sink)
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
	RGNode* N = Sink;
	while(N != OARG->getDrivingNode())
	{
		
		RGNode* nextN = NULL;
		RGEdge* selectEdge = NULL;
		
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
		if(nextN != OARG->getDrivingNode())
		{	
			double subtree_cap = nextN->getDowntreamCap() - N->getDowntreamCap() - (selectEdge->getLength() * unit_c);
			
			Delay_by_Sub_Tree = Delay_by_Sub_Tree + subtree_cap * (nextN->getDistFromDriving() * unit_r / 1000);		
		}

		//[3-3] keep back-tracing
		N = nextN;
	}


	//[4] Delay caused by driving node (const value)
	Delay_by_Driving_Node = this->Rd / 1000 * OARG->getDrivingNode()->getDowntreamCap();


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

void elmoreDelay::Compute_OARG_DownstreamCap()
{
	for(int i=0;i<OARG->getNumNode();i++)
	{	
						
		RGNode* N = OARG->getNode(i);		

		if(N->getType()==_PIN && N!=OARG->getDrivingNode())
		{	//N->setDowntreamCap(N->getDowntreamCap()+this->Cloading);
			N->setDowntreamCap(N->getDowntreamCap()+N->getLoading());
		}

		double cap = 0;
		if(N->checkIsLeaf()) //case 1 : leaf node
		{	cap = N->getDowntreamCap();
		}
		else if(N->getType()==_PIN && N!=OARG->getDrivingNode()) //case 2 : internal node 
		{	//cap = this->Cloading; //avoid recompute cap of this internal node
			cap = N->getLoading(); //avoid recompute cap of this internal node
		}


#ifdef DEBUG_ELMORE_DOWNSTREAM_CAPACITANCE
		cout<<"Path starts from node :\n";
		PrintRGNode_Delay(N);
#endif

		RGEdge* selectEdge = NULL;
		bool traceOneEdge = false;

		while(N != OARG->getDrivingNode())
		{
			//select next internode
			RGNode* nextN = NULL;
			

			for(int j=0;j<N->getNumEdge();j++)
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
						cout<<"Error occured in elmoreDelay::Compute_OARG_DownstreamCap()\n";
						exit(0);
					}
				}
			}

			if(nextN==NULL)
			{		
				cout<<"Error occured in elmoreDelay::Compute_OARG_DownstreamCap()\n";
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

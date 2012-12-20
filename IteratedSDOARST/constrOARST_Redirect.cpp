#include "OARST.h"
using namespace auxiliary;


RGNode* OARSteinerTree::Find_SubTreeRoot_Upstream(RGNode* Sink)
{	
	RGNode* N = Sink;
	RGNode* nextN = NULL;
	for(int j=0;j<N->getNumEdge();j++)
	{
		if(N->getEdge(j)->getToNode()->getDistFromDriving() < N->getDistFromDriving() 
			&& N->getEdge(j)->getToNode()!=this->getOARG()->getDrivingNode())
		{	
			nextN = N->getEdge(j)->getToNode();
			return nextN;
		}
	}

	return getOARG()->getDrivingNode();

}

//avoid to connect disconnected sink to a subtree that rooted on itself
//that will cause a disconnection
bool OARSteinerTree::NRootOnReduceWantedSink(RGNode* ReduceWanted_Sink,RGNode* N)
{
	//while(N!=this->getOARG()->getDrivingNode())
	while(N->getDistFromDriving() > ReduceWanted_Sink->getDistFromDriving())
	{	
		N = Find_SubTreeRoot_Upstream(N);
		if(N == ReduceWanted_Sink)
		{	return true;
		}
	}
	return false;
}

RGNode* OARSteinerTree::ConnectWNS_Sink(RGNode* ReduceWanted_Sink)
{

	SGNode* ToN = ReduceWanted_Sink->getSGNode();
	//select the best edge that can reduce the distance from driver to WNS Sink
	SGNode* FromN = NULL;
	SGEdge* SelectEdge = NULL;
	
	
	//method 1 : pick minimum delay node
	double min_delay = _INFINITE;

	for(int i=0;i<ToN->getNumEdge();i++)
	{	
		SGEdge* E = ToN->getEdge(i);
		RGNode* Ref_RG_N = E->getToNode()->getRefRGNode();

		if(E->getToNode()->getType() != _TURNING && E->getToNode()->checkInTree() == true && Ref_RG_N != NULL)
		{
		
#ifdef DEBUG_OARST_BRANCH_MOVING_CONNECT_WNS_SINK
		cout<<"Cand (RGNode): "; PrintRGNode(Ref_RG_N);
#endif

		if(min_delay > Ref_RG_N->getDelay() && NRootOnReduceWantedSink(ReduceWanted_Sink,Ref_RG_N) == false)
		{	
			FromN = E->getToNode();
			SelectEdge = E;
			min_delay = Ref_RG_N->getDelay();
		}
		}
	}

	if(SelectEdge==NULL)
	{		
		return NULL;
	}

#ifdef DEBUG_OARST_BRANCH_MOVING_CONNECT_WNS_SINK
	cout<<"Pick : "; PrintNode(FromN);
	cout<<"Pick (RGNode): "; PrintRGNode(FromN->getRefRGNode());
	cout<<"Select SG Edge = ";
	PrintEdge(SelectEdge);
#endif

	RouteEdge(SelectEdge);
	RouteEdge(SelectEdge->getDualEdge());
	SelectEdge->setRectState(false);
	SelectEdge->getDualEdge()->setRectState(false);
	ToN->setTreeState(true);

	return  FromN->getRefRGNode();
}

//make that sink unconnectable from tree
//if the isolation fails, return false
bool OARSteinerTree::DisconnectSink(RGNode* ReduceWanted_Sink,RGNode* upstreamNode)
{
		//Rip-up Edge in OARG
#ifdef DEBUG_OARST_BRANCH_MOVING_DISCONNECT_WNS_SINK
		cout<<"\nupstreamNode : "; PrintRGNode(upstreamNode);
		cout<<"# of edge in upstreamNode = "<<upstreamNode->getNumEdge()<<endl;
		cout<<"edges of upstreamNode :\n";
		for(int j=0;j<upstreamNode->getNumEdge();j++)
		{	PrintRGEdge(upstreamNode->getEdge(j));
		}
#endif
		if(upstreamNode == getOARG()->getDrivingNode() || upstreamNode->getNumEdge()==1)
		{	
#ifdef DEBUG_OARST_BRANCH_MOVING_DISCONNECT_WNS_SINK			
			cout<<"cannot isolate this ReduceWanted_Sink\n";
			cout<<"upstreamNode = driver or upstreamNode->getNumEdge()==1, so return\n";
#endif
			return false;
		}
		RGEdge* breakableEdge = getOARG()->Find_connEdge(upstreamNode,ReduceWanted_Sink);
#ifdef DEBUG_OARST_BRANCH_MOVING_DISCONNECT_WNS_SINK
		cout<<"\ndel edge in OARG : "; PrintRGEdge(breakableEdge);
		cout<<"then check upstreamNode, further delete if only edge exist and node isn't sink\n\n";		
#endif
		getOARG()->DelRGEdge(breakableEdge);	


		//Rip-up the edges connecting ReduceWanted_Sink between previous node in OASG
		int RipupNum=0;
		SGNode* ToN = ReduceWanted_Sink->getSGNode();
		SGEdge* EdgeToDriver = NULL;

		for(int i=ToN->getNumEdge()-1;i>=0;i--)
		{	
			if(ToN->getEdge(i)->checkRouted() && ToN->getEdge(i)->getToNode()->getDistFromDriving() < ToN->getDistFromDriving())
			{	
#ifdef DEBUG_OARST_BRANCH_MOVING_DISCONNECT_WNS_SINK				
				cout<<"ripup edge in OASG : "; PrintEdge(ToN->getEdge(i));
				RipupNum++;
#endif	
				UnRouteEdge(ToN->getEdge(i));
				break;
			}
		}
#ifdef DEBUG_OARST_BRANCH_MOVING_DISCONNECT_WNS_SINK
		cout<<"Rip-up the edges (# = "<<RipupNum<<") to disconnect WNS Sink from driver in OASG\n";
#endif	

		return true;
}

void OARSteinerTree::Remove_Redundant_Edges(RGNode* upstreamNode,RGNode* fromNode)
{
	int delNum=0;
	while(upstreamNode->getType()!=_PIN && upstreamNode->getNumEdge()==1 && upstreamNode != fromNode)
	{	
#ifdef DEBUG_OARST_BRANCH_MOVING_DISCONNECT_WNS_SINK
		cout<<"Remove Edge : ";
		PrintRGEdge(upstreamNode->getEdge(0));
#endif
		RGNode* tempN = upstreamNode->getEdge(0)->getToNode();
		getOARG()->DelRGEdge(upstreamNode->getEdge(0));			
		getOARG()->DelRGNode(upstreamNode);
		upstreamNode = tempN;
		delNum++;
	}
#ifdef DEBUG_OARST_BRANCH_MOVING_DISCONNECT_WNS_SINK
	cout<<"Total delete # = "<<delNum<<" edges\n\n";
#endif
}

//avoid doing nothing
bool OARSteinerTree::MoveToPreviousNode(RGNode* ReduceWanted_Sink)
{	if(ReduceWanted_Sink->getType()!=_PIN)
	{	return true;
	}
	

	RGNode* upstreamNode = Find_SubTreeRoot_Upstream(ReduceWanted_Sink);
	while(upstreamNode->getType()!=_PIN)
	{	upstreamNode = Find_SubTreeRoot_Upstream(upstreamNode);
	}

	SGNode* ToN = ReduceWanted_Sink->getSGNode();

	//-------------------------------------------------------------------------------

	//select the best edge that can reduce the distance from driver to WNS Sink
	SGEdge* SelectEdge = NULL;
	SGNode* FromN = NULL;

	//method 1 : pick minimum delay node
	double min_delay = _INFINITE;

	for(int i=0;i<ToN->getNumEdge();i++)
	{	
		SGEdge* E = ToN->getEdge(i);

		RGNode* Ref_RG_N = E->getToNode()->getRefRGNode();
		if(E->getToNode()->checkInTree() == true && Ref_RG_N != NULL
			&& min_delay > Ref_RG_N->getDelay())
		{	
			FromN = E->getToNode();
			SelectEdge = E;
			min_delay = Ref_RG_N->getDelay();
		}
	}

	//-------------------------------------------------------------------------------

	if(upstreamNode == FromN->getRefRGNode())
	{	return true;
	}
	else
	{	return false;
	}
}

//---------------------------------------------------------
//perform branch moving (method from A Novel Performance-Driven Topology Design Algorithm 08)
//to reduce WNS and # of timing violation sinks
//---------------------------------------------------------

bool OARSteinerTree::Redirect(RGNode* ReduceWanted_Sink)
{
	if(ReduceWanted_Sink->getType()==_PIN)
	{			
#ifdef DEBUG_REDIRECT_FLOW
		cout<<"\n--[Compute each sink's distance from driver in OASG]\n";
#endif
		getOASG()->computeDistFromDriver();

#ifdef DEBUG_REDIRECT_FLOW
		cout<<"\n--[Check WNS Sink, Moving forward to previous ?]\n";
#endif
		while(MoveToPreviousNode(ReduceWanted_Sink))
		{	ReduceWanted_Sink = Find_SubTreeRoot_Upstream(ReduceWanted_Sink);
#ifdef DEBUG_REDIRECT_FLOWs
			cout<<"MOVE FORWARD!!\n";
#endif
		}

		RGNode* FromRGNode = NULL;
		RGNode* upstreamNode = NULL;
#ifdef DEBUG_REDIRECT_FLOW
		cout<<"\n--[Disconnect WNS Sink in OASG and OASG]\n";
#endif
		upstreamNode =	Find_SubTreeRoot_Upstream(ReduceWanted_Sink);

		getOARG()->setDisConnect_Sink(ReduceWanted_Sink);

		if(DisconnectSink(ReduceWanted_Sink,upstreamNode)==false)
		{	return true;
		}
#ifdef DEBUG_REDIRECT_FLOW
		cout<<"\n--[Reconnect WNS Sink in OASG]\n";
#endif
		FromRGNode = ConnectWNS_Sink(ReduceWanted_Sink);

		if(FromRGNode == NULL)
		{	
			return false;
		}


#ifdef DEBUG_REDIRECT_FLOW
		cout<<"\n--[Remove Redundant Edges]\n";
#endif
		Remove_Redundant_Edges(upstreamNode,FromRGNode);

#ifdef DEBUG_REDIRECT_FLOW
		cout<<"\n--[Turn Rectilinear]\n";		
#endif
		getOARG()->TurnRectilinear_From(FromRGNode);
	
#ifdef DEBUG_REDIRECT_FLOW
		cout<<"\n--[Compute each sink's distance from driver in OARG]\n";		
#endif
		getOARG()->computeDistFromDriver();	
	}

	return true;
}


bool OARSteinerTree::ReduceNegativeSlack()
{	
	RGNode* WNS_Sink =  getOARG()->getWorstSlackSink();
#ifdef DEBUG_REDIRECT_FLOW
	cout<<"WNS sink = ";
	PrintRGNode_Delay(WNS_Sink);
#endif
	return Redirect(WNS_Sink);
}
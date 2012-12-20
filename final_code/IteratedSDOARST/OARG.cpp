#include "OARST.h"

OARectilinearGraph::OARectilinearGraph(SGNode* SG_DrivingNode,OASpanningGraph* OASG) : WorstDelaySink(NULL), WorstSlackSink(NULL), longestRadius(0)
{
	setDisConnect_Sink(NULL); //for redirecting use
	ReduceUOK = false; //turn on when doing reducing U shape
	vUn_RGNode.clear();
	vUn_SGEdge.clear();
	this->DrivingNode = AddRGNode(SG_DrivingNode);
	this->OASG = OASG;
	this->DrivingNode->setDistFromDriving(0);
	this->DrivingNode->setIsLeaf(true); //now only driving node in OASG
	vUn_RGNode.push_back(DrivingNode);
	
}


RGNode::RGNode(SGNode* N)
{
	this->point = N->getPoint();
	this->type  = N->getType();
	this->setTreeState(N->checkInTree());

	this->setIsLeaf(false);
	this->setCheckedConn(false);
	distFromDriving=_INFINITE;	//init for routing use
	SG_Node=N;
	
	
	//constrain-related variables
	required_time = N->getRequiredTime();
	loading = N->getLoading();


	//delay-related
	downtreamCap = 0;
	delay = 0;	
	Delay_by_Pi_Model = 0;
	Delay_by_Itself = 0;
	Delay_by_Sub_Tree = 0;
	Delay_by_Driving_Node = 0;
	index = -1;
}


RGNode::RGNode(int X,int Y,int Layer,int type)
{
	this->type=type;
	this->point = new Point(X,Y,Layer);

	this->setIsLeaf(false);
	this->setCheckedConn(false);
	distFromDriving=_INFINITE;	//init for routing use
	SG_Node=NULL;
	this->setTreeState(true);

	//constrain-related variables
	required_time = 0;
	loading = 0;

	//delay-related
	downtreamCap = 0;
	delay = 0;
	Delay_by_Pi_Model = 0;
	Delay_by_Itself = 0;
	Delay_by_Sub_Tree = 0;
	Delay_by_Driving_Node = 0;
	index = -1;
}

void RGNode::EraseEdge(RGEdge* delE)
{	for(int i=0;i<this->getNumEdge();i++)
	{	if(this->getEdge(i) == delE)
		{	
			
#ifdef DEBUG_OARG_ERASE_EDGE
			cout<<"\nErase Edge from Node:\n";
			PrintRGEdge(vRGEdge[i]);
#endif			
			///////////////////////////////////////////////////////////////////
			this->vRGEdge.erase(this->vRGEdge.begin()+i);

			return;
		}
	}

	cout<<"Error occured in RGNode::EraseEdge(RGEdge* delE), delE in node isn't found\n";
	cout<<"delE = ";
	PrintRGEdge(delE);
	cout<<"its dual = ";
	PrintRGEdge(delE->getDualEdge());
	cout<<"\nOutput all the edge from Root:\n";
	for(int i=0;i<this->getNumEdge();i++)
	{	PrintRGEdge(getEdge(i));
	}
	exit(0);
}


RGEdge::RGEdge(RGNode* FromNode,RGNode* ToNode) : FromNode(FromNode), ToNode(ToNode), length(0), dualEdge(NULL)
{
	this->setLength(ComputeLength(FromNode,ToNode));
	this->used = false;
}

void UnProcessSGEdge::ComputeCost()
{
	Cost = this->SG_edge->getLength();
}





RGNode* OARectilinearGraph::AddRGNode(int X,int Y,int Layer,int type)
{	
	if(getDisConnect_Sink() != NULL)
	{	
		if(DisConnect_Sink->getX() == X && DisConnect_Sink->getY() == Y && DisConnect_Sink->getLayer() == Layer)
		{	return DisConnect_Sink;
		}
	}

	if(getDisConnect_Sink() != NULL || ReduceUOK==true)
	{
		//check already exist ?
		for(int i=0;i<this->getNumNode();i++)
		{	RGNode* N = this->getNode(i);
			if(N->getX() == X && N->getY() == Y && N->getLayer() == Layer)
			{	return N;
			}
		}
	}
	
	RGNode* RG_N = new RGNode(X,Y,Layer,type);
	vRGNode.push_back(RG_N);
	

#ifdef DEBUG_OARG_DO_RECTILINEAR_ADD_RGNODE
	cout<<"=> New RGNode:\n";
	PrintRGNode(RG_N);
#endif

	return vRGNode[vRGNode.size()-1];
}

RGNode* OARectilinearGraph::AddRGNode(SGNode* N)
{	
	if(getDisConnect_Sink() != NULL)
	{	
		if(DisConnect_Sink->getX() == N->getX() && DisConnect_Sink->getY() == N->getY() && DisConnect_Sink->getLayer() == N->getLayer())
		{	return DisConnect_Sink;
		}
	}

	if(getDisConnect_Sink() != NULL || ReduceUOK==true)
	{
		//check already exist ?
		for(int i=0;i<this->getNumNode();i++)
		{	RGNode* CheckN = this->getNode(i);
			if(N->getX() == CheckN->getX() && N->getY() == CheckN->getY() && N->getLayer() == CheckN->getLayer())
			{	return CheckN;
			}
		}
	}

	RGNode* RG_N = new RGNode(N);
	N->setRefRGNode(RG_N);
	vRGNode.push_back(RG_N);				

#ifdef DEBUG_OARG_DO_RECTILINEAR_ADD_RGNODE
	cout<<"=> New RGNode:\n";
	PrintRGNode(RG_N);
#endif

	return vRGNode[vRGNode.size()-1];
}

void OARectilinearGraph::DelRGNode(RGNode* delN)
{
	//delete delN from vector of OARG
	for(int i=0;i<this->vRGNode.size();i++)
	{	
		if(vRGNode[i] == delN)
		{	
#ifdef DEBUG_OARG_ERASE_NODE
			cout<<"\nErase delN OK from Vector .....\n";
			PrintRGNode(vRGNode[i]);
#endif
			this->vRGNode.erase(this->vRGNode.begin()+i);

			break;
		}
	}

	if(delN->getSGNode() != NULL)
	{	//set delN's SGNode reference = non Tree
		delN->getSGNode()->setTreeState(false);
	}

	//delete delN from memory
	delete(delN);

}


void OARectilinearGraph::DelRGEdge(RGEdge* delE)
{	
	RGNode* Root = delE->getFromNode();
	RGNode* N = delE->getToNode();
	

	//delete RE1 from vector of OARG
	for(int i=0;i<this->vRGEdge.size();i++)
	{	
		if(vRGEdge[i] == delE)
		{	
#ifdef DEBUG_OARG_ERASE_EDGE
			cout<<"\nErase delE OK from Vector .....\n";
			PrintRGEdge(vRGEdge[i]);
#endif
			this->vRGEdge.erase(this->vRGEdge.begin()+i);

			break;
		}
	}

	//delete its dual from vector of OARG
	for(int i=0;i<this->vRGEdge.size();i++)
	{	
		if(vRGEdge[i] == delE->getDualEdge())
		{	

#ifdef DEBUG_OARG_ERASE_EDGE
			cout<<"\nErase dual delE from Vector OK .....\n";
			PrintRGEdge(vRGEdge[i]);
#endif

			this->vRGEdge.erase(this->vRGEdge.begin()+i);

			break;
		}
	}


	//delete RE1 and its dual from Root and N
	Root->EraseEdge(delE);
	N->EraseEdge(delE->getDualEdge());


	//delete RE1 and its dual from memory
	delete(delE->getDualEdge());
	delete(delE);

}


void OARectilinearGraph::AddSlantRGEdge(StruFromRGNode FromNode,StruToRGNode ToNode,double assignDist,bool checkOverlap)
{
	if(FromNode.Node->getX() == ToNode.Node->getX() || FromNode.Node->getY() == ToNode.Node->getY())
	{	AddRGEdge(FromNode,ToNode,assignDist,checkOverlap);
	}
	else
	{	
		//-----------------------------------
		//is slant edge, so add two edges		
		//random select upper L or lower L
		//-----------------------------------
		RGNode* Vb = AddRGNode(FromNode.Node->getX(),ToNode.Node->getY(),FromNode.Node->getLayer(),_TURNING);
		RGEdge* RE1 = AddRGEdge(StruFromRGNode(FromNode),StruToRGNode(Vb),_INFINITE,false);
		RGEdge* RE2 = AddRGEdge(StruFromRGNode(Vb),StruToRGNode(ToNode),_INFINITE,false);
		//EliminateOverlap(RE1);
		//EliminateOverlap(RE2);
	}

}


RGEdge* OARectilinearGraph::AddRGEdge(StruFromRGNode FromNode,StruToRGNode ToNode,double assignDist,bool checkOverlap)
{	

	if(FromNode.Node->getX() == ToNode.Node->getX() && FromNode.Node->getY() == ToNode.Node->getY())
	{	return NULL;
	}


	RGEdge* RG_E = new RGEdge(FromNode.Node,ToNode.Node);  //create a new RGEdge then set the from node and to node of this edge
	RGEdge* dual_RG_E = new RGEdge(ToNode.Node,FromNode.Node);  //create a dual RGEdge then set the from node and to node of this edge

	
	RG_E->setDualEdge(dual_RG_E); //set reference of RG_E
	dual_RG_E->setDualEdge(RG_E); //set reference of dual_RG_E

	FromNode.Node->AddEdge(RG_E);
	ToNode.Node->AddEdge(dual_RG_E);

	vRGEdge.push_back(RG_E);
	vRGEdge.push_back(dual_RG_E);

#ifdef DEBUG_OARG_DO_RECTILINEAR_ADD_RGEDGE
	cout<<"=> New RG Edge:\n";
	PrintRGEdge(RG_E);
	cout<<"=> Its dual Edge:\n";
	PrintRGEdge(dual_RG_E);
#endif

	if(checkOverlap)
		EliminateOverlap(RG_E); //After adding a new RG edge, do overlapped edge check and eliminate overlapped edge if any
	//EliminateOverlap(dual_RG_E); ///< ================================================================== mark

#ifdef DEBUG_OARG_DO_RECTILINEAR_ADD_RGEDGE
	cout<<"=> (2)New RG Edge:\n";
	PrintRGEdge(RG_E);
	cout<<"=> (2)Its dual Edge:\n";
	PrintRGEdge(dual_RG_E);
#endif


	if(checkOverlap)
	{	return false;
	}
	else
	{	return RG_E;
	}

}

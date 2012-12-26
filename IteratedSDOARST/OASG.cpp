#include "OARST.h"

SGNode::SGNode(int X, int Y, int Layer, int type,double Req_time,double loading) : RefRGNode(NULL)
{
	this->type=type;

	point = new Point(X,Y,Layer);
	PE_R1 = new ProcessEntry(X,Y,Layer,_R1,type,this);
	PE_R2 = new ProcessEntry(X,Y,Layer,_R2,type,this);
	PE_R3 = new ProcessEntry(X,Y,Layer,_R3,type,this);
	PE_R4 = new ProcessEntry(X,Y,Layer,_R4,type,this);
	inTree=false;
	index=0;
	ConnOK =false;
	this->setIsLeaf(false);

	distFromDriving=_INFINITE;	//init for routing use
	referenceRadius=_INFINITE;	//init for estimating delay
	delayRatio=_INFINITE;	//init for estimating delay

	tempDistFromSource=_INFINITE;	//init for routing use
	setTempProCost(_INFINITE);		//init for routing use
	tempSource=NULL;			    //init for routing use
	tempPreviousNode=NULL;			//init for routing use
	tempPreviousEdge=NULL;			//init for routing use
	tempDistFromTree==_INFINITE;    //init for prim's algorithms

	//----------------------------------------
	//constrain-related variables
	//this->timing_constrain = constrain;
	required_time = Req_time;
	this->loading = loading;

	//----------------------------------------
	//Critical sink related variables
	isCriticalSink = false;
	inCriticalPath = false;
	DPF = 0;
	SID = 0;

	//delay-related
	downtreamCap = 0;
	delay = 0;
	Delay_by_Pi_Model = 0;
	Delay_by_Itself = 0;
	Delay_by_Sub_Tree = 0;
	Delay_by_Driving_Node = 0;

#ifdef PRIORITY_BASED
	priority = _NINFINITE;
	slack = _INFINITE;
	distFromDriver = 0;
	tempDistFromDriver = 0;
	//mazeDelay=_INFINITE;
	mazeDelay=0;
	referencePriority = _INFINITE;
	slackUsed = false ;
	preSlackNode = 0 ;
	initTree=false;
#endif
#ifdef BUF_DRIVEN
	bufferedDSC=0;
	DSC=0;
	isBuffer=false;
#endif
}

SGNode::SGNode(SGNode* N) //copy construction
{
	this->type=N->getType();
	

	point = new Point(N->getX(),N->getY(),N->getLayer());
	inTree=N->checkInTree();
	index=N->getIndex();
	ConnOK = N->checkedConn();
	this->setIsLeaf(false);

	distFromDriving=N->getDistFromDriving();
	referenceRadius=_INFINITE;	//init for estimating delay
	delayRatio=_INFINITE;	//init for estimating delay
	
	tempDistFromSource=N->getTempDistFromSource();
	setTempProCost(N->getTempProCost());
	tempSource=N->getTempSource();
	tempPreviousEdge=N->getTempPrevEdge();
	tempPreviousNode=N->getTempPrevNode();

	//----------------------------------------
	//constrain-related variables
	required_time = N->getRequiredTime();
	loading = N->getLoading();

	//----------------------------------------
	//Critical sink related variables
	isCriticalSink = N->checkCriticalSink();
	inCriticalPath = N->checkInCriticalPath();
	DPF = N->getDPF();
	SID = N->getSID();

	//delay-related
	downtreamCap = 0;
	delay = 0;
	Delay_by_Pi_Model = 0;
	Delay_by_Itself = 0;
	Delay_by_Sub_Tree = 0;
	Delay_by_Driving_Node = 0;
#ifdef PRIORITY_BASED
	priority = N->priority;
	slack=N->slack;
	mazeDelay=N->mazeDelay;
	referencePriority = N->referencePriority ;
	initTree=false;
#endif
#ifdef BUF_DRIVEN
	bufferedDSC=0;
	DSC=0;
	isBuffer=false;
#endif
}

void SGNode::AddEdge(SGEdge* E) 
{	vSGEdge.push_back(E);
}

SGEdge::SGEdge(SGNode* FromNode,SGNode* ToNode) : FromNode(FromNode), ToNode(ToNode), length(0), TurnRectilinear(false), inCriticalPath(false)
{	
	//remember that don't use manhattan distance that will change the topology very dramatically
	double len = ComputeLength(FromNode,ToNode);
	setLength(len);
	setSqrLength(len*len);
	//setLength(ComputeManhattanDist(FromNode,ToNode));
	dualEdge=NULL;
	Routed=false;
	Enable=true;
#ifdef PRIORITY_BASED
	used=false;
#endif
#ifdef BUF_DRIVEN
	inInitTree=false;
#endif
}

ProcessEntry::ProcessEntry(int X, int Y, int Layer, int region, int type, SGNode *parent)
{
	point = new Point(X,Y,Layer); 
	this->region=region;
	this->type=type;
	this->parent=parent;
}


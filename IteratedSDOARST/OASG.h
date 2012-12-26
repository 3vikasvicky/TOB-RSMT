#ifndef OASG_H //Avoid duplicate definition
#define OASG_H

#include "OARST.h"
#include "RTree.h"
#include <list>

class ProcessEntry;
class Interval;
class AEntry;
class SweepEntry;
class SGNode;
class SGEdge;
class RGNode;
typedef Interval IEntry;


//Class SGNode is a fundamental data structure of OARST
class SGNode
{
public:
	SGNode(int X,int Y,int Layer,int type,double Req_time=0,double loading=0);
	SGNode(SGNode* N); //copy construction
	void setRefRGNode(RGNode* N) { RefRGNode = N; }
	RGNode* getRefRGNode() { return RefRGNode; }

#ifdef PRIORITY_BASED
	void setPriority(double p) { priority=p; }
	double getPriority() { return priority; }
	void setSlack(double s) { slack=s; }
	double getSlack() { return slack; }
	void setDistFromDriver(double d) { distFromDriver=d; }
	double getDistFromDriver() { return distFromDriver; }
	void setTempDistFromDriver(double d) { tempDistFromDriver=d; }
	double getTempDistFromDriver() { return tempDistFromDriver; }
	void setDelayForPrim(double d) { delayForPrim=d; }
	double getDelayForPrim() { return delayForPrim; }
	void setMazeDelay(double d) { mazeDelay=d; }
	double getMazeDelay() { return mazeDelay; }
	void setReferencePriority(double p) { referencePriority = p ; }
	double getReferencePriority() { return referencePriority ; }
	void setSlackUsed() { slackUsed = true ; }
	void setSlackUnused() { slackUsed = false ; }
	bool isSlackUsed() { return slackUsed ; }
	void setPreSlackNode(SGNode* n) { preSlackNode = n ; }
	SGNode* getPreSlackNode() { return preSlackNode ; }
	void InInitTree() { initTree=true; }
	bool isInitTree() { return initTree; }
#endif
#ifdef BUF_DRIVEN
	void setAbsDistFromDriving(double d){ absDistFromDriving=d; }
	double getAbsDistFromDriving() {return absDistFromDriving; }
	void setBufferedDSC(double dsc) { bufferedDSC=dsc; }
	double getBufferedDSC() { return bufferedDSC; }
	void setDSC(double dsc) { DSC=dsc; }
	double getDSC() { return DSC; }
	void setDFSFlag(bool f) { DFSFlag=f; }
	bool getDFSFlag() { return DFSFlag; }
	void setReducedDSC(double dsc) { reducedDSC=dsc; }
	double getReducedDSC() { return reducedDSC; }
	void setTmpDistFromLastBuffer(double d) { tmpDistFromLastBuffer=d; }
	double getTmpDistFromLastBuffer() { return tmpDistFromLastBuffer; }
	void setPreDistFromSource(double d) { preDistFromSource; }
	double getPreDistFromSource() { return preDistFromSource; }
	void setLastBufferAbsDist(double d) { lastBufferAbsDist=d; }
	double getLastBufferAbsDist() { return lastBufferAbsDist; }
	void insertBuffer() { isBuffer=true; }
	bool isBuffered() { return isBuffer; }
#endif
	ProcessEntry* getR1(){ return PE_R1; }	//It's a little bit of defect that no using const as return type. 
	ProcessEntry* getR2(){ return PE_R2; }
	ProcessEntry* getR3(){ return PE_R3; }
	ProcessEntry* getR4(){ return PE_R4; }

	int getX() { return point->getX(); }
	int getY() { return point->getY(); }
	int getLayer() { return point->getLayer(); }
	Point* getPoint() { return point; } 

	size_t getNumEdge() { return vSGEdge.size(); }
	SGEdge* getEdge(size_t index) { return vSGEdge[index]; }
	void AddEdge(SGEdge* E);

	int getType() { return type; }	//return the node's type that is a pin or an obstacle
	void setDetailType(int val) { detail_type=val;}
	int getDetailType() {return detail_type;}

	bool checkInTree() { return inTree; }
	void setTreeState(bool val) { inTree=val; }

	int getIndex() { return index; }
	void setIndex(int val) { index=val; }
	
	
	double getDistFromDriving() { return distFromDriving; }
	void setDistFromDriving(double val) { distFromDriving=val; }

	double getReferenceRadius() { return referenceRadius; }
	void setReferenceRadius(double val) { referenceRadius=val; }

	float getDelayRatio() { return delayRatio; }
	void setDelayRatio(float val) { delayRatio=val; }

	void setTempProCost(double val) { tempPropagateCost=val; }
	double getTempProCost() { return tempPropagateCost; }

	double getTempDistFromSource() { return tempDistFromSource; }
	void setTempDistFromSource(double val) { tempDistFromSource=val; }

	double getTempDistFromTree() { return tempDistFromSource; }      //only use in prim algorithm
	void setTempDistFromTree(double val) { tempDistFromSource=val; } //only use in prim algorithm

	void setTempSource(SGNode* N) { tempSource=N; } 
	SGNode* getTempSource() { return tempSource; } 

	void setTempPrevNode(SGNode* N) { tempPreviousNode=N; } 
	SGNode* getTempPrevNode() { return tempPreviousNode; } 

	void setTempPrevEdge(SGEdge* E) { tempPreviousEdge=E; } 
	SGEdge* getTempPrevEdge() { return tempPreviousEdge; } 

	bool checkedConn() { return ConnOK; } //if yes, this pin connect to driving correctly
	void setCheckedConn(bool val) { ConnOK=val; }

	//----------------------------------------
	//constrain-related functions
	double getRequiredTime() const { return required_time; }
	double getLoading() const { return loading; }
	void setLoading(double val) { loading = val; }


	//----------------------------------------
	//Critical sink related functions
	bool checkCriticalSink() { return isCriticalSink; } 
	void setCriticalSink(bool val) { isCriticalSink=val; } 
	bool checkInCriticalPath() { return inCriticalPath; } 
	void setInCriticalPath(bool val) { inCriticalPath=val; } 
	float getDPF() { return DPF; }
	void setDPF(float val) { DPF=val; }
	int getSID() {return SID; } //SID = sink impact delay
	void setSID(int val) { SID=val; } //SID = sink impact delay

	//----------------------------------------
	//delay-related functions
	void setDowntreamCap(double val) { downtreamCap=val; }
	//downtreamCap contains the cap of tree rooted on itself and its own loading
	double getDowntreamCap() { return downtreamCap; }
	void setDelay(double val) { delay=val; }
	double getDelay() { return delay; }
	
	void setDelay_by_Pi_Model(double val) { Delay_by_Pi_Model=val; }
	double getDelay_by_Pi_Model() { return Delay_by_Pi_Model; }
	void setDelay_by_Itself(double val) { Delay_by_Itself=val; }
	double getDelay_by_Itself() { return Delay_by_Itself; }
	void setDelay_by_Sub_Tree(double val) { Delay_by_Sub_Tree=val; }
	double getDelay_by_Sub_Tree() { return Delay_by_Sub_Tree; }
	void setDelay_by_Driving_Node(double val) { Delay_by_Driving_Node=val; }
	double getDelay_by_Driving_Node() { return Delay_by_Driving_Node; }

	bool checkIsLeaf() { return isLeaf; } //if yes, this node is a leaf of this steiner tree. we will compute leaf's delay
	void setIsLeaf(bool val) { isLeaf=val; }

private:
	Point *point;
	RGNode* RefRGNode;
	vector<SGEdge*> vSGEdge;
	int  type; 
	int  detail_type; 
	bool inTree;	//this node is in the present routing tree or not. routed or not routed
	int  index;		//node's index number
	bool ConnOK; //if yes, this pin connect to driving correctly
	bool isLeaf; //if yes, this node is a leaf of this steiner tree. we will compute leaf's delay
	
	double distFromDriving;	//distance between this node and driving node	

	//the following three members will be used when routing
	double tempPropagateCost;	//used when constructing the OARST
	double tempDistFromSource;	//distance between this node and source node
	double tempDistFromTree;	//only use in prim algorithm
	SGNode* tempSource; 
	SGNode* tempPreviousNode;
	SGEdge* tempPreviousEdge;

	//The following four members are used by OASG when building spanning graph
	ProcessEntry* PE_R1;	//By the paper of OARST '07 ,the effective region of one node is divided into four parts
	ProcessEntry* PE_R2;
	ProcessEntry* PE_R3;
	ProcessEntry* PE_R4;


	//---------------------------------------------------------
	//route the spanning graph with radius-factor = 1 to get the radius infomation of each sink (referenceRadius)
	//then we regard the radius/max-radius of each sink as delay ratio
	//radius/max-radius = 1 means this sink is far away driving
	//so we can use that infomation to do timing-driven routing (we will route spanning graph again!!!)
	//---------------------------------------------------------
	double referenceRadius;	//distance between this node and driving node (estimate delay)
	float delayRatio; //referenceRadius /maxRadiusInOASG


	//----------------------------------------
	//constrain-related variables
	double required_time;
	double loading; //sink's capacitance

	//----------------------------------------
	//Critical sink related variables
	bool isCriticalSink;
	bool inCriticalPath;
	float DPF; //DPF: Delay Penalty Factor, = referenceRadius /maxRadiusInOASG
	int SID; //SID: sink impact delay. New data member for new cost function use. 6/28/2008

	//----------------------------------------
	//delay-related variables
	double downtreamCap;
	double delay;
	double Delay_by_Pi_Model;
	double Delay_by_Itself;
	double Delay_by_Sub_Tree;
	double Delay_by_Driving_Node;
	
#ifdef PRIORITY_BASED
	double priority;
	double slack;
	double distFromDriver;
	double tempDistFromDriver;
	double delayForPrim;
	double mazeDelay;
	double referencePriority;
	bool slackUsed ;
	SGNode* preSlackNode ;
	bool initTree;
public:
	list<SGNode*> passedPin;
#endif
#ifdef BUF_DRIVEN
private:
	double absDistFromDriving;
	double tmpDistFromLastBuffer;
	double bufferedDSC; double DSC;
	double reducedDSC;
	double preDistFromSource;
	double lastBufferAbsDist;
	bool DFSFlag;
	bool isBuffer;
#endif

};



//Class SGEdge is a fundamental data structure of OARST
class SGEdge
{
public:
	SGEdge(SGNode* FromNode,SGNode* ToNode);
	SGNode* getParent()			{ return FromNode; }
	SGNode* getFromNode()		{ return FromNode; }
	SGNode* getToNode()			{ return ToNode; }
	
	double getCost()			{ return cost; }
	double getSqrLength()                   { return sqrLength; }
	double getLength()			{ return length; }
	void   setCost(double val)	{ cost=val; }
	void   setSqrLength(double val) { sqrLength=val; }
	void   setLength(double val) { if(length!=0){ cout<<"edge length has been initialized!\n"; exit(0); } 
	                               cost=val; length=val; }

	bool checkRouted()			{ return Routed; }
	void setRoutingState(bool val) { Routed=val; }

	SGEdge* getDualEdge()		{ return dualEdge; }
	void setDualEdge(SGEdge* E) { dualEdge=E; }


	void addIntersectEdge(SGEdge* E) {vIntersectEdge.push_back(E);}
	bool checkEnable()			{ return Enable; } //true for routable , false for non-routable
	void setEnable(bool val)	{ Enable=val; } //true for routable , false for non-routable
	SGEdge* getIntersectedEdge(int index) { return vIntersectEdge[index];}
	int getNumIntersectedEdge() { return vIntersectEdge.size();}

	bool checkTurnRect()		{ return TurnRectilinear; }
	void setRectState(bool val) { TurnRectilinear=val; }

	//----------------------------------------
	//Critical edge related functions
	bool checkInCriticalPath() { return inCriticalPath; } 
	void setInCriticalPath(bool val) { inCriticalPath=val; } 

#ifdef PRIORITY_BASED
	void setUsed() { used = true; }
	void setUnused() { used = false; }
	bool isUsed() { return used; }
#endif
#ifdef BUF_DRIVEN
	void setInInitTree() { inInitTree=true; }
	bool isInInitTree() { return inInitTree; }
#endif

private:
	SGEdge* dualEdge;	//edge that his nodes are the same but the direction is reverse
	SGNode* ToNode;	//connect to this SGNode
	SGNode* FromNode;
	double length;
	double sqrLength;
	double cost;
	bool Routed; //true for routed , false for not routed yet	
	bool Enable; //true for routable , false for non-routable
	bool TurnRectilinear; //true for been translated rectilinear, false for not
	vector<SGEdge*> vIntersectEdge; //edges intersect this one


	//----------------------------------------
	//Critical edge related variables
	bool inCriticalPath;

#ifdef PRIORITY_BASED
	bool used;
	bool inInitTree; 
#endif
};


//The following four classes such as Point,Interval,ProcessEntry and SweepEntry are for OASG usage

class Interval{
public:
	Interval(int X,int Y1,int Y2,int Layer,size_t index,int type) : x(X) , y1(Y1) , y2(Y2) , layer(Layer) , ObstacleIndex(index) ,type(type) {}
	int getX() const {return x;} //Read only
	int getY1() const {return y1;}
	int getY2() const {return y2;}
	int getLayer() const {return layer;}
	size_t getIndex() const {return ObstacleIndex;}
	int getType() const {return type;}
private:
	int x;
	int y1;
	int y2;
	int layer;
	size_t ObstacleIndex; //Only for Obstacle
	int type;
};

class AEntry {
public:
	AEntry(SGNode* N) : N(N)  {}
	int getX() const { return N->getX();} //Read only
	int getY() const { return N->getY();}
	int getLayer() const {return N->getLayer();}
	SGNode* getNode() { return N; }
private:
	SGNode* N;
};

class ProcessEntry {
public:
	ProcessEntry(int X,int Y,int Layer,int region,int type,SGNode* parent);
	int getX() const {return point->getX();} //Read only
	int getY() const {return point->getY();}
	int getLayer() const {return point->getLayer();}	

	int getType() const {return type;}
	int getRegion() const {return region;}
	SGNode* getParent() { return parent; }

	vector<IEntry*> ISet; // Obstacle Y interval
	vector<AEntry*> ASet; // SGEdge Candidate Container

private:
	Point* point;
	SGNode* parent;	//point out who is his parnet. So PE can use this pointer to back trace which node has this PE.
	int region;
	int type;

};

//SweepEntry will constain Pin or Obstacle
class SweepEntry {
public:
	SweepEntry(SGNode* N,int type) : N(N), type(type), ID(_UNSET) {}
	
	int getX() const {
		return N->getX();
	}
	int getY() const {
		return N->getY();
	}
	int getLayer() const {
		return N->getLayer();
	}
	SGNode* getN() { 
		return N; 
	}

	void set_Left_Lower_N(SweepEntry* N) { 
		N_Left_Lower = N;
	}
	void set_Left_Upper_N(SweepEntry* N) { 
		N_Left_Upper = N;
	}
	void set_Right_Lower_N(SweepEntry* N) { 
		N_Right_Lower = N;
	}
	void set_Right_Upper_N(SweepEntry* N) { 
		N_Right_Upper = N;
	}
	SweepEntry* get_Left_Lower_N() { 
		return N_Left_Lower;
	}
	SweepEntry* get_Left_Upper_N() { 
		return N_Left_Upper;
	}
	SweepEntry* get_Right_Lower_N() { 
		return N_Right_Lower;
	}
	SweepEntry* get_Right_Upper_N() { 
		return N_Right_Upper;
	}
	void add_His_vA(SweepEntry* N,int CheckDirection);
	void Check_vA(SweepEntry* N,int CheckDirection);

	SweepEntry* get_His_vA(int index)
	{	return vA_own[index];
	}
	int get_vA_size() { return vA_own.size(); }
	void erase_vA(int index) { vA_own.erase(vA_own.begin()+index); }

	void add_His_vAs(SweepEntry* N) 
	{	vAs_own.push_back(N);
	}
	SweepEntry* get_His_vAs(int index)
	{	return vAs_own[index];
	}
	int get_vAs_size() { return vAs_own.size(); }
	
	void clear_vA() { vA_own.clear(); }
	void clear_vAs() { vAs_own.clear(); }


	int getType() const {return type;}

	int getID() { return ID; }
	void setID(int val) { ID = val; }

private:
	int ID;
	SGNode* N;
	SweepEntry* N_Left_Lower;
	SweepEntry* N_Left_Upper;
	SweepEntry* N_Right_Lower;
	SweepEntry* N_Right_Upper;
	vector<SweepEntry*> vA_own; //contain his own all visible nodes
	vector<SweepEntry*> vAs_own; //contain his own all edge connection candidates
	int type;
};




class Vertical_Block_Edge
{
public:
	Vertical_Block_Edge(SweepEntry* N_1,SweepEntry* N_2)
	{	if(N_1->getX() != N_2->getX())
		{	cout<<"Block_Edge egde_type wrong\n";
			exit(0);
		}


		if(N_1->getY() < N_2->getY())
		{	N1=N_1;
			N2=N_2;
		}
		else
		{	N1=N_2;
			N2=N_1;
		}
	}
	int getX() const {
		return N1->getX();
	}

	int getY1() const {
		return N1->getY();
	}

	int getY2() const {
		return N2->getY();
	}

	SweepEntry* getN1() { return N1; }
	SweepEntry* getN2() { return N2; }

private:
	SweepEntry* N1;
	SweepEntry* N2;
};

class Horizontal_Block_Edge
{
public:
	Horizontal_Block_Edge(SweepEntry* N_1,SweepEntry* N_2)
	{	if(N_1->getY() != N_2->getY())
		{	cout<<"Block_Edge egde_type wrong\n";
			exit(0);
		}

		if(N_1->getX() < N_2->getX())
		{	N1=N_1;
			N2=N_2;
		}
		else
		{	N1=N_2;
			N2=N_1;
		}
	}

	int getY() const {
		return N1->getY();
	}

	int getX1() const {
		return N1->getX();
	}

	int getX2() const {
		return N2->getX();
	}

	SweepEntry* getN1() { return N1; }
	SweepEntry* getN2() { return N2; }

private:
	SweepEntry* N1;
	SweepEntry* N2;
};

struct Horizontal_Line {
	explicit Horizontal_Line(Horizontal_Block_Edge* EH) : EH(EH) {}
	Horizontal_Block_Edge* EH;
};

struct Vertical_Line {
	explicit Vertical_Line(Vertical_Block_Edge* EV) : EV(EV) {}
	Vertical_Block_Edge* EV;
};







class OASpanningGraph {
public:	
	OASpanningGraph(Database &data,elmoreDelay *delay); 
	Database* getDatabase() { return data; }
	
												 //import the data that needs to be processed and initialize some member data
	void buildOASG(Database &data,int numLayer); //parameter numLayer specify the number of layers to build
	
	void RTreeInsertNode(SGNode* node); //functions that manipulate region tree
	//int nodeRTreeSearch(const Point *P11,const Point *P22,bool __cdecl a_resultCallback(int a_data, void* a_context));
	int nodeRTreeSearch(const Point *P11,const Point *P22,bool a_resultCallback(int a_data, void* a_context));

	double getLongestRadius() { return longestRadius; }
	void setLongestRadius(double val) { longestRadius=val; }	
	SGNode* getWorstSlackSink() { return WorstSlackSink; }
	void setWorstSlackSink(SGNode* N) { WorstSlackSink=N; }
	double computeRadius();
      void setDrivingNode(SGNode* node) { DrivingNode=node; DrivingNode->setDistFromDriving(0); }
	SGNode* getDrivingNode() { return DrivingNode; }
	void computeDistFromDriver();
	double getRoutedSGEdgeLength();

	void AddingEdge(ProcessEntry* PE);	//Adding edge. Besides, analyze the candidates in set A to avoid adding redundant edge 
	SGEdge* AddingEdge(SGNode* FromNode,SGNode* ToNode);	//main use : add boundary of obstacle or dual edge. and function will return edge that it has just added


	//-------------------------------------
	//OASG check methods
	//-------------------------------------
	void Check_Timing_Violation(vector<SGNode*> &vViolationNode,double driver_arrival_time);
	bool Corner_Connection_Check(SweepEntry* U,SweepEntry* V);
	void Check_Conn();
	void Check_Duplicate_Node();
	void Check_Edge_Cost_Length();
	void Check_Sink_Loading();
	void CheckEdgeIntersection();
	bool DetectLineIntersection(SGEdge* E1,SGEdge* E2);

	//-------------------------------------
	//We exploit ISPD 08 Long's method to construct OASG
	//-------------------------------------
	void Quad1();
	void Quad2();
	void Quad3();
	void Quad4();

	bool Block_By_vAev_vAeh(SweepEntry* N1,SweepEntry* N2);
	bool Block_By_vAev(int Y,int X1,int X2); //input H line
	bool Block_By_vAeh(int X,int Y1,int Y2); //input V line
	
	bool U_On_Block_Edge_vAev(SweepEntry* U);
	bool U_On_Block_Edge_vAeh(SweepEntry* U);

	void Add_Block_Edge(SweepEntry* N,int AddType);
	void Remove_Block_Edge(SweepEntry* N,int RemoveType);
	//----------------------------------------------------------------------------------------------

	int getNumEdge() {return vSGEdge.size();}
	int getNumNode() {return vSGNode.size();}	
	SGNode* getNode(int index) { return vSGNode[index]; }
	SGEdge* getEdge(int index) { return vSGEdge[index]; }

	void AddCriticalPin(SGNode* N) { vCritical_SGPin.push_back(N); vCritical_SGNode.push_back(N); }
	void DelCriticalPin(int index) { vCritical_SGPin.erase(vCritical_SGPin.begin()+index); }
	int getCriricalNumPin() {return vCritical_SGPin.size();}
	SGNode* getCriricalPin(int index) { return vCritical_SGPin[index]; }
	void AddCriticalNode(SGNode* N) { vCritical_SGNode.push_back(N); }
	int getCriricalNumNode() {return vCritical_SGPin.size();}
	SGNode* getCriricalNode(int index) { return vCritical_SGNode[index]; }


private:

	Database* data;
	SGNode* DrivingNode;	//driving node in the whole net
	SGNode* WorstSlackSink;
#ifdef PRIORITY_BASED
	elmoreDelay *delay;
#endif


	//The purpose of seperating R-Tree is that we want to speed up the time of querying obstacle
	RTree<int,int,3,float> nodeRTree; //for fast querying to generate sources when do multi-source single target routing
									   //set ELEMTYPEREAL as float, or compiling will be failed

	int numRTreeNodes; //stands for the number of nodes in R Tree
	//-----------------------------------------------------------------------------
	friend void display(); //in order to drawing Spanning Graph more easily
	//-----------------------------------------------------------------------------
	//record the longest radius in spanning graph
	double longestRadius;
	vector<SGNode*> vCritical_SGPin; //contain the sinks with its radius are longer than X% of longest radius
	vector<SGNode*> vCritical_SGNode; //contain the nodes in critical path
	vector<SGEdge*> vCritical_SGEdge; //contain all the edges in critical path

	vector<SweepEntry*> vAs; //contain all edge connection candidates
	//-----------------------------------------------------------------------------	
	vector<SweepEntry*> vSweepEntry;
	vector<SweepEntry*> vA; //contain active nodes
	vector<Vertical_Block_Edge*> vAev; //contain vertical block edge
	vector<Horizontal_Block_Edge*> vAeh; //contain horizontal block edge
	//-----------------------------------------------------------------------------
	vector<SGNode*> vSGNode; //including all Pins and the obstacles' vertices, it's a basic data structure when routing steiner tree						 
	vector<SGEdge*> vSGEdge; //Contain all the spanning egdes
	//-----------------------------------------------------------------------------
};








#endif

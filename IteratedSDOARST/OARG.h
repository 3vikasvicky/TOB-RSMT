#ifndef OARG_H //Avoid duplicate definition
#define OARG_H

#include "OARST.h"
//using namespace auxiliary; //for auxiliary use
class RGNode;
class RGEdge;



class RGNode
{	
public:
	RGNode(int X,int Y,int Layer,int type);
	RGNode(SGNode* N); //copy SGNode constructor

	int getType() { return type; }	//return the node's type that is a pin or an obstacle
	bool checkInTree() { return inTree; }
	void setTreeState(bool val) { inTree=val; }

	bool checkIsLeaf() { return isLeaf; } //if yes, this node is a leaf of this steiner tree. we will compute leaf's delay
	void setIsLeaf(bool val) { isLeaf=val; }

	bool checkedConn() { return ConnOK; } //if yes, this pin connect to driving correctly
	void setCheckedConn(bool val) { ConnOK=val; }

	int getX() { return point->getX(); }
	int getY() { return point->getY(); }
	int getLayer() { return point->getLayer(); }
	Point* getPoint() { return point; } 

	size_t getNumEdge() { return vRGEdge.size(); }
	RGEdge* getEdge(size_t index) { return vRGEdge[index]; }
	void AddEdge(RGEdge* E) { vRGEdge.push_back(E); };
	void EraseEdge(RGEdge* delE);

	SGNode* getSGNode() { /*if(SG_Node==NULL)
						  {		cout<<"Error occured! this RG node has no SG node reference\n(this RG node must be a turning node)\n";
								exit(0);
						  }*/
						  return SG_Node; }
	
	double getDistFromDriving() { return distFromDriving; }
	void setDistFromDriving(double val) { distFromDriving=val; }

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

	//----------------------------------------
	//constrain-related functions
	double getRequiredTime() const { return required_time; }
	void setRequiredTime(double val) { required_time=val; }
	double getLoading() const { return loading; }
	void setLoading(double val) { loading = val; }
	int getIndex() { return index; }
	void setIndex(int id) { index=id; }

private:
	Point* point;
	vector<RGEdge*> vRGEdge;
	SGNode* SG_Node;

	bool isLeaf; //if yes, this node is a leaf of this steiner tree. we will compute leaf's delay
	bool ConnOK; //if yes, this pin connect to driving correctly
	int type; 
	bool inTree;	//this node is in the present routing tree or not. routed or not routed
	
	double distFromDriving;	//distance between this node and driving node

	//----------------------------------------
	//delay-related variables
	double downtreamCap;
	double delay;
	double Delay_by_Pi_Model;
	double Delay_by_Itself;
	double Delay_by_Sub_Tree;
	double Delay_by_Driving_Node;

	//----------------------------------------
	//constrain-related variables
	double required_time;
	double loading; //sink's capacitance
	
	int index;

};


class RGEdge
{
public:
	RGEdge(RGNode* FromNode,RGNode* ToNode);
	RGNode* getFromNode() { return FromNode; }
	RGNode* getToNode() { return ToNode; }
	
	double getLength() { return length; }
	void setLength(double val) { if(length!=0){ cout<<"edge length has been initialized!\n"; exit(0); } 
								length=val; }


	RGEdge* getDualEdge() { return dualEdge; }
	void setDualEdge(RGEdge* E) { dualEdge=E; }

	void setUsed() {used=true;}
	bool isUsed() { return used; }

private:
	RGNode* ToNode;	
	RGNode* FromNode;
	double length;
	RGEdge* dualEdge;
	bool used;
};

class UnProcessSGEdge
{
public:
	UnProcessSGEdge(SGEdge* edge) : SG_edge(edge) { ComputeCost(); }
	void ComputeCost();
	double getCost() { return Cost; }
	
private:
	SGEdge* SG_edge;
	double Cost;
	
};

class UnProcessed_SGEdge_CostLessThan
{	public:
		bool operator() (SGEdge* E1,SGEdge* E2)
		{	return E1->getCost() < E2->getCost();
		}
};

//The following seven structs are wrapper types (make interfaces easy to use correctly)
struct StruFromRGNode {	
	RGNode* Node;
	explicit StruFromRGNode(RGNode* N) : Node(N) {}
};
struct StruToRGNode {	
	RGNode* Node;
	explicit StruToRGNode(RGNode* N) : Node(N) {}
};


//-------------------------------------------------------------------------------------------------------
//refer to OASG's information, then build a whole new graph "OARectilinearGraph" to represent rectilinear graph.
//(note that OASG is for spanning graph)
//after OARectilinearGraph is builded, you can disgard OASG because we won't refer to OASG again.
//-------------------------------------------------------------------------------------------------------
//method to build OARectilinearGraph: starting with driving node and propagating to other SG node,
//                      in the meanwhile rectilinearize the encountered SG edges.
//						the whole process won't stop until the OASG has been traversed.
//-------------------------------------------------------------------------------------------------------
//by the way, the distance from driving node has to been recomputed.


typedef vector<SGEdge*> vUn_Processed_SGEdge;
typedef vector<RGNode*> vUn_Propagated_RGNode;

class OARectilinearGraph
{
public:
	OARectilinearGraph(SGNode* DrivingNode,OASpanningGraph* OASG);	

	void TurnRectilinear_From_Driver();
	void TurnRectilinear_From(RGNode* StartNode);
	void Turn_One_Edge_Rectilinear(RGNode* StartNode);
	void TurnRectilinear_Process_One_Node();	
	void computeDistFromDriver(); //trace from source to sinks and compute the right distance from driver
	void UShapeRemove();
	bool SmoothEdge();
	bool FlipEdge();
	void FlipEdge_CheckOne(RGNode* checkN,int& remove_count);
	bool PushEdge1();
	bool PushEdge2();
	void PushEdge_CheckOne1(RGNode* checkN,int& remove_count);
	bool PushEdge_CheckOne2(RGNode* checkN,int& remove_count);
	
	void AddUnPropNode(RGNode*);
	void SearchSlantEdges(RGNode* ProRGNode);
	

	int getNumUnPropRGNode(){ return vUn_RGNode.size(); }
		
	RGNode* Pick_ProRGNode(vUn_Propagated_RGNode &vUn_RGNode);
	SGEdge* Pick_E(vUn_Processed_SGEdge &vUn_SGEdge);

	//SGEdge* Pick_E0(SGEdge* E, SGNode* FromNode,int &E_E0_Case);
	SGEdge* Pick_SE0(SGEdge* E, SGNode* FromNode,int &E_E0_Case,double &Score_SG);
	RGEdge* Pick_RE0(SGEdge* E, RGNode* FromNode,double &Score_RG);

	void TurnRectilinear(SGEdge* E);

	double Compute_Share_Length(SGNode* N1,SGNode* N2,SGNode* Root,int &E_E0_Case);	
	double Compute_Share_Length(SGNode* N1,RGNode* N2,RGNode* Root,int &E_E0_Case);

	RGNode* AddRGNode(int X,int Y,int Layer,int type);
	RGNode* AddRGNode(SGNode* N);	
	RGEdge* AddRGEdge(StruFromRGNode FromNode,StruToRGNode ToNode,double assignDist=0,bool checkOverlap=true);
	void AddSlantRGEdge(StruFromRGNode FromNode,StruToRGNode ToNode,double assignDist=0,bool checkOverlap=true);
	void DelRGEdge(RGEdge* delE);
	void DelRGNode(RGNode* delN);

	bool EliminateOverlap(RGEdge* RE); //avoid overlaped edge; true for removing, false for no overlap
	void Merge2RGEdge(RGEdge* RE,RGEdge* Overlap_RE); 

	bool IncludePin(double x,double y,int z);
	bool IncludePin(RGNode* N);

	RGEdge* Find_connEdge(RGNode* fromNode,RGNode* toNode);
	int getNumEdge() {return vRGEdge.size();}
	int getNumNode() {return vRGNode.size();}
	RGNode* getNode(int index) { return vRGNode[index]; }
	RGEdge* getEdge(int index) { return vRGEdge[index]; }
	RGNode* getDrivingNode() { return DrivingNode; }

	RGNode* getWorstDelaySink() { return WorstDelaySink; }
	void setWorstDelaySink(RGNode* N) { WorstDelaySink=N; }
	RGNode* getWorstSlackSink() { return WorstSlackSink; }
	void setWorstSlackSink(RGNode* N) { WorstSlackSink=N; }
	double getLongestRadius() { return longestRadius; }
	void setLongestRadius(double val) { longestRadius=val; }
	double computeRadius();
	double getRoutedRGEdgeLength();

	void setDisConnect_Sink(RGNode* N) { DisConnect_Sink = N;}
	RGNode* getDisConnect_Sink() {return DisConnect_Sink;}


	void setReduceUOK(bool val) { ReduceUOK=val; }
	bool getReduceUOK() { return ReduceUOK; }

	//----------------------------------------
	//verification-related functions
	void Check_Edge(); //Check does duplicated Edge exist in OARG ?
	void OverlapCorrect(); //Correct overlap edge in OARG if any
	void Check_Node();
	void Check_Node_All_In_Tree();
	void Check_All_Pin_In_Tree(Database &data);
	void Check_Node_Type();
	void Check_Duplicate_Node();
	void Check_Timing_Violation(double driver_arrival_time);
	void Check_Conn(); //from sink trace back to driving node by the value of distFromDriving
					   //if cannot reach driving node, that's must be a error when rectilinearizing slant edges

private:
	OASpanningGraph* OASG;
	bool ReduceUOK;

	RGNode* DrivingNode;
	RGNode* WorstDelaySink;
	RGNode* WorstSlackSink;
	RGNode* DisConnect_Sink;
	double longestRadius;
	vector<RGNode*> vRGNode;
	vector<RGEdge*> vRGEdge;

	friend double routedRGEdgeLength(vector<RGEdge*> &vRGEdge); 
	friend double ResultOfRectiSG();

	//force to store un-propagated node as RG node.
	vUn_Propagated_RGNode vUn_RGNode;
	vUn_Processed_SGEdge vUn_SGEdge;



};

#endif

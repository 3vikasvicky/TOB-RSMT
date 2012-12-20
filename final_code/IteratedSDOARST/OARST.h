#ifndef OARST_H //Avoid duplicate definition
#define OARST_H

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <stack>
//#include <gl\glaux.h>
//#include <gl\glut.h>
#include "database.h"
#include "OASG.h"
#include "OARG.h"
#include "RTree.h"
#include "elmoreDelay.h"
#include "sorting.h"


#include <algorithm>
#include <list>
#include <vector>
#include <set>
#include <math.h>
#include <queue>
#include "basic.h"
#include "RunTime.h"
#include "definition.h"


namespace auxiliary //This namespace is only for auxiliary and debuging
{
	void PrintPins(Database& database);
	void PrintPin(Database& database,size_t index);
	void PrintObstacles(Database& database);
	void PrintObstacle(Database& database,size_t index);
	void PrintPE(ProcessEntry* PE);
	void PrintNode(SGNode* N);
	void PrintRGNode(RGNode* N);
	void PrintRGNode_Delay(RGNode* N);
	void PrintEdge(SGEdge* E);
	void PrintRGEdge(RGEdge* E);
	double ComputeLength(SGNode* N1,SGNode* N2);
	double ComputeLength(RGNode* N1,RGNode* N2);
	double ComputeManhattanDist(SGNode* N1,SGNode* N2);
	bool Same_RGNode(RGNode* RN1,RGNode* RN2);

	bool Is_Vertical_Line(SGEdge* SG_E);
	bool Is_Horizontal_Line(SGEdge* SG_E);
	bool Is_Vertical_Line(RGEdge* RG_E);
	bool Is_Horizontal_Line(RGEdge* RG_E);
	int Compute_Quadrant(SGNode* N,SGNode* Root);
	int Compute_Quadrant(SGNode* N,RGNode* Root);
	int Compute_Quadrant(RGNode* N,RGNode* Root);

	bool IsOverlap(RGEdge* RE1,RGEdge* RE2);
	bool Horizontal_Overlap(RGEdge* RE1,RGEdge* RE2);
	bool Vertical_Overlap(RGEdge* RE1,RGEdge* RE2);

	RGEdge* FindOverlapEdge(RGNode* Root,RGEdge* RE);
	RGEdge* FindOverlapEdge(RGEdge* RE);

	bool V_In_U_Quad1(SweepEntry* U,SweepEntry* V);
	bool V_In_U_Quad2(SweepEntry* U,SweepEntry* V);
	bool V_In_U_Quad3(SweepEntry* U,SweepEntry* V);
	bool V_In_U_Quad4(SweepEntry* U,SweepEntry* V);

	bool IsCorner(RGNode* N);
}

using namespace std;
using namespace auxiliary; //for auxiliary use

class TwoPinNet {
public:
	TwoPinNet(SGNode* Source,SGNode* Target) :Source(Source), Target(Target) { }
	TwoPinNet() {}
	SGNode* getSource() { return Source; }
	SGNode* getTarget() { return Target; }
	void setSource(SGNode* Source) { this->Source=Source; }
	void setTarget(SGNode* Target) { this->Target=Target; }
	double getEstimatedDist() { return estimated_distance; }
	void setEstimatedDist(int val) { estimated_distance=val; }
private:
	SGNode* Source;
	SGNode* Target;
	double estimated_distance;
};

class OARSteinerTree {
public:     //DelayMapOK(false),
	OARSteinerTree(OASpanningGraph* OASG, elmoreDelay* DelayModel) : OASG(OASG), DelayModel(DelayModel), OARG(NULL), maxRadiusInOASG(0), CriticalTrunkOK(false), RedirectOK(false)
	{ 
		radiusFactor=1; searchScale=20; // default 20
		penaltyScale=10; // 10
		cout<<"radiusFactor = "<<radiusFactor<<"  (range 0 ~ 1, 0 for MST, 1 for SPT)\n";
		cout<<"searchScale  = "<<searchScale<<"  (searching range of R-Tree)\n";
		cout<<"penaltyScale = "<<penaltyScale<<" (2-pin net order)\n";
#ifdef PRIORITY_BASED
		maxSlack = _NINFINITE ;
		minSlack = _INFINITE ;
		PriorityOK = false;
#endif

	}
	elmoreDelay* getDelayModel() { return DelayModel; }
	void doRefinedPrim(Database &data);	//divide the whole single net into several two-pin nets, only trace connected SGNode
	void doPrim(Database &data); //mainly consider pins, then use R-Tree to capture obstacle
	
	void designateDrivingNode(Database &database);
	void setDrivingNode(SGNode* N) { DrivingNode=N; DrivingNode->setDistFromDriving(0); }
	SGNode* getDrivingNode() { return DrivingNode; }
	int getNumTwoPinNet() { return vTwoPinNet.size(); }
	TwoPinNet* getTwoPinNet(int index) { return vTwoPinNet[index]; }
	
	void initOARG() { OARG = new OARectilinearGraph(DrivingNode,OASG); }
	OARectilinearGraph* getOARG() { return OARG; }
	OASpanningGraph* getOASG() { return OASG; }

	void RouteSpanningGraph();	//start routing all the two-pin nets
	void ConnectTwoPin(SGNode* FromN,SGNode* ToN);	//connect two node by Spanning Graph
	void computeMultiSources(SGNode* Source,SGNode* Target,vector<SGNode*> &vMultiSource); //use R-Tree to get multi-source candidates
	void routeTwoPinNet(int index,SGNode* Source,SGNode* Target,vector<SGNode*> &vMultiSource);	//route one two-pin net
	void mazePropagate(SGNode* SOURCE,vector<SGNode*> &vMultiSource,SGNode* Target,vector<SGNode*> &UsedNodeSet); //part 1 of routing two-pin net
	SGNode* backTracing(vector<SGNode*> &vMultiSource,SGNode* Target); //part 2 of routing two-pin net, return meeting source
	void recomputeDistFromDriver(SGNode* Source,SGNode* Target); //part 3 of routing two-pin net
	void RouteEdge(SGEdge* E);  //route this edge physically
	void UnRouteEdge(SGEdge* E);//un-route this edge physically
	double tempPropagateCost(SGNode* Source,SGNode* Target,SGNode* ByNode);	//estimate needed cost if routing to target by this ToNode.

	//---------------------------------------------------------
	//perform branch moving (method from A Novel Performance-Driven Topology Design Algorithm 08)
	//to reduce WNS and # of timing violation sinks
	//branch here means edge that connects two nodes
	//---------------------------------------------------------
	bool ReduceNegativeSlack();
	RGNode* DisConnect_Sink;
	bool MoveToPreviousNode(RGNode* ReduceWanted_Sink); //avoid doing nothing
	bool Redirect(RGNode* ReduceWanted_Sink);
	bool DisconnectSink(RGNode* ReduceWanted_Sink,RGNode* upstreamNode); //make that sink unconnectable from tree; if fail return false
	RGNode* ConnectWNS_Sink(RGNode* ReduceWanted_Sink);
	bool NRootOnReduceWantedSink(RGNode* ReduceWanted_Sink,RGNode* N); //avoid to connect disconnected sink to a subtree that rooted on itself
	                                        //that will cause a disconnection
	void Remove_Redundant_Edges(RGNode* upstreamNode,RGNode* fromNode);
	RGNode* Find_SubTreeRoot_Upstream(RGNode* Sink);

	//---------------------------------------------------------
	//route the spanning graph with radius-factor = 1 to get the radius infomation of each sink
	//then designate critical sink its radius (distance from driving node) is longer than 90%
	//set the sub-tree containing these critical sink unchanged
	//and rip-up other sub-tree.
	//---------------------------------------------------------
	//In the following process, we can reroute other sinks with radius-factor = 0, 
	//and by our sharing methodology we can avoid too much sub-tree that growing from critical path
	//---------------------------------------------------------
	void GrowCriticalPath(Database &database,elmoreDelay* elmoreModel,double driver_arrival_time);
	void GrowOneCriticalPath(SGNode* Sink);
	//---------------------------------------------------------
	void setCriticalPathOK() { CriticalTrunkOK=true; }
#ifdef PRIORITY_BASED
	void setPriorityOK() { PriorityOK=true ; }
	//---------------------------------------------------------
	void ReConstrCriticalPath(SGNode* Source,SGNode* Target);
	//---------------------------------------------------------
	void Restore_exceptCriticalPath();
	//Find Critical Sink : first use SPT to route tree, then compute sink delay (worst sink delay and average sink delay)
	//then set Critical-Sink-Ratio = average sink delay / worst sink delay)
	//if Critical-Sink-Ratio approach 1, means total wire length dominate tree performance
	//we should add fewer critical sink as better (in order not to restrict tree topology to SPT)
	//if Critical-Sink-Ratio approach 0, means radius dominate tree performance and Rd is not very fatal
	//we should add some essential critical sink as better (in order to keep good radius)
	//---------------------------------------------------------
	//we use actual delay information as reference "only" in this function
	//we still use radius of each sink as our major reference (such as set each sink's share-ratio)
	//---------------------------------------------------------
	void Compute_CTF(Database &database,elmoreDelay* elmoreModel);

	//---------------------------------------------------------
	//Designate timing constrain of each sink
	//after performing SPT construction (radius-factor = 1)
	//then do Elmore delay model
	//finally in function GenTimingConstrain, each sink's delay multiplies a constant (2.5) as its timing bound
	//and write result to a text file
	//---------------------------------------------------------
	void GenTimingConstrain(char* filename,Database& database,float timing_constrain_mul,elmoreDelay* elmoreModel);
	void setRunTimeSet(RunTime* r) { RunTimeSet=r; }
	RunTime* getRunTimeSet() { return RunTimeSet; }

#ifdef PRIORITY_BASED
	double getMinSlack() { return minSlack ; }
	void setMinSlack(double s) { minSlack = s ; }
	double getMaxSlack() { return maxSlack ; }
	void setMaxSlack(double s) { maxSlack = s ; }
	void computeSlackForEachNode(Database &database);
	void Compute_Priority_CTF(Database &database,elmoreDelay* elmoreModel);
	void Assign_Reference_Priority(Database &database);
#endif
#ifdef BUF_DRIVEN
	void Compute_DSC();
	void Pseudo_BI();
#endif

private:
	OASpanningGraph* OASG;	//container stores Spanning Graph info
	elmoreDelay* DelayModel;
	OARectilinearGraph* OARG; 
	vector<TwoPinNet*> vTwoPinNet;	//store the routing nets in order
	RunTime* RunTimeSet;

	friend double routedSGEdgeLength(vector<SGEdge*> &vEdge); 
	friend double computeRadius(OASpanningGraph* OASG);
	friend void ResultOfRouteSG();
	friend void display(void);

	SGNode* DrivingNode;	//driving node in the whole net
	
	double maxRadiusInOASG;
	float  radiusFactor;	  //affect the routing topology
	int    searchScale;       //affect R-tree search area
	int    penaltyScale;      //give two-pin net a penalty when this net is blocked by obstacle
#ifdef PRIORITY_BASED
	double maxSlack ;
	double minSlack ;
	float PriorityCTF;
#endif
	float  CTF;               //CTF: Criticality threshold factor, designate critical sink its radius longer than [CTF] x longest radius
	                          //CTF => 1 : MST,  CTF => 0 : SPT

	//---------------------------------------------------------
	//after delay map is generated, this bool variable will be set OK
	//then when routing spanning graph, we can regard each sink's (delay-ratio)^2 * distFromDriving as new gVal
	//(In A* algorithm g(node) stands for the 'exact' distance from source)
	//---------------------------------------------------------
	//bool DelayMapOK;

	//---------------------------------------------------------
	//after critical path is computed, this bool variable will be set OK
	//then when routing spanning graph, we can regard each sink's share-ratio^2 * distFromDriving as new gVal
	//(In A* algorithm g(node) stands for the 'exact' distance from source)
	//---------------------------------------------------------
	bool CriticalTrunkOK;
#ifdef PRIORITY_BASED
	bool PriorityOK;
#endif


	//---------------------------------------------------------
	//now major purpose is reduce WNS sink, so don't use other parameters such as 
	//(1-this->CTF) or pow(Source->getAntiShareRatio(),2) to adjust the tree not too like SPT
	//We now just want a direct link to WNS sink (so we want a pure SPT¡I¡I¡I¡I)
	//---------------------------------------------------------
	bool RedirectOK;
};

#endif
#endif

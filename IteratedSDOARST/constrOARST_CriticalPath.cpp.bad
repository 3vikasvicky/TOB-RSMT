#include "OARST.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;
extern bool secondTime;
extern string argv1_path;

void OARSteinerTree::Compute_CTF(Database &database,elmoreDelay* elmoreModel)
{
	//------------------------------------------------------
	//compute average sink delay and worst sink delay
	//------------------------------------------------------
	cout<<"--[Apply Elmore delay model to compute sink delay (OASG)]\n";
	double average_sink_delay = elmoreModel->Compute_OASG_Delay(database);
	//------------------------------------------------------

	//------------------------------------------------------
	//compute CTF
	//------------------------------------------------------
	CTF = (float) average_sink_delay / elmoreModel->getWorseDelaySinkInOASG()->getDelay();
	cout<<"==> now CTF (Criticality Threshold Factor) = "<<this->CTF<<"\n";
}
#ifdef PRIORITY_BASED
void OARSteinerTree::Compute_Priority_CTF(Database &database,elmoreDelay* elmoreModel)
{
	//------------------------------------------------------
	//compute average sink delay and worst sink delay
	//------------------------------------------------------
	cout<<"--[Apply Elmore delay model to compute sink delay (OASG)]\n";
	double average_sink_delay = elmoreModel->Compute_OASG_Delay(database); // Homer: useless in priority_based mode
	double average_priority = 0;
	if(secondTime==false){
		average_priority = elmoreModel->Compute_OASG_Priority(database);
	}
	else{
		vector< vector<int> > delays;
		ifstream infile;
		string file_name = "../benchmarks/" + argv1_path + "/timing.txt";
		cout<<file_name<<endl;
		const char * c = file_name.c_str();
		infile.open(c);
		assert(infile.is_open()==true);		
		while(infile.good()){
			string line;
			getline (infile, line);
			int index1 = line.find("(");
			int index2 = line.find(",");
			int index3 = line.find(")");
			int index4 = line.find("is");
			if(index1==-1 || index2==-1 || index3==-1 || index4==-1){
				break;
			}
			int x = atoi(line.substr(index1+1, index2).c_str());
			int y = atoi(line.substr(index2+2, index3).c_str());
			int delay = atoi(line.substr(index4+3).c_str());
			vector<int> innode (3);
			innode.at(0)=x;
			innode.at(1)=y;
			innode.at(2)=delay;
//			cout<<x<<"\t"<<y<<"\t"<<innode.at(0)<<"\t"<<innode.at(1)<<endl;
			delays.push_back(innode);
		}
		infile.close();

//		for (int j =0; j<delays.size();j++){
//			cout<<delays.at(j).at(0)<<"\t"<<delays.at(j).at(1)<<endl;
//		}
		
		double PRIORITYSUM=0;
		double WORSTPRIORITY=_INFINITE;
		double BESTPRIORITY = _NINFINITE;
		SGNode* worstPriorityNode = NULL;
		SGNode* bestPriorityNode = NULL;
		  for(int i=0;i < OASG->getNumNode();i++)
		  {
			SGNode* N = OASG->getNode(i);
			if(N->getType() == _PIN && N !=  OASG->getDrivingNode())
			{
			  double priority;
			  assert(N->getSlack()!=_INFINITE);
			  for (int j =0; j<delays.size();j++){
				  //cout<<N->getX()<<"\t"<<N->getY()<<"\t"<<delays.at(j).at(0)<<"\t"<<delays.at(j).at(1)<<endl;
				  if((delays.at(j).at(0)==N->getX()) && (delays.at(j).at(1)==N->getY())){
					  priority=N->getRequiredTime()-delays.at(j).at(2);
				  }
			  }
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
		  elmoreModel->setWorstPrioritySinkInOASG(worstPriorityNode);
		  elmoreModel->setBestPrioritySinkInOASG(bestPriorityNode);
		  elmoreModel->setOrgWorstPriority( WORSTPRIORITY ) ;
		  elmoreModel->setOrgBestPriority( BESTPRIORITY ) ;
		  average_priority = (double) (PRIORITYSUM / (database.getPinNum() - 1));// - WORSTPRIORITY;
	}
	//------------------------------------------------------

	//------------------------------------------------------
	//compute PriorityCTF
	//------------------------------------------------------
	//PriorityCTF = (double) ((0-elmoreModel->getOrgWorstPriority()) / elmoreModel->getBestPrioritySinkInOASG()->getPriority());
	//PriorityCTF = (double) (average_priority / elmoreModel->getBestPrioritySinkInOASG()->getPriority());
	//PriorityCTF = PriorityCTF * elmoreModel->getWorseDelaySinkInOASG()->getDelay() ;
	//PriorityCTF = average_priority * getOASG()->getNumNode()/ (getOASG()->getDatabase()->getPinNum() *50);
	//assert(elmoreModel->getWorseDelaySinkInOASG()->getDelay()!=0);
	//CTF = average_sink_delay / elmoreModel->getWorseDelaySinkInOASG()->getDelay();
	PriorityCTF = average_priority ;// * getOASG()->getNumNode()/ (getOASG()->getDatabase()->getPinNum() *50);
	//PriorityCTF = 0 - elmoreModel->getOrgBestPriority() ;
	cout<<"==> now Priority_CTF (Criticality Threshold Factor) = "<<this->PriorityCTF<<"\n";
}
#endif

void OARSteinerTree::GrowCriticalPath(Database &database,elmoreDelay* elmoreModel,double driver_arrival_time)
{
	double maxRadius = 0;

	//set radius factor = 1 to sinks to get radius (distance from driver)
	cout<<"--[set radius-factor = 1 to sinks to get radius (distance from driver)]\n";
	float radiusFactor_bak = radiusFactor;
	this->radiusFactor = 1;
#ifdef PRIORITY_BASED
	this->getDrivingNode()->setMazeDelay(0);
#endif

	RouteSpanningGraph();

	//copy ReferenceRadius from DistFromDriving
	for(int i=0;i<this->OASG->getNumNode();i++)
	{
	  SGNode* N = this->OASG->getNode(i);
	  N->setReferenceRadius(N->getDistFromDriving());

	  if(N->checkInTree() && N->getReferenceRadius() > maxRadius)
	  {	
	    maxRadius = N->getReferenceRadius();
	  }
	}

#ifdef PRIORITY_BASED
	cout<<"\n--[compute PriorityCTF]\n";
	Compute_Priority_CTF(database,elmoreModel);
#else
	cout<<"\n--[compute CTF]\n";
	Compute_CTF(database,elmoreModel);
#endif

	this->maxRadiusInOASG = maxRadius;
	cout<<"max radius in spanning graph = "<<maxRadiusInOASG<<" (um)\n";
#ifndef PRIORITY_BASED
	cout<<"\n--[designate critical sink]\n";
	cout<<"--(its radius longer than [CTF] x maxRadiusInOASG)\n";
	double threshold = maxRadiusInOASG * CTF;
	cout<<"threshold = "<<threshold<<" (um)\n";
#endif

	for(int i=0;i<this->OASG->getNumNode();i++)
	{
	  SGNode* N = this->OASG->getNode(i);
	  
	  if(N->getType() == _PIN  && N != OASG->getDrivingNode() )
	  {	
#ifdef PRIORITY_BASED
	    if(N->getPriority() < PriorityCTF)
#else
	    if(N->getDistFromDriving() > threshold)
#endif
	    {
	      N->setCriticalSink(true); //set critical pin
	      N->setInCriticalPath(true); //in critical pin
	      this->OASG->AddCriticalPin(N);
	    }
	  }
	}
	cout<<"add # "<<this->OASG->getCriricalNumPin()<<" critical sinks\n";

#ifdef PRIORITY_BASED
	// reassign the referenced priority of each SGNode acccording to the adjacent minimal priorities
	//Assign_Reference_Priority(database);

	cout<<"--[set DPF(Priority) of each sink (radius/max-radius)]\n";
	double MAXPRI = elmoreModel->getBestPrioritySinkInOASG()->getPriority() ;
	for(int i=0 ; i < this->OASG->getNumNode() ; i++)
	{
	  SGNode* N = this->OASG->getNode(i);
	  //N->setDPF( ((MAXPRI - N->getReferencePriority())/MAXPRI)*(N->getReferenceRadius()/this->maxRadiusInOASG));
	  N->setDPF( (N->getReferenceRadius()/this->maxRadiusInOASG));
	}
#else
	cout<<"--[set DPF of each sink (radius/max-radius)]\n";
	for(int i=0;i<this->OASG->getNumNode();i++)
	{
	  SGNode* N = this->OASG->getNode(i);
	  N->setDPF( (N->getReferenceRadius()/this->maxRadiusInOASG));
	}
#endif

	//------------------------------------------------------------------------------
	//GROW TRUNK by our critical sink
	//------------------------------------------------------------------------------
	//trace from critical pin to driver
	//set passing node and edge in-critical-path (for edge, its dual edge is also be set)
	//------------------------------------------------------------------------------
	cout<<"\n--[GROW TRUNK by our critical sink]\n";
	for(int i=0;i<this->OASG->getCriricalNumPin();i++)
	{	
		SGNode* Sink = OASG->getCriricalPin(i);		
		GrowOneCriticalPath(Sink);
	}


	cout<<"--[restore data in OASG]\n";
	Restore_exceptCriticalPath();

	//restore radius factor to original value
	cout<<"--[restore radius-factor to original value]\n";
	this->radiusFactor = radiusFactor_bak;

	//finally set critical path computation OK, set CriticalTrunkOK for true
	this->setCriticalPathOK();
#ifdef PRIORITY_BASED
	this->setPriorityOK() ;
#endif
}


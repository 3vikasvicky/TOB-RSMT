#include "OARST.h"

void OARSteinerTree::mazePropagate(SGNode* SOURCE,vector<SGNode*> &vMultiSource,SGNode* Target,vector<SGNode*> &UsedNodeSet)
{
#ifdef DEBUG_ANALYSIS_RUN_TIME
	clock_t SG_maze_route_start = clock();
#endif
	vector<SGNode*> propagateCands;	  //propagation candidates	
	vector<SGNode*> propagatedNodes;  //nodes that have propagated
	
	UsedNodeSet.push_back(Target);    //adding used node into used set

#ifdef DEBUG_OARST_MULTI_SOURCE_DETAIL_INFO
	cout<<"--[Add multi-sources into propagation candidate set]\n";
#endif

	//adding multi-sources into propagation candidate set
	for(size_t i=0;i<vMultiSource.size();i++)
	{	
		SGNode* Source=vMultiSource[i];		
		Source->setTempSource(Source);	    //set candidate's temporary source
		Source->setTempDistFromSource(0);   //set the distance from source
		Source->setTempProCost(tempPropagateCost(Source,Target,Source));			//set candidate's propagation cost 
		propagateCands.push_back(Source);
		UsedNodeSet.push_back(Source); // adding used node into used set

#ifdef DEBUG_OARST_MULTI_SOURCE_DETAIL_INFO
		cout<<"Can Source DistFromDriving="<<Source->getDistFromDriving()<<" progCost="<<Source->getTempProCost()<<"\n";
		PrintNode(Source);
#endif
	}

	sort(propagateCands.begin(),propagateCands.end(),Node_ProCostGreaterThan());
	
	bool meetTarget=false;

#ifdef DEBUG_OARST_MULTI_SOURCE_DETAIL_INFO
	cout<<"--[Start propagating]\n";
#endif

	while(!meetTarget && !propagateCands.empty())
	{
		//pick up the lowest cost of candidate node
		//SGNode* proNode=propagateCands[propagateCands.size()-1];		
		SGNode* proNode=propagateCands.back();
		propagateCands.pop_back(); //pop out 
		++RunTimeSet->times_of_maze;
		

#ifdef DEBUG_OARST_MAZE_ROUTE				
		cout<<"\nPropagating SGNode: Dist from source="<<proNode->getTempDistFromSource()<<"\n";
		PrintNode(proNode);
		cout<<"Previous SGNode:"<<"\n";
		PrintNode(proNode->getTempPrevNode());	
#endif

		if(proNode==Target) 
		{	meetTarget=true;
			break;
		}

		//compute propagation candidates
		for(size_t i=0;i<proNode->getNumEdge();i++)
		{	
			//(1) FromNode , (2) ToNode , (3) E
			SGNode* FromNode=proNode;
			SGNode* ToNode=proNode->getEdge(i)->getToNode();
			SGEdge* E=proNode->getEdge(i);

			//check this edge is able to route ?
			//If this edge intersect other edges that are routed, then this edge is not routable.
			if(E->checkEnable() == false)
			{	continue;
			}

			//avoid loop propagation
			bool ToNodePropagated=false;
			for(size_t j=0;j<propagatedNodes.size();j++)
			{	
				SGNode* N=propagatedNodes[j];

				if(N->getTempSource()==ToNode->getTempSource() && N->getX() == ToNode->getX() && N->getY() == ToNode->getY() && N->getLayer() == ToNode->getLayer())
				{	
					ToNodePropagated=true;
					break;					
				}
			}			

			// Homer: propagate to a new node
			if(!ToNodePropagated)
			{	
				//If we cannot add this node into candidate set eventually, using the following temp value to restore ToNode
				SGNode* tempSource=ToNode->getTempSource();
				SGNode* tempPrevNode=ToNode->getTempPrevNode();
				SGEdge* tempPrevEdge=ToNode->getTempPrevEdge();
				double tempDistFromSource=ToNode->getTempDistFromSource();
				double tempProCost=ToNode->getTempProCost();
				//=========================================================================================================

				ToNode->setTempSource(FromNode->getTempSource());	//set candidate's temporary source				
				ToNode->setTempPrevNode(FromNode);
				ToNode->setTempPrevEdge(E);
				
				//[Consider Routed SG SGEdge]
				//considerRoutedSGEdge(FromNode,ToNode,E); //recompute the cost of edge
				//ToNode->setTempDistFromSource(FromNode->getTempDistFromSource()+E->getCost()); //set ToNode's Distance from Source (temporary info)
				if(CriticalTrunkOK&&ToNode->getSlack()!=_INFINITE && ToNode->getSlack()<FromNode->getSlack()){
				  double W = (FromNode->getSlack()-ToNode->getSlack())/FromNode->getSlack();
				  //W = pow(W,0.1);
				  //double W(0.9);
				  assert(W<=1);
				  W = 1;
				  ToNode->setTempDistFromSource(FromNode->getTempDistFromSource()+W*E->getCost());
				}
				else
				  ToNode->setTempDistFromSource(FromNode->getTempDistFromSource()+E->getCost()); //set ToNode's Distance from Source (temporary info)
				
				//[Consider Routed SG SGEdge]
				//if(E->getCost()!=0) { E->setCost(E->getLength()); } //restore cost of SGEdge

				ToNode->setTempProCost(tempPropagateCost(FromNode->getTempSource(),Target,ToNode)); //set ToNode's Propagation Cost (temporary info)
				
				//check this candidate already exists in the vector ?
				bool AddToNode=true;
				for(size_t j=0;j<propagateCands.size();j++)
				{	
					SGNode* N=propagateCands[j];					
					
					if(N==ToNode)
					{
						//compare these two candidate to decide which one should be erased
						if(N->getTempDistFromSource() > ToNode->getTempDistFromSource())
						{	//erase N
							propagateCands.erase(propagateCands.begin()+j);
							break;
						}
						else
						{	AddToNode=false;
							break;
						}
					}
					
				}

				if(AddToNode)
				{
					propagateCands.push_back(ToNode);
					UsedNodeSet.push_back(ToNode);

#ifdef DEBUG_OARST_MAZE_ROUTE				
					cout<<"\n>Add the following node into propagateCands\n";
					cout<<" Cand SGNode: Dist from source="<<ToNode->getTempDistFromSource()<<" Cost="<<ToNode->getTempProCost()<<"\n";
					PrintNode(ToNode);
					cout<<" His previous node:"<<"\n";
					PrintNode(ToNode->getTempPrevNode());
#endif
				}
				else
				{
					//restore the parameter
					ToNode->setTempSource(tempSource);	//set candidate's temporary source				
					ToNode->setTempPrevNode(tempPrevNode);
					ToNode->setTempPrevEdge(tempPrevEdge);
					ToNode->setTempDistFromSource(tempDistFromSource); //set ToNode's Distance from Source (temporary info)	
					ToNode->setTempProCost(tempProCost); //set ToNode's Propagation Cost (temporary info)
				}
			}
		}
		
		//update propagated Nodes
		propagatedNodes.push_back(new SGNode(proNode));

		//sorting candidate nodes from higher cost to lower cost
		sort(propagateCands.begin(),propagateCands.end(),Node_ProCostGreaterThan());

	}

#ifdef DEBUG_ANALYSIS_RUN_TIME
	clock_t SG_maze_route_end = clock();
	RunTime* RunTime = OASG->getDatabase()->getRunTimeSet();
	RunTime->SG_routing_maze_route = RunTime->SG_routing_maze_route + float(SG_maze_route_end - SG_maze_route_start)/CLK_TCK;
#endif

}

//back trace the routing path by querying record that maze routing generate
SGNode* OARSteinerTree::backTracing(vector<SGNode*> &vMultiSource,SGNode* Target)
{
#ifdef DEBUG_ANALYSIS_RUN_TIME
	clock_t SG_back_trace_start = clock();
#endif

	SGNode* backTraceNode=Target;	 //now back trace node
	SGNode* meetSource=NULL;		 //meeting source

	bool meetTree=false;

#ifdef DEBUG_OARST_BACK_TRACE
	cout<<"--[Start back-tracing]\n";
#endif

	while(!meetTree)
	{
		SGNode* FromNode  =backTraceNode; //back-tracing node
		SGNode* ToNode    =backTraceNode->getTempPrevNode(); //next back-tracing node
		SGEdge* E = backTraceNode->getTempPrevEdge(); //edge between back-tracing node and next back-tracing node
				
		RouteEdge(E);
							
		//set dual edge
		if(E->getDualEdge()==NULL)
		{	PrintEdge(E);
			cout<<"Dual SGEdge is NULL\n";
			exit(0);
		}
		
		RouteEdge(E->getDualEdge());

#ifdef DEBUG_OARST_BACK_TRACE
		cout<<"Skip SGNode:\n";
		PrintNode(ToNode);
		cout<<"# Set SGEdge Length = 0 \n"; //because now this SGEdge is 'routed' !!!
		PrintEdge(E);					    //set this SGEdge
		PrintEdge(E->getDualEdge());        //and its dual edge
#endif				
		//update routing information
		backTraceNode->setTreeState(true); //set node's tree state
		//--------------------------------------------------------------------------------
		//remain backTraceNode's DistFromDriving = _INFINITE
		//so we can use recomputeDistFromDriver() to trace from actually source to target
		//and compute the right distance from driver
		//--------------------------------------------------------------------------------
		//backTraceNode->setDistFromDriving(backTraceNode->getTempDistFromSource()+backTraceNode->getTempSource()->getDistFromDriving()); //set node's distance from driving node

#ifdef DEBUG_OARST_BACK_TRACE
		cout<<"\nBackTrace 'Routed' (Dist from driving ="<<backTraceNode->getDistFromDriving()<<")\n";
		PrintNode(backTraceNode);
		cout<<"Next SGNode ";
		PrintNode(backTraceNode->getTempPrevNode());	
		cout<<endl;
#endif
		backTraceNode=backTraceNode->getTempPrevNode();

		if(backTraceNode->checkInTree()==true) 
		{	
			meetSource = backTraceNode;
			meetTree=true;		
		}
		else if(vMultiSource.size()==1 && backTraceNode==this->getDrivingNode()) //starting case
		{	
			meetSource = backTraceNode;
			meetTree=true;		

			//check is this node (backTraceNode) driving node ? 
			//if the anwser is yes, set this node in tree
			if(backTraceNode->getPoint()->getX() == this->getDrivingNode()->getPoint()->getX() &&
				backTraceNode->getPoint()->getY() == this->getDrivingNode()->getPoint()->getY() &&
				backTraceNode->getPoint()->getLayer() == this->getDrivingNode()->getPoint()->getLayer())
			{	backTraceNode->setTreeState(true); //set driving node's tree state
			}
		}	
	}

#ifdef DEBUG_ANALYSIS_RUN_TIME
	clock_t SG_back_trace_end = clock();
	RunTime* RunTime = OASG->getDatabase()->getRunTimeSet();
	RunTime->SG_routing_back_trace = RunTime->SG_routing_back_trace + float(SG_back_trace_end - SG_back_trace_start)/CLK_TCK;
#endif
	return meetSource;
}

//trace from source to target and compute the right distance from driver
void OARSteinerTree::recomputeDistFromDriver(SGNode* Source,SGNode* Target)
{
	SGNode* passingNode = Source;
	SGNode* nextNode = NULL;
	SGEdge* passingEdge =NULL;


#ifdef DEBUG_OARST_COMPUTE_DISTANCE_FROM_DRIVER
	cout<<"--[Start compute the right distance from driver]\n";
	cout<<"from - to\n";
	PrintNode(Source);
	PrintNode(Target);
#endif
	


	while(passingNode != Target)
	{
		bool findNextNode = false;
		for(int i=0;i<passingNode->getNumEdge();i++)
		{	

#ifdef DEBUG_OARST_COMPUTE_DISTANCE_FROM_DRIVER
			cout<<"node connect to passingNode, in tree ?"<<passingNode->getEdge(i)->getToNode()->checkInTree()<<"\n";
			PrintNode(passingNode->getEdge(i)->getToNode());
#endif

			if(passingNode->getEdge(i)->getToNode()->checkInTree() == true
				&& passingNode->getEdge(i)->getToNode()->getDistFromDriving() == _INFINITE)
			{	nextNode = passingNode->getEdge(i)->getToNode();
				passingEdge = passingNode->getEdge(i);
				findNextNode =true;
				break;
			}
		}

		if(findNextNode==false)
		{	cout<<"trace from source to target, compute distance from driver .. fail\n";
			cout<<"in OARSteinerTree::recomputeDistFromDriver(SGNode* Source,SGNode* Target)\n";
			exit(0);
		}
		nextNode->setDistFromDriving(passingNode->getDistFromDriving() + passingEdge->getLength());
		passingNode = nextNode;

#ifdef DEBUG_OARST_COMPUTE_DISTANCE_FROM_DRIVER
		cout<<"-> passing node\n";
		PrintNode(passingNode);
#endif
	}
}


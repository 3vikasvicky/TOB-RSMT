
#include "OARST.h"

void  OASpanningGraph::Check_Duplicate_Node()
{

	for(int i=0;i<this->getNumNode();i++)
	{
		SGNode* N1 = this->getNode(i);

		for(int j=i+1;j<this->getNumNode();j++)
		{	SGNode* N2 = this->getNode(j);
			if(N1->getX() == N2->getX() && N1->getY() == N2->getY() && N1->getLayer() == N2->getLayer())
			{	
					cout<<"\n--[Check does duplicated nodes exist in OASG ?]\n";
					cout<<"Catch duplication!\n";
					PrintNode(this->getNode(i));
					PrintNode(this->getNode(j));
					exit(0);
			}
		}
	}

}

void OASpanningGraph::Check_Edge_Cost_Length()
{
	for(int i=0;i<this->getNumEdge();i++)
	{
		SGEdge* E =this->getEdge(i);
		if(E->getLength()==0)
		{	
			cout<<"\n--[Check edges' length and cost in OASG ?]\n";
			cout<<"Catch error! length = 0\n";
			PrintEdge(E);
			exit(0);		
		}
		if(E->getCost()!=E->getLength())
		{
			cout<<"\n--[Check edges' length and cost in OASG ?]\n";
			cout<<"Catch error! cost != length\n";
			PrintEdge(E);
			exit(0);		
		}
	}
}


//from each node traces back to driving node by the value of distFromDriving
//if cannot reach driving node, that's must be a error when rectilinearizing slant edges
void OASpanningGraph::Check_Conn()
{
	if(this->getNumNode()==0)
	{	cout<<"There is no node in OASG\n";
		return ;
	}


	for(int i=0;i<this->getNumNode();i++)
	{	
						
		SGNode* N = this->getNode(i);		

		if(N->getType()==_PIN && N->checkInTree()==true)
		{
#ifdef DEBUG_OASG_CHECK_CONNECTION
			cout<<"Path starts from node :\n";
			PrintNode(N);
#endif

#ifdef DEBUG_OASG_CHECK_CONNECTION
			if(N->checkedConn()==true)
			{	cout<<"this node is checked\n";
			}
#endif

#ifndef DEBUG_OASG_CHECK_CONNECTION_STRICTLY
			while(N != this->DrivingNode && N->checkedConn()!=true)
#endif
#ifdef DEBUG_OASG_CHECK_CONNECTION_STRICTLY
			while(N != this->DrivingNode)
#endif
			{
				//select next internode
				SGNode* nextN = NULL;
				

				for(int j=0;j<N->getNumEdge();j++)
				{
					if(N->getEdge(j)->checkRouted())
					{

						if(N->getEdge(j)->getToNode()->getDistFromDriving() == N->getDistFromDriving()) //check error
						{	
							cout<<"\n--[Check Tree's Connection in OASG]\n";					
							cout<<"Error occured in OASpanningGraph::Check_Conn()\n";
							cout<<"N->getEdge(j)->getToNode()->getDistFromDriving() is wrong\n";
									cout<<"N' = ";
									PrintNode(N->getEdge(j)->getToNode());							
									cout<<"N = ";
									PrintNode(N);
							exit(0);
						}

						
						if(N->getEdge(j)->getToNode()->getDistFromDriving() < N->getDistFromDriving())
						{
							if(nextN == NULL)
							{	nextN = N->getEdge(j)->getToNode();
								//because slant edge its length may be not an integer
								//so give it an inaccuracy range (range = 1)
								if(  abs(int(N->getDistFromDriving() - (N->getEdge(j)->getToNode()->getDistFromDriving() + N->getEdge(j)->getLength())) ) > 1)
								{									
									cout<<"\n--[Check Tree's Connection in OASG]\n";					
									cout<<"Error occured in OASpanningGraph::Check_Conn()\n";
									cout<<"N->getEdge(j)->getToNode()->getDistFromDriving() is wrong\n";
									cout<<"nextN = ";
									PrintNode(N->getEdge(j)->getToNode());							
									cout<<"N = ";
									PrintNode(N);
									cout<<"Length = "<<N->getEdge(j)->getLength()<<endl;
									exit(0);
								}
							}
							else
							{
								cout<<"\n--[Check Tree's Connection in OASG]\n";					
								cout<<"Error occured in OASpanningGraph::Check_Conn()\n";
								cout<<"N->getEdge(j) is wrong\n";
								PrintEdge(N->getEdge(j));
								cout<<"nextN is wrong\n";
								PrintNode(nextN);
								exit(0);
							}
						}
					}
				}

#ifdef DEBUG_OASG_CHECK_CONNECTION			
				PrintNode(nextN);
#endif

				if(nextN==NULL)
				{	
					cout<<"\n--[Check Tree's Connection in OASG]\n";					
					cout<<"Error occured in OASpanningGraph::Check_Conn()\n";
					cout<<"nextN cannot be NULL\n";
					cout<<"Driving Node : ";
					PrintNode(this->DrivingNode);
					cout<<"N : ";
					PrintNode(N);
					cout<<"edges of N :\n";
					for(int j=0;j<N->getNumEdge();j++)
					{	PrintEdge(N->getEdge(j));
					}
					exit(0);
				}

				N->setCheckedConn(true);
				N = nextN;
			}

#ifdef DEBUG_OASG_CHECK_CONNECTION
			cout<<"One path is checked!\n";
			cout<<"===========================\n";
#endif
		}
	}


}


//compare sink's delay with its timing constrain
//pick the sink with timing violation
void OASpanningGraph::Check_Timing_Violation(vector<SGNode*> &vViolationNode,double driver_arrival_time)
{

	int count = 0;
	double TNS = 0 ; //total negative slack
	double WNS = 0 ; //worst negative slack
	double slack = 0; //required arrival time - delay
	SGNode* WNS_Sink = NULL;


	for(int i=0;i<this->getNumNode();i++)
	{	
						
		SGNode* N = this->getNode(i);

		if(N->getType()==_PIN && N != this->getDrivingNode())
		{			
			count++;

			slack = (N->getRequiredTime() - driver_arrival_time) - N->getDelay();

			if(slack < 0)
			{	
				if(slack < WNS)
				{	WNS = slack;	
					WNS_Sink = N;
				}

				TNS = TNS + slack;

				//cout<<"Delay = "<<N->getDelay()<<", Required-Time = "<<N->getRequiredTime()<<", slack = "<<slack<<endl;
				//cout<<" - delay from sub-tree = "<<N->getDelay_by_Sub_Tree()<<endl;

				vViolationNode.push_back(N);
			}
		}
	}
	
	if(WNS_Sink!=NULL)
	{
		this->setWorstSlackSink(WNS_Sink);

#ifdef DEBUG_SLACK_INFO
		cout<<"\nWNS sink's delay  = "<<WNS_Sink->getDelay()<<" (ps)"<<endl;
		cout<<"WNS sink's req time  = "<<WNS_Sink->getRequiredTime()<<" (ps)"<<endl;
		cout<<"Arrival time of driver = "<<driver_arrival_time<<" (ps)"<<endl;
		cout<<"WNS sink  node : leaf ? "<<WNS_Sink->checkIsLeaf()<<endl;
		cout<<" (1) Delay_by_Pi_Model     = "<<WNS_Sink->getDelay_by_Pi_Model()<<endl;
		cout<<" (2) Delay_by_Itself       = "<<WNS_Sink->getDelay_by_Itself()<<endl;
		cout<<" (3) Delay_by_Sub_Tree     = "<<WNS_Sink->getDelay_by_Sub_Tree()<<endl;
		cout<<" (4) Delay_by_Driving_Node = "<<WNS_Sink->getDelay_by_Driving_Node()<<endl;
		cout<<"WNS Sink : "; PrintNode(WNS_Sink);
#endif
	}


	if(!vViolationNode.empty())
	{	cout<<"\n# of violation sink = "<<vViolationNode.size()<<endl;	
		cout<<"WNS (Worst Negative Slack) = "<<WNS<<" (ps)"<<endl;
		cout<<"TNS (Total Negative Slack) = "<<TNS<<" (ps)"<<endl;
	}
	else
	{	cout<<"check # of pin = "<<count<<endl;
		cout<<"all meet timing constrain!\n";
	}
}


//check sink's loading !=0 except driver
void OASpanningGraph::Check_Sink_Loading()
{
	for(int i=0;i<this->getNumNode();i++)
	{	SGNode* N = this->getNode(i);
		if(N->getType()==_PIN && N->getLoading()==0 && N!=this->getDrivingNode())
		{	cout<<"Error occured in OASpanningGraph::Check_Sink_Loading()\n";
			cout<<"Catch #"<<i<<" loading of Sink(Pin) = 0\n";
			PrintNode(N);
			exit(0);
		}
	}
}

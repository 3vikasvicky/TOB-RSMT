#include "OARST.h"

void OARectilinearGraph::Check_Duplicate_Node()
{
	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N1 = this->getNode(i);

		if(N1->checkInTree())
		{
			for(int j=i+1;j<this->getNumNode();j++)
			{	RGNode* N2 = this->getNode(j);
				if(N1->getX() == N2->getX() && N1->getY() == N2->getY())
				{	
						cout<<"\n--[Check does duplicated nodes exist in OARG ?]\n";
						cout<<"Catch duplication!\n";
						PrintRGNode(this->getNode(i));
						PrintRGNode(this->getNode(j));
						exit(0);
				}
			}
		}
	}
}

void OARectilinearGraph::OverlapCorrect()
{
	int correct_overlap_num = 0;
	for(int i=0;i<this->getNumEdge();i++)
	{	if(EliminateOverlap(getEdge(i)))
		{	correct_overlap_num++;
		}
	}

	
	if(correct_overlap_num!=0)
	{	cout<<"correct #"<<correct_overlap_num<<" overlap\n";
		cout<<"press any key to continue\n";
		getchar();
	}
}

//Check does duplicated Edge exist in OARG ?
void OARectilinearGraph::Check_Edge()
{
	if(this->getNumEdge()==0)
	{	cout<<"There is no edge in OARG\n";
		return ;
	}

	for(int i=0;i<this->getNumEdge();i++)
	{
		for(int j=i+1;j<this->getNumEdge();j++)
		{
			if(this->getEdge(i)->getDualEdge()!=this->getEdge(j)) //avoid compare dual edge
			{	if(IsOverlap(this->getEdge(i),this->getEdge(j)))
				{
					cout<<"\n--[Check does overlaped Edge exist in OARG ?]\n";
					cout<<"Catch overlap!\n";
					PrintRGEdge(this->getEdge(i));
					PrintRGEdge(this->getEdge(j));
					exit(0);
				}
			}
		}
	}

}

void OARectilinearGraph::Check_All_Pin_In_Tree(Database &data)
{
	int pin_count = 0;
	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N = this->getNode(i);
		if(N->getType()==_PIN)
		{
			if(!N->checkInTree())
			{
				cout<<"\n--[Check all pins in tree in OARG]\n";
				cout<<"Catch wrong node!\n";
				PrintRGNode(N);
				exit(0);
			}
			else
			{	pin_count++;
			}
		}
		
	}

	if(pin_count!=data.getPinNum())
	{
		cout<<"\n--[Check all pin in tree in OARG]\n";
		cout<<"Catch error!\n";
		cout<<"only #"<<pin_count<<" in tree in OARG\n";
		cout<<"actual # of pin = "<<data.getPinNum()<<endl;
		exit(0);
	}
}

void OARectilinearGraph::Check_Node_All_In_Tree()
{
	int node_not_in_tree_num = 0;
	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N = this->getNode(i);
		if(!N->checkInTree())
		{	node_not_in_tree_num++;
		}
	}
	if(node_not_in_tree_num!=0)
	{
		cout<<"\n--[Check nodes all in tree in OARG]\n";
		cout<<"Catch error!\n";
		cout<<"#"<<node_not_in_tree_num<<" RG nodes are not in tree\n";	
	}
}

void  OARectilinearGraph::Check_Node()
{

	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N = this->getNode(i);

		if(N->checkInTree())
		{

			for(int j=0;j<N->getNumEdge();j++)
			{

				RGEdge* RE = N->getEdge(j);

				bool find = false;
				for(int k=0;k<this->getNumEdge();k++)
				{
					if(RE == this->getEdge(k))
					{	find = true;	
						break;
					}
				}
				if(!find)
				{

					cout<<"\n--[Check do node's edges reference exist in OARG ?]\n";
					cout<<"Catch wrong node!\n";
					PrintRGNode(N);
					cout<<"Catch wrong edge!\n";
					PrintRGEdge(RE);
					exit(0);
				}
			}
		}
	}

}

void  OARectilinearGraph::Check_Node_Type()
{

	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N = this->getNode(i);
		
		if(N->checkInTree() && N->getType() != _PIN && N->getType() != _OBSTACLE_LEFT && N->getType() != _OBSTACLE_RIGHT && N->getType() != _TURNING)
		{
			cout<<"\n--[Check type of nodes in OARG]\n";
			cout<<"Catch wrong node!\n";
			PrintRGNode(N);
			cout<<"RGNode type is wrong!\n";
			exit(0);
		}
	}

}

//from each node traces back to driving node by the value of distFromDriving
//if cannot reach driving node, that's must be a error when rectilinearizing slant edges
void OARectilinearGraph::Check_Conn()
{
	if(this->getNumNode()==0)
	{	cout<<"There is no node in OARG\n";
		return ;
	}


	for(int i=0;i<this->getNumNode();i++)
	{	
						
		RGNode* N = this->getNode(i);		


		if(N->checkInTree())
		{

	#ifdef DEBUG_OARG_CHECK_CONNECTION
			cout<<"Path starts from node :\n";
			PrintRGNode(N);
	#endif

	#ifdef DEBUG_OARG_CHECK_CONNECTION
			if(N->checkedConn()==true)
			{	cout<<"this node is checked\n";
			}
	#endif

	#ifndef DEBUG_OARG_CHECK_CONNECTION_STRICTLY
			while(N != this->DrivingNode && N->checkedConn()!=true)
	#endif
	#ifdef DEBUG_OARG_CHECK_CONNECTION_STRICTLY
			while(N != this->DrivingNode)
	#endif
			{
				//select next internode
				RGNode* nextN = NULL;
				

				for(int j=0;j<N->getNumEdge();j++)
				{
					if(N->getEdge(j)->getToNode()->getDistFromDriving() == N->getDistFromDriving()) //check error
					{	
						cout<<"\n--[Check Tree's Connection in OARG]\n";					
						cout<<"Error occured in OARectilinearGraph::Check_Conn()\n";
						cout<<"N->getEdge(j)->getToNode()->getDistFromDriving() is wrong\n";
								cout<<"N's ToNode = ";
								PrintRGNode(N->getEdge(j)->getToNode());							
								cout<<"N = ";
								PrintRGNode(N);
						exit(0);
					}

					
					if(N->getEdge(j)->getToNode()->getDistFromDriving() < N->getDistFromDriving())
					{
						if(nextN == NULL)
						{	nextN = N->getEdge(j)->getToNode();
							if(N->getEdge(j)->getToNode()->getDistFromDriving() + N->getEdge(j)->getLength() != N->getDistFromDriving())
							{
								cout<<"\n--[Check Tree's Connection in OARG]\n";					
								cout<<"Error occured in OARectilinearGraph::Check_Conn()\n";
								cout<<"N->getEdge(j)->getToNode()->getDistFromDriving() is wrong\n";
								cout<<"nextN = ";
								PrintRGNode(N->getEdge(j)->getToNode());							
								cout<<"N = ";
								PrintRGNode(N);
								cout<<"Length = "<<N->getEdge(j)->getLength()<<endl;
								exit(0);
							}
						}
						else
						{
							cout<<"\n--[Check Tree's Connection in OARG]\n";					
							cout<<"Error occured in OARectilinearGraph::Check_Conn()\n";
							cout<<"N->getEdge(j) is wrong\n";
							PrintRGEdge(N->getEdge(j));
							cout<<"nextN is wrong : 2 candidates\n";							
							PrintRGNode(nextN);
							PrintRGNode(N->getEdge(j)->getToNode());
							exit(0);
						}
					}
				}

	#ifdef DEBUG_OARG_CHECK_CONNECTION			
				PrintRGNode(nextN);
	#endif

				if(nextN==NULL)
				{	
					cout<<"\n--[Check Tree's Connection in OARG]\n";					
					cout<<"Error occured in OARectilinearGraph::Check_Conn()\n";
					cout<<"nextN cannot be NULL\n";
					cout<<"Driving Node : ";
					PrintRGNode(this->DrivingNode);
					cout<<"N : ";
					PrintRGNode(N);
					cout<<"edges of N :\n";
					for(int j=0;j<N->getNumEdge();j++)
					{	PrintRGEdge(N->getEdge(j));
					}
					exit(0);
				}

				N->setCheckedConn(true);
				N = nextN;
			}
	#ifdef DEBUG_OARG_CHECK_CONNECTION
			cout<<"One path is checked!\n";
			cout<<"===========================\n";
	#endif
		}

	}
}


//compare sink's delay with its timing constrain
//pick the sink with timing violation
void OARectilinearGraph::Check_Timing_Violation(double driver_arrival_time)
{
	int count = 0;
	int violation_count = 0;
	double TNS = 0 ; //total negative slack
	double WNS = 0 ; //worst negative slack
	double slack = 0; //required arrival time - delay
	RGNode* WNS_Sink = NULL;
	double MMIINN(100000000);


	for(int i=0;i<this->getNumNode();i++)
	{	
						
		RGNode* N = this->getNode(i);

		if(N->getType()==_PIN && N != this->getDrivingNode())
		{			
			count++;

			slack = (N->getRequiredTime() - driver_arrival_time) - N->getDelay();
			if(MMIINN>slack){
			  MMIINN=slack;
			}

			if(slack < 0)
			{	
				if(slack < WNS)
				{	WNS = slack;	
					WNS_Sink = N;
				}

				TNS = TNS + slack;
				violation_count ++;
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
		cout<<"WNS Sink : "; PrintRGNode(WNS_Sink);
#endif
	}


	if(violation_count!=0)
	{	cout<<"\n# of violation sink = "<<violation_count<<endl;	
		cout<<"WNS (Worst Negative Slack) = "<<WNS<<" (ps)"<<endl;
		cout<<"TNS (Total Negative Slack) = "<<TNS<<" (ps)"<<endl;
	}
	else
	{	cout<<"\ncheck # of pin = "<<count<<endl;
		cout<<"all meet timing constrain! ("<<MMIINN<<")"<<endl;
	}
}


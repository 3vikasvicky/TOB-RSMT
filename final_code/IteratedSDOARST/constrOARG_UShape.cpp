#include "OARST.h"
using namespace auxiliary;



void OARectilinearGraph::FlipEdge_CheckOne(RGNode* checkN,int& remove_count)
{
	
			for(int j=0;j<checkN->getNumEdge();j++)
			{	
				RGEdge* connN_E = checkN->getEdge(j);
				RGNode* turnN = connN_E->getToNode();

				if(turnN == checkN)
				{	cout<<"U shape remove .. fail, turnN == N\n";					
					exit(0);
				}

				//bool eliminate_one_U = false;
				//------------------------------------------
				//means this node only connect 2 egde => flipping safe edge
				//and we can remove the node that is PIN
				//PIN must in the tree
				//------------------------------------------
				if(turnN->getNumEdge()==2 && turnN->getType()!=_PIN)
				{						
					RGEdge* flipE = NULL;
					RGEdge* flipE_conn_checkN = NULL;
					RGNode* toN = NULL;

					//set toN
					if(turnN->getEdge(1)->getToNode()==checkN)
					{	flipE = turnN->getEdge(0);
						flipE_conn_checkN = turnN->getEdge(1);
						toN = turnN->getEdge(0)->getToNode();
					}
					else if(turnN->getEdge(0)->getToNode()==checkN)
					{	flipE = turnN->getEdge(1);
						flipE_conn_checkN = turnN->getEdge(0);
						toN = turnN->getEdge(1)->getToNode();
					}
					else
					{	cout<<"U shape remove .. fail\n";					
						cout<<"now connN_E\n";
						PrintRGEdge(connN_E);
						cout<<"print edges of turn node\n";
						for(int i=0;i<turnN->getNumEdge();i++)
						{	PrintRGEdge(turnN->getEdge(i));
						}
						exit(0);
					}

					//avoid flip edge cross obstacle
					//(1) toN and turnN //(2) turnN and checkN
					bool toNisCorner = IsCorner(toN);
					bool turnNisCorner = IsCorner(turnN);
					bool checkNisCorner = IsCorner(checkN);
					if((toNisCorner && turnNisCorner) || (turnNisCorner && checkNisCorner))
					{	continue;
					}
					

					for(int k=0;k<checkN->getNumEdge();k++)
					{	
						RGEdge* connN_E_2 = checkN->getEdge(k);

						if(Vertical_Overlap(flipE,connN_E_2) || Horizontal_Overlap(flipE,connN_E_2))
						{
										

							RGNode* newTurnN = NULL;
							//if(turnN->getX() == toN->getX() && turnN->getY() == checkN->getY())
							if(Vertical_Overlap(flipE,connN_E_2))
							{	
								newTurnN = AddRGNode(checkN->getX(),toN->getY(),checkN->getLayer(),_TURNING); 
							}
							//else
							else if(Horizontal_Overlap(flipE,connN_E_2))
							{
								newTurnN = AddRGNode(toN->getX(),checkN->getY(),checkN->getLayer(),_TURNING); 
							}
							else
							{	cout<<"Error occured in OARectilinearGraph::FlipEdge_CheckOne()\n";
							}

							//remove this U shape, turn upper L to lower L or lower L to upper L
							DelRGEdge(flipE);
							DelRGEdge(flipE_conn_checkN);	
							turnN->setTreeState(false);


							RGEdge* RE1 = AddRGEdge(StruFromRGNode(checkN),StruToRGNode(newTurnN),_INFINITE,false);
							RGEdge* RE2 = AddRGEdge(StruFromRGNode(newTurnN),StruToRGNode(toN),_INFINITE,false);
							EliminateOverlap(RE1);
							EliminateOverlap(RE2);

							remove_count++;				
							break;							

						}
					}
				}
			}
}



void OARectilinearGraph::PushEdge_CheckOne1(RGNode* checkN,int& remove_count)
{
			for(int j=0;j<checkN->getNumEdge();j++)
			{	
				RGEdge* connN_E = checkN->getEdge(j);

				for(int k=0;k<checkN->getNumEdge();k++)
				{	
					if(checkN->getEdge(k)!=connN_E)
					{
						RGNode* N = checkN->getEdge(k)->getToNode(); //checkN -> N -> ToN2 (E)


						//avoid flip edge cross obstacle
						//N and checkN
						bool NisCorner = IsCorner(N);
						bool checkNisCorner = IsCorner(checkN);
						if(NisCorner && checkNisCorner)
						{	return;
						}						


						for(int m=0;m<N->getNumEdge();m++)
						{
							if(N->getEdge(m)!=checkN->getEdge(k)->getDualEdge())
							{
								RGEdge* E = N->getEdge(m);
								if(Vertical_Overlap(E,connN_E) || Horizontal_Overlap(E,connN_E))
								{	
#ifdef DEBUG_OARG_U_SHAPE_PUSH_EDGE
									cout<<"\nE : ";
									PrintRGEdge(E);
									cout<<"connN_E : ";
									PrintRGEdge(connN_E);
#endif
									RGNode* ToN1 = connN_E->getToNode(); //checkN -> ToN1 (connN_E)
									RGNode* ToN2 = E->getToNode();


									if(Vertical_Overlap(E,connN_E))
									{	if(abs(ToN1->getX()-ToN2->getX()) > min(E->getLength(),connN_E->getLength()))	
										{	return; //no use, so return
										}
									}
									else 
									{
										if(abs(ToN1->getY()-ToN2->getY()) > min(E->getLength(),connN_E->getLength()))
										{	return; //no use, so return
										}
									}


									//find reduction edge
									if(E->getLength() > connN_E->getLength()) //remove connN_E, ToN1 connect to E
									{
#ifdef DEBUG_OARG_U_SHAPE_PUSH_EDGE
										cout<<"remove connN_E, ToN1 connect to E\n";										
#endif
										RGNode* newTurnN = NULL;
										if(ToN1->getY() == ToN2->getY() && Vertical_Overlap(E,connN_E))
										{	DelRGEdge(connN_E);
											newTurnN = ToN2;
										}
										else if(ToN1->getX() == ToN2->getX() && Horizontal_Overlap(E,connN_E))
										{	DelRGEdge(connN_E);
											newTurnN = ToN2;
										}
										else
										{	
											if(Vertical_Overlap(E,connN_E))
											{	newTurnN = AddRGNode(ToN2->getX(),ToN1->getY(),checkN->getLayer(),_TURNING); 
											}
											else
											{	newTurnN = AddRGNode(ToN1->getX(),ToN2->getY(),checkN->getLayer(),_TURNING); 
											}
											DelRGEdge(connN_E);
											DelRGEdge(E);	
											RGEdge* RE2 = AddRGEdge(StruFromRGNode(N),StruToRGNode(newTurnN),_INFINITE,false);
											RGEdge* RE3 = AddRGEdge(StruFromRGNode(newTurnN),StruToRGNode(ToN2),_INFINITE,false);
											EliminateOverlap(RE2);
											EliminateOverlap(RE3);
										}

#ifdef DEBUG_OARG_U_SHAPE_PUSH_EDGE
										cout<<"checkN : ";
										PrintRGNode(checkN);
										cout<<"N : ";
										PrintRGNode(N);
										cout<<"ToN1 : ";
										PrintRGNode(ToN1);
										cout<<"ToN2 : ";
										PrintRGNode(ToN2);
										cout<<"newTurnN : ";
										PrintRGNode(newTurnN);
#endif
										RGEdge* RE1 = AddRGEdge(StruFromRGNode(ToN1),StruToRGNode(newTurnN),_INFINITE,true);
										remove_count++;										
									}
									else //remove E, ToN2 connect to connN_E
									{	
#ifdef DEBUG_OARG_U_SHAPE_PUSH_EDGE
										cout<<"remove E, ToN2 connect to connN_E\n";										
#endif
										RGNode* newTurnN = NULL;
										if(ToN1->getY() == ToN2->getY() && Vertical_Overlap(E,connN_E))
										{	DelRGEdge(E);
											newTurnN = ToN1;
										}
										else if(ToN1->getX() == ToN2->getX() && Horizontal_Overlap(E,connN_E))
										{	DelRGEdge(E);
											newTurnN = ToN1;
										}
										else
										{
											if(Vertical_Overlap(E,connN_E))
											{	
												newTurnN = AddRGNode(ToN1->getX(),ToN2->getY(),checkN->getLayer(),_TURNING); 
											}
											else
											{	
												newTurnN = AddRGNode(ToN2->getX(),ToN1->getY(),checkN->getLayer(),_TURNING); 
											}
											DelRGEdge(E);
											DelRGEdge(connN_E);	
											RGEdge* RE2 = AddRGEdge(StruFromRGNode(checkN),StruToRGNode(newTurnN),_INFINITE,false);
											RGEdge* RE3 = AddRGEdge(StruFromRGNode(newTurnN),StruToRGNode(ToN1),_INFINITE,false);
											EliminateOverlap(RE2);
											EliminateOverlap(RE3);
										}
#ifdef DEBUG_OARG_U_SHAPE_PUSH_EDGE
										cout<<"checkN : ";
										PrintRGNode(checkN);
										cout<<"N : ";
										PrintRGNode(N);
										cout<<"ToN1 : ";
										PrintRGNode(ToN1);
										cout<<"ToN2 : ";
										PrintRGNode(ToN2);
										cout<<"newTurnN : ";
										PrintRGNode(newTurnN);
#endif

										RGEdge* RE1 = AddRGEdge(StruFromRGNode(ToN2),StruToRGNode(newTurnN),_INFINITE,true);
										remove_count++;
									}

									if(checkN->getNumEdge()==1 && checkN->getType()!=_PIN)
									{	DelRGEdge(checkN->getEdge(0));	
										checkN->setTreeState(false);
									}
									if(ToN1->getNumEdge()==1 && ToN1->getType()!=_PIN)
									{	DelRGEdge(ToN1->getEdge(0));	
										ToN1->setTreeState(false);
									}
									if(ToN2->getNumEdge()==1 && ToN2->getType()!=_PIN)
									{	DelRGEdge(ToN2->getEdge(0));	
										ToN2->setTreeState(false);
									}
									if(N->getNumEdge()==1 && N->getType()!=_PIN)
									{	DelRGEdge(N->getEdge(0));	
										N->setTreeState(false);
									}
									return;
								}
							}
						}
					}
				}
			}
}

bool OARectilinearGraph::PushEdge_CheckOne2(RGNode* checkN,int& remove_count)
{	
	if(checkN->getNumEdge()!=2 || checkN->getType()==_PIN)
	{	return false;
	}

	RGNode* toN = NULL;
	vector<RGEdge*> vRemoveEdge;
	RGEdge* checkN_Edge = NULL;

	for(int j=0;j<checkN->getNumEdge();j++)
	{	
		RGNode* tempN = checkN->getEdge(j)->getToNode();
		if(tempN->getNumEdge()==2 && tempN->getType()!=_PIN)
		{	toN = tempN;
			vRemoveEdge.push_back(checkN->getEdge(j));
			if(j==0)
			{	checkN_Edge = checkN->getEdge(1);
			}
			else
			{	checkN_Edge = checkN->getEdge(0);
			}
			break;
		}
	}

	if(toN == NULL || (IsCorner(checkN) && IsCorner(toN))) //no found pushable edge
	{	return false;
	}

	//now checkN -> toN
	//further check
	RGNode* preToN = checkN;
	while(  (Is_Horizontal_Line(toN->getEdge(0)) && Is_Horizontal_Line(toN->getEdge(1)))
		|| (Is_Vertical_Line(toN->getEdge(0)) && Is_Vertical_Line(toN->getEdge(1))) )
	{	

		bool MoveToN = false;
		for(int j=0;j<toN->getNumEdge();j++)
		{	
			RGNode* tempN = toN->getEdge(j)->getToNode();
			if(tempN != preToN && tempN->getNumEdge()==2 && tempN->getType()!=_PIN)
			{	preToN = toN;
				toN = tempN;
				vRemoveEdge.push_back(preToN->getEdge(j));
				MoveToN = true;
				break;
			}
		}

		if(IsCorner(preToN) && IsCorner(toN))
		{	return false;
		}

		if(MoveToN == false)
		{	break;
		}
	}

	if(toN->getNumEdge()!=2 || toN->getType()==_PIN)
	{	return false;
	}

	RGEdge* toN_Edge = NULL;
	for(int j=0;j<toN->getNumEdge();j++)
	{	
		RGNode* tempN = toN->getEdge(j)->getToNode();
		if(tempN != preToN)
		{	toN_Edge = toN->getEdge(j);
			break;
		}
	}

	//check overlap ?
	if(Horizontal_Overlap(checkN_Edge,toN_Edge) || Vertical_Overlap(checkN_Edge,toN_Edge))
	{
		for(int i=vRemoveEdge.size()-1;i>=0;i--)
		{	
			vRemoveEdge[i]->getFromNode()->setTreeState(false);
			vRemoveEdge[i]->getToNode()->setTreeState(false);
			DelRGEdge(vRemoveEdge[i]);
		}

		remove_count++;

		if(checkN_Edge->getLength() == toN_Edge->getLength())
		{	
			DelRGEdge(toN_Edge);
			toN->setTreeState(false);
			DelRGEdge(checkN_Edge);
			checkN->setTreeState(false);
		}
		else if(checkN_Edge->getLength() > toN_Edge->getLength())
		{				

			RGNode* checkN_conn = checkN->getEdge(0)->getToNode();
			RGNode* toN_conn = toN->getEdge(0)->getToNode();
						
			int newX = 0;
			int newY = 0;
			int newLayer = checkN->getLayer();

			if(Horizontal_Overlap(checkN_Edge,toN_Edge))
			{	newX = toN_conn->getX();
				newY = checkN->getY();
			}
			else
			{	newX = checkN->getX();
				newY = toN_conn->getY();
			}
			
			DelRGEdge(checkN_Edge);
			DelRGEdge(toN_Edge);

			toN->setTreeState(false);
			checkN->setTreeState(false);
			
			RGNode* newN = AddRGNode(newX,newY,newLayer,_TURNING);
			
			AddRGEdge(StruFromRGNode(newN),StruToRGNode(checkN_conn),_INFINITE,true);
			AddRGEdge(StruFromRGNode(newN),StruToRGNode(toN_conn),_INFINITE,true);
		}
		else
		{
			RGNode* tempN = checkN->getEdge(0)->getToNode();

			RGNode* checkN_conn = checkN->getEdge(0)->getToNode();
			RGNode* toN_conn = toN->getEdge(0)->getToNode();

			int newX = 0;
			int newY = 0;
			int newLayer = checkN->getLayer();

			if(Horizontal_Overlap(checkN_Edge,toN_Edge))
			{	newX = tempN->getX();
				newY = toN->getY();
			}
			else
			{	newX = toN->getX();
				newY = tempN->getY();
			}

			DelRGEdge(checkN_Edge);
			DelRGEdge(toN_Edge);

			toN->setTreeState(false);
			checkN->setTreeState(false);
			
			RGNode* newN = AddRGNode(newX,newY,newLayer,_TURNING);	

			AddRGEdge(StruFromRGNode(newN),StruToRGNode(checkN_conn),_INFINITE,true);
			AddRGEdge(StruFromRGNode(newN),StruToRGNode(toN_conn),_INFINITE,true);
		}

		return true;
	}

	return false;
}

bool OARectilinearGraph::PushEdge1()
{

	vector<RGNode*> vOld_Node_Set;
	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N = this->getNode(i);
		vOld_Node_Set.push_back(N);
	}


	int remove_count=0;

	for(int i=0;i<vOld_Node_Set.size();i++)
	{
		RGNode* checkN = vOld_Node_Set[i];		
		
		if(checkN->checkInTree())
		{
			PushEdge_CheckOne1(checkN,remove_count);
		}
	}

	int remove_node=0;
	//delete all the node not in tree
	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N = this->getNode(i);
		if(N->checkInTree()==false)
		{	this->DelRGNode(N);
			remove_node++;
			i--;
		}
	}

	cout<<"PUSH EDGE(1): remove #"<<remove_count<<" U shape\n";
	cout<<"PUSH EDGE(1): remove #"<<remove_node<<" node in OARG\n";

	if(remove_count == 0)
	{	return false;
	}
	else
	{	return true;
	}
}

bool OARectilinearGraph::PushEdge2()
{

	vector<RGNode*> vOld_Node_Set;
	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N = this->getNode(i);
		vOld_Node_Set.push_back(N);
	}


	int remove_count=0;

	for(int i=0;i<vOld_Node_Set.size();i++)
	{
		RGNode* checkN = vOld_Node_Set[i];		
		
		if(checkN->checkInTree())
		{
			if(PushEdge_CheckOne2(checkN,remove_count)==true)
			{	break;
			}
		}
	}

	int remove_node=0;
	//delete all the node not in tree
	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N = this->getNode(i);
		if(N->getNumEdge()==0)
		{	this->DelRGNode(N);
			remove_node++;
			i--;
		}
		else if(N->checkInTree()==false)
		{	this->DelRGNode(N);
			remove_node++;
			i--;
		}
	}

	cout<<"PUSH EDGE(2): remove #"<<remove_count<<" U shape\n";
	cout<<"PUSH EDGE(2): remove #"<<remove_node<<" node in OARG\n";

	if(remove_count == 0)
	{	return false;
	}
	else
	{	return true;
	}
}

bool OARectilinearGraph::FlipEdge()
{
	//------------------------------------------
	//for each in-tree node in OARG
	//check the connecting edge, if it can be flipped safely (doesen't make any disconnect)
	//and reduce the wirelength, then we do the flip
	//that's a easy U shape elimination
	//------------------------------------------

	vector<RGNode*> vOld_Node_Set;
	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N = this->getNode(i);
		vOld_Node_Set.push_back(N);
	}


	int remove_count=0;
	for(int i=0;i<vOld_Node_Set.size();i++)
	{
		RGNode* checkN = vOld_Node_Set[i];	

		if(checkN->checkInTree())
		{
			FlipEdge_CheckOne(checkN,remove_count);
		}
	}

	int remove_node = 0;
	//delete all the node not in tree
	for(int i=0;i<this->getNumNode();i++)
	{
		RGNode* N = this->getNode(i);
		if(N->checkInTree()==false)
		{	this->DelRGNode(N);
			remove_node++;
			i--;
		}
	}

	cout<<"FLIP EDGE: remove #"<<remove_count<<" U shape\n";
	cout<<"FLIP EDGE: remove #"<<remove_node<<" node in OARG\n";

	if(remove_count == 0)
	{	return false;
	}
	else
	{	return true;
	}
}

bool OARectilinearGraph::SmoothEdge()
{
	int remove_count = 0;
	int remove_node  = 0;
	for(int i=this->getNumNode()-1;i>=0;i--)
	{
		RGNode* N = this->getNode(i);

		if(N->getType()!=_PIN && N->getNumEdge()==0)
		{
			DelRGNode(N);
			remove_node++;
		}
		else if(N->getType()!=_PIN && N->getNumEdge()==1)
		{		
			DelRGEdge(N->getEdge(0));
			remove_count++;
			DelRGNode(N);
			remove_node++;
		}		
	}
	cout<<"SMOOTH EDGE: remove #"<<remove_count<<" edge\n";
	cout<<"SMOOTH EDGE: remove #"<<remove_node<<" node in OARG\n";

	if(remove_count == 0)
	{	return false;
	}
	else
	{	return true;
	}
}

void OARectilinearGraph::UShapeRemove()
{
		bool didFlip = false;
		bool didPush1 = false;
		bool didPush2 = false;
		bool didSmooth = false;
				

		int limit = 2;
		int times = 0;
		PushEdge2();
		do
		{
			while(SmoothEdge()) {};
			didFlip = FlipEdge();
			didPush1 = PushEdge1();
			while(SmoothEdge()) {};
			times++;

		}while((didFlip==true || didPush1==true) && times != limit);
}
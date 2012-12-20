#include "OARST.h"
#include "definition.h"
using namespace auxiliary;




//erase the SG edge that has been traslated into Rect shape
void Check_vUn_SGEdge(vUn_Processed_SGEdge &vUn_SGEdge)
{

	for(int i=vUn_SGEdge.size()-1;i>=0;i--)
	{
		if(vUn_SGEdge[i]->checkTurnRect()==true)
		{	vUn_SGEdge.erase(vUn_SGEdge.begin()+i);
		}
	}
}




SGEdge* OARectilinearGraph::Pick_SE0(SGEdge* E, SGNode* FromNode,int &E_E0_Case,double &Score_SG)
{
	SGNode* Root = FromNode;
	SGNode* N1 = E->getToNode();

	double Score = _UNSET;
	double temp;
	SGEdge* Current_E0 = NULL;
	int Current_Case = _E_E0_CASE0;
	E_E0_Case = _E_E0_CASE0;

#ifdef DEBUG_OARG_PICK_SE0
	cout<<"    Root :";
	PrintNode(Root);
	cout<<"    N1 :";
	PrintNode(N1);
#endif 

	int valid_SGEdge_num=0;

	//the "from node" of slant Edge SG_E (Pick_E(vUn_SGEdge))
	for(int i=0;i<Root->getNumEdge();i++)
	{
		SGEdge* SG_E = Root->getEdge(i);

		if(SG_E != E && SG_E->checkTurnRect()==false && SG_E->checkRouted()==true)
		{
			valid_SGEdge_num++;

			SGNode* N2 = SG_E->getToNode();			

#ifdef DEBUG_OARG_PICK_SE0
			cout<<"    Can N2 :";
			PrintNode(N2);
#endif 
			if(Score < (temp = Compute_Share_Length(N1,N2,Root,Current_Case))) //check case_2 case_3 case_4
			{	Current_E0 = SG_E;
				Score = temp;
				E_E0_Case = Current_Case;
			}

#ifdef DEBUG_OARG_PICK_SE0
			cout<<"    Score :"<<Score<<", E_E0_Case:"<<E_E0_Case<<endl;
#endif 
		}
	}

#ifdef DEBUG_OARG_PICK_SE0
	cout<<"# of slant edges that connect to Root = "<<valid_SGEdge_num<<endl;
#endif 

	Score_SG = Score;

	if(Score == 0 || Score==_UNSET) //no reuseful edge exists
	{
		E_E0_Case = _E_E0_CASE0;
#ifdef DEBUG_OARG_PICK_SE0
		cout<<"Score = 0 or _UNSET, no reuseful edge exists!"<<endl;				
#endif 	
	}


#ifdef DEBUG_OARG_PICK_SE0
	cout<<"E_E0_Case = "<<E_E0_Case<<endl;				
#endif 
	return Current_E0;
}


RGEdge* OARectilinearGraph::Pick_RE0(SGEdge* E, RGNode* FromNode,double &Score_RG)
{
	RGNode* Root = FromNode;
	SGNode* N1 = E->getToNode();

	double Score = _UNSET;
	double temp;
	
	RGEdge* Current_E0 = NULL;
	
	int Current_Case = 0; //no use

#ifdef DEBUG_OARG_PICK_RE0
	cout<<"    Root :";
	PrintRGNode(Root);
	cout<<"    N1 :";
	PrintNode(N1);
#endif 


	for(int i=0;i<Root->getNumEdge();i++)
	{
		RGEdge* RG_E = Root->getEdge(i);

		RGNode* N2 = RG_E->getToNode();			

#ifdef DEBUG_OARG_PICK_RE0
		cout<<"    Can N2 :";
		PrintRGNode(N2);
#endif 
		if(Score < (temp = Compute_Share_Length(N1,N2,Root,Current_Case))) //check case_2 case_3 case_4
		{	Current_E0 = RG_E;
			Score = temp;
			//E_E0_Case = Current_Case;
		}

#ifdef DEBUG_OARG_PICK_RE0
		cout<<"    Score :"<<Score<<endl;
#endif 
	}


#ifdef DEBUG_OARG_PICK_RE0
	cout<<"# of rectilinearized edges that connect to Root = "<<Root->getNumEdge()<<endl;
#endif 

	Score_RG = Score;
	return Current_E0;
}

//select the first SG Edge (slant edge) that we want to rectilinearize
SGEdge* OARectilinearGraph::Pick_E(vUn_Processed_SGEdge &vUn_SGEdge)
{
	if(vUn_SGEdge.empty())
	{	return NULL;
	}
	
	SGEdge* E = NULL;
	sort(vUn_SGEdge.begin(),vUn_SGEdge.end(),UnProcessed_SGEdge_CostLessThan());
	E = vUn_SGEdge[vUn_SGEdge.size()-1];
	vUn_SGEdge.pop_back();

	//
	TurnRectilinear(E);  		
						//E->setRectState(true);
						//E->getDualEdge()->setRectState(true);

	return E;

}

RGNode* OARectilinearGraph::Pick_ProRGNode(vUn_Propagated_RGNode &vUn_RGNode)
{	
	if(vUn_RGNode.empty())
	{	cout<<"Error! vUn_RGNode.empty()\n";
		exit(0);
	}

	RGNode* ProRGNode = vUn_RGNode[vUn_RGNode.size()-1];
	vUn_RGNode.pop_back();

#ifdef DEBUG_OARG_DO_RECTILINEAR		
	cout<<"============================================\nNow ProRGNode:\n";
	PrintRGNode(ProRGNode);
#endif

	return ProRGNode;

}



//adding relative SG edges that are valid and connected to ProRGNode
void OARectilinearGraph::SearchSlantEdges(RGNode* ProRGNode)
{

#ifdef DEBUG_OARG_DO_RECTILINEAR_SEARCH_SLANT_EDGES
	cout<<"\n--[Print adjacent unrectilinearized slant edges]\n";	
#endif
	

	for(int i=0;i<ProRGNode->getSGNode()->getNumEdge();i++)
	{				
			
		SGEdge* SG_E = ProRGNode->getSGNode()->getEdge(i);

		if(SG_E->checkRouted()==true && SG_E->checkTurnRect()==false)
		{	
			vUn_SGEdge.push_back(SG_E);

#ifdef DEBUG_OARG_DO_RECTILINEAR_SEARCH_SLANT_EDGES
			PrintEdge(SG_E);
#endif
		}
	}


#ifdef DEBUG_OARG_DO_RECTILINEAR_SEARCH_SLANT_EDGES
	cout<<"# of adjacent unrectilinearized slant edges = "<<vUn_SGEdge.size()<<"\n";	
#endif
}

void OARectilinearGraph::Turn_One_Edge_Rectilinear(RGNode* StartNode)
{
	vUn_RGNode.clear();
	StartNode->setIsLeaf(true);
	vUn_RGNode.push_back(StartNode);

	//while(getNumUnPropRGNode()!=0)
	//{	
		TurnRectilinear_Process_One_Node();
	//}
}

void OARectilinearGraph::TurnRectilinear_From(RGNode* StartNode)
{
	vUn_RGNode.clear();
	StartNode->setIsLeaf(true);
	vUn_RGNode.push_back(StartNode);

	while(getNumUnPropRGNode()!=0)
	{	
		TurnRectilinear_Process_One_Node();
	}
}

void OARectilinearGraph::TurnRectilinear_From_Driver()
{
	while(getNumUnPropRGNode()!=0)
	{	TurnRectilinear_Process_One_Node();
	}
}

void OARectilinearGraph::TurnRectilinear_Process_One_Node()
{
#ifdef DEBUG_OARG_DO_RECTILINEAR		
		cout<<"# of RGNode = "<<vRGNode.size()<<endl;
		cout<<"# of Un Processed Node = "<<vUn_RGNode.size()<<endl;
#endif

		//[1] select propagating node
		RGNode* ProRGNode = Pick_ProRGNode(vUn_RGNode);


		//[2] adding relative SG edges that are valid and connected to ProRGNode
		//    check do exist unrectilinearized slant edges with connecting to ProRGNode ?
		//    if the anwser is yes, add these slant edges into vUn_SGEdge.
		//    then we will rectilinearize these slant edges in step [3]
		//after a spanning edge is turned rectilinear, it's set SG_E->checkTurnRect() = true
		SearchSlantEdges(ProRGNode); //put un-rectilinear edges connecting to ProRGNode into vUn_SGEdge



		//[3] propagating until all relative SG Edge are processed
		while(!vUn_SGEdge.empty())
		{	


			//
			Check_vUn_SGEdge(vUn_SGEdge);

			//[4] pick the 'first' slant edge we want to rectilinearize
			SGEdge* SG_E = Pick_E(vUn_SGEdge); //<=========== this function can be modified to meet timing constrain !!!!!!! pick what ?

#ifdef DEBUG_OARG_DO_RECTILINEAR
			cout<<"\n--[1: Pick first slant Edge]\n";
			PrintEdge(SG_E);
#endif


						
			if(SG_E==NULL)
			{	continue;
			}

			int E_E0_Case = _UNSET;


			ProRGNode->setIsLeaf(false); //becuase some edges will generate from this node


#ifdef DEBUG_OARG_DO_RECTILINEAR
			cout<<"\n--[2: Compute the rest of slant Edges in SG]\n";
#endif
			//[5] pick the 'second' slant edge we want to rectilinearize
			int E_E0_Case_SG = _UNSET;
			double Score_SG = _UNSET;
			SGEdge* SG_E0 = Pick_SE0(SG_E,ProRGNode->getSGNode(),E_E0_Case_SG,Score_SG);


#ifdef DEBUG_OARG_DO_RECTILINEAR
			cout<<"\n--[3: Pick second slant Edge]\n";
			PrintEdge(SG_E0);
#endif

#ifdef DEBUG_OARG_DO_RECTILINEAR
			cout<<"\n--[4: Compute the horizontal or vertical Edges in RG]\n";
#endif
			//[5] pick the horizontal or vertical Edges in RG
			double Score_RG = _UNSET;
			RGEdge* RG_E0 = Pick_RE0(SG_E,ProRGNode,Score_RG);

#ifdef DEBUG_OARG_DO_RECTILINEAR
			cout<<"\n--[5: Pick horizontal or vertical Edges in RG]\n";
			PrintRGEdge(RG_E0);
#endif

			if(Score_RG > Score_SG && Score_RG != 0 && Score_RG != _UNSET)
			{	E_E0_Case = _E_E0_CASE1;
			}
			else
			{
				E_E0_Case = E_E0_Case_SG;
				TurnRectilinear(SG_E0);  		
			}


#ifdef DEBUG_OARG_DO_RECTILINEAR
			cout<<"\n--[6: Start to rectilinearizing]\n";
#endif

			//premise: vb stands for turning point of RGEdge
			//         vc stands for end point of RGEdge

			
			//Case 0: only one edge to rectilinearize
			if(E_E0_Case == _E_E0_CASE0)
			{	

#ifdef DEBUG_OARG_DO_RECTILINEAR
				cout<<"CASE 0: only one slant edge\n";
#endif

				RGNode* Root = ProRGNode;
				SGNode* SG_N = SG_E->getToNode();
					
				//Vc
				RGNode* Vc = AddRGNode(SG_N);				
									
				if(Is_Horizontal_Line(SG_E) || Is_Vertical_Line(SG_E)) //This is a H or V Line, so there is no need to have turning node
				{
					AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc));
				}
				else
				{
					//Vb -- TURNING Node : random select lower or upper L shape
					RGNode* Vb = NULL;

					Vb = AddRGNode(SG_N->getX(),Root->getY(),Root->getLayer(),_TURNING);
					//
					AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb));
					AddRGEdge(StruFromRGNode(Vb),StruToRGNode(Vc));
				}

				//
				AddUnPropNode(Vc);
			}
			else if(E_E0_Case == _E_E0_CASE1) //one slant edge and one H/V edge
			{	
#ifdef DEBUG_OARG_DO_RECTILINEAR
				cout<<"CASE 1: one slant edge and one H/V edge\n";
#endif

				RGNode* Root = ProRGNode;
				SGNode* SG_N = SG_E->getToNode();
					
				//Vc
				RGNode* Vc = AddRGNode(SG_N);				
					
				//Vb -- TURNING Node : select lower or upper L shape by RG_E0
				RGNode* Vb=NULL;
				if(Is_Horizontal_Line(RG_E0) && Is_Horizontal_Line(SG_E)) //SG_E is H or V
				{

					AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc));
				}
				else if(Is_Vertical_Line(RG_E0) && Is_Vertical_Line(SG_E)) //SG_E is H or V
				{

					AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc));
				}
				else if(Is_Horizontal_Line(RG_E0)) //SG_E isn't H or V, selecting lower or upper L shape by RG_E0
				{	
					if(RG_E0->getToNode()->getX() == SG_N->getX() && RG_E0->getToNode()->getY() == Root->getY())
					{	Vb = RG_E0->getToNode();
					}
					else
					{	
						//check need to create a new RG node ?
						bool needNewNode = true;
						for(int i=0;i<RG_E0->getToNode()->getNumEdge();i++)
						{	if(RG_E0->getToNode()->getEdge(i) != RG_E0->getDualEdge())
							{	RGNode* N = RG_E0->getToNode()->getEdge(i)->getToNode();
								if(N->getX() == SG_N->getX() && N->getY() == Root->getY())
								{	Vb = N;
									//cout<<"check need to create a new RG node ? => NO! (H)\n";
									//PrintRGNode(Vb);
									needNewNode = false;
									//getchar();
									break;
								}								
								
							}
						}
						if(needNewNode)
						{	Vb = AddRGNode(SG_N->getX(),Root->getY(),Root->getLayer(),_TURNING);
						}
					}

					AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb));
					AddRGEdge(StruFromRGNode(Vb),StruToRGNode(Vc));

				}
				else if(Is_Vertical_Line(RG_E0)) //SG_E isn't H or V, selecting lower or upper L shape by RG_E0
				{	
					if(RG_E0->getToNode()->getY() == SG_N->getY() && RG_E0->getToNode()->getX() == Root->getX())
					{	Vb = RG_E0->getToNode();
					}
					else
					{	
						//check need to create a new RG node ?
						bool needNewNode = true;
						for(int i=0;i<RG_E0->getToNode()->getNumEdge();i++)
						{	if(RG_E0->getToNode()->getEdge(i) != RG_E0->getDualEdge())
							{	RGNode* N = RG_E0->getToNode()->getEdge(i)->getToNode();
								if(N->getY() == SG_N->getY() && N->getX() == Root->getX())
								{	Vb = N;
									//cout<<"check need to create a new RG node ? => NO! (V)\n";
									//PrintRGNode(Vb);
									needNewNode = false;
									//getchar();
									break;
								}								
								
							}
						}						
						if(needNewNode)
						{	Vb = AddRGNode(Root->getX(),SG_N->getY(),Root->getLayer(),_TURNING);
						}
					}

					AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb));
					AddRGEdge(StruFromRGNode(Vb),StruToRGNode(Vc));

				}
				else //SG_E isn't H or V, selecting lower or upper L shape by RG_E0
				{	
					if(RG_E0->getToNode()->getX() == Root->getX() && RG_E0->getToNode()->getY() == SG_N->getY())
					{	Vb = RG_E0->getToNode();
					}
					else					
					{	Vb = AddRGNode(Root->getX(),SG_N->getY(),Root->getLayer(),_TURNING);
					}

					
					AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb));
					AddRGEdge(StruFromRGNode(Vb),StruToRGNode(Vc));
				}

				//
				AddUnPropNode(Vc);
			}
			else if((Is_Vertical_Line(SG_E0) && Is_Vertical_Line(SG_E))
				|| (Is_Horizontal_Line(SG_E0) && Is_Horizontal_Line(SG_E)) )
			{
#ifdef DEBUG_OARG_DO_RECTILINEAR
				cout<<"Merge two direct edges (all V or H) into one\n";
#endif
				RGNode* Root = ProRGNode;					
				SGNode* SG_N1 = SG_E->getToNode();	//due to two slant edges, so we need to pick two nodes
				SGNode* SG_N2 = SG_E0->getToNode(); //due to two slant edges, so we need to pick two nodes

				//Vc1 Vc2
				RGNode* Vc1 = AddRGNode(SG_N1);
				RGNode* Vc2 = AddRGNode(SG_N2);
				//
				AddUnPropNode(Vc1);
				AddUnPropNode(Vc2);

				AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc1));
				AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc2));
			}
			else //merge two slant edges into one, so generate two RG nodes
			{
#ifdef DEBUG_OARG_DO_RECTILINEAR
				cout<<"Merge two slant edges into one\n";
#endif

				RGNode* Root = ProRGNode;					
				SGNode* SG_N1 = SG_E->getToNode();	//due to two slant edges, so we need to pick two nodes
				SGNode* SG_N2 = SG_E0->getToNode(); //due to two slant edges, so we need to pick two nodes

				//Vc1 Vc2
				RGNode* Vc1 = AddRGNode(SG_N1);
				RGNode* Vc2 = AddRGNode(SG_N2);
				//
				AddUnPropNode(Vc1);
				AddUnPropNode(Vc2);

				//three cases: case2, case3, case4
				//within each case, only add new turing node (name as Vb...) and add new rectilinear edge					

				if(E_E0_Case == _E_E0_CASE2)
				{	
#ifdef DEBUG_OARG_DO_RECTILINEAR
					cout<<"CASE 2: adjacent in the same horizontal quadrant\n";
#endif

					//Vb1 Vb2
					RGNode* Vb1 = NULL;
					RGNode* Vb2 = NULL;

					if(Vc1->getY() == Vc2->getY()) //check Vb1 = Vb2 ?
					{	
						Vb1 = AddRGNode(Root->getX(),Vc1->getY(),Root->getLayer(),_TURNING);
						Vb2 = Vb1;
					}
					else
					{
						Vb1 = AddRGNode(Root->getX(),Vc1->getY(),Root->getLayer(),_TURNING);
						Vb2 = AddRGNode(Root->getX(),Vc2->getY(),Root->getLayer(),_TURNING);
					}

					//
					if(abs(Root->getY() - Vb1->getY()) < abs(Root->getY() - Vb2->getY()))
					{	AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb1));
						if(Vb1 != Vb2)
							AddRGEdge(StruFromRGNode(Vb1),StruToRGNode(Vb2));
					}
					else
					{	AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb2));
						if(Vb1 != Vb2)	
							AddRGEdge(StruFromRGNode(Vb2),StruToRGNode(Vb1));
					}

					//
					if(Vb1->getY() == Vc1->getY())
					{
						AddRGEdge(StruFromRGNode(Vb1),StruToRGNode(Vc1));
						AddRGEdge(StruFromRGNode(Vb2),StruToRGNode(Vc2));
					}
					else
					{
						AddRGEdge(StruFromRGNode(Vb1),StruToRGNode(Vc2));
						AddRGEdge(StruFromRGNode(Vb2),StruToRGNode(Vc1));
					}

				}
				else if(E_E0_Case == _E_E0_CASE3)
				{					

#ifdef DEBUG_OARG_DO_RECTILINEAR
					cout<<"CASE 3: adjacent in the same vertical quadrant\n";
#endif
	
					//Vb1 Vb2
					RGNode* Vb1 = NULL;
					RGNode* Vb2 = NULL;

					if(Vc1->getX() == Vc2->getX()) //check Vb1 = Vb2 ?
					{	
						Vb1 = AddRGNode(SG_N1->getX(),Root->getY(),Root->getLayer(),_TURNING);
						Vb2 = Vb1;						
					}
					else
					{
						Vb1 = AddRGNode(Vc1->getX(),Root->getY(),Root->getLayer(),_TURNING);
						Vb2 = AddRGNode(Vc2->getX(),Root->getY(),Root->getLayer(),_TURNING);
					}

					if(abs(Root->getX() - Vb1->getX()) < abs(Root->getX() - Vb2->getX()))
					{	

						AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb1));
						if(Vb1 != Vb2)
							AddRGEdge(StruFromRGNode(Vb1),StruToRGNode(Vb2));
					}
					else
					{	
						AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb2));
						if(Vb1 != Vb2)
							AddRGEdge(StruFromRGNode(Vb2),StruToRGNode(Vb1));
					}

					//
					if(Vb1->getX() == Vc1->getX())
					{

						AddRGEdge(StruFromRGNode(Vb1),StruToRGNode(Vc1));
						AddRGEdge(StruFromRGNode(Vb2),StruToRGNode(Vc2));
					}
					else
					{
						AddRGEdge(StruFromRGNode(Vb1),StruToRGNode(Vc2));
						AddRGEdge(StruFromRGNode(Vb2),StruToRGNode(Vc1));
					}

				}
				else if(E_E0_Case == _E_E0_CASE4)
				{	
#ifdef DEBUG_OARG_DO_RECTILINEAR
					cout<<"CASE 4: the same quadrant\n";
#endif

					if(Is_Horizontal_Line(SG_E)) //SG_E is H, only need one turning point
					{

						//Vb
						RGNode* Vcommon = NULL;
						if(Vc2->getX() == Root->getX())
						{	//Vcommon = AddRGNode(SG_N2->getX(),Root->getY(),Root->getLayer(),_PIN); 
							Vcommon = Root;

							//
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc1));
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc2));

						}
						else
						{	Vcommon = AddRGNode(Vc2->getX(),Root->getY(),Root->getLayer(),_TURNING); 

							//
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vcommon));
							AddRGEdge(StruFromRGNode(Vcommon),StruToRGNode(Vc1));
							AddRGEdge(StruFromRGNode(Vcommon),StruToRGNode(Vc2));
						}
						
					}
					else if(Is_Horizontal_Line(SG_E0)) //SG_E0 is H, only need one turning point
					{
						//Vb
						RGNode* Vcommon = NULL;
						if(Vc1->getX() == Root->getX())
						{	//Vcommon = AddRGNode(SG_N2->getX(),Root->getY(),Root->getLayer(),_PIN); 
							Vcommon = Root;

							//
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc1));
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc2));

						}
						else
						{	Vcommon = AddRGNode(Vc1->getX(),Root->getY(),Root->getLayer(),_TURNING); 

							//
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vcommon));
							AddRGEdge(StruFromRGNode(Vcommon),StruToRGNode(Vc1));
							AddRGEdge(StruFromRGNode(Vcommon),StruToRGNode(Vc2));
						}
					}
					else if(Is_Vertical_Line(SG_E)) //SG_E is V, only need one turning point
					{

						//Vb
						RGNode* Vcommon = NULL;
						if(Vc2->getY() == Root->getY())
						{	//Vcommon = AddRGNode(Root->getX(),SG_N1->getY(),Root->getLayer(),_PIN); 
							Vcommon = Root;

							//
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc1));
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc2));
						}
						else
						{	Vcommon = AddRGNode(Root->getX(),Vc2->getY(),Root->getLayer(),_TURNING); 

							//
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vcommon));
							AddRGEdge(StruFromRGNode(Vcommon),StruToRGNode(Vc1));
							AddRGEdge(StruFromRGNode(Vcommon),StruToRGNode(Vc2));
						}
					}
					else if(Is_Vertical_Line(SG_E0)) //SG_E0 is V, only need one turning point
					{

						//Vb
						RGNode* Vcommon = NULL;
						if(Vc1->getY() == Root->getY())
						{	//Vcommon = AddRGNode(Root->getX(),SG_N1->getY(),Root->getLayer(),_PIN); 
							Vcommon = Root;

							//
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc1));
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vc2));
						}
						else
						{	Vcommon = AddRGNode(Root->getX(),Vc1->getY(),Root->getLayer(),_TURNING); 

							//
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vcommon));
							AddRGEdge(StruFromRGNode(Vcommon),StruToRGNode(Vc1));
							AddRGEdge(StruFromRGNode(Vcommon),StruToRGNode(Vc2));
						}
					}
					else //SG_E isn't H or V, need two turning point
					{
						//three situations
						if(abs(Vc1->getX()-Root->getX()) >= abs(Vc2->getX()-Root->getX())
							&& abs(Vc1->getY()-Root->getY()) >= abs(Vc2->getY()-Root->getY()))
						{
							//Vb1 Vb2
							RGNode* Vb1 = AddRGNode(Vc2->getX(),Root->getY(),Root->getLayer(),_TURNING);
							RGNode* Vb2 = NULL;
							if(Vc1->getX() == Vc2->getX() || Vc1->getY() == Vc2->getY())
							{
								Vb2 = Vc2;
								//
								AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb1));
								AddRGEdge(StruFromRGNode(Vb1),StruToRGNode(Vc2));
								AddRGEdge(StruFromRGNode(Vc2),StruToRGNode(Vc1));
							}
							else
							{
								Vb2 = AddRGNode(Vc1->getX(),Vc2->getY(),Root->getLayer(),_TURNING);
								//
								AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb1));
								AddRGEdge(StruFromRGNode(Vb1),StruToRGNode(Vc2));
								AddRGEdge(StruFromRGNode(Vc2),StruToRGNode(Vb2));
								AddRGEdge(StruFromRGNode(Vb2),StruToRGNode(Vc1));
							}
						}
						else if(abs(Vc1->getX()-Root->getX()) <= abs(Vc2->getX()-Root->getX())
							&& abs(Vc1->getY()-Root->getY()) <= abs(Vc2->getY()-Root->getY()))
						{
							//Vb1 Vb2
							RGNode* Vb1 = AddRGNode(Vc1->getX(),Root->getY(),Root->getLayer(),_TURNING);
							RGNode* Vb2 = NULL;
							if(Vc1->getX() == Vc2->getX() || Vc1->getY() == Vc2->getY())
							{
								Vb2 = Vc2;
								//
								AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb1));
								AddRGEdge(StruFromRGNode(Vb1),StruToRGNode(Vc1));
								AddRGEdge(StruFromRGNode(Vc1),StruToRGNode(Vc2));
							}
							else
							{
								Vb2 = AddRGNode(Vc2->getX(),Vc1->getY(),Root->getLayer(),_TURNING);
								//
								AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb1));
								AddRGEdge(StruFromRGNode(Vb1),StruToRGNode(Vc1));
								AddRGEdge(StruFromRGNode(Vc1),StruToRGNode(Vb2));
								AddRGEdge(StruFromRGNode(Vb2),StruToRGNode(Vc2));
							}
						}
						else
						{
							int quadrant_N = Compute_Quadrant(SG_N1,Root->getSGNode());

							int X=_UNSET,Y=_UNSET;

							switch(quadrant_N)
							{
								case _QUADRANT1:
									X = min(SG_N1->getX(),SG_N2->getX());
									Y = min(SG_N1->getY(),SG_N2->getY());
									break;
								case _QUADRANT2:
									X = max(SG_N1->getX(),SG_N2->getX());
									Y = min(SG_N1->getY(),SG_N2->getY());
									break;
								case _QUADRANT3:
									X = max(SG_N1->getX(),SG_N2->getX());
									Y = max(SG_N1->getY(),SG_N2->getY());
									break;
								case _QUADRANT4:
									X = min(SG_N1->getX(),SG_N2->getX());
									Y = max(SG_N1->getY(),SG_N2->getY());
									break; 
								default:
									quadrant_N = Compute_Quadrant(SG_N2,Root->getSGNode());
									switch(quadrant_N)
									{
										case _QUADRANT1:
											X = min(SG_N1->getX(),SG_N2->getX());
											Y = min(SG_N1->getY(),SG_N2->getY());
											break;
										case _QUADRANT2:
											X = max(SG_N1->getX(),SG_N2->getX());
											Y = min(SG_N1->getY(),SG_N2->getY());
											break;
										case _QUADRANT3:
											X = max(SG_N1->getX(),SG_N2->getX());
											Y = max(SG_N1->getY(),SG_N2->getY());
											break;
										case _QUADRANT4:
											X = min(SG_N1->getX(),SG_N2->getX());
											Y = max(SG_N1->getY(),SG_N2->getY());
											break;
										default:
											cout<<"Error! quadrant_N is wrong\n";
											exit(0);
									}
							}

							//Vb1 Vb2
							RGNode* Vcommon = AddRGNode(X,Y,Root->getLayer(),_TURNING); 
							RGNode* Vb = AddRGNode(X,Root->getY(),Root->getLayer(),_TURNING);

				
							//
							AddRGEdge(StruFromRGNode(Root),StruToRGNode(Vb));
							AddRGEdge(StruFromRGNode(Vb),StruToRGNode(Vcommon));
							AddRGEdge(StruFromRGNode(Vcommon),StruToRGNode(Vc1));
							AddRGEdge(StruFromRGNode(Vcommon),StruToRGNode(Vc2));
						}
					}

				}
				else
				{	cout<<"Error! E_E0_Case is wrong!\n";
					exit(0);
				}
			}

		}		

#ifdef DEBUG_OARG_DO_RECTILINEAR		
		cout<<"===========================\n# of Un Processed Node = "<<vUn_RGNode.size()<<endl;
#endif


}





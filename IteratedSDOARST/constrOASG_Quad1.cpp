#include "OARST.h"

void OASpanningGraph::Quad1()
{
	vA.clear(); //active set
	vAev.clear();
	vAeh.clear();


	sort(vSweepEntry.begin(),vSweepEntry.end(),SE_NonDecreaseXplusY()); //non-decrease = increase

	cout<<"Quad1 : handle all nodes' Quad1 (Pos,Pos) Connection (X+Y:Small->Big)\n";

	for(int i=0;i<vSweepEntry.size();i++)
	{
		SweepEntry* V = vSweepEntry[i];

		for(int j=vA.size()-1;j>=0;j--)
		{
			SweepEntry* U = vA[j];

			if(V_In_U_Quad1(U,V))
			{
				if(U->getType()==_PIN && (U_On_Block_Edge_vAeh(U) || U_On_Block_Edge_vAev(U)))				
				{
					//Remove U from vA (the view of U is blocked by obstacle)
					vA.erase(vA.begin()+j);
				}
				else if(Block_By_vAev_vAeh(U,V)==false)
				{	
					if(Corner_Connection_Check(U,V) == false)
					{	//Remove U from vA
						vA.erase(vA.begin()+j);
					}
					else
					{	AddingEdge(U->getN(),V->getN());
						//Remove U from vA
						vA.erase(vA.begin()+j);
					}
				}
			}
		}

		if(V->getType() == _OBSTACLE_LEFT_LOWER) //If V is a corner
		{	Add_Block_Edge(V,_OBSTACLE_LEFT_LOWER);
		}
		else if(V->getType() == _OBSTACLE_RIGHT_UPPER) //If V is a corner
		{	Remove_Block_Edge(V->get_Left_Lower_N(),_OBSTACLE_LEFT_LOWER);
		}

		vA.push_back(V);
	}
}

//================================================================================================

void OASpanningGraph::Remove_Block_Edge(SweepEntry* N,int RemoveType)
{	
	switch(RemoveType)
	{	case _OBSTACLE_LEFT_LOWER:		
			//remove eh
			for(int i=vAeh.size()-1;i>=0;i--)
			{	if(vAeh[i]->getN1() == N)
				{	vAeh.erase(vAeh.begin()+i);
					break;
				}
			}
			//remove ev
			for(int i=vAev.size()-1;i>=0;i--)
			{	if(vAev[i]->getN1() == N)
				{	vAev.erase(vAev.begin()+i);
					break;
				}
			}
			break;
		case _OBSTACLE_LEFT_UPPER:
			//remove eh
			for(int i=vAeh.size()-1;i>=0;i--)
			{	if(vAeh[i]->getN1() == N)
				{	vAeh.erase(vAeh.begin()+i);
					break;
				}
			}
			//remove ev
			for(int i=vAev.size()-1;i>=0;i--)
			{	if(vAev[i]->getN2() == N)
				{	vAev.erase(vAev.begin()+i);
					break;
				}
			}
			break;
		case _OBSTACLE_RIGHT_LOWER:
			//remove eh
			for(int i=vAeh.size()-1;i>=0;i--)
			{	if(vAeh[i]->getN2() == N)
				{	vAeh.erase(vAeh.begin()+i);
					break;
				}
			}
			//remove ev
			for(int i=vAev.size()-1;i>=0;i--)
			{	if(vAev[i]->getN1() == N)
				{	vAev.erase(vAev.begin()+i);
					break;
				}
			}
			break;
		case _OBSTACLE_RIGHT_UPPER:
			for(int i=vAeh.size()-1;i>=0;i--)
			{	if(vAeh[i]->getN2() == N)
				{	vAeh.erase(vAeh.begin()+i);
					break;
				}
			}
			//remove ev
			for(int i=vAev.size()-1;i>=0;i--)
			{	if(vAev[i]->getN2() == N)
				{	vAev.erase(vAev.begin()+i);
					break;
				}
			}
			break;
		default:
			cout<<"OASpanningGraph::CheckVisibleNode(), Check Type Error!\n";
			exit(0);
	}
}

void OASpanningGraph::Add_Block_Edge(SweepEntry* N,int AddType)
{
	Vertical_Block_Edge* EV = NULL;
	Horizontal_Block_Edge* EH = NULL;

	switch(AddType)
	{	case _OBSTACLE_LEFT_LOWER:		
				EV = new Vertical_Block_Edge(N,N->get_Left_Upper_N());
				vAev.push_back(EV);
				EH = new Horizontal_Block_Edge(N,N->get_Right_Lower_N());
				vAeh.push_back(EH);
			break;
		case _OBSTACLE_LEFT_UPPER:
				EV = new Vertical_Block_Edge(N->get_Left_Lower_N(),N);
				vAev.push_back(EV);
				EH = new Horizontal_Block_Edge(N,N->get_Right_Upper_N());
				vAeh.push_back(EH);
			break;
		case _OBSTACLE_RIGHT_LOWER:
				EV = new Vertical_Block_Edge(N,N->get_Right_Upper_N());
				vAev.push_back(EV);
				EH = new Horizontal_Block_Edge(N->get_Left_Lower_N(),N);
				vAeh.push_back(EH);
			break;
		case _OBSTACLE_RIGHT_UPPER:
				EV = new Vertical_Block_Edge(N->get_Right_Lower_N(),N);
				vAev.push_back(EV);
				EH = new Horizontal_Block_Edge(N->get_Left_Upper_N(),N);
				vAeh.push_back(EH);
			break;
		default:
			cout<<"OASpanningGraph::CheckVisibleNode(), Check Type Error!\n";
			exit(0);
	}
}
bool OASpanningGraph::Block_By_vAev(int Y,int X1,int X2) //input H line
{
	for(int j=0;j<vAev.size();j++)
	{
		Vertical_Block_Edge* E = vAev[j];
		if(E->getY1() < Y && Y < E->getY2())
		{	
			if(X1 < E->getX() && E->getX() < X2)
			{	return true;
			}
		}
	}
	return false;
}

bool OASpanningGraph::Block_By_vAeh(int X,int Y1,int Y2) //input V line
{
	for(int j=0;j<vAeh.size();j++)
	{
		Horizontal_Block_Edge* E = vAeh[j];
		if(E->getX1() < X && X < E->getX2())
		{	
			if(Y1 < E->getY() && E->getY() < Y2)
			{	return true;
			}
		}


	}
	return false;
}

bool OASpanningGraph::U_On_Block_Edge_vAev(SweepEntry* U)
{	
	for(int j=0;j<vAev.size();j++)
	{
		Vertical_Block_Edge* E = vAev[j];
		if(U->getX() == E->getX())
		{
			if(E->getY1() < U->getY() && U->getY() < E->getY2())
			//if(E->getY1() <= U->getY() && U->getY() <= E->getY2())
			{	
				return true;				
			}
		}
	}
	return false;
}

bool OASpanningGraph::U_On_Block_Edge_vAeh(SweepEntry* U)
{
	for(int j=0;j<vAeh.size();j++)
	{
		Horizontal_Block_Edge* E = vAeh[j];
		if(U->getY() == E->getY())
		{
			if(E->getX1() < U->getX() && U->getX() < E->getX2())
			//if(E->getX1() <= U->getX() && U->getX() <= E->getX2())
			{	
				return true;
			}
		}
	}
	return false;
}

bool OASpanningGraph::Corner_Connection_Check(SweepEntry* U,SweepEntry* V)
{
	//begin corner connection check
	if(U->getType()!=_PIN && V->getType()!=_PIN && U->getID()!=V->getID())
	{
		if(V->getX() == U->getX()) //Vertical connection
		{
			int Y1 = min(U->getY(),V->getY());
			int Y2 = max(U->getY(),V->getY());

			int U_Y1 = U->get_Left_Lower_N()->getY();
			int U_Y2 = U->get_Left_Upper_N()->getY();
			int V_Y1 = V->get_Left_Lower_N()->getY();
			int V_Y2 = V->get_Left_Upper_N()->getY();

			//check edge (Y1->Y2) in both obstacles ?
			if(U_Y1 <= Y1 && Y1 <= U_Y2 && U_Y1 <= Y2 && Y2 <= U_Y2
				&& V_Y1 <= Y1 && Y1 <= V_Y2 && V_Y1 <= Y2 && Y2 <= V_Y2 )
			{								
				return false;
			}
		}
		else if(V->getY() == U->getY()) //Horizontal connection
		{
			int X1 = min(U->getX(),V->getX());
			int X2 = max(U->getX(),V->getX());

			int U_X1 = U->get_Left_Lower_N()->getX();
			int U_X2 = U->get_Right_Upper_N()->getX();
			int V_X1 = V->get_Left_Lower_N()->getX();
			int V_X2 = V->get_Right_Upper_N()->getX();

			//check edge (X1->X2) in both obstacles ?
			if(U_X1 <= X1 && X1 <= U_X2 && U_X1 <= X2 && X2 <= U_X2
				&& V_X1 <= X1 && X1 <= V_X2 && V_X1 <= X2 && X2 <= V_X2 )
			{								
				return false;
			}
		}
	}
	return true;
}


bool OASpanningGraph::Block_By_vAev_vAeh(SweepEntry* U,SweepEntry* V)
{	
	int Y1 = min(U->getY(),V->getY());
	int Y2 = max(U->getY(),V->getY());

	int X1 = min(U->getX(),V->getX());
	int X2 = max(U->getX(),V->getX());

	if(U->getX() == V->getX())
	{	//check block by H line
		int Common_X = U->getX();	
		//Block_By_vAeh() //input V line
		return Block_By_vAeh(Common_X,Y1,Y2);
	}
	else if(U->getY() == V->getY())
	{	//check block by V line
		int Common_Y = U->getY();
		//Block_By_vAev() //input H line
		return Block_By_vAev(Common_Y,X1,X2);
	}
	else
	{	//manhantan line check : Upper L lower L

		bool block_1_L = false;
		bool block_2_L = false;

		//Block_By_vAeh() //input V line
		//Block_By_vAev() //input H line

		if(Block_By_vAev(U->getY(),X1,X2) || Block_By_vAeh(V->getX(),Y1,Y2))
		{	block_1_L = true;
		}
		if(Block_By_vAev(V->getY(),X1,X2) || Block_By_vAeh(U->getX(),Y1,Y2))
		{	block_2_L = true;
		}

		//if(block_1_L==false || block_2_L==false)
		if(block_1_L==false && block_2_L==false)
		{	return false;
		}
		else
		{	return true;
		}
	}
}

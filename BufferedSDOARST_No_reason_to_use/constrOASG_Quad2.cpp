#include "OARST.h"

void OASpanningGraph::Quad2() //(X,Y) (Neg,Pos)
{
	vA.clear(); //active set
	vAev.clear();
	vAeh.clear();


	sort(vSweepEntry.begin(),vSweepEntry.end(),SE_NonDecreaseYminusX()); //non-decrease = increase

	cout<<"Quad2 : handle all nodes' Quad2 (Neg,Pos) Connection (Y-X:Small->Big)\n";

	for(int i=0;i<vSweepEntry.size();i++)
	{
		SweepEntry* V = vSweepEntry[i];

		for(int j=vA.size()-1;j>=0;j--)
		{
			SweepEntry* U = vA[j];

			if(V_In_U_Quad2(U,V))
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

		if(V->getType() == _OBSTACLE_RIGHT_LOWER) //If V is a corner
		{	Add_Block_Edge(V,_OBSTACLE_RIGHT_LOWER);
		}
		else if(V->getType() == _OBSTACLE_LEFT_UPPER) //If V is a corner
		{	Remove_Block_Edge(V->get_Right_Lower_N(),_OBSTACLE_RIGHT_LOWER);
		}

		vA.push_back(V);
	}
}
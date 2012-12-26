#include "OARST.h"

void OASpanningGraph::Quad3()
{
	vA.clear(); //active set
	vAev.clear();
	vAeh.clear();


	sort(vSweepEntry.begin(),vSweepEntry.end(),SE_NonDecreaseXplusY()); //non-decrease = increase

	cout<<"Quad3 : handle all nodes' Quad3 (Neg,Neg) Connection (X+Y:Big->Small)\n";

	for(int i=vSweepEntry.size()-1;i>=0;i--)
	{
		SweepEntry* V = vSweepEntry[i];

		for(int j=vA.size()-1;j>=0;j--)
		{
			SweepEntry* U = vA[j];

			if(V_In_U_Quad3(U,V))
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

		if(V->getType() == _OBSTACLE_RIGHT_UPPER) //If V is a corner
		{	Add_Block_Edge(V,_OBSTACLE_RIGHT_UPPER);
		}
		else if(V->getType() == _OBSTACLE_LEFT_LOWER) //If V is a corner
		{	Remove_Block_Edge(V->get_Right_Upper_N(),_OBSTACLE_RIGHT_UPPER);
		}

		vA.push_back(V);
	}
}
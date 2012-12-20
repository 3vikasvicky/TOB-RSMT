#include "OARST.h"

void OASpanningGraph::Quad4()
{
	vA.clear(); //active set
	vAev.clear();
	vAeh.clear();


	sort(vSweepEntry.begin(),vSweepEntry.end(),SE_NonDecreaseYminusX()); //non-decrease = increase

	cout<<"Quad4 : handle all nodes' Quad4 (Pos,Neg) Connection (Y-X:Big->Small)\n";

	for(int i=vSweepEntry.size()-1;i>=0;i--)
	{
		SweepEntry* V = vSweepEntry[i];

		for(int j=vA.size()-1;j>=0;j--)
		{
			SweepEntry* U = vA[j];

			if(V_In_U_Quad4(U,V))
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

		if(V->getType() == _OBSTACLE_LEFT_UPPER) //If V is a corner
		{	Add_Block_Edge(V,_OBSTACLE_LEFT_UPPER);
		}
		else if(V->getType() == _OBSTACLE_RIGHT_LOWER) //If V is a corner
		{	Remove_Block_Edge(V->get_Left_Upper_N(),_OBSTACLE_LEFT_UPPER);
		}

		vA.push_back(V);
	}
}
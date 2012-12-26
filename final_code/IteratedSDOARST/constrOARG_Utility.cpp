#include "OARST.h"
using namespace auxiliary;


double OARectilinearGraph::getRoutedRGEdgeLength()
{	
/*	int method=1;

	if(method==1)
	{
		double val=0;
		vector<RGEdge*> vAddedEdge;
		for(size_t i=0;i<vRGEdge.size();i++)
		{	
			RGEdge* E = vRGEdge[i];

			bool added = false;
			for(size_t j=0;j<vAddedEdge.size();j++)
			{	RGEdge* E2 = vAddedEdge[j];				
				if(Same_RGNode(E->getFromNode(),E2->getFromNode()) && Same_RGNode(E->getToNode(),E2->getToNode()))
				{	added = true;
					break;
				}
				else if(Same_RGNode(E->getFromNode(),E2->getToNode()) && Same_RGNode(E->getToNode(),E2->getFromNode()))
				{
					added = true;
					break;
				}
			}
			
			if(added==false)
			{	val=val+E->getLength();
				vAddedEdge.push_back(E);
			}
		}
		//return val/2;  //eliminate dual edges
		return val;  //eliminate dual edges
	}
	else
	{*/
		double val=0;
		for(size_t i=0;i<vRGEdge.size();i++)
		{	RGEdge* E = vRGEdge[i];
			val=val+E->getLength();
		}
		return val/2;  //eliminate dual edges
//	}
}

RGEdge* OARectilinearGraph::Find_connEdge(RGNode* fromNode,RGNode* toNode)
{
	for(int j=0;j<fromNode->getNumEdge();j++)
	{	
		if(fromNode->getEdge(j)->getToNode() == toNode)
		{	
			return fromNode->getEdge(j);
		}
	}

	cout<<"didn't find edge (OARectilinearGraph::Find_connEdge())\n";
	exit(0);
}

double OARectilinearGraph::computeRadius()
{	
	double val=0;
	for(size_t i=0;i<getNumNode();i++)
	{	
		RGNode* n=getNode(i);
		if(n->checkIsLeaf() && val<n->getDistFromDriving())
		{	val=n->getDistFromDriving();
		}
	}
	setLongestRadius(val);

	return val;
}

double OARectilinearGraph::Compute_Share_Length(SGNode* N1,RGNode* N2,RGNode* Root,int &E_E0_Case)
{
	double share_length=_UNSET;

	//[1] Check the quadrant
	int quadrant_N1 = Compute_Quadrant(N1,Root);
	int quadrant_N2 = Compute_Quadrant(N2,Root);

	//[2] Compute share length
	if(quadrant_N1 == quadrant_N2 //in the same quadrant
		|| (quadrant_N1 == _QUADRANT1 && (quadrant_N2 == _QUADRANT14 || quadrant_N2== _QUADRANT12))
		|| (quadrant_N1 == _QUADRANT2 && (quadrant_N2 == _QUADRANT23 || quadrant_N2== _QUADRANT12))
		|| (quadrant_N1 == _QUADRANT3 && (quadrant_N2 == _QUADRANT23 || quadrant_N2== _QUADRANT34))
		|| (quadrant_N1 == _QUADRANT4 && (quadrant_N2 == _QUADRANT41 || quadrant_N2== _QUADRANT34)) )

	{	double diffX = min( abs(N1->getX()-Root->getX()) , abs(N2->getX()-Root->getX()));
		double diffY = min( abs(N1->getY()-Root->getY()) , abs(N2->getY()-Root->getY()));
		share_length = diffX+diffY;
		E_E0_Case = _E_E0_CASE4;
	}
	else if( (quadrant_N1==_QUADRANT1 && quadrant_N2==_QUADRANT2) || (quadrant_N1==_QUADRANT2 && quadrant_N2==_QUADRANT1) //in the same horizontal quadrant
		|| (quadrant_N1==_QUADRANT3 && quadrant_N2==_QUADRANT4) || (quadrant_N1==_QUADRANT4 && quadrant_N2==_QUADRANT3) )
	{	double diffY = min( abs(N1->getY()-Root->getY()) , abs(N2->getY()-Root->getY()));
		share_length = diffY;
		E_E0_Case = _E_E0_CASE2;
	}
	else if( (quadrant_N1==_QUADRANT1 && quadrant_N2==_QUADRANT4) || (quadrant_N1==_QUADRANT4 && quadrant_N2==_QUADRANT1) //in the same vertical quadrant
		|| (quadrant_N1==_QUADRANT3 && quadrant_N2==_QUADRANT2) || (quadrant_N1==_QUADRANT2 && quadrant_N2==_QUADRANT3) )
	{	double diffX = min( abs(N1->getX()-Root->getX()) , abs(N2->getX()-Root->getX()));
		share_length = diffX;
		E_E0_Case = _E_E0_CASE3;
	}

	return share_length;
}

double OARectilinearGraph::Compute_Share_Length(SGNode* N1,SGNode* N2,SGNode* Root,int &E_E0_Case)
{
	double share_length=_UNSET;

	//[1] Check the quadrant
	int quadrant_N1 = Compute_Quadrant(N1,Root);
	int quadrant_N2 = Compute_Quadrant(N2,Root);

	//[2] Compute share length
	if(quadrant_N1 == quadrant_N2) //in the same quadrant
	{	double diffX = min( abs(N1->getX()-Root->getX()) , abs(N2->getX()-Root->getX()));
		double diffY = min( abs(N1->getY()-Root->getY()) , abs(N2->getY()-Root->getY()));
		share_length = diffX+diffY;
		E_E0_Case = _E_E0_CASE4;
	}
	else if( (quadrant_N1==_QUADRANT1 && quadrant_N2==_QUADRANT2) || (quadrant_N1==_QUADRANT2 && quadrant_N2==_QUADRANT1) //in the same horizontal quadrant
		|| (quadrant_N1==_QUADRANT3 && quadrant_N2==_QUADRANT4) || (quadrant_N1==_QUADRANT4 && quadrant_N2==_QUADRANT3) )
	{	double diffY = min( abs(N1->getY()-Root->getY()) , abs(N2->getY()-Root->getY()));
		share_length = diffY;
		E_E0_Case = _E_E0_CASE2;
	}
	else if( (quadrant_N1==_QUADRANT1 && quadrant_N2==_QUADRANT4) || (quadrant_N1==_QUADRANT4 && quadrant_N2==_QUADRANT1) //in the same vertical quadrant
		|| (quadrant_N1==_QUADRANT3 && quadrant_N2==_QUADRANT2) || (quadrant_N1==_QUADRANT2 && quadrant_N2==_QUADRANT3) )
	{	double diffX = min( abs(N1->getX()-Root->getX()) , abs(N2->getX()-Root->getX()));
		share_length = diffX;
		E_E0_Case = _E_E0_CASE3;
	}

	return share_length;
}







//this function will be called in Pick_E(..) and Pick_E0(..)
void OARectilinearGraph::TurnRectilinear(SGEdge* E)
{	
	if(E!=NULL)
	{
		E->setRectState(true);
		E->getDualEdge()->setRectState(true);
	}
}

void OARectilinearGraph::AddUnPropNode(RGNode* node)
{	
	node->setIsLeaf(true); //unpropagated node is leaf now for now
	
	vUn_RGNode.push_back(node);
}


vector<int> vNodeResult2;

bool nodeRTree_SearchCallback2(int id, void* arg) 
{
  vNodeResult2.push_back(id);
  return true; // keep going
}


bool OARectilinearGraph::IncludePin(double x,double y,int z)
{
		
		vNodeResult2.clear();

		double x1 = x-0.1;
		double x2 = x+0.1;
		double y1 = y-0.1;
		double y2 = y+0.1;

		OASG->nodeRTreeSearch(new Point(x1,y1,z),new Point(x2,y2,z),nodeRTree_SearchCallback2);


		for(size_t i=0 ; i<vNodeResult2.size() ; i++)
		{
			SGNode* hitNode = OASG->getNode(vNodeResult2[i]);

			if(hitNode->getType()==_PIN)
			{	
				if(!(hitNode->getX()==x && hitNode->getY()==y && hitNode->getLayer()==z))
				{	return true;
				}
			}
		}

		return false;

}


bool OARectilinearGraph::IncludePin(RGNode* N)
{
		
		int x = N->getX();
		int y = N->getY();
		int z = N->getLayer();

		vNodeResult2.clear();

		double x1 = x-0.1;
		double x2 = x+0.1;
		double y1 = y-0.1;
		double y2 = y+0.1;

		OASG->nodeRTreeSearch(new Point(x1,y1,z),new Point(x2,y2,z),nodeRTree_SearchCallback2);


		for(size_t i=0 ; i<vNodeResult2.size() ; i++)
		{
			SGNode* hitNode = OASG->getNode(vNodeResult2[i]);

			if(hitNode->getType()==_PIN)
			{	
				if(!(hitNode->getX()==x && hitNode->getY()==y && hitNode->getLayer()==z))
				{	return true;
				}
			}
		}

		return false;

}



//trace from source to sinks and compute the right distance from driver
void OARectilinearGraph::computeDistFromDriver()
{


	//--------------------------------------------------------------------
	//in order to test this function whether it can work correctly, 
	//and we don't have to change our DoOneRectilinear() function
	//so we initialize the distance from driver of each sink
	//--------------------------------------------------------------------

	//first set all the node's distance from driver = _INFINITE
	for(int i=0;i<this->getNumNode();i++)
	{	getNode(i)->setDistFromDriving(_INFINITE);
	}
	//set driver's distance from driver = 0
	this->getDrivingNode()->setDistFromDriving(0);

	//--------------------------------------------------------------------


	vector<RGNode*> vPropNode;
	vPropNode.push_back(this->getDrivingNode());


	while(!vPropNode.empty())
	{
		RGNode* propN = vPropNode[vPropNode.size()-1];
		vPropNode.pop_back();
		for(int j=0;j<propN->getNumEdge();j++)
		{	
			RGNode* toN = propN->getEdge(j)->getToNode();
			if(toN->getDistFromDriving()==_INFINITE)
			{	toN->setDistFromDriving(propN->getDistFromDriving() + propN->getEdge(j)->getLength());
				vPropNode.push_back(toN);
			}
		}	
	}

}



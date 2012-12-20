#include "OARST.h"

double OASpanningGraph::getRoutedSGEdgeLength()
{	
	double val=0;
	for(size_t i=0;i<this->vSGEdge.size();i++)
	{	
		if(this->vSGEdge[i]->checkRouted()==true)
		{	val=val+this->vSGEdge[i]->getLength();
		}
	}
	return val/2;  //eliminate dual edges
}

double OASpanningGraph::computeRadius()
{	
	double val=0;
	for(size_t i=0;i<getNumNode();i++)
	{	SGNode* n=getNode(i);
		if(n->getType()==_PIN && val< n->getDistFromDriving())
		{	val=n->getDistFromDriving();
		}
	}
	setLongestRadius(val);

	return val;
}


//trace from source to sinks and compute the right distance from driver
void OASpanningGraph::computeDistFromDriver()
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


	vector<SGNode*> vPropNode;
	vPropNode.push_back(this->getDrivingNode());


	while(!vPropNode.empty())
	{
		SGNode* propN = vPropNode[vPropNode.size()-1];
		vPropNode.pop_back();
		for(int j=0;j<propN->getNumEdge();j++)
		{	
			if(propN->getEdge(j)->checkRouted())
			{
				SGNode* toN = propN->getEdge(j)->getToNode();
				if(toN->getDistFromDriving()==_INFINITE)
				{	toN->setDistFromDriving(propN->getDistFromDriving() + propN->getEdge(j)->getLength());
					vPropNode.push_back(toN);
				}
			}
		}	
	}

}
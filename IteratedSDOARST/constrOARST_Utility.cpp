#include "OARST.h"

//route this edge physically
void OARSteinerTree::RouteEdge(SGEdge* E)
{	

	E->setRoutingState(true); //set edge's routing state
	E->setCost(0); //set edge's length as 0 because it's routed	
	//vRoutedSGEdge.push_back(E); //add routed spanning edge into container.
        //By doing this, after routing, we can caculatre the whole routing length

#ifndef DELAUNAY_TRIANGULATION
	for(int i=0;i<E->getNumIntersectedEdge();i++)
	{	SGEdge* disable_e = E->getIntersectedEdge(i);
		disable_e->setEnable(false);
	}
#endif
}

//route this edge physically
void OARSteinerTree::UnRouteEdge(SGEdge* E)
{	
	//restore routing information
	E->setRectState(false);
	E->setEnable(true);
	E->setRoutingState(false);
	E->getDualEdge()->setEnable(true);
	E->getDualEdge()->setRoutingState(false);
	//E->setCost(E->getLength());					//set edge's length as E->getLength() because it's un-routed
	E->setCost(ComputeLength(E->getFromNode(),E->getToNode()));					//set edge's length as E->getLength() because it's un-routed
}


#include "OARST.h"


void OARSteinerTree::ReConstrCriticalPath(SGNode* Source,SGNode* Target)
{
	if(Source->checkInCriticalPath()==true)
	{
		//trace from target to source and set passing node's share-ratio and set node in critical
		SGNode* passingNode = Target;

		while(passingNode!=Source)
		{
			passingNode->setInCriticalPath(true);
			passingNode->setDPF(Source->getDPF());
			passingNode = passingNode->getTempPrevNode();
		}
	}
}
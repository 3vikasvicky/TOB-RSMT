#include "OARST.h"

vector<int> vNodeResult;

bool nodeRTree_SearchCallback(int id, void* arg) 
{
  vNodeResult.push_back(id);
  return true; // keep going
}

//use R-Tree to get multi-source candidates
void OARSteinerTree::computeMultiSources(SGNode* Source,SGNode* Target,vector<SGNode*> &vMultiSource)
{
#ifdef DEBUG_ANALYSIS_RUN_TIME
	clock_t SG_gen_multi_sources_start = clock();
#endif


	if(Target->checkInTree()==false)
	{
		vNodeResult.clear();
		vMultiSource.clear();

		//now only consider one layer routing
		//Specify search region
		double diffX = abs(Source->getX()-Target->getX());
		double diffY = abs(Source->getY()-Target->getY());
		double diff = max(diffX,diffY); //avoid the special case such as source and target both with the same y-coordinate


#ifdef DEBUG_OARST_MULTI_SOURCE
		//cout<<"Search Region diffX = "<<diffX<<"\n";
		//cout<<"Search Region diffY = "<<diffY<<"\n";
		//cout<<"Search Region diff = "<<diff<<"\n";
#endif

		int X11 = min(Source->getX(),Target->getX()) - searchScale*diff;
		int Y11 = min(Source->getY(),Target->getY()) - searchScale*diff;
		int Z11 = min(Source->getLayer(),Target->getLayer());
		int X22 = max(Source->getX(),Target->getX()) + searchScale*diff;
		int Y22 = max(Source->getY(),Target->getY()) + searchScale*diff;
		int Z22 = max(Source->getLayer(),Target->getLayer());

		OASG->nodeRTreeSearch(new Point(X11,Y11,Z11),new Point(X22,Y22,Z11),nodeRTree_SearchCallback);

#ifdef DEBUG_OARST_MULTI_SOURCE
		cout<<"Number of Source Candidates = "<<vNodeResult.size()<<"\n";
#endif

		//[0] build multi-sources : add searching result that is made by R-Tree into MultiSource	
		for(size_t i=0 ; i<vNodeResult.size() ; i++)
		{
			SGNode* hitNode = OASG->getNode(vNodeResult[i]);

#ifdef DEBUG_OARST_MULTI_SOURCE
			cout<<"hitNode : InTree = "<<hitNode->checkInTree()<<"\n";
			PrintNode(hitNode);
#endif

			if(hitNode->checkInTree()==true)
			{	vMultiSource.push_back(hitNode);
			}
		}
		if(vMultiSource.size()==0)
		{		
			vMultiSource.push_back(Source);
		}
#ifdef DEBUG_OARST_MULTI_SOURCE
		cout<<"Search Region (left buttom) ("<<X11<<","<<Y11<<","<<Source->getLayer()<<")\n";
		cout<<"Search Region (right top)   ("<<X22<<","<<Y22<<","<<Source->getLayer()<<")\n";
		cout<<"Number of Source Candidates = "<<vMultiSource.size()<<"\n";
#endif
	}

#ifdef DEBUG_ANALYSIS_RUN_TIME
	clock_t SG_gen_multi_sources_end = clock();
	RunTime* RunTime = OASG->getDatabase()->getRunTimeSet();
	RunTime->SG_routing_gen_multi_sources = RunTime->SG_routing_gen_multi_sources + float(SG_gen_multi_sources_end - SG_gen_multi_sources_start)/CLK_TCK;
#endif
}
#include "OARST.h"
#include "prim.h"

//set up routing net order
//In order to achieve time-complxity VlogV
//operate Pins directly instead of generating a lot of edges
//see Update Process, you will understand what i say


#ifdef PRIORITY_BASED
void OARSteinerTree::doRefinedPrim(Database &data)
{

  cout<<"--[Refined idea : only trace connected SGNode]\n";
  size_t numNodes = OASG->getNumNode();
  cout<<"# of Pin = "<<data.getPinNum()<<" (include driver)"<<endl;
  cout<<"# of SGNode "<<numNodes<<endl;
  cout<<"* of SGEdges "<<OASG->getNumEdge()<<endl;

  //assume the node with index number equals firstPinNode is driving node of the tree
  //setDrivingNode(OASG->getNode(0));
  //cout<<"Driving SGNode : ";
  //PrintNode(DrivingNode);
  
  SGNode* expandNode = this->getDrivingNode(); //the tree starts growing from this node

  expandNode->setTreeState(true);       //temporary setting, we will restore this value
  expandNode->setDistFromDriver(0);
  expandNode->setTempDistFromDriver(0);
  expandNode->setDelayForPrim(0);
  expandNode->setTempDistFromTree( 0 ) ; // Homer: ver1.03
  expandNode->setPreSlackNode( expandNode ) ;
  expandNode->setSlackUsed() ;
  //expandNode->setPriority(_INFINITE);

  int NonTreePinNum = OASG->getDatabase()->getPinNum() -1 ;
  int TreePinNum = 1 ;
  double MinPriority=_INFINITE;

  vector<SGNode*> vReadyToPropNode;  //=====> use vector's sorting algo to choose the closet node (for propagating nodes)
  vector<SGNode*> vInTreePin;    //=====> contain all the in-tree Pins	
  set<SGNode*> pool;
  
  vInTreePin.push_back(expandNode);	

  while(NonTreePinNum!=0) //the prim's algo will continue to find 2-pin net until all the pins are in the tree
  {

#ifdef DEBUG_PRIM
    cout<<"# of NonTreePin = "<<NonTreePinNum<<endl;
    cout<<"# of TreePinNum = "<<TreePinNum<<endl;
#endif


    do  //start to find 'one' 2-pin net
    {

#ifdef DEBUG_PRIM
      cout<<"Now expanding node"<<endl;		
      PrintNode(expandNode);
      cout<<"# of vReadyToPropNode = "<<vReadyToPropNode.size()<<endl;
      cout<<"# of expandNode's branches = "<<expandNode->getNumEdge()<<endl;
#endif

      //Update Process : update node's distanceFromSource
      for(int indexEdge = 0; indexEdge < expandNode->getNumEdge() ; indexEdge++)
      {					
	SGNode* Target = expandNode->getEdge(indexEdge)->getToNode();   //refined idea : only trace connected SGNode
	//double DIST = ComputeLength( expandNode, Target ) ;
	double DIST = expandNode->getEdge(indexEdge)->getLength() ;
	//DIST = pow(DIST, 2) ;
	if( Target->getSlack()!=_INFINITE && Target->isSlackUsed()==false )
	{
	  double W = (Target->getSlack() - getMinSlack())/pow(getMaxSlack(),1);
	  //double W = (Target->getSlack() - getMinSlack())/Target->getSlack() ;
	  //DIST = DIST * pow(W,1.019) ;
	  //DIST=DIST*atan(W);
	  //if( W < 0.068 ) W = 0.068 ;
	  DIST = DIST * W  ;
	}
	else if( Target->getSlack() != _INFINITE && Target->isSlackUsed() )
	{
	  double W = (Target->getSlack() - getMinSlack())/pow(getMaxSlack(),1.1);
	  //double W = (Target->getSlack() - getMinSlack())/Target->getSlack() ;
	  //DIST = DIST * pow(W, 1.300);
	  //DIST=DIST*atan(W);
	  //if( W < 0.7 ) W = 0.7 ;
	  DIST = DIST * W ;
	}
	double distFromTree = expandNode->getTempDistFromTree() + DIST ;

	if(Target->checkInTree() == false && Target->getTempDistFromTree() > distFromTree )
	{	
	  Target->setTempDistFromTree( distFromTree ) ;
	  Target->setPreSlackNode( expandNode ) ;

	  if(expandNode->getType()==_PIN)
	  {
	    Target->setTempSource(expandNode);      //designate the source of this 2-pin net				
	  }
	  else
	  {	
	    Target->setTempSource(expandNode->getTempSource());      //designate the source of this 2-pin net
	  }

	  vReadyToPropNode.push_back(Target); //push expanding candidates into vector
	  pool.insert(Target);
#ifdef DEBUG_PRIM
	  cout<<"Can Length "<<Target->getTempDistFromTree()<<endl;
	  PrintNode(Target);
#endif
	}
      }

      assert(vReadyToPropNode.size()!=0);
#ifdef DEBUG_PRIM
      cout<<"after searching branches"<<endl;		
      cout<<"# of vReadyToPropNode = "<<vReadyToPropNode.size()<<endl;
#endif
      sort(vReadyToPropNode.begin(),vReadyToPropNode.end(),EstimatedTreeDist_GreaterThan());

      expandNode = vReadyToPropNode.back();
      vReadyToPropNode.pop_back();

      if(expandNode->getPriority()<MinPriority) MinPriority = expandNode->getPriority();
#ifdef DEBUG_PRIM
      cout<<"@ Net :"<<vTwoPinNet.size()<<"\n";
      cout<<"Prim Algorithm select expanding node, its length = "<<expandNode->getTempDistFromTree()<<endl;		
      PrintNode(expandNode);
      cout<<"================================================\n";
#endif

    }while(expandNode->getType()!=_PIN || expandNode->checkInTree()==true); //end finding one two-pin net

    expandNode->setTreeState(true);
    NonTreePinNum -- ;
    TreePinNum ++ ;
    vInTreePin.push_back(expandNode); //record finded pin
    
    //Adding two-pin net into vTwoPinNet
    TwoPinNet* net = new TwoPinNet(expandNode->getTempSource(),expandNode);
    vTwoPinNet.push_back(net);

    SGNode* T = expandNode->getTempSource() ;
    SGNode* S = expandNode ;
    while( S!=T )
    {
      if(S->getSlack()==_INFINITE) break;
      S->setSlackUsed() ;
      S = S->getPreSlackNode() ;
    }
    S->setSlackUsed() ;
    expandNode->setPreSlackNode( expandNode ) ;
    //getchar();

#ifdef DEBUG_PRIM
    cout<<"Net :"<<vTwoPinNet.size()<<" Source -> Target\n";
    PrintNode(expandNode->getTempSource());
    PrintNode(expandNode);
    cout<<"�סססססססססססססססססססססס�\n";
#endif
    /*
    for(set<SGNode*>::iterator it=pool.begin();it!=pool.end();++it)
      (*it)->setPriority((*it)->getPriority()-MinPriority);
      */
    for(int i=0; i < vInTreePin.size() ; i++)
      vInTreePin[i]->setTempDistFromTree( 0 ) ;
  }


  // Homer: shoule we reset something here?
  //restore temporary used value
  for(size_t i=0;i<OASG->getNumNode();i++)
  {	
    OASG->getNode(i)->setTreeState(false);				
    OASG->getNode(i)->setTempSource(NULL);
    OASG->getNode(i)->setTempDistFromSource(_INFINITE) ;
  }
  cout<<"# of Two-Pin-Net = "<<vTwoPinNet.size()<<endl;
}
#endif

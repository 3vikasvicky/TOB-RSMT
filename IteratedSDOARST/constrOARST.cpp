#include "OARST.h"
#include <algorithm>

class PRI_TEST_GreaterThan //Using function object instead of function pointer because of inlining effect
{	public:
		bool operator() (SGNode* pn1,SGNode* pn2)
		{	return pn1->getTempDistFromSource() > pn2->getTempDistFromSource();
		}
};

//I M P O R T A N T !
//this function just return the cost without modifying node's any member datas
double OARSteinerTree::tempPropagateCost(SGNode* Source,SGNode* Target,SGNode* ByNode)	//estimate needed cost if routing to target by this ToNode.
{	
	if(Source->getDistFromDriving() ==_INFINITE)
	{	cout<<"Error occured in OARSteinerTree::tempPropagateCost(..). DistFromDriving cannot be infinite.\n";
		exit(0);
	}
	
	double dds = Source->getDistFromDriving(); //distance from driver to source
	double dsn = ByNode->getTempDistFromSource(); //distance from source to node n
	double dnt = ComputeManhattanDist(ByNode,Target); //predicted distance from node n to target
#ifdef DEBUG_NEW_COST_FUNCTION
	double unit_c = getDelayModel()->get_Unit_C(); //unit wire capacitance
#endif


	//Original A* estimation formula : f(node) = g(node) + h(node). 
	//	g(node) stands for the 'exact' distance from source. 
	//	h(node) stands for the predicted distance from target
	//But we do some modifications to adapt to our new approach
	double gVal=0,hVal=0;	


	//(0) plus delay-aware value into gVal
	if(RedirectOK == true)
	{	
#ifdef DEBUG_NEW_COST_FUNCTION
		gVal = radiusFactor * dds * unit_c;
#else
		gVal = radiusFactor * dds;
#endif
	}
	else if(CriticalTrunkOK == true) //grow critical-trunk step turns on
	{
		if(Source->checkInCriticalPath()!=true) //no critical-trunks exist
		{	
#ifdef debug_new_cost_function
			gval = radiusfactor * dds * unit_c;
#else
			gVal = radiusFactor * dds;
#endif
		}	
		else //critical-trunks exist
		{	
		  /*
#ifdef DEBUG_NEW_COST_FUNCTION			
			gVal = ( dds * unit_c  + Source->getSID() * CTF ) * pow(Source->getDPF(),2) * (1-CTF); //apply SID
#else
#ifdef PRIORITY_BASED
			gVal = ( dds * pow(Source->getDPF(),2) ) ;  //no apply SID, 7/1/2008
#else
			gVal = ( dds * pow(Source->getDPF(),2) ) * (1-this->CTF);  //no apply SID, 7/1/2008
#endif
#endif
*/
			gVal = ( dds * pow(Source->getDPF(),2) ) ;  //no apply SID, 7/1/2008
		}
	}
	else
	{
		//firt initialize the g(n)	
#ifdef DEBUG_NEW_COST_FUNCTION			
		gVal = radiusFactor * dds * unit_c;
#else
		gVal = radiusFactor * dds;
#endif
	}

	//(1) compute g(n)
#ifdef DEBUG_NEW_COST_FUNCTION			
	gVal = gVal + dsn * unit_c;
#else
	gVal = gVal + dsn;
#endif

	//(2) compute h(n)
#ifdef DEBUG_NEW_COST_FUNCTION			
	hVal = dnt * unit_c;
#else
	hVal = dnt;
#endif

	return gVal + hVal;
}


void OARSteinerTree::routeTwoPinNet(int index,SGNode* Source,SGNode* Target,vector<SGNode*> &vMultiSource)
{
#ifdef DEBUG_OARST_ROUTING_INFO
	cout<<"\nNet:"<<index<<" Source (Dist from driving node="<<Source->getDistFromDriving()<<") -> Target\n";
	PrintNode(Source);
	PrintNode(Target);
#endif

	if(Target->checkInTree()==false)
	{

#ifdef DEBUG_OARST_ROUTING_INFO
		cout<<"\n--[Step1 : Do maze propagating]\n";
#endif

		//[1] do maze propagate : propagate from source to target
		vector<SGNode*> UsedNodeSet;	//it can help us to restore the nodes' temporary records
		mazePropagate(Source,vMultiSource,Target,UsedNodeSet);

#ifdef DEBUG_OARST_ROUTING_INFO
		cout<<"\n--[Step2 : Do back tracing]\n";
#endif

		//[2] do back trace : trace back from target to source
		SGNode* meetSource = backTracing(vMultiSource,Target);

#ifdef DEBUG_OARST_ROUTING_INFO
		cout<<"\n--[Step3 : trace from source to target, compute distance from driver]\n";
#endif
		//[3] trace from source to target and compute the right distance from driver
		recomputeDistFromDriver(meetSource,Target);

		if(CriticalTrunkOK == true)
		{
#ifdef DEBUG_OARST_ROUTING_INFO
			cout<<"\n--[Step4 : Recontruct Critical Path]\n";
#endif
			//[4] reconstruct critical path (record sub-tree grow from critical path)
			ReConstrCriticalPath(meetSource,Target);
		}

		//[5] restore non-routed nodes' information
#ifdef DEBUG_ANALYSIS_RUN_TIME
		clock_t SG_restore_variable_start = clock();
#endif

#ifdef DEBUG_OARST_ROUTING_INFO
		cout<<"\n";
#endif

		for(size_t i=0;i<UsedNodeSet.size();i++)
		{	
			SGNode* N=UsedNodeSet[i];

			//in order to make recomputeDistFromDriver(meetSource,Target) process correctly
			//node is not in tree, its distance from driver must be _INFINITE
			if(N->checkInTree()==false) 
			{	N->setDistFromDriving(_INFINITE);
			}

			N->setTempProCost(_INFINITE);
			N->setTempDistFromSource(_INFINITE);
			N->setTempSource(NULL);
			N->setTempPrevNode(NULL);
			N->setTempPrevEdge(NULL);
#ifdef DEBUG_OARST_RESTORE_USED_NODES		
			cout<<"Restore SGNode's ProCost and DistFromSource :";
			PrintNode(N);
#endif
		}
#ifdef DEBUG_ANALYSIS_RUN_TIME
		clock_t SG_restore_variable_end = clock();
		RunTime* RunTime = OASG->getDatabase()->getRunTimeSet();
		RunTime->SG_routing_restore_variable = RunTime->SG_routing_restore_variable + float(SG_restore_variable_end - SG_restore_variable_start)/CLK_TCK;
#endif
	}

#ifdef DEBUG_OARST_ROUTING_INFO
	cout<<"\nNet:"<<index<<" finish routing\n";
	cout<<"========================================\n";
#endif
}

//designate driving node of net
void OARSteinerTree::designateDrivingNode(Database &database)
{	
	for(int i=0;i<OASG->getNumNode();i++)
	{
		SGNode* N = OASG->getNode(i);
		if(N->getX()==database.getDriver()->getX() && N->getY()==database.getDriver()->getY()
			&& N->getLayer()==database.getDriver()->getLayer())
		{
			setDrivingNode(N);
			OASG->setDrivingNode(N);
#ifdef PRIORITY_BASED
			N->setTempDistFromDriver(0);
			N->setDistFromDriver(0);
#endif
			
			cout<<"Driving SGNode : ";
			PrintNode(getDrivingNode());
			return ;
		}
	}
}

//connect two node by Spanning Graph
void OARSteinerTree::ConnectTwoPin(SGNode* FromN,SGNode* ToN)
{
		SGNode* Source = FromN;
		SGNode* Target = ToN;

		vector<SGNode*> vMultiSource;
		computeMultiSources(FromN,ToN,vMultiSource);

		//---------------------------------
		//set node is leaf or not
		//for computing elmore delay usage when calculate downstream cap
		//---------------------------------
		Source->setIsLeaf(false);
		if(Target->checkInTree()==false)
		{	Target->setIsLeaf(true);	
		}
		routeTwoPinNet(_UNSET,Source,Target,vMultiSource);
}

//start routing
void OARSteinerTree::RouteSpanningGraph()
{
  //this->vRoutedSGEdge.clear();
  for(size_t i=0;i<vTwoPinNet.size();i++)
  {	
    TwoPinNet* net=getTwoPinNet(i);
    SGNode* Source = net->getSource();
    SGNode* Target = net->getTarget();

    vector<SGNode*> vMultiSource;
#ifndef PRIORITY_BASED
    computeMultiSources(Source,Target,vMultiSource);
#else
    /*
    if(CriticalTrunkOK&&Source->checkInCriticalPath() || Target->checkInCriticalPath())
    {
      cout<<"\nBCASE 1\n";
      computeMultiSources(Source,Target,vMultiSource);

      bool sourceExist(true);
      for(int i=0; i < vMultiSource.size(); i++)
      {
	if(vMultiSource[i]->getSlack() != _INFINITE && vMultiSource[i]->getPriority() < Source->getPriority())// && vMultiSource[i]->getDistFromDriving() < Target->getDistFromDriving())
	{
	  if(vMultiSource[i]==Source)
	    sourceExist = false ;
	  vMultiSource.erase( vMultiSource.begin()+i ) ;
	  i--;
	}
      }
      if(vMultiSource.empty())
      {
	vMultiSource.push_back( Source ) ;
	sourceExist = true ;
      }
      if(sourceExist==false)// && Source->getSlack()!=_INFINITE)
      {
	vMultiSource.insert( vMultiSource.begin(), Source ) ;
      }
    }
    else 
    {
      vMultiSource.push_back(Source);
    }
    */
    if(CriticalTrunkOK==false)
      computeMultiSources(Source,Target,vMultiSource);
    else
      vMultiSource.push_back(Source);
#endif

    //---------------------------------
    //set node is leaf or not
    //for computing elmore delay usage when calculate downstream cap
    //---------------------------------
    Source->setIsLeaf(false);
    bool Compute_Target_SID = false;
    if(Target->checkInTree()==false)
    {
      Target->setIsLeaf(true);	
      Compute_Target_SID = true; //when target is not in tree, we compute target's SID
    }

    routeTwoPinNet(i,Source,Target,vMultiSource);
    
    //---------------------------------
    //Sink impact delay (SID) only affect sub-tree growing
    //As for critical trunk, SID won't do anything.
    //---------------------------------
    if(CriticalTrunkOK==true && Compute_Target_SID)
    {	
      //Compute SID :Sink Impact Delay for target
      double sink_loading = Target->getLoading(); // Homer: original: sink_loading = getDelayModel()->get_Loading();
      Target->setSID( sink_loading * Target->getDistFromDriving() + Source->getSID());
      //        SID = sink loading x path-length from driver to this sink
    }
  }
}

#ifdef PRIORITY_BASED
// Homer: compute the slack of each node in OASG
void OARSteinerTree::computeSlackForEachNode(Database &database)
{
  // Step 1.
  // construct the shortest path from the source to each node
  queue<SGNode*> PRO ;
  SGNode* SOURCE = OASG->getDrivingNode() ;

  for(int i=0;i<OASG->getNumNode();i++)
  {
    OASG->getNode(i)->setTempDistFromSource(_INFINITE);
    OASG->getNode(i)->setAbsDistFromDriving(_INFINITE);
  }
  OASG->getDrivingNode()->setTempDistFromSource( 0 ) ;
  OASG->getDrivingNode()->setAbsDistFromDriving( 0 ) ;
  OASG->getDrivingNode()->setTempPrevNode( NULL ) ;
  PRO.push( SOURCE ) ;

  while( !PRO.empty() )
  {
    //sort(PRO.begin(),PRO.end(),PRI_TEST_GreaterThan());
    SGNode* S = PRO.front() ;
    PRO.pop() ;
    S->InInitTree();
    
    for(int i=0; i < S->getNumEdge() ; i++ )
    {
      SGEdge* E = S->getEdge( i ) ;
      if( E->isUsed() )
	continue;
      SGNode* T = E->getToNode() ;
      //double DIST = S->getTempDistFromSource() + (pow(ComputeLength( S, T ),2.069)*DelayModel->get_Unit_R()*DelayModel->get_Unit_C()/1000);//2.069, 2.33, 
      double DIST_2 = ComputeLength(S,T);
      double DIST = S->getTempDistFromSource() + pow(DIST_2,2.069);//2.069, 2.33, 
      //double DIST = S->getTempDistFromSource() + ComputeManhattanDist(S,T);
      if( T->getTempDistFromSource() > DIST )
      {
	T->setTempDistFromSource( DIST ) ;
	T->setTempPrevEdge( E ) ;
	T->setTempPrevNode( S ) ;
#ifdef BUF_DRIVEN
	T->setAbsDistFromDriving( S->getAbsDistFromDriving()+DIST_2);
	E->setInInitTree();
	E->getDualEdge()->setInInitTree();
#endif
	PRO.push( T ) ;
      }
      E->setUsed() ;
      E->getDualEdge()->setUsed() ;
    }
  }

#ifdef BUF_DRIVEN
  // GOAL: According to CapUpperBound to update the potential delays
  // 1. Compute potential Downstream Capacitance (DSC) in the constructed tree 
  // 2. Update the potential delay to reflect pseudo buffer insertion according to CapUpperBound
  
  
  // STEP 1
  Compute_DSC();
  // STEP 2
  Pseudo_BI();
#endif

  // Step 2.
  // decrease slack in the constructed path
  vector<int> BACKUPSLACK;
  for(int i=0;i<OASG->getNumNode();i++)
  {
    BACKUPSLACK.push_back( OASG->getNode(i)->getSlack() ) ;
  }

  vector<SGEdge*> USED;
  SOURCE->setTreeState( true ) ;
  for(int i=0; i < OASG->getNumNode() ; i++)
  {
    SGNode* C = OASG->getNode( i ) ;
    if( C->getType() != _PIN || C == SOURCE )
      continue;
    //double BDIST = C->getTempDistFromSource() ;
    //SGNode* N = C->getTempPrevNode() ;
    SGNode* N = C->getTempPrevNode() ;
    double TL(0);
    SGNode* PP = C;
    while( N != NULL || N!=OASG->getDrivingNode())
    {
      //TL += ComputeManhattanDist(PP,N);
      if(N->isBuffered())
	TL=0;
      TL += ComputeLength(PP,N);
      //TL = ComputeLength(PP,N);
      double SLACK = C->getSlack() - pow( TL ,2) * DelayModel->getHalfRC() ;
      //double SLACK = PP->getSlack() - pow( TL ,2) * DelayModel->getHalfRC() ;
      //double SLACK = C->getSlack() - labs( (BDIST - N->getTempDistFromSource()) ) * DelayModel->getHalfRC() ;
      assert(SLACK>=0);
      /*
      if(N->getType()==_PIN){
	C=N;
	TL=0;
      }
      */
      if(getMinSlack() > SLACK) setMinSlack( SLACK ) ;
      if(getMaxSlack() < SLACK) setMaxSlack( SLACK ) ;
      if( N->getSlack() > SLACK )
      {
	N->setSlack( SLACK ) ;
	//C = N ;
	N = N->getTempPrevNode() ;
	PP = PP->getTempPrevNode() ;
	//PRO.push( N ) ;
	//USED.push_back( C->getTempPrevEdge() ) ;
      }
      else
	break ;
    }
  }

  // reset the used data
  for(int i=0; i < OASG->getNumNode() ; i++)
  {
    /*
    if( OASG->getNode(i)!=SOURCE&&OASG->getNode(i)->getType() == _PIN )
    {
      OASG->getNode(i)->setSlack( BACKUPSLACK[i] ) ;
    }
    */
    OASG->getNode(i)->setTempPrevNode( NULL ) ;
    OASG->getNode(i)->setTempPrevEdge( NULL ) ;
    OASG->getNode(i)->setTempDistFromSource( _INFINITE ) ;
  }
}

void OARSteinerTree::Assign_Reference_Priority(Database &database)
{
  assert( false ) ;
}
#endif

#ifdef BUF_DRIVEN
void OARSteinerTree::Compute_DSC(){
  for(int i=0;i<OASG->getNumNode();i++){	
				    
    SGNode* N = OASG->getNode(i);		

    if(N->getType()==_PIN && N->isInitTree()==true){
      if(N->getType()==_PIN && N!=OASG->getDrivingNode()){
	N->setDSC(N->getDSC()+N->getLoading());
      }

      double cap = 0;
      // if(N->checkIsLeaf()) {//case 1 : leaf node
      // 	cap = N->getDowntreamCap();
      // }
      // else if(N->getType()==_PIN && N!=OASG->getDrivingNode()) {//case 2 : internal node 
      // 	//cap = this->Cloading; //avoid recompute cap of this internal node
      //   cap = N->getLoading(); //avoid recompute cap of this internal node
      // }
      if(N->getType()==_PIN && N!=OASG->getDrivingNode()) {
         cap = N->getLoading(); //avoid recompute cap of this internal node
      }

      SGEdge* selectEdge = NULL;
      bool traceOneEdge = false;

      while(N != OASG->getDrivingNode()){
	//select next internode
	SGNode* nextN = N->getTempPrevNode();
	selectEdge = N->getTempPrevEdge();
	

	if(nextN==NULL){	
	  assert(false);
	  cout<<"Error occured in elmoreDelay::Compute_OASG_DownstreamCap()\n";
	  exit(0);
	}

	if(!traceOneEdge){	
	  traceOneEdge = true;
	  cap = cap + selectEdge->getLength()*this->getDelayModel()->get_Unit_C();
	}

	nextN->setDSC(nextN->getDSC() + cap);

	N = nextN;
      }
    }
  }
}

void OARSteinerTree::Pseudo_BI(){
  for(int i=0;i<this->OASG->getNumNode();i++)
  {
    SGNode* N=OASG->getNode(i);
    N->setDFSFlag(false); // false: unvisited; true: visited
    N->setTmpDistFromLastBuffer(N->getAbsDistFromDriving());
    N->setBufferedDSC(N->getDSC());
    N->setReducedDSC(0.0);
  }

  stack<SGNode*> STACK;
  this->getDrivingNode()->setPreDistFromSource(0.0);
  this->getDrivingNode()->setLastBufferAbsDist(0.0);
  STACK.push(this->getDrivingNode());
  const double CUB(this->getDelayModel()->getCapUpperBound());

  while(!STACK.empty()){
    SGNode* N = STACK.top();
    STACK.pop();
    if(N->getDFSFlag()==false&&N->getTempDistFromSource()!=_INFINITE){
      // 1. compute buffered DSC
      //double DECREASE(N->getReducedDSC());
      double DECREASE(0.0);
      double ABSDIST(0.0);
      N->setDFSFlag(true);
      if(N->getBufferedDSC()>CUB){ // buffered
	//printf("BUFFERED!!!!!!!!!(%lf; %lf; %lf)\n",N->getLastBufferAbsDist(),N->getPreDistFromSource(),N->getTempDistFromSource());
	printf("BUFFERED!!!!!!!!!(%lf; %lf)\n",N->getLastBufferAbsDist(),N->getAbsDistFromDriving());
	//DECREASE += floor(N->getBufferedDSC()/CUB)*CUB;
	DECREASE = floor(N->getDSC()/CUB)*CUB;
	//printf("N->getBufferedDSC()=%lf, CUB=%lf, DECREASE=%lf\n",N->getBufferedDSC(),CUB,DECREASE);
	//ABSDIST=N->getPreDistFromSource();
	ABSDIST=N->getAbsDistFromDriving();
	N->insertBuffer();
      }else{ // unbuffered
	ABSDIST=N->getLastBufferAbsDist();
      }
      //printf("Original DSC = %lf\t", N->getDSC());
      double currentDSC(N->getDSC()-DECREASE);
      //printf("BufferedDSC=%lf, DECREASE=%lf\t",N->getBufferedDSC(),DECREASE);
      //printf("currentDSC=%lf\n",currentDSC);
      assert(currentDSC>=0);
      N->setBufferedDSC(currentDSC);
      N->setLastBufferAbsDist(ABSDIST);
      N->setTmpDistFromLastBuffer(N->getAbsDistFromDriving()-N->getLastBufferAbsDist());
      //printf("(%lf; %lf; %lf)\n",N->getLastBufferAbsDist(),N->getPreDistFromSource(),N->getAbsDistFromDriving());
      //printf("tempDistFromSource()=%lf, lastBufferAbsDist=%lf\n", N->getTempDistFromSource(), N->getLastBufferAbsDist());
      assert(N->getTmpDistFromLastBuffer()>=0);
      //printf("TmpDistFromLastBuffer=%lf\n",N->getTmpDistFromLastBuffer());
      // 2. compute distFromLastBuffer
      for(int j=0;j<N->getNumEdge();j++){
	SGNode* NN = N->getEdge(j)->getToNode();
	if(NN->getDFSFlag()==false&&N->getTempDistFromSource()<NN->getTempDistFromSource()&&NN->getTempPrevNode()==N){
	  //NN->setReducedDSC(DECREASE);
	  //NN->setPreDistFromSource(N->getTempDistFromSource());
	  NN->setPreDistFromSource(N->getAbsDistFromDriving());
	  NN->setLastBufferAbsDist(N->getLastBufferAbsDist());
	  printf("NN:(%lf; %lf)\n",NN->getLastBufferAbsDist(),NN->getAbsDistFromDriving());
	  printf("N(%lf; %lf)\n",N->getLastBufferAbsDist(),N->getAbsDistFromDriving());
	  STACK.push(NN);
	}
      }
    }
  }
}

#endif

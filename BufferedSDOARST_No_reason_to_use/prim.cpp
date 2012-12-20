#include "OARST.h"
#include "prim.h"

vector<int> vObstacleResult;
const int LEFT_BUTTOM=1;
const int LEFT_TOP=2;
const int RIGHT_TOP=3;
const int RIGHT_BUTTOM=4;

bool obstacleRTree_SearchCallback(int id, void* arg) 
{
  vObstacleResult.push_back(id);
  return true; // keep going
}

//check horizontal line cuts block ?
bool checkHLine_CutBlock(int py,int px1,int px2,Obstacle* O)//int ox1,int oy1,int ox2,int oy2)
{
	int ox1=O->getX1();
	int ox2=O->getX2();
	int oy1=O->getY1();
	int oy2=O->getY2();

	if(oy1<py && py<oy2)
	{
		if((px1<ox1 && ox1<px2) || (px1<ox2 && ox2<px2))
		{	return true;
		}
	}
	return false;
}
//check vertical line cuts block ?
bool checkVLine_CutBlock(int px,int py1,int py2,Obstacle* O)//,int ox1,int oy1,int ox2,int oy2)
{

	int ox1=O->getX1();
	int ox2=O->getX2();
	int oy1=O->getY1();
	int oy2=O->getY2();

	if(ox1<px && px<ox2)
	{
		if((py1<oy1 && oy1<py2) || (py1<oy2 && oy2<py2))
		{	return true;
		}
	}
	return false;
}

int specifyTheGeoRelation(SGNode* Source,SGNode* Target)
{
	int condition=0;
	//specify the relation between source and target
	//condition=LEFT_BUTTOM  => source is located at the left-buttom side of target
	//condition=LEFT_TOP     => source is located at the left-top side of target
	//condition=RIGHT_TOP    => source is located at the right-top side of target
	//condition=RIGHT_BUTTOM => source is located at the right-buttom side of target
	Point* S=Source->getPoint();
	Point* T=Target->getPoint();
	if(S->getX() <= T->getX() && S->getY() <= T->getY())
	{	condition=LEFT_BUTTOM;
	}
	else if(S->getX() <= T->getX() && S->getY() >= T->getY())
	{	condition=LEFT_TOP;
	}
	else if(S->getX() >= T->getX() && S->getY() >= T->getY())
	{	condition=RIGHT_TOP;
	}
	else if(S->getX() >= T->getX() && S->getY() <= T->getY())
	{	condition=RIGHT_BUTTOM;
	}

	return condition;
}

void ConsiderObstacle(Database &data,SGNode* Source,SGNode* Target,double &Len)
{
#ifdef DEBUG_ANALYSIS_RUN_TIME
	clock_t prim_consider_obstacle_start = clock();
#endif

	//query R-Tree to fine-tune the length
	vObstacleResult.clear();
	int penaltyScale=10;

	int x1=min(Source->getX(),Target->getX());
	int x2=max(Source->getX(),Target->getX());
	int y1=min(Source->getY(),Target->getY());
	int y2=max(Source->getY(),Target->getY());

	int X11 = min(Source->getX(),Target->getX());
	int Y11 = min(Source->getY(),Target->getY());
	int Z11 = min(Source->getLayer(),Target->getLayer());
	int X22 = max(Source->getX(),Target->getX());
	int Y22 = max(Source->getY(),Target->getY());
	int Z22 = max(Source->getLayer(),Target->getLayer());

	int hit=data.obstacleRTreeSearch(new Point(X11,Y11,Z11),new Point(X22,Y22,Z11),obstacleRTree_SearchCallback);

	
	//check the geo relation between Source and Target
	int geoCondition=specifyTheGeoRelation(Source,Target);

	bool route_way_1=true; //the net starts with two routing paths
	bool route_way_2=true;

	//check the mahattan distance between source and target is blocked by obstacle ?
	bool T_BLOCKED=false; //top side
	bool B_BLOCKED=false; //buttom side
	bool R_BLOCKED=false; //right side
	bool L_BLOCKED=false; //left side

	for(size_t i=0 ; i<hit ; i++)
	{
		Obstacle* O = data.GetObstacle(vObstacleResult[i]);
				
#ifdef DEBUG_PRIM_CONSIDER_OBSTACLE
		cout<<"index :"<<vObstacleResult[i]<<", ";
		PrintObstacle(data,vObstacleResult[i]);
#endif

		if(T_BLOCKED==false)
		{	
			if(checkHLine_CutBlock(y2,x1,x2,O) )
			{	T_BLOCKED=true;
			}						
		}
		if(B_BLOCKED==false)
		{
			if(checkHLine_CutBlock(y1,x1,x2,O) )
			{	B_BLOCKED=true;
			}						
		}
		if(L_BLOCKED==false)
		{
			if(checkVLine_CutBlock(x1,y1,y2,O) )
			{	L_BLOCKED=true;
			}						
		}
		if(R_BLOCKED==false)
		{
			if(checkVLine_CutBlock(x2,y1,y2,O) )
			{	R_BLOCKED=true;
			}						
		}

		switch(geoCondition)
		{	case LEFT_BUTTOM:
				if(B_BLOCKED || R_BLOCKED)
					route_way_1=false;
				if(T_BLOCKED || L_BLOCKED)
					route_way_2=false;
				break;
			case LEFT_TOP:
				if(T_BLOCKED || R_BLOCKED)
					route_way_1=false;
				if(B_BLOCKED || L_BLOCKED)
					route_way_2=false;
				break;
			case RIGHT_TOP:
				if(T_BLOCKED || L_BLOCKED)
					route_way_1=false;
				if(B_BLOCKED || R_BLOCKED)
					route_way_2=false;
				break;
			case RIGHT_BUTTOM:
				if(T_BLOCKED || R_BLOCKED)
					route_way_1=false;
				if(B_BLOCKED || L_BLOCKED)
					route_way_2=false;
				break;
		}

		if(route_way_1==false && route_way_2==false)
		{
			//this mahattan distance will be blocked, so give it some penalty
			Len=Len*penaltyScale;
#ifdef DEBUG_PRIM_CONSIDER_OBSTACLE
			cout<<"\nThis Net: Source -> Target\n";
			PrintNode(Source);
			PrintNode(Target);
			cout<<"Is Blocked!\n";
#endif
			break;
		}
	}

#ifdef DEBUG_PRIM_CONSIDER_OBSTACLE
	//cout<<"B_BLOCKED = "<<B_BLOCKED<<"\n";
	//cout<<"T_BLOCKED = "<<T_BLOCKED<<"\n";
	//cout<<"R_BLOCKED = "<<R_BLOCKED<<"\n";
	//cout<<"L_BLOCKED = "<<L_BLOCKED<<"\n";

	//cout<<"route_way_1 = "<<route_way_1<<"\n";
	//cout<<"route_way_2 = "<<route_way_2<<"\n";
#endif

#ifdef DEBUG_ANALYSIS_RUN_TIME
	clock_t prim_consider_obstacle_end = clock();
	RunTime* RunTime = data.getRunTimeSet();
	RunTime->prim_consider_obstacle = RunTime->prim_consider_obstacle + float(prim_consider_obstacle_end - prim_consider_obstacle_start)/CLK_TCK;
#endif

}



//==============================================================================

//set up routing net order
//In order to achieve time-complxity VlogV
//operate Pins directly instead of generating a lot of edges
//see [3] Update Process, you will understand what i say
void OARSteinerTree::doPrim(Database &data)
{		

	cout<<"--[mainly consider pins, don't care obstacles]\n";
	size_t numNodes = OASG->getNumNode();
	cout<<"# of Pin = "<<data.getPinNum()<<" (include driver)"<<endl;
	cout<<"# of SGNode "<<numNodes<<endl;
	
	vector<prim_node*> vPrimNode; //all nodes copy
	vector<prim_node*> vTreeNode;  //node in the present tree
	vector<prim_node*> vNonTreeNode; //node not in the present tree


	//[1] initialize vPrimNode
	//	  and give each node that type equals _PIN an index number
	prim_node* expandNode = NULL;
	for(size_t i=0;i<numNodes;i++)
	{	
		if(OASG->getNode(i)->getType()==_PIN)
		{
			OASG->getNode(i)->setIndex(i);
			prim_node* pn = new prim_node(OASG->getNode(i));
			pn->setIndex(i);
			vPrimNode.push_back(pn);

			if(expandNode == NULL &&
				pn->getNode()->getX() == getDrivingNode()->getX() && pn->getNode()->getY() == getDrivingNode()->getY())
			{	expandNode = pn;
			}	
		}
	}

	//the tree starts growing from this node
	vTreeNode.push_back(expandNode);
	expandNode->setTreeState(true);
	
	for(size_t i=0;i<vPrimNode.size();i++)
	{
		if(vPrimNode[i] != expandNode)
		{	vNonTreeNode.push_back(vPrimNode[i]);
		}
	}
	cout<<"init: # of vNonTreeNode = "<<vNonTreeNode.size()<<endl;
	cout<<"init: # of vTreeNode = "<<vTreeNode.size()<<endl;


	//[2] In order to use R-Tree to fine-tune length of spanning edge
	//    we need to initialize obstacleRTree
/*	for(size_t i=0;i<data.GetObstacleSize();i++)
	{	
		Obstacle* O = data.GetObstacle(i);		
		data.RTreeInsertObstacle(O);	 //Adding obstacle into Region Tree
#ifdef DEBUG_PRIM_CONSIDER_OBSTACLE
		cout<<"Index :"<<i<<", ";
		PrintObstacle(data,i);
#endif
		
	}
*/

	while(!vNonTreeNode.empty())
	{

		//[3] Update Process : update prim-node's distanceFromTree
		for(vector<prim_node*>::iterator To=vNonTreeNode.begin(); To!=vNonTreeNode.end(); To++)
		{
			SGNode* Source = expandNode->getNode();
			SGNode* Target = (*To)->getNode();

			double diffX = Source->getX() - Target->getX() ;
			double diffY = Source->getY() - Target->getY() ;

			//(3-1) compute length between source and target
			//    remember that don't use manhattan distance that will change the topology very dramatically
			double Len= sqrt(diffX*diffX + diffY*diffY);

			//(3-2) query R-Tree to fine-tune the length
			//ConsiderObstacle(data,Source,Target,Len);
			
#ifdef PRIORITY_BASED
			double Delay(Len * DelayModel->get_Unit_R() * DelayModel->get_Unit_C());//??
			double Priority(Target->getSlack()-Target->getDistFromDriver()-Delay);
			if(Priority>Target->getPriority()){
			        Target->setDistFromDriver(Source->getDistFromDriver()+Len);
				Target->setPriority(Priority);
				(*To)->setDistFromDriver(Source->getDistFromDriver()+Len,expandNode);
			}
#else
			if(Len < (*To)->getDistFromTree())
			{	(*To)->setDistFromTree(Len,expandNode);   //update
			}
#endif

#ifdef DEBUG_PRIM
			cout<<"Can Length "<<(*To)->getDistFromTree()<<endl;
#endif
		}


#ifdef DEBUG_PRIM
		cout<<"before sorting\n";
#endif
#ifdef PRIORITY_BASED
		sort(vNonTreeNode.begin(),vNonTreeNode.end(),Priority_GreaterThan());
#else
		sort(vNonTreeNode.begin(),vNonTreeNode.end(),EstimatedDist_GreaterThan());
#endif
#ifdef DEBUG_PRIM
		cout<<"after sorting\n";
		cout<<"(1) NonTreeNodes have been updated\n";
#endif

		//[4] min NonTreeNode is selected from vector
		expandNode = vNonTreeNode[vNonTreeNode.size()-1];					
		if(! (expandNode->checkInTree() == false && expandNode->getFromPrimNode()->checkInTree() == true) )
		{	cout<<"Error: occured in¡@OARSteinerTree::doPrim(). expandNode is wrong !\n";
			exit(0);
		}		
		
#ifdef DEBUG_PRIM
		cout<<"Prim Algorithm select : "<<expandNode->getDistFromTree()<<endl;		
		cout<<"(2) min NonTreeNode is selected from vector\n";
#endif


		expandNode->setTreeState(true);
		vTreeNode.push_back(expandNode);
		vNonTreeNode.pop_back(); //remove this vNonTreeNode because it's on the tree now


		//[5] Adding two-pin net into vTwoPinNet
		TwoPinNet* net = new TwoPinNet(expandNode->getFromPrimNode()->getNode(),expandNode->getNode());
		vTwoPinNet.push_back(net);

#ifdef DEBUG_PRIM
		cout<<"Net :"<<vTwoPinNet.size()<<" Source -> Target\n";
		PrintNode(expandNode->getFromPrimNode()->getNode());
		PrintNode(expandNode->getNode());
		cout<<"vNonTreeNode size = "<<vNonTreeNode.size()<<endl;
#endif

	}

	cout<<"# of Two-Pin-Net = "<<vTwoPinNet.size()<<endl;
}

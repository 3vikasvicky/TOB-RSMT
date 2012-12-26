#include "OARST.h"
#include "elmoreDelay.h"


//constructor of OASG : it will build OASG layer by layer
OASpanningGraph::OASpanningGraph(Database &data,elmoreDelay *delay) : numRTreeNodes(0), longestRadius(0), WorstSlackSink(NULL) //, numRTreeObstacles(0)
{
	this->data=&data;
#ifdef PRIORITY_BASED
	this->delay=delay;
#endif

#ifndef DELAUNAY_TRIANGULATION
	//the following is another spanning graph construction!!!!!!!!!!!!!!!!!!!!!!!!!!!!! (zhou and nctu chang)
	//build from buttom layer to top layer
	for(int numLayer=data.getButtomLayerNum();numLayer<=data.getTopLayerNum();numLayer++)
	{
		buildOASG(data,numLayer);
		vSweepEntry.clear();
	}
#endif
}

//constructor of OASG initializes some member data(vR1_ProcessEntry,vR2_ProcessEntry,....,,vSGNode) ,and sorts the data we have added.
//parameter numLayer specify the number of layers to build
void OASpanningGraph::buildOASG(Database &data,int numLayer)
{	

	cout<<"\n--[Create Pins' SGNodes in OASG]";
	cout<<"\n--[Add Pins' SGNodes in R-Tree]\n";
	//Adding Pin
	for(size_t i=0;i<data.GetPinSize();i++)
	{	
		Pin* Pref = data.GetPin(i);
		Pin P = *Pref;

		//only create one layer spanning graph
		if(P.getLayer()==numLayer)
		{
			//Initialize SGNode Entry
			SGNode *N=new SGNode(P.getX(),P.getY(),P.getLayer(),_PIN,P.getRequiredTime(),P.getLoading());
#ifdef PRIORITY_BASED
			N->setSlack(P.getRequiredTime()-delay->get_Driver_Arrival_Time());
#endif
			N->setDetailType(_PIN);
			//Adding SGNode¡@Entry
			vSGNode.push_back(N);
			//Adding SGNode into Region Tree
			RTreeInsertNode(N);
			//Adding SweepLine Entry
			SweepEntry* S = new SweepEntry(N,_PIN);
			vSweepEntry.push_back(S);
		}
	}

	cout<<"--# of SGNodes in R-Tree = "<<this->numRTreeNodes<<endl;

	cout<<"\n--[Create Obstacles' SGNodes in OASG]";
	cout<<"\n--[Add Obstacles' SGNodes in R-Tree]\n";

	//Adding Obstacle
	for(size_t i=0;i<data.GetObstacleSize();i++)
	{	
		Obstacle* O = data.GetObstacle(i);
				
		//only create one layer spanning graph
		if(O->getLayer()==numLayer)
		{

			//X1 Y1
			//Initialize SGNode Entry
			SGNode *N11=new SGNode(O->getX1(),O->getY1(),O->getLayer(),_OBSTACLE_LEFT);
			N11->setDetailType(_OBSTACLE_LEFT_LOWER);
			//Adding SGNode¡@Entry
			vSGNode.push_back(N11);
			//Adding SGNode into Region Tree
			RTreeInsertNode(N11);
			//Adding SweepLine Entry
			SweepEntry* S11 = new SweepEntry(N11,_OBSTACLE_LEFT_LOWER);
			vSweepEntry.push_back(S11);

			//X1 Y2
			//Initialize SGNode Entry
			SGNode *N12=new SGNode(O->getX1(),O->getY2(),O->getLayer(),_OBSTACLE_LEFT);
			N12->setDetailType(_OBSTACLE_LEFT_UPPER);
			//Adding SGNode¡@Entry
			vSGNode.push_back(N12);
			//Adding SGNode into Region Tree
			RTreeInsertNode(N12);
			//Adding SweepLine Entry
			SweepEntry* S12 = new SweepEntry(N12,_OBSTACLE_LEFT_UPPER);
			vSweepEntry.push_back(S12);

			//X2 Y1
			//Initialize SGNode Entry
			SGNode *N21=new SGNode(O->getX2(),O->getY1(),O->getLayer(),_OBSTACLE_RIGHT);
			N21->setDetailType(_OBSTACLE_RIGHT_LOWER);
			//Adding SGNode¡@Entry
			vSGNode.push_back(N21);
			//Adding SGNode into Region Tree
			RTreeInsertNode(N21);
			//Adding SweepLine Entry
			SweepEntry* S21 = new SweepEntry(N21,_OBSTACLE_RIGHT_LOWER);
			vSweepEntry.push_back(S21);

			//X2 Y2
			//Initialize SGNode Entry
			SGNode *N22=new SGNode(O->getX2(),O->getY2(),O->getLayer(),_OBSTACLE_RIGHT);
			N22->setDetailType(_OBSTACLE_RIGHT_UPPER);
			//Adding SGNode¡@Entry
			vSGNode.push_back(N22);
			//Adding SGNode into Region Tree
			RTreeInsertNode(N22);
			//Adding SweepLine Entry
			SweepEntry* S22 = new SweepEntry(N22,_OBSTACLE_RIGHT_UPPER);
			vSweepEntry.push_back(S22);

			//----------------------------------------------------------------
			S11->set_Left_Lower_N(S11);
			S11->set_Left_Upper_N(S12);
			S11->set_Right_Lower_N(S21);
			S11->set_Right_Upper_N(S22);
			S11->setID(i);

			S12->set_Left_Upper_N(S12);
			S12->set_Left_Lower_N(S11);
			S12->set_Right_Upper_N(S22);
			S12->set_Right_Lower_N(S21);
			S12->setID(i);

			S21->set_Right_Lower_N(S21);
			S21->set_Left_Lower_N(S11);
			S21->set_Right_Upper_N(S22);
			S21->set_Left_Upper_N(S12);
			S21->setID(i);

			S22->set_Right_Upper_N(S22);
			S22->set_Left_Upper_N(S12);
			S22->set_Right_Lower_N(S21);
			S22->set_Left_Lower_N(S11);
			S22->setID(i);

		}
	}

	cout<<"--# of SGNodes in R-Tree = "<<this->numRTreeNodes<<endl;
	cout<<"\n--[Start OASG Construction]";
	cout<<"\n# of Sweep Entry = "<<vSweepEntry.size()<<endl;	

	Quad1();//perform edge connection for Quad1
	Quad2();//perform edge connection for Quad2
	Quad3();//perform edge connection for Quad3
	Quad4();//perform edge connection for Quad4
	

	//cout<<"\n--[Check Intersections Of Spanning Edge]\n";
	//check intersection
	//CheckEdgeIntersection();

	cout<<"--[End OASG Construction]\n";

	cout<<"# of SGEdge = "<<getNumEdge()<<endl;
	cout<<"# of SGEdge = "<<getNumEdge()/2<<" (no dual edges)"<<endl;
	cout<<"# of SGNode = "<<getNumNode()<<endl;
}






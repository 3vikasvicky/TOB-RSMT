#include "OARST.h"
#include "temp.h"
#include "intersect.h"

//main use : add boundary of obstacle
SGEdge* OASpanningGraph::AddingEdge(SGNode* FromNode,SGNode* ToNode)
{
	bool AddingOK=true;

	if(FromNode->getX()==ToNode->getX() && FromNode->getY()==ToNode->getY())
	{	return NULL;
	}

	//check 'existing edges' to avoid adding duplicated edge
	for(size_t j=0;j<FromNode->getNumEdge(); j++)
	{	
		SGNode* ToNode2=FromNode->getEdge(j)->getToNode();
							
		if(ToNode2->getX()==ToNode->getX() && ToNode2->getY()==ToNode->getY())
		{	
			AddingOK=false;	//because edge already exists
#ifdef DEBUG_OASG_ADDING_EDGE_CHECK
			cout<<"~ ToNode can be replaced with ToNode2 (ToNode ... ToNode2)\n";
			PrintNode(ToNode);
			PrintNode(ToNode2);
#endif
			return FromNode->getEdge(j); //return this existed edge
		}
	}

	if(AddingOK)
	{
		SGEdge* E=new SGEdge(FromNode,ToNode);
		vSGEdge.push_back(E);
		FromNode->AddEdge(E);	//actually adding edge
#ifdef DEBUG_OASG_ADDING_EDGE
		cout<<"@ Adding SGEdge from -> to \n";
		PrintNode(FromNode);
		PrintNode(ToNode);
		cout<<"Check dual\n";
#endif	
		//edge must be dual.
		if(E->getDualEdge()==NULL)
		{	SGEdge* dualEdge=AddingEdge(ToNode,FromNode);
			E->setDualEdge(dualEdge);
			dualEdge->setDualEdge(E);
		}

		return E;
	}

	return NULL;
}

//Adding edge. Besides, analyze the candidates in set A to avoid adding redundant edge 
//definition of redundant edge : it can ne replaced with other edge.
void OASpanningGraph::AddingEdge(ProcessEntry* PE)
{	
	
	SGNode* FromNode=PE->getParent();

#ifdef DEBUG_OASG_ADDING_EDGE
	cout<<"Check ";
	PrintPE(PE);
#endif

	//!!!!!!!!!!!!!!!!!!!!!!!modify this part to fit Zion Shen's algorithm (only add one edge)
	double score = _INFINITE;
	SGNode* ClosetNode = NULL;
	//!!!!!!!!!!!!!!!!!!!!!!!modify this part to fit Zion Shen's algorithm (only add one edge)


	for(size_t i=0;i<PE->ASet.size();i++)
	{	
		//candidate of Adding SGNode
		SGNode* CandNode=PE->ASet[i]->getNode();

		bool AddingOK=true;

		//check 'existing edges' to avoid adding duplicated edge
		for(size_t j=0;j<FromNode->getNumEdge(); j++)
		{	
			SGNode* ToNode=FromNode->getEdge(j)->getToNode();
							
			if(ToNode->getX()==CandNode->getX() && ToNode->getY()==CandNode->getY())
			{	
				AddingOK=false;	//because edge already exists
#ifdef DEBUG_OASG_ADDING_EDGE_CHECK
				cout<<"~ CanNode can be replaced with ToNode (Can ... To)\n";
				PrintNode(CandNode);
				PrintNode(ToNode);
#endif
				break;
			}
		}

		if(AddingOK)
		{	

#ifdef YAO_WEN_METHOD
			SGEdge* E=new SGEdge(FromNode,CandNode);
			vSGEdge.push_back(E);
			FromNode->AddEdge(E);	//actually adding edge
#ifdef DEBUG_OASG_ADDING_EDGE
			cout<<"@ Adding SGEdge from -> to \n";
			PrintNode(FromNode);
			PrintNode(CandNode);
			cout<<"Check dual\n";
#endif
			//edge must be dual.
			if(E->getDualEdge()==NULL)
			{	SGEdge* dualEdge=AddingEdge(CandNode,FromNode);
				E->setDualEdge(dualEdge);
				dualEdge->setDualEdge(E);
			}
#endif


#ifdef ZHOU_METHOD
			//!!!!!!!!!!!!!!!!!!!!!!!modify this part to fit Zion Shen's algorithm (only add one edge)
			if(score > ComputeManhattanDist(FromNode,CandNode))
			{	score = ComputeManhattanDist(FromNode,CandNode);
				ClosetNode = CandNode;
			}
			//!!!!!!!!!!!!!!!!!!!!!!!modify this part to fit Zion Shen's algorithm (only add one edge)
#endif
		}
	}

#ifdef ZHOU_METHOD
	//!!!!!!!!!!!!!!!!!!!!!!!modify this part to fit Zion Shen's algorithm (only add one edge)
	if(ClosetNode != NULL)
	{
		SGEdge* E=new SGEdge(FromNode,ClosetNode);
		vSGEdge.push_back(E);
		FromNode->AddEdge(E);	//actually adding edge
#ifdef DEBUG_OASG_ADDING_EDGE
		cout<<"@ Adding SGEdge from -> to \n";
		PrintNode(FromNode);
		PrintNode(ClosetNode);
		cout<<"Check dual\n";
#endif
		//edge must be dual.
		if(E->getDualEdge()==NULL)
		{	SGEdge* dualEdge=AddingEdge(ClosetNode,FromNode);
			E->setDualEdge(dualEdge);
			dualEdge->setDualEdge(E);
		}
	}
	//!!!!!!!!!!!!!!!!!!!!!!!modify this part to fit Zion Shen's algorithm (only add one edge)
#endif

}


//check intersection of edges. build the relation of edges that intersect each other.
//for more detail, refer to algorithm textbook (sweep-line implementation)
void OASpanningGraph::CheckEdgeIntersection()
{
#ifdef DEBUG_ANALYSIS_RUN_TIME
	clock_t SG_check_intersection_start = clock();
#endif

	//(1) build the event vector
	//    the definition of event : refer to sweep line algorithm
	vector<Event*> vEvent;
	for(int i=0;i<getNumEdge();i++)
	{
		
		SGEdge* E = getEdge(i);
		//No need to consider edges of obstacle
		if(E->getToNode()->getType() != _PIN && E->getFromNode()->getType() != _PIN)
		{	continue;
		}
		
		Point* P1=E->getFromNode()->getPoint();
		Point* P2=E->getToNode()->getPoint();

		if(P1->getX() > P2->getX())
		{	Point* temp = P2;
			P2 = P1;
			P1 = temp;
		}

		//true for leftmost point of edge
		Event* ev1 = new Event(P1->getX(),P1->getY(),P1->getLayer(),true,E);
		Event* ev2 = new Event(P2->getX(),P2->getY(),P2->getLayer(),false,E);
		vEvent.push_back(ev1);
		vEvent.push_back(ev2);
	}

	//(2) sort event vector by increasing x-coordinate
	//when the line is vertical 
	//it may come the rightmost of edge first, then leftmost of edge
	//so the order is reverse, In order to make sure the order, we use stable_sort() instead of sort()
	stable_sort(vEvent.begin(),vEvent.end(),Event_XLessThan());

	//(3) sweep from left to right
	vector<SGEdge*> State;

	for(int i=0;i<vEvent.size();i++)
	{	Event* ev=vEvent[i];
		SGEdge* E=ev->getEdge();

#ifdef DEBUG_OASG_DETECT_INTERSECTION
		cout<<"\nEvent\n";
		PrintEdge(E);
#endif

		if(ev->LeftmostOfEdge() == true)
		{
#ifdef DEBUG_OASG_DETECT_INTERSECTION
			cout<<":LeftmostOfEdge\n";
#endif
			//compare with the edges in state
			for(int j=0;j<State.size();j++)
			{
				SGEdge* detectEdge=State[j];
				if(E == detectEdge)
				{	cout<<"Error: occured in¡@OASpanningGraph::CheckEdgeIntersection().\n";
					exit(0);
				}
				else if(DetectLineIntersection(E,detectEdge) == true)
				{	
#ifdef DEBUG_OASG_DETECT_INTERSECTION
					cout<<"Intersect this edge\n";
					PrintEdge(detectEdge);
#endif
					E->addIntersectEdge(detectEdge);
					detectEdge->addIntersectEdge(E);
				}
			}
			//add this edge into State
			State.push_back(E);
		}
		else
		{
#ifdef DEBUG_OASG_DETECT_INTERSECTION			
			cout<<":RightmostOfEdge\n";
#endif			
			//remove edge from State
			int deleteIndex=-1;
			for(int j=0;j<State.size();j++)
			{
				SGEdge* detectEdge=State[j];
				if(detectEdge == E)
				{	
					deleteIndex=j;
#ifdef DEBUG_OASG_DETECT_INTERSECTION
					cout<<"delete this edge from State\n";
					PrintEdge(detectEdge);
#endif
					break;
				}
			}

			if(deleteIndex==-1)
			{
				cout<<"Error: occured in¡@OASpanningGraph::CheckEdgeIntersection(). Index cannot be negative\n";
				exit(0);
			}
			else
			{	State.erase(State.begin()+deleteIndex);
			}
		}
	}
	
#ifdef DEBUG_ANALYSIS_RUN_TIME
	clock_t SG_check_intersection_end = clock();
	RunTime* RunTime = data->getRunTimeSet();
	RunTime->SG_check_intersection = RunTime->SG_check_intersection + float(SG_check_intersection_end - SG_check_intersection_start)/CLK_TCK;
	cout<<"RunTime->SG_check_intersection = "<<RunTime->SG_check_intersection<<endl;
#endif

}

bool OASpanningGraph::DetectLineIntersection(SGEdge* E1,SGEdge* E2)
{
	Point* E1_P1 = E1->getFromNode()->getPoint();
	Point* E1_P2 = E1->getToNode()->getPoint();

	Point* E2_P1 = E2->getFromNode()->getPoint();
	Point* E2_P2 = E2->getToNode()->getPoint();


	Vector E1_V1(E1_P1->getX(),E1_P1->getY());
	Vector E1_V2(E1_P2->getX(),E1_P2->getY());

	Vector E2_V1(E2_P1->getX(),E2_P1->getY());
	Vector E2_V2(E2_P2->getX(),E2_P2->getY());


    LineSegment linesegment0(E1_V1, E1_V2);
    LineSegment linesegment1(E2_V1, E2_V2);

    Vector intersection;


    switch(linesegment0.Intersect(linesegment1, intersection))
    {
		case LineSegment::PARALLEL:
#ifdef DEBUG_OASG_DETECT_INTERSECTION_DETAIL
			std::cout << "The lines are parallel\n\n";
#endif
			return false;
		break;
		case LineSegment::COINCIDENT:
#ifdef DEBUG_OASG_DETECT_INTERSECTION_DETAIL
			std::cout << "Line Segment 0: (" << E1_V1.x_ << ", " << E1_V1.y_ << ") to (" << E1_V2.x_ << ", " << E1_V2.y_ << ")\n"
				  << "Line Segment 1: (" << E2_V1.x_ << ", " << E2_V1.y_ << ") to (" << E2_V2.x_ << ", " << E2_V2.y_ << ")\n";
			std::cout << "The lines are coincident\n\n";
#endif
			return true;
		break;
		case LineSegment::NOT_INTERESECTING:
#ifdef DEBUG_OASG_DETECT_INTERSECTION_DETAIL
			std::cout << "The lines do not intersect\n\n";
#endif
			return false;
		break;
		case LineSegment::INTERESECTING:


			if((intersection.x_==E1_V1.x_ && intersection.y_==E1_V1.y_) || (intersection.x_==E1_V2.x_ && intersection.y_==E1_V2.y_)
				|| (intersection.x_==E1_V2.x_ && intersection.y_==E1_V2.y_) || (intersection.x_==E2_V2.x_ && intersection.y_==E2_V2.y_))
			{	return false;
			}
			else
			{
#ifdef DEBUG_OASG_DETECT_INTERSECTION_DETAIL
				std::cout << "Line Segment 0: (" << E1_V1.x_ << ", " << E1_V1.y_ << ") to (" << E1_V2.x_ << ", " << E1_V2.y_ << ")\n"
					  << "Line Segment 1: (" << E2_V1.x_ << ", " << E2_V1.y_ << ") to (" << E2_V2.x_ << ", " << E2_V2.y_ << ")\n";
				std::cout << "The lines intersect at (" << intersection.x_ << ", " << intersection.y_ << ")\n\n";
#endif
				return true;
			}
			
		break;
    }
	
	return false;
}
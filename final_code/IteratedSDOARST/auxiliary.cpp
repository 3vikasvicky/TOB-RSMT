#include "OARST.h"

namespace auxiliary 
{
	//Notice : this cpp is just for auxiliary and debuging, such as printing the specified data

	void PrintPin(Database& database,size_t index)
	{
		Pin* P = database.GetPin(index);
		cout<<P->getX()<<" "<<P->getY()<<" "<<P->getLayer()<<endl;
	}

	void PrintPins(Database& database)
	{	
		cout<<"Pin number is "<<database.GetPinSize()<<endl;
		for(size_t i=0;i<database.GetPinSize();i++)
		{	
			PrintPin(database,i);
		}
	}

	void PrintObstacle(Obstacle* O)
	{
		cout<<"LB ("<<O->getX1()<<","<<O->getY1()<<") TR ("<<O->getX2()<<","<<O->getY2()<<") L:"<<O->getLayer()<<endl;
	}

	void PrintObstacle(Database& database,size_t index)
	{
		Obstacle* O = database.GetObstacle(index);
		cout<<"LB ("<<O->getX1()<<","<<O->getY1()<<") TR ("<<O->getX2()<<","<<O->getY2()<<") L:"<<O->getLayer()<<endl;
	}


	void PrintObstacles(Database& database)
	{	
		cout<<"Obstacle number is "<<database.GetObstacleSize()<<endl;		
		for(size_t i=0;i<database.GetObstacleSize();i++)
		{
			PrintObstacle(database,i);
		}
	}

	void PrintPE(ProcessEntry* PE)
	{	
		switch(PE->getType())
		{	
			case _PIN :
				cout<<"PE Type: Pin ";
				break;
			case _OBSTACLE_LEFT :
				cout<<"PE Type: Obstacle left ";
				break;
			case _OBSTACLE_RIGHT :
				cout<<"PE Type: Obstacle right ";
				break;
			default:
				cout<<"Type Error has occured in printPE(...)\n"; exit(0);
		}

		switch(PE->getRegion())
		{
			case _R1 : 
				cout<<"Region = _R1 ";
				break;
			case _R2 : 
				cout<<"Region = _R2 ";
				break;
			case _R3 : 
				cout<<"Region = _R3 ";
				break;
			case _R4 : 
				cout<<"Region = _R4 ";
				break;
			default:
				cout<<"Region Error has occured in printPE(...)\n"; exit(0);
		}
		cout<<" ("<<PE->getX()<<","<<PE->getY()<<")"<<endl;
	}

	void PrintNode(SGNode* N)
	{
		if(N==NULL)
		{	cout<<"NULL\n";	
		}
		else if(N->getType()==_PIN)
		{	cout<<"SGNode Pin ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
		}
		else if(N->getDetailType()==_OBSTACLE_LEFT_LOWER)
		{	cout<<"SGNode Object Left Lower ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
		}
		else if(N->getDetailType()==_OBSTACLE_LEFT_UPPER)
		{	cout<<"SGNode Object Left Upper ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
		}	
		else if(N->getDetailType()==_OBSTACLE_RIGHT_LOWER)
		{	cout<<"SGNode Object Right Lower ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
		}
		else if(N->getDetailType()==_OBSTACLE_RIGHT_UPPER)
		{	cout<<"SGNode Object Right Upper ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
		}	
		else
		{	cout<<"Error occured in void PrintNode(SGNode* N)\n";
			cout<<"SGNode Type = "<<N->getType()<<" ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
			exit(0);
		}
	}

	void PrintRGNode(RGNode* N)
	{
		if(N==NULL)
		{	cout<<"NULL\n";	
		}
		else if(N->getType()==_PIN)
		{	cout<<"RGNode Pin ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
		}
		else if(N->getType()==_OBSTACLE_LEFT)
		{	cout<<"RGNode Object Left ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
		}
		else if(N->getType()==_OBSTACLE_RIGHT)
		{	cout<<"RGNode Object Right ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
		}		
		else if(N->getType()==_TURNING)
		{	cout<<"RGNode Turning ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
		}		
		else
		{	cout<<"Error occured in void PrintRGNode(RGNode* N)\n";
			cout<<"RGNode ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
			cout<<"Its type = "<<N->getType()<<endl;
			exit(0);
		}
	}

	void PrintRGNode_Delay(RGNode* N)
	{
		if(N==NULL)
		{	cout<<"NULL\n";	
		}
		else if(N->getType()==_PIN)
		{	cout<<"RGNode Pin ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
			cout<<"       Leaf node ? "<<N->checkIsLeaf()<<endl;
			cout<<"       Its downstream capacitance = "<<N->getDowntreamCap()<<" (Ff)"<<endl;
			cout<<"       Delay (Driving node to this Sink)= "<<N->getDelay()<<" (ps)"<<endl;
		}
		else if(N->getType()==_OBSTACLE_LEFT)
		{	cout<<"RGNode Object Left ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
			cout<<"       Leaf node ? "<<N->checkIsLeaf()<<endl;
			cout<<"       Its downstream capacitance = "<<N->getDowntreamCap()<<" (Ff)"<<endl;
			cout<<"       Delay (Driving node to this Sink)= "<<N->getDelay()<<" (ps)"<<endl;
		}
		else if(N->getType()==_OBSTACLE_RIGHT)
		{	cout<<"RGNode Object Right ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
			cout<<"       Leaf node ? "<<N->checkIsLeaf()<<endl;
			cout<<"       Its downstream capacitance = "<<N->getDowntreamCap()<<" (Ff)"<<endl;
			cout<<"       Delay (Driving node to this Sink)= "<<N->getDelay()<<" (ps)"<<endl;
		}		
		else if(N->getType()==_TURNING)
		{	cout<<"RGNode Turning ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
			cout<<"       Leaf node ? "<<N->checkIsLeaf()<<endl;
			cout<<"       Its downstream capacitance = "<<N->getDowntreamCap()<<" (Ff)"<<endl;
			cout<<"       Delay (Driving node to this Sink)= "<<N->getDelay()<<" (ps)"<<endl;
		}		
		else
		{	cout<<"Error occured in void PrintRGNode(RGNode* N)\n";
			cout<<"RGNode ("<<N->getX()<<","<<N->getY()<<","<<N->getLayer()<<") dist from Driving: "<<N->getDistFromDriving()<<endl;
			cout<<"Its type = "<<N->getType()<<endl;
			cout<<"       Leaf node ? "<<N->checkIsLeaf()<<endl;
			cout<<"       Its downstream capacitance = "<<N->getDowntreamCap()<<" (Ff)"<<endl;
			cout<<"       Delay (Driving node to this Sink)= "<<N->getDelay()<<" (ps)"<<endl;
			exit(0);
		}
	}

	void PrintEdge(SGEdge* E)
	{	
		if(E!=NULL)
		{
			cout<<"Print SGEdge from -> to (Routed="<<E->checkRouted()<<", Enable="<<E->checkEnable()<<" TurnRect="<<E->checkTurnRect()<<")\n";
			PrintNode(E->getFromNode());
			PrintNode(E->getToNode());
		}
		else
		{	cout<<"NULL\n";
		}
	}


	void PrintRGEdge(RGEdge* E)
	{	
		if(E!=NULL)
		{
			cout<<"Print RGEdge from -> to , Length ("<<E->getLength()<<")\n";
			PrintRGNode(E->getFromNode());
			PrintRGNode(E->getToNode());
		}
		else
		{	cout<<"NULL\n";
		}
	}

	double ComputeLength(SGNode* N1,SGNode* N2)
	{	
		double diffX = N1->getX() - N2->getX() ;
		double diffY = N1->getY() - N2->getY() ;

		return sqrt(diffX*diffX + diffY*diffY);		
	}

	double ComputeSqrLength(SGNode* N1,SGNode* N2)
	{
	  double diffX = N1->getX() - N2->getX();
	  double diffY = N1->getY() - N2->getY();
	  return (diffX*diffX+diffY*diffY);
	}

	double ComputeLength(RGNode* N1,RGNode* N2)
	{	
		double diffX = N1->getX() - N2->getX() ;
		double diffY = N1->getY() - N2->getY() ;

		return sqrt(diffX*diffX + diffY*diffY);		
	}
	double ComputeSqrLength(RGNode* N1, RGNode* N2)
	{
	  double diffX = N1->getX() - N2->getX();
	  double diffY = N1->getY() - N2->getY();
	  return (diffX*diffX+diffY*diffY);
	}

	double ComputeManhattanDist(SGNode* N1,SGNode* N2)
	{	return abs(N1->getX()-N2->getX()) + abs(N1->getY()-N2->getY());
	}

	bool Same_RGNode(RGNode* RN1,RGNode* RN2)
	{
		if(RN1->getX()==RN2->getX() && RN1->getY()==RN2->getY())
		{	return true;
		}
		else
		{	return false;
		}
	}

	bool Is_Vertical_Line(SGEdge* SG_E)
	{	
		if(SG_E->getToNode()->getPoint()->getX() == SG_E->getFromNode()->getPoint()->getX())
		{	return true;
		}
		else
		{	return false;
		}
	}

	bool Is_Horizontal_Line(SGEdge* SG_E)
	{	
		if(SG_E->getToNode()->getPoint()->getY() == SG_E->getFromNode()->getPoint()->getY())
		{	return true;
		}
		else
		{	return false;
		}
	}

	bool Is_Vertical_Line(RGEdge* RG_E)
	{	
		if(RG_E->getToNode()->getPoint()->getX() == RG_E->getFromNode()->getPoint()->getX())
		{	return true;
		}
		else
		{	return false;
		}
	}

	bool Is_Horizontal_Line(RGEdge* RG_E)
	{	
		if(RG_E->getToNode()->getPoint()->getY() == RG_E->getFromNode()->getPoint()->getY())
		{	return true;
		}
		else
		{	return false;
		}
	}

	int Compute_Quadrant(SGNode* N,SGNode* Root)
	{
		if(N->getX() < Root->getX())
		{	if(N->getY() < Root->getY())	
			{	return _QUADRANT3;
			}
			else if(N->getY() > Root->getY())
			{	return _QUADRANT2;
			}
			else //N->getY() = Root->getY()
			{	return _QUADRANT23;
			}
		}
		else if(N->getX() > Root->getX())
		{	if(N->getY() < Root->getY())	
			{	return _QUADRANT4;
			}
			else if(N->getY() > Root->getY())	
			{	return _QUADRANT1;
			}
			else //N->getY() = Root->getY()
			{	return _QUADRANT14;
			}
		}
		else //(N->getX() = Root->getX())
		{
			if(N->getY() < Root->getY())	
			{	return _QUADRANT34;
			}
			else if(N->getY() > Root->getY())	
			{	return _QUADRANT12;
			}
			else //N->getY() = Root->getY()   N == Root?
			{	cout<<"Error occured in Compute_Quadrant(), N = Root\n";
				exit(0);
			}
		}
	}

	int Compute_Quadrant(SGNode* N,RGNode* Root)
	{
		if(N->getX() < Root->getX())
		{	if(N->getY() < Root->getY())	
			{	return _QUADRANT3;
			}
			else if(N->getY() > Root->getY())
			{	return _QUADRANT2;
			}
			else //N->getY() = Root->getY()
			{	return _QUADRANT23;
			}
		}
		else if(N->getX() > Root->getX())
		{	if(N->getY() < Root->getY())	
			{	return _QUADRANT4;
			}
			else if(N->getY() > Root->getY())	
			{	return _QUADRANT1;
			}
			else //N->getY() = Root->getY()
			{	return _QUADRANT14;
			}
		}
		else //(N->getX() = Root->getX())
		{
			if(N->getY() < Root->getY())	
			{	return _QUADRANT34;
			}
			else if(N->getY() > Root->getY())	
			{	return _QUADRANT12;
			}
			else //N->getY() = Root->getY()   N == Root?
			{	cout<<"Error occured in Compute_Quadrant(), N = Root\n";
				exit(0);
			}
		}
	}

	int Compute_Quadrant(RGNode* N,RGNode* Root)
	{
		if(N->getX() < Root->getX())
		{	if(N->getY() < Root->getY())	
			{	return _QUADRANT3;
			}
			else if(N->getY() > Root->getY())
			{	return _QUADRANT2;
			}
			else //N->getY() = Root->getY()
			{	return _QUADRANT23;
			}
		}
		else if(N->getX() > Root->getX())
		{	if(N->getY() < Root->getY())	
			{	return _QUADRANT4;
			}
			else if(N->getY() > Root->getY())	
			{	return _QUADRANT1;
			}
			else //N->getY() = Root->getY()
			{	return _QUADRANT14;
			}
		}
		else //(N->getX() = Root->getX())
		{
			if(N->getY() < Root->getY())	
			{	return _QUADRANT34;
			}
			else if(N->getY() > Root->getY())	
			{	return _QUADRANT12;
			}
			else //N->getY() = Root->getY()   N == Root?
			{	cout<<"Error occured in Compute_Quadrant(), N = Root\n";
				PrintRGNode(Root);
				PrintRGNode(N);
				//exit(0);
			}
		}
	}

	bool Horizontal_Overlap(RGEdge* RE1,RGEdge* RE2)
	{
		if(Is_Horizontal_Line(RE1) && Is_Horizontal_Line(RE2))
		{
				int RE1_X1 = min(RE1->getFromNode()->getX(),RE1->getToNode()->getX());
				int RE1_X2 = max(RE1->getFromNode()->getX(),RE1->getToNode()->getX());
				int RE2_X1 = min(RE2->getFromNode()->getX(),RE2->getToNode()->getX());
				int RE2_X2 = max(RE2->getFromNode()->getX(),RE2->getToNode()->getX());

				bool RE2_X1_IN_RE1 = false;
				bool RE2_X2_IN_RE1 = false;

				if(RE1_X1 <= RE2_X1 && RE2_X1 <= RE1_X2)
				{	RE2_X1_IN_RE1 = true;
				}

				if(RE1_X1 <= RE2_X2 && RE2_X2 <= RE1_X2)
				{	RE2_X2_IN_RE1 = true;
				}

				if((RE2_X1_IN_RE1 && RE2_X2_IN_RE1) || (RE2_X1_IN_RE1 && (RE1_X2!=RE2_X1)) || (RE2_X2_IN_RE1 && (RE1_X1!=RE2_X2)))
				{
					return true;
				}
				else
				{	return false;
				}
		}
		else
		{	return false;
		}
	}
	bool Vertical_Overlap(RGEdge* RE1,RGEdge* RE2)
	{
		if(Is_Vertical_Line(RE1) && Is_Vertical_Line(RE2))
		{
				int RE1_Y1 = min(RE1->getFromNode()->getY(),RE1->getToNode()->getY());
				int RE1_Y2 = max(RE1->getFromNode()->getY(),RE1->getToNode()->getY());
				int RE2_Y1 = min(RE2->getFromNode()->getY(),RE2->getToNode()->getY());
				int RE2_Y2 = max(RE2->getFromNode()->getY(),RE2->getToNode()->getY());

				bool RE2_Y1_IN_RE1 = false;
				bool RE2_Y2_IN_RE1 = false;

				if(RE1_Y1 <= RE2_Y1 && RE2_Y1 <= RE1_Y2)
				{	RE2_Y1_IN_RE1 = true;
				}

				if(RE1_Y1 <= RE2_Y2 && RE2_Y2 <= RE1_Y2)
				{	RE2_Y2_IN_RE1 = true;
				}

				if((RE2_Y1_IN_RE1 && RE2_Y2_IN_RE1) || (RE2_Y1_IN_RE1 && (RE1_Y2!=RE2_Y1)) || (RE2_Y2_IN_RE1 && (RE1_Y1!=RE2_Y2)))
				{
					return true;
				}
				else
				{	return false;
				}
		}
		else
		{	return false;
		}
	}

	bool IsOverlap(RGEdge* RE1,RGEdge* RE2)
	{
		//since the tree is rectilinear, so the edge must be horizontal or vetical
		if(Is_Vertical_Line(RE1) && Is_Vertical_Line(RE2))
		{	
			if(RE1->getFromNode()->getX() != RE2->getFromNode()->getX())
			{	return false;
			}
			else
			{
				int RE1_Y1 = min(RE1->getFromNode()->getY(),RE1->getToNode()->getY());
				int RE1_Y2 = max(RE1->getFromNode()->getY(),RE1->getToNode()->getY());
				int RE2_Y1 = min(RE2->getFromNode()->getY(),RE2->getToNode()->getY());
				int RE2_Y2 = max(RE2->getFromNode()->getY(),RE2->getToNode()->getY());

				bool RE2_Y1_IN_RE1 = false;
				bool RE2_Y2_IN_RE1 = false;

				if(RE1_Y1 <= RE2_Y1 && RE2_Y1 <= RE1_Y2)
				{	RE2_Y1_IN_RE1 = true;
				}

				if(RE1_Y1 <= RE2_Y2 && RE2_Y2 <= RE1_Y2)
				{	RE2_Y2_IN_RE1 = true;
				}

				if((RE2_Y1_IN_RE1 && RE2_Y2_IN_RE1) || (RE2_Y1_IN_RE1 && (RE1_Y2!=RE2_Y1)) || (RE2_Y2_IN_RE1 && (RE1_Y1!=RE2_Y2)))
				{
					return true;
				}
				else
				{	return false;
				}
			}
		}
		else if(Is_Horizontal_Line(RE2) && Is_Horizontal_Line(RE1))
		{	
			if(RE1->getFromNode()->getY() != RE2->getFromNode()->getY())
			{	return false;
			}
			else
			{
				int RE1_X1 = min(RE1->getFromNode()->getX(),RE1->getToNode()->getX());
				int RE1_X2 = max(RE1->getFromNode()->getX(),RE1->getToNode()->getX());
				int RE2_X1 = min(RE2->getFromNode()->getX(),RE2->getToNode()->getX());
				int RE2_X2 = max(RE2->getFromNode()->getX(),RE2->getToNode()->getX());

				bool RE2_X1_IN_RE1 = false;
				bool RE2_X2_IN_RE1 = false;

				if(RE1_X1 <= RE2_X1 && RE2_X1 <= RE1_X2)
				{	RE2_X1_IN_RE1 = true;
				}

				if(RE1_X1 <= RE2_X2 && RE2_X2 <= RE1_X2)
				{	RE2_X2_IN_RE1 = true;
				}

				if((RE2_X1_IN_RE1 && RE2_X2_IN_RE1) || (RE2_X1_IN_RE1 && (RE1_X2!=RE2_X1)) || (RE2_X2_IN_RE1 && (RE1_X1!=RE2_X2)))
				{
					return true;
				}
				else
				{	return false;
				}
			}
		}
		else
		{	return false;
		}
		
	}

	RGEdge* FindOverlapEdge(RGNode* Root,RGEdge* RE)
	{
		for(int i=0;i<Root->getNumEdge();i++)
		{	
			if(Root->getEdge(i) != RE && Root->getEdge(i) != RE->getDualEdge())
			{	
				if(IsOverlap(Root->getEdge(i),RE))
				{	return Root->getEdge(i);
				}
			}
		}

		return NULL;
	}

	RGEdge* FindOverlapEdge(RGEdge* RE) //be careful that do not return its dual edge
	{
		RGNode* FromNode = RE->getFromNode();
		//RGNode* ToNode = RE->getToNode();


		for(int i=0;i<FromNode->getNumEdge();i++)
		{	
			if(FromNode->getEdge(i) != RE && FromNode->getEdge(i) != RE->getDualEdge())
			{	
				if(IsOverlap(FromNode->getEdge(i),RE))
				{	return FromNode->getEdge(i);
				}
			}
		}

		return NULL;
	}

	bool V_In_U_Quad1(SweepEntry* U,SweepEntry* V)
	{	if(U->getX() <= V->getX() && U->getY() <= V->getY())
		{	return true;
		}
		else
		{	return false;
		}
	}
	bool V_In_U_Quad2(SweepEntry* U,SweepEntry* V)
	{	if(U->getX() >= V->getX() && U->getY() <= V->getY())
		{	return true;
		}
		else
		{	return false;
		}
	}
	bool V_In_U_Quad3(SweepEntry* U,SweepEntry* V)
	{	if(U->getX() >= V->getX() && U->getY() >= V->getY())
		{	return true;
		}
		else
		{	return false;
		}
	}
	bool V_In_U_Quad4(SweepEntry* U,SweepEntry* V)
	{	if(U->getX() <= V->getX() && U->getY() >= V->getY())
		{	return true;
		}
		else
		{	return false;
		}
	}

	bool IsCorner(RGNode* N)
	{	if(N->getSGNode()==NULL)
		{	return false;
		}
		else if(N->getSGNode()->getType()==_PIN)
		{	return false;
		}
		else
		{	return true;
		}
	}
};


//auxiliary function of printing private member

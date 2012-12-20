#ifndef SORTING_H //Avoid duplicate definition
#define SORTING_H

#include "OARST.h"
//#include "RTree.h"

//-----------------------------------------------------------------------------
//OASG.h
//-----------------------------------------------------------------------------

	class PE_XLessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const ProcessEntry* P1,const ProcessEntry* P2)
			{	return P1->getX() < P2->getX();
			}
	};
	class PE_XGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const ProcessEntry* P1,const ProcessEntry* P2)
			{	return P1->getX() > P2->getX();
			}
	};
	class PE_YLessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const ProcessEntry* P1,const ProcessEntry* P2)
			{	return P1->getY() < P2->getY();
			}
	};
	class PE_YGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const ProcessEntry* P1,const ProcessEntry* P2)
			{	return P1->getY() > P2->getY();
			}
	};
	class PE_XYGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const ProcessEntry* P1,const ProcessEntry* P2)
			{	return (P1->getY()+P1->getX()) > (P2->getY()+P2->getX());
			}
	};
	class PE_XYLessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const ProcessEntry* P1,const ProcessEntry* P2)
			{	return (P1->getY()+P1->getX()) < (P2->getY()+P2->getX());
			}
	};
	class PE_Y_minus_X_LessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const ProcessEntry* P1,const ProcessEntry* P2)
			{	return (P1->getY()-P1->getX()) < (P2->getY()-P2->getX());
			}
	};

	class AE_XLessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const AEntry* A1,const AEntry* A2)
			{	return A1->getX() < A2->getX();
			}
	};
	class AE_XGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const AEntry* A1,const AEntry* A2)
			{	return A1->getX() > A2->getX();
			}
	};
	class AE_YLessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const AEntry* A1,const AEntry* A2)
			{	return A1->getY() < A2->getY();
			}
	};
	class AE_YGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const AEntry* A1,const AEntry* A2)
			{	return A1->getY() > A2->getY();
			}
	};
	class SE_NonDecreaseX //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (SweepEntry* E1,SweepEntry* E2)
			{	
				if(E1->getX() != E2->getX() || E1->getType()==_PIN || E2->getType()==_PIN)
				{	return E1->getX() < E2->getX();
				}
				else
				{	return E1->get_Left_Lower_N()->getX() < E2->get_Left_Lower_N()->getX();
				}
			}
	};
	class SE_NonDecreaseXplusY //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const SweepEntry* E1,const SweepEntry* E2)
			{	
				return (E1->getX()+E1->getY()) < (E2->getX()+E2->getY());
			}
	};
	class SE_NonDecreaseYminusX //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const SweepEntry* E1,const SweepEntry* E2)
			{	
				return (E1->getY()-E1->getX()) < (E2->getY()-E2->getX());
			}
	};
	class SE_XGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const SweepEntry* E1,const SweepEntry* E2)
			{	return E1->getX() > E2->getX();
			}
	};
	class SE_NonDecreaseY //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (SweepEntry* E1,SweepEntry* E2)
			{	
				if(E1->getY() != E2->getY() || E1->getType()==_PIN || E2->getType()==_PIN)
				{	return E1->getY() < E2->getY();
				}
				else
				{	return E1->get_Left_Lower_N()->getY() < E2->get_Left_Lower_N()->getY();
				}
			}
	};
	class SE_YGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const SweepEntry* E1,const SweepEntry* E2)
			{	return E1->getY() > E2->getY();
			}
	};
	class SE_XYLessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const SweepEntry* E1,const SweepEntry* E2)
			{	return (E1->getX() + E1->getY()) < (E2->getX()+E2->getY());
			}
	};
	class SE_XYGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const SweepEntry* E1,const SweepEntry* E2)
			{	return (E1->getX()+E1->getY()) > (E2->getX()+E2->getY());
			}
	};
	class SE_Y_minus_X_LessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (const SweepEntry* E1,const SweepEntry* E2)
			{	return (E1->getY() - E1->getX()) < (E2->getY() - E2->getX());
			}
	};


	class Node_ProCostGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (SGNode* N1,SGNode* N2)
			{	return N1->getTempProCost() > N2->getTempProCost();
			}
	};

	class EstimatedSourceDist_GreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (SGNode* N1,SGNode* N2)
			{	return N1->getTempDistFromSource() > N2->getTempDistFromSource();
			}
	};

	class EstimatedTreeDist_GreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (SGNode* N1,SGNode* N2)
			{	
				return N1->getTempDistFromTree() > N2->getTempDistFromTree();			
			}
	};
#ifdef PRIORITY_BASED
	class SGNPriority_GreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (SGNode* N1,SGNode* N2)
			{	
			  /*
				return N1->getPriority() > N2->getPriority();			
				*/
			  if(N1->getPriority() > N2->getPriority())
			  {
			    return true ;
			  }
			  else
			  {
			    return N1->getTempDistFromTree() > N2->getTempDistFromTree() ;
			  }
			}
	};
#endif

	class Node_XGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (SGNode* N1,SGNode* N2)
			{	return N1->getX() > N2->getX();
			}
	};

	class Node_YGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (SGNode* N1,SGNode* N2)
			{	return N1->getY() > N2->getY();
			}
	};

	class Node_XLessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (SGNode* N1,SGNode* N2)
			{	return N1->getX() < N2->getX();
			}
	};

	class Node_YLessThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (SGNode* N1,SGNode* N2)
			{	return N1->getY() < N2->getY();
			}
	};

//-----------------------------------------------------------------------------
//OASG.h
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
//OARG.h : Branch Moving
//-----------------------------------------------------------------------------
	class Node_NSGreaterThan //Using function object instead of function pointer because of inlining effect
	{	public:
			bool operator() (RGNode* N1,RGNode* N2)
			{	return (N1->getRequiredTime() - N1->getDelay()) > (N2->getRequiredTime() - N2->getDelay());
			}
	};


#endif

#include "OARST.h"
using namespace auxiliary;


bool OARectilinearGraph::EliminateOverlap(RGEdge* RE) //check RE
{

	if(RE == NULL)
	{	return false;
	}

#ifdef DEBUG_OARG_ELIMINATE_OVERLAP_EDGE
	cout<<"\n--[1: Start eliminating overlapped edge]\n";
	cout<<"Now check : ";
	PrintRGEdge(RE);
#endif

	//getchar();

	//[1] search overlapped edge
	RGEdge* Overlap_RE = NULL;
	Overlap_RE = FindOverlapEdge(RE);

#ifdef DEBUG_OARG_ELIMINATE_OVERLAP_EDGE
		cout<<"\n--[2: Find overlapped edge]\n";
		PrintRGEdge(Overlap_RE);
#endif

	//[2] merge overlap edges
	bool remove_overlap = false;
	if(RE != NULL && Overlap_RE != NULL)
	{	Merge2RGEdge(RE,Overlap_RE);
		remove_overlap = true;
	}
	
#ifdef DEBUG_OARG_ELIMINATE_OVERLAP_EDGE
	cout<<"\n--[3: End eliminating overlapped edge ... OK]\n";
#endif

	return remove_overlap;
	
}



//RE : the edge we need to merge because that will overlap with other edges
//Overlap_RE : the edge we can reuse
void OARectilinearGraph::Merge2RGEdge(RGEdge* RE,RGEdge* Overlap_RE)    
{

	double RE1_Length = RE->getLength();
	double RG_E0_Length = Overlap_RE->getLength();


#ifdef DEBUG_OARG_MERGE_TWO_EGEDGE
	cout<<"\n--[Merge the following 2 RG edges]\n";
	cout<<"RE   => ";
	PrintRGEdge(RE);
	cout<<"Overlap_RE => ";
	PrintRGEdge(Overlap_RE);
	cout<<"\n";
#endif



	RGNode* Root = NULL; //end point of RE and Overlap_RE
	RGNode* RE_N = NULL;   //the other end point of RE
	RGNode* Overlap_RE_N = NULL;   //the other end point of Overlap_RE

	RGEdge* Next_RE1 = NULL; //we will check this edge next (avoid overlapping)

	//[1] designate Root
	if(Same_RGNode(RE->getFromNode(),Overlap_RE->getFromNode()))
	{	Root = RE->getFromNode();
		RE_N = RE->getToNode();
		Overlap_RE_N = Overlap_RE->getToNode();
	}
	else if(Same_RGNode(RE->getToNode(),Overlap_RE->getToNode()))
	{	Root = RE->getToNode();
		RE_N = RE->getFromNode();
		Overlap_RE_N = Overlap_RE->getFromNode();
	}
	else if(Same_RGNode(RE->getFromNode(),Overlap_RE->getToNode()))
	{	Root = RE->getFromNode();
		RE_N = RE->getToNode();
		Overlap_RE_N = Overlap_RE->getFromNode();
	}
	else if(Same_RGNode(RE->getFromNode(),Overlap_RE->getFromNode()))
	{	Root = RE->getToNode();
		RE_N = RE->getFromNode();
		Overlap_RE_N = Overlap_RE->getToNode();
	}
	else
	{	cout<<"Error occured in Merge2RGEdge(), cannot designate Root node\n";
		exit(0);		
	}



	//===================================================================
	


	//[2] delete RE and its dual from Root and N and RGEdge vector
#ifdef DEBUG_OARG_MERGE_TWO_EGEDGE
	cout<<"\n--[Delete RE]\n";
#endif
	DelRGEdge(RE);

#ifdef DEBUG_OARG_MERGE_TWO_EGEDGE
	cout<<"\n--[Delete Overlap_RE]\n";
#endif
	DelRGEdge(Overlap_RE);
#ifdef DEBUG_OARG_MERGE_TWO_EGEDGE
	cout<<"\n";
#endif



	//assign RE_N_2_Driving
	if(RE1_Length <= RG_E0_Length)
	{	
		RGEdge* RE1 = AddRGEdge(StruFromRGNode(Root),StruToRGNode(RE_N),_INFINITE,false);
		RGEdge* RE2 = AddRGEdge(StruFromRGNode(RE_N),StruToRGNode(Overlap_RE_N),_INFINITE,false);
		EliminateOverlap(RE1);
		EliminateOverlap(RE2);
	}
	else
	{	
		RGEdge* RE1 = AddRGEdge(StruFromRGNode(Root),StruToRGNode(Overlap_RE_N),_INFINITE,false);
		RGEdge* RE2 = AddRGEdge(StruFromRGNode(Overlap_RE_N),StruToRGNode(RE_N),_INFINITE,false);
		EliminateOverlap(RE1);
		EliminateOverlap(RE2);
	}

}


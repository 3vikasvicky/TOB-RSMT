#ifndef PRIM_H //Avoid duplicate definition
#define PRIM_H

#include "OARST.h"

class prim_node
{
public:
	prim_node(SGNode* node): node(node), distanceFromTree(_INFINITE), inTree(false) {}
	SGNode* getNode() { return node; }
	prim_node* getFromPrimNode() { return fromPrimNode; }
	void setIndex(int index) { nodeIndex=index; }
	double getDistFromTree()  { return distanceFromTree; }
	void setDistFromTree(double val, prim_node* fromThisPrimNode)  { distanceFromTree=val; fromPrimNode=fromThisPrimNode; }
	void setTreeState(bool val) { inTree=val; }
	bool checkInTree() { return inTree; }
#ifdef PRIORITY_BASED
	double getDistFromDriver() { return distanceFromDriver; }
	void setDistFromDriver(double d, prim_node* fromThisPrimNode) { distanceFromDriver=d; fromPrimNode=fromThisPrimNode; }
#endif
private:
	SGNode* node;
	prim_node* fromPrimNode;
	int nodeIndex;
	double distanceFromTree;
	bool inTree;
#ifdef PRIORITY_BASED
	double distanceFromDriver;
#endif
};

typedef TwoPinNet prim_edge;

class EstimatedDist_LessThan //Using function object instead of function pointer because of inlining effect
{	public:
		bool operator() (prim_node* pn1,prim_node* pn2)
		{	return pn1->getDistFromTree() < pn2->getDistFromTree();
		}
};

#ifdef PRIORITY_BASED
class Priority_GreaterThan //Using function object instead of function pointer because of inlining effect
{	public:
		bool operator() (prim_node* pn1,prim_node* pn2)
		{	return pn1->getNode()->getPriority() > pn2->getNode()->getPriority();
		}
};

#endif

/*class EstimatedDist_LessThan //Using function object instead of function pointer because of inlining effect
{	public:
		bool operator() (TwoPinNet* N1,TwoPinNet* N2)//(const TwoPinNet* N1,const TwoPinNet* N2)
		{	return N1->getEstimatedDist() < N2->getEstimatedDist();
		}
};

class EstimatedDist_GreaterThan //Using function object instead of function pointer because of inlining effect
{	public:
		bool operator() (TwoPinNet* N1,TwoPinNet* N2)//(const TwoPinNet* N1,const TwoPinNet* N2)
		{	return N1->getEstimatedDist() > N2->getEstimatedDist();
		}
};*/

//Heap
template<class _type>
class Heap {
public:
    Heap(){
        make_heap(data.begin(), data.end());//,EstimatedDist_LessThan());
    }
   
    void push(_type entry){
        data.push_back(entry);
		push_heap(data.begin(), data.end(),EstimatedDist_LessThan());
		//push_heap(data.begin(), data.end());
    }

    _type top(){
        return *(data.begin());
    }
   
    _type pop(){
        _type ret;
        ret = top();
        pop_heap(data.begin(), data.end(),EstimatedDist_LessThan());
		//pop_heap(data.begin(), data.end());
        data.pop_back();
        return ret;
    }

	void do_heap()
	{	make_heap(data.begin(), data.end(),EstimatedDist_LessThan());
	}
   
    bool empty(){
        return size() == 0;
    }
   
    void clear(){
        data.clear();
        make_heap(data.begin(), data.end(),EstimatedDist_LessThan());
    }
   
    int size(){
        return data.size();
    }
private:
    vector<_type> data;
};

#endif

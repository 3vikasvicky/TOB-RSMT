#ifndef ROO_H
#define ROO_H
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <algorithm>
#include <iostream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cassert>
#include <climits>
#include <cfloat>
#include <sstream>
#include <cmath>
#include <ctime>
#include <tr1/tuple>

#include "BinaryHeap.h"
using namespace std;
using std::tr1::tuple;
using std::tr1::tie;
using std::tr1::make_tuple;
/*****************************/
/*  User-Defined Parameters  */
/*****************************/
#ifndef DTYPE   // Data type for distance
#define DTYPE float
#endif
#define BUFFERSIZE 1000
#define LINESIZE 2000 
#define CUNIT 1
#define RUNIT 1
#define Rs 0
#define Db 0
#define Cb 0
#define SLEWSPEC 2400.0
//ALPHA is in objective
#define ALPHA 0.0001
//EPSILON is for float comparison
#define EPSILON 0.01
//FACTOR is how to add lengthInBlock into lengthSoFar, because when we compare same lengthSoFar, we like the one with less lengthInBlock
#define FACTOR 0.00001
//#define DEBUG
#define INFO
enum DIRECTION {UNREACHED, RIGHT, LEFT, UP, DOWN};


class Poly;
class Edge;
class Point{
public:
    DTYPE x, y;
    Point (DTYPE a, DTYPE b):x(a), y(b){}
    Point (){}
    DTYPE distance (const Point& other)const{return abs(x-other.x)+abs(y-other.y);}
    DTYPE distance (const Edge& e)const;//return the shortest Manhatan distance to e
    Point projection (const Edge& e) const;
    bool isProjectedIn (const Edge& e) const;
    Point findClosestPoint (const Edge& e) const;
    bool IsOnEdge (const Edge& edge) const;
    bool IsOnPoly (const Poly& poly) const;
    bool isOutside (const Poly& poly) const;
    bool PointsOnSameLine (const Point&that)const{return abs(x-that.x)<EPSILON || abs(y-that.y)<EPSILON;}
    bool PointsOnSameHorizontalLine (const Point&that)const{return abs(y-that.y)<EPSILON;}
    bool PointsOnSameVerticalLine (const Point&that)const{return abs(x-that.x)<EPSILON;}
    bool isEndpoint (const Edge& edge)const;
    Point operator + (const Point & that)const{return Point(x+that.x, y+that.y);}
    Point operator - (const Point & that)const{return Point(x-that.x, y-that.y);}
    Point operator * (float mul)const{return Point(x*mul, y*mul);}
    Point operator / (float div)const{assert (div!=0); return Point(x/div, y/div);}
    bool operator == (const Point & that)const{return (abs(x-that.x)<EPSILON && abs(y-that.y)<EPSILON);}
    bool operator != (const Point & that)const{return ! this->operator == (that);}
    void print () const{printf("(%f,%f)",(float)x, (float)y);}
} ;
class Branch
{
public:
    int i;
    Point p;   // starting point of the branch
    int n;   // index of neighbor
    float Ct;
    Branch(){};
    Branch(int ii, Point ip, int in, float iCt = 0): i(ii), p(ip), n(in), Ct(iCt){};
};
/*
class MaxSegment{
public:
    Branch start, end;
    bool v;// vertical-true, horizontal-false
    DTYPE c;
    MaxSegment(pair<bool,DTYPE> maxSeg):v(maxSeg.first), c(maxSeg.second){}//if needed, add Branch start and end
    MaxSegment(){};
    edge(){return Edge()}
};*/
class Edge{
    Point a, b;
public:
    DTYPE c_;        // verical - x, horizontal - y
    DTYPE c1_, c2_;  // vertical - y1, y2, horizontal - x1, x2. c1_ always < c2_
    bool v_;       // vertical - true, horizontal - false
    //bool o_;       // open - true, close - false
    inline Edge(const Point& p1, const Point& p2):a(p1), b(p2){
        assert(p1.PointsOnSameLine(p2));
        v_ = (p1.PointsOnSameVerticalLine(p2));
        if (v_){
            c_ = p1.x; c1_=min(p1.y, p2.y); c2_=max(p1.y, p2.y);
        }
        else{
            c_ = p1.y; c1_=min(p1.x, p2.x); c2_=max(p1.x, p2.x);
        }
    }
    inline Edge(bool v, DTYPE c, DTYPE c1, DTYPE c2):v_(v), c_(c), c1_(c1), c2_(c2){
        if (v_){
            a = Point (c_, c1_);
            b = Point (c_, c2_);
        }else {
            a = Point (c1_,c_);
            b = Point (c2_,c_);
        }
        assert (c1<c2);
    }
    inline Edge(){}
    
    bool intersects(const Edge& e) const//it's a pure intersection
    {
        //if (v_ == e.v_) return (c_ == e.c_ && c2_ >= e.c1_ && c1_ <= e.c2_);
        if (v_ == e.v_) return false;
        //else return (c_ >= e.c1_ && c_ <= e.c2_ && c1_ <= e.c_ && c2_ >= e.c_);
        else return (c_ > e.c1_ && c_ < e.c2_ && c1_ < e.c_ && c2_ > e.c_);
    }
    inline Point point1() const
    {
        if (v_ == true) return Point(c_, c1_);
        else return Point(c1_, c_);
    }
    inline Point point2() const
    {
        if (v_ == true) return Point(c_, c2_);
        else return Point(c2_, c_);
    }
    inline DIRECTION direction() const{
        if (v_ == true) return a.y>b.y?DOWN:UP;
        else return a.x>b.x?RIGHT:LEFT;
    }
    DTYPE length()const{return c2_ - c1_;}
    bool overlap(const Edge that)const{
        return (v_ == that.v_ && abs(c_ - that.c_)<EPSILON && max(c2_, that.c2_)-min(c1_,that.c1_) < length() + that.length());//*this and that overlaps
    }

};
class Poly{//any rectilinear shape
    vector<Point> pVect;
    public:
    Poly(const vector<Point>& that):pVect(that){}
    Poly(){}
    vector<Point>::iterator begin(){return pVect.begin();}
    vector<Point>::iterator end(){return pVect.end();}
    Point at(int i)const{//point index
        return pVect.at(i);
    }
    Point p1_edge(int edge)const{//edge index. get the first 
        return pVect.at(edge);
    }
    Point p2_edge(int edge)const{
        return pVect.at((edge+1)%size());
    }
    int size() const {return pVect.size();}
    Edge edge(int i)const{
        return Edge(p1_edge(i), p2_edge(i));
    }
    Edge PreviousEdge(int i)const{
        return edge((i+size()-1)%size());
    }
    void print() const{
        int iEdge;
        for (iEdge=0; iEdge < size(); iEdge++){
            p1_edge(iEdge).print();
            printf (" to ");
            p2_edge(iEdge).print();
            if (iEdge+1<size()) printf (" , ");
        }
        printf ("\n");
    }
    bool contain(const Point& p) const{//strictly inside
        int left = 0,  up= 0, right = 0, down = 0;
        vector<Point>::iterator it;
        int i;
        for (i = 0; i < size(); ++i){
            Edge e = edge(i);
            if (p.IsOnEdge(e))  return false; //on the edge
            if (e.v_ && e.c1_<=p.y && p.y<=e.c2_){//because p is not on edge, = here to eliminate Br6 not inside situation
                left += (e.c_ < p.x)?1:0;
                right += (e.c_ > p.x)?1:0;
            }
            else if (!e.v_ && e.c1_<=p.x && p.x<=e.c2_){
                down += (e.c_ < p.y)?1:0;
                up += (e.c_ > p.y)?1:0;
            }
        } 
        return (left>=1 && right>=1 && down>=1 && up>=1);
    }
    bool contain(const Edge& e){//this function is not in used now
        Point p1 = e.point1();
        Point p2 = e.point2();
        bool b1, b2;
        b1 = contain(p1);
        b2 = contain(p2);
        return contain(p1) && contain(p2);
    }
    void push_back (const Point& p){
        pVect.push_back(p);
    }
    bool IsAdjacent (const Poly& that) const{
        for (int i = 0; i < size(); i++){
            Edge edge1 = edge(i);
            for (int j = 0; j < that.size(); j++){
                Edge edge2 = that.edge(j);
                if (edge1.overlap(edge2))   return true;
            }
        }
        return false;
    }
    int FindAdjacent (const Poly& that, int *index_pointA, int *index_pointC) const{//num_adjacent_edges is how many edges are overlapped(usually one), use pointers to change values
        vector<int> points1, points2;
        int num_adjacent_edges = 0;
        for (int i = 0; i < size(); i++){
            Edge edge1 = edge(i);
            for (int j = 0; j < that.size(); j++){
                Edge edge2 = that.edge(j);
                if (edge1.overlap(edge2)){
                    if(num_adjacent_edges == 0){
                        *index_pointA = i;
                        *index_pointC = j;
                    }
                    num_adjacent_edges++;
                }   
            }
        }
        *index_pointC -= num_adjacent_edges - 1;//Because for block2, the recorded one is the last edge index, but we need to return the first edge index
        return num_adjacent_edges;
    }
    Poly cut(int point1, int point2){//return a copy of part of current block, [point1, point2],both point1 and point2 are non-negative
        Poly that;
        for (int i = point1; i <= point2; i++){
            that.push_back(at(i));
        }
        return that;
    }
    Poly operator + (const Poly& that)const{
        Poly result = *this;
        for (int i = 0; i < that.size(); i++){
            result.push_back(that.at(i));
        }
        return result;
    }
    Poly operator += (const Poly& that){
        *this = *this + that;
        return *this;
    }
    Poly operator = (const Poly& that){
        pVect = that.pVect;
        return *this;
    }
    void Plot()const;
};
class Tree{
protected:
    vector<Branch>* pbranchVect;
    vector<Poly>* pblockVect;
    int root;
    //MaxSegment ms;
    set<int> index;//which branch is in this tree
public:
    bool isRootMoved;
    vector<pair<int,float> > EP;//EP is leaf node in tree, root not included
    Edge ms;//maxsegment
    Tree ( vector<Branch>* pb, vector<Poly>* pp, set<int> branchs, vector<pair<int,float> > nodeBranchs, int r): pbranchVect(pb), pblockVect(pp), index(branchs), EP(nodeBranchs), root(r){isRootMoved = false;}//this version has index
    Tree ( vector<Branch>* pb, vector<Poly>* pp, int r): pbranchVect(pb), pblockVect(pp), root(r){isRootMoved = false;}//this version has not index and node
    Tree (){}
    int getRoot()const{return root;}
    void setRoot(const Branch& b){root=b.i;}
    void eraseIndex (int ibranch){
        set<int>::iterator it = index.find(ibranch);
        assert (it != index.end());
        index.erase(it);
    }
    void clearIndex() {
        index.clear();
    }
    void insertIndex (const Branch& b){
        set<int>::iterator it;
        it= index.find(b.i);
        assert (it == index.end());
        index.insert(b.i);
    }
    void insertEP(const Branch& b, float f=0){
        EP.push_back( (make_pair<int,float>)(b.i,f) );
    }
    bool empty (){
        return index.empty();
    }
    void unite (const Tree & that);
    virtual void print ()const;
    pair<bool,DTYPE> buildMaxSegment();
    void breakLShape();
    DTYPE getLength(int ibranch, int ibranch2);
    bool isIndex (int ibranch){return index.find(ibranch)!=index.end();}
    set<int> getIndex ()const {return index;}//only copy
    void generateDangling(vector<pair<int,int> >& unionR);
    void removeDanglingInUnionR(vector<pair<int,int> >& unionR);
    int findBranch (const Point& p) const;
    Edge computeEdge (int ibranch)const{
        return Edge (  pbranchVect->at(ibranch).p, 
            pbranchVect->at(pbranchVect->at(ibranch).n).p  ); 
    }
    bool isEP (int ibranch) const{
        vector<pair<int,float> >::const_iterator it;
        for (it=EP.begin(); it != EP.end(); ++it){
            if (ibranch==it->first) return true;
        }
        return false;
    }
    bool isFP(int ibranch)const{
        return isEP(ibranch)||ibranch==root;
    }
    void trim();
    void removeUseless();
    void combine();
    void removeZeroLength();
    int CountChildren(int) const;
    int findOnlyChild (int ibranch)const;
    set<int> findConnectedBranch(int ibranch)const;//ibranch may not in tree, may be in unionR, but found branchs are all in tree. ibranch could also be some middle point because previous MR connect to it
    /*bool checkStraight(){//all branchs need be a straightline. (connectivity from leaf to root)
        bool isStraight = true;
        set<int>::iterator it;
        for (it = index.begin(); it!= index.end(); ++it){
            if (!pbranchVect->at(*it).p.PointsOnSameLine(pbranchVect->at(pbranchVect->at(*it).n).p)){
                isStraight = false;
                printf ("branch%d is not same line with branch %d\n", *it, pbranchVect->at(*it).n);
            }
        }
    }*/
    void CleanFloatingTerminal(int iparentBranch);
    bool isConnectedBranch(int ibranch, int ibranch2)const;//same branch or being parent-child relation

};
class PR;
class PRS;
//Tree_ is the tree inside one block
class Tree_ :public Tree {
    int iblock;//index of block
    int itree;//treeVect[itree] is this
public:
    vector<pair<int, float> > MP;//still movable escaping points
    Tree_ (vector<Branch>* pbranchVect, vector<Poly>* pblockVect, int root, int ib, int it): Tree(pbranchVect, pblockVect, root), iblock(ib), itree(it){}
    Tree_(){}
    virtual void print ()const;
    void erasePoint(vector<pair<int,float> > & EP, int ibranch){//EP or MP
       int i, j;
       for (i = 0; i < EP.size();i++){
           if (EP[i].first==ibranch){
               for (j=i+1;j<EP.size();j++){
                   EP[j-1] = EP[j];
               }
               EP.resize(EP.size()-1);
               break;
           }
       }
    }
    int getEdge(int ibranch)const;
    int findNextBranch (int ibranch, int step)const;
    int getStep(int);//get step direction. step is dependant on the shape of corner at max segment
    float elmore (int ibranch);
    void PropagateCap ();
    void CalculateElmore ();
    int firstParentOnMaxSeg (int ibranch) const{
        int iparent = ibranch;
        Point p = pbranchVect->at(iparent).p;
        while (!p.IsOnEdge(ms)){
            iparent =  pbranchVect->at(iparent).n;
            p = pbranchVect->at(iparent).p;
        }
        return iparent;
    }
    int rootOnMaxSeg ()const{
        int iparent =root;
        set<int>::iterator it;
        Point p = pbranchVect->at(root).p;
        while (!p.IsOnEdge(ms)){
            for (it=index.begin(); it!= index.end();++it){
                if (pbranchVect->at(*it).n == iparent) {
                    iparent = pbranchVect->at(*it).i;
                    break;
                }
            }
            p = pbranchVect->at(iparent).p;
        }
        return iparent;
    }
    DTYPE getOutLength(int step, int ibranch, int inextBranch, int startEdge, int stopEdge)const;
    void addCombinationPR (const PRS &prs, PR & pr, int ibranch, int inextBranch, int iworstBranch, int ibranchOnMaxSeg, bool bendOfMaxSeg, int startEdge, int stopEdge);
    //bool updateTree (const PRS &, vector<pair<int,int> >&, set<int>&, const Tree&);
    void buildPR (PRS &);
    void addNonlinearPR (float fbase, float& fbase2, DTYPE dWLbase, DTYPE& dWLbase2, float& fCa, float& fRb, const float&fRc, const float& fCc, Point pstart, Point pend, Branch branch, PR & pr, bool bendOfMaxSeg, DTYPE deltaX);
/*    bool behindOnMaxSeg (int ibranch, int ibranch2) const{
        if (ibranch==ibranch2)  return false;
        int iparent = firstParentOnMaxSeg(ibranch);
        int iparent2 = firstParentOnMaxSeg(ibranch2);
        while (iparent!=root){
            if (iparent==iparent2)  return true;
            iparent = pbranchVect->at(iparent).n;
        }
        return false;
    }*/
    bool IsJoggingLeg(int ibranch)const{//will find all jogging leg which is not at max segment, should be 2 jogging legs
        Branch branch1 = pbranchVect->at(ibranch);
        Branch branch2 = pbranchVect->at(branch1.n);
        Point point1 = branch1.p;
        Point point2 = branch2.p;
        if (point1.IsOnEdge(ms) || point2.IsOnEdge(ms))   return false;//Include both joggingleg
        return true;
    }
    int FindFirstSteinerBranch(int ibranch)const;//find first steiner point (ibranch,root]. Will return -1 if no such point
    int FindFirstCommonAncestor(int ibranch1, int ibranch2)const;//find first common ancestor of ibranch1 and ibranch2
    bool IsAncestorBranch(int index_branch, int index_ancestor_branch)const;////a branch is considered as ancestor of itself
};
class PR{
public:
    vector<float> base;
    vector<float> slope;
    vector<DTYPE> length;
    vector<DTYPE> WLbase;
    vector<Point> pstart;
    vector<Point> pend;
    vector<float> shareMaxSeg;//the length form ibranchOnMaxSeg to root
    vector<bool> endOfMaxSeg;
    int iworstBranch;
    int ibranch;
    vector<int> coefA;
    vector<float> coefY;//not using DTYPE because recalculate elmore would be different
    int step;//needed for reconnection
    

    PR(){}
    PR(int ib, int st):ibranch(ib), step(st){}
    void prune (float& fbase, float& fslope, DTYPE& dlength, DTYPE dWLbase){
    }
    void add(float fbase, float fslope, DTYPE dlength, DTYPE dWLbase, float fshareMaxSeg, bool bendOfMaxSeg, Point p1, Point p2){
        prune (fbase, fslope, dlength,dWLbase);
        base.push_back(fbase);
        slope.push_back(fslope);
        length.push_back(dlength);
        WLbase.push_back(dWLbase);
        shareMaxSeg.push_back (fshareMaxSeg);
        shareMaxSeg.push_back (fshareMaxSeg);
        endOfMaxSeg.push_back (bendOfMaxSeg);
        pstart.push_back(p1);
        pend.push_back(p2);

        coefA.push_back(0);
        coefY.push_back(0);
    }
    void show() const;
    int size() const{return base.size();}
};
class PRS{
public:
    vector <class PR> PRV;
    float slew0, slew_spec_;//slew0 is the starting slew of iworstBranchm
    DTYPE deltaX;
    int iworstBranch;
    PRS(int iworstBr, float currentslew, float slew, DTYPE dX):iworstBranch(iworstBr), slew0(currentslew), slew_spec_ (slew), deltaX(dX){}
    void add(class PR & pri){
        PRV.push_back (pri);
    }
    void write() const;
    void read();
    int size()const{return PRV.size();}
};

class MRP{
    public:
    int ixgrid;
    int iygrid;
    Point p;
    float lengthInBlock;
    bool operator == (const MRP & that)const{return (ixgrid==that.ixgrid && iygrid==that.iygrid);}
    MRP(){ixgrid = 0; iygrid = 0; p = Point(0,0); lengthInBlock = 0;}
};

class Design{
    int root; //the root for a whole Steiner Tree
    DTYPE deltaX;//The linearize step length
    float xmin_, xmax_, ymin_, ymax_;//use float because FLT_MAX and -FLT_MAX
    float slew_percentage_;
    float slew_spec_;//slew requirement of this Steiner Tree
    float length_limit_;//for a single wire, how long it can go in the block
    Tree tree;//the outside blockage tree
    char benchmarkPath[BUFFERSIZE], blockFile[BUFFERSIZE], branchFile[BUFFERSIZE];
    float slew_percentage;//the slew percentage of this run, will be 0.2 0.5 or 0.8
    vector<Point> sinks_;//vector of all sinks
    vector<Poly > blockVect;
    vector<Branch> branchVect;
    vector<Tree_> treeVect;
    vector<pair<int,int> > unionR;//branch index and itree(was step for reconnect function)
    vector<float> xgrids;
    vector<float> ygrids;
    vector<vector<float> > gridMarks;
    vector<vector<bool> > blockMarks;
    vector<vector<DIRECTION> > directions;//direction from previos grid cross to (ixgrid, iygrid)


public:
    Design (char argv1[], char argv2[], char argv3[], float argv4){
        strcpy(benchmarkPath, argv1);
        strcpy(blockFile, argv2);
        strcpy(branchFile, argv3);
        slew_percentage_ = argv4;
        xmin_ = FLT_MAX; xmax_ = -FLT_MAX;
        ymin_ = FLT_MAX; ymax_ = -FLT_MAX;
    }
    Design (){
        xmin_ = FLT_MAX; xmax_ = -FLT_MAX;
    }
    /*--------------  IO functions      ------------------*/
    void readBlockFile ();
    void sequenceBranch (int);
    void initialSequenceBranch (int);//sequence anyway, even the first one is orderd
    void readBranchFile ();
    void splitBranch ();
    void reformBranch ();
    void legitimate ();
    void SortAndRecordSlew ();//sort slew in each inside tree, and record min and max for all
    void printBranchs()const;
    void printBlocks()const;
    Tree& getTree() {return tree;}
    void uniteAllTrees();
    void plotAndSaveTree(int)const;
    pair<float,float> getOutsideWL() const;
    float getInsideWL() const;
//    void reconnect();
    void clean();
    Edge computeEdge (int ibranch)const{
        return Edge (  branchVect.at(ibranch).p, 
            branchVect.at(branchVect.at(ibranch).n).p  ); 
    }
    void mazeRouting();
    pair<float, MRP > findPath (BinaryHeap<float, MRP> &mheap, int ibranch) ;
    void traceBack(MRP);
    void generateGrid();
    int findBranchOrFP(const Point & p)const;//FP is EP+root
    bool isBranchOrFP(const Point & p)const;//FP is EP+root
    bool PointIsInBlocks(const Point & p)const;//Judge if p is in any block
    bool ContainSinkInBlock(const Poly & block)const;//Judge if block contains any sink inside
    int findBranch(const Point & point)const ;
    void clearGridMarksAndDirections(){
        for (int i = 0; i < gridMarks.size(); i++){
            gridMarks.at(i) = vector<float> (gridMarks.at(i).size(), FLT_MAX);
            directions.at(i) = vector<DIRECTION> (gridMarks.at(i).size(), UNREACHED);
        }
    }
    /*bool checkStraight(){//all branchs need be a straightline. (connectivity from leaf to root)
        bool isStraight = true;
        isStraight = isStraight & tree.checkStraight();
        for (int i = 0; i < treeVect.size(); i++){
            isStraight = isStraight & treeVect[i].checkStraight();
        }
        return isStraight;
    }*/
    bool ischanging;
    bool updateTree (int, const PRS &, vector<pair<int,int> >&);
    template <class T>
        void print( vector<T>const& vT){
            typename vector<T>::const_iterator it;
            for (it=vT.begin(); it!= vT.end(); ++it){
                it->print();
            }
        }
    void reformBlock();
    void AddPlacedLogics();
};
/*--------------  Operator class  ------------------*/
/*class MRPComparator{
public:
    bool operator () (MRP p1, MRP p2){
        if (p1.lengthSoFar == p2.lengthSoFar)
            return p1.lengthInBlock < p2.lengthInBlock;
        return p1.lengthSoFar < p2.lengthSoFar;
    }
};
typedef priority_queue< MRP, vector<MRP>, MRPComparator> min_heap_of_MRP;*/
class PairSecond{
public:
    bool operator () (pair<int,float>p1, pair<int,float>p2){
        if (p1.second == p2.second)
            return p1.first < p2.first;
        return p1.second > p2.second;
    }
};
/*--------------  Extern Variables  ------------------*/
/*blocks;
fluteBranch;
branch;*/
#endif

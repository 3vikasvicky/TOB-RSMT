#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <vector>
#include <map>
#include <set>
#include <list>
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

#include "param.h"
#include "BinaryHeap.h"
using namespace std;
using std::tr1::tuple;
using std::tr1::tie;
using std::tr1::make_tuple;
//DIRECTION used both in mazerouting and continousbuffering
enum DIRECTION {UNTOUCHED, RIGHT, LEFT, UP, DOWN};
//OMIT:Omit blockage, ADAPTIVE: The way used in 07 03 TCAD, BOB: My way of buffered over the block RSMT
enum BUFFERINGMODE {OMIT,ADAPTIVE, BOB};


class Poly;
class Edge;
class Point{
public:
    //verified point functions
    DTYPE x, y;
    Point (DTYPE a, DTYPE b);
    bool PointsOnSameLine (const Point&that)const{return abs(x-that.x)<EPSILON || abs(y-that.y)<EPSILON;}
    bool PointsOnSameHorizontalLine (const Point&that)const{return abs(y-that.y)<EPSILON;}
    bool PointsOnSameVerticalLine (const Point&that)const{return abs(x-that.x)<EPSILON;}
    bool IsInsidePoly(const Poly& block) const;//strictly inside. Only work on convex block
    bool IsOnEdge (const Edge& edge) const;//Include at two ends of the edge
    bool IsOnPoly (const Poly& poly) const;
    DTYPE DistanceToPoint (const Point& other)const;
    //return the shortest Manhatan distance to the edge. The edge might not has the projection of the point within
    DTYPE DistanceToEdge (const Edge& edge) const;
    bool operator == (const Point & that)const;
    bool operator != (const Point & that)const;
    Point operator + (const Point & that)const;
    Point operator - (const Point & that)const;
    Point operator * (double mul)const;
    Point operator / (double div)const;
    Point Projection (const Edge& e) const;
    bool IsProjectedIn (const Edge& e) const;
    bool IsProjectedOn (const Edge& e) const;


    Point (): x(0), y(0){}
    DTYPE distance (const Edge& e)const;//return the shortest Manhatan distance to e
    Point findClosestPoint (const Edge& e) const;
    bool isOutside (const Poly& poly) const;
    bool isEndpoint (const Edge& edge)const;
    void print () const{printf("(%f,%f)",(double)x, (double)y);}
};
class Edge{
public:
    DTYPE c_;        // verical - x, horizontal - y
    DTYPE c1_, c2_;  // vertical - y1, y2; horizontal - x1, x2. c1_ < c2_
    bool v_;       // vertical - true, horizontal - false
    //bool o_;       // open - true, close - false
    //verified
    Edge();
    Edge(const Point& p1, const Point& p2);
    bool IsSameLine(const Edge that)const;//Return true if two edges are in same line
    bool Overlap(const Edge that)const;//Return true if two edges are in same direction and have overlaps in between
    DTYPE Length()const;//Return the length of the edge
    bool IsIntersecting(const Edge& that) const;//Return true if it's a pure intersection between vertical and horizontaledge

    Point point1() const;//Return the left bottom point
    Point point2() const;//Return the up right point
};
class DirectionalEdge:public Edge{
    Point start_point_, end_point_;
public:
    DirectionalEdge(const Point& start_point, const Point& end_point);
    DirectionalEdge();
    DIRECTION Direction() const;//Return RIGHT/DOWN/LEFT/UP
    Point get_start_point() const;
    Point get_end_point() const;
    Point set_start_point(const Point&);
    Point set_end_point(const Point&);
    Point GetProportionPoint(double segment_portion)const;
};
class Poly{//any rectilinear shape
    vector<Point> points_;
    public:
    Poly(const vector<Point>& that);
    Poly();
    void PushBack (const Point& point);
    Point At(int i)const;//point index
    int Size() const ;
    Edge GetEdge(int i)const;
    bool IsAdjacent (const Poly& that) const;
    //This function will find the adjacent part of two poly. Can only handle non-hole situation
    //num_adjacent_edges is how many edges are overlapped(usually one), use pointers to change values
    int FindAdjacent (const Poly& that, int *index_pointA, int *index_pointC) const;
    Poly Cut(int point1, int point2);//Return a copy of part of current block, [point1, point2],both point1 and point2 are non-negative
    Poly operator + (const Poly& that)const;
    Poly operator = (const Poly& that);
    Poly operator += (const Poly& that);
    pair<Point,Point> GenerateBoundingBox()const;
    DTYPE GetOutsideLength(Point point1, Point point2, int step)const;
    int FindEdgeIndex(Point point)const;//In GetOutsideLength, find which edge the sink(root) is on
};
class Node{
public:
    int index_;
    int parent_;//parent index in the set of node
    set<int> children_;
    Point point_;
    double ct_;//Capacitance downstream
    set<int> get_children()const;
    void set_children(const set<int> & children);
};
#endif

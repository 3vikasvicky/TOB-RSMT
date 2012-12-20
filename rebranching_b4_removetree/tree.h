#ifndef TREE_H
#define TREE_H
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
#include "geometry.h"
#include "buffering.h"
using namespace std;
using std::tr1::tuple;
using std::tr1::tie;
using std::tr1::make_tuple;

class Node{
public:
    enum NodeType{//we can split NORMAL into sink/ee/
        NORMAL, ROOT, SINK, BUFFER
    };
    int index_;
    int parent_;//parent index in the set of node
    set<int> children_;
    Point point_;
    double ct_;//Capacitance downstream
    Buffer buffer_;
    NodeType type_;
    //Dangerous to use initialize a node without explictly about type,
    //but initially you don't know what's their types
    Node(NodeType = NORMAL);
    set<int> get_children()const;
    void set_children(const set<int> & children);
    void add_child(int child);
    void remove_child(int child);
};
class Tree{
    int root_;
    set<int> indexes_;//which branch node is in this tree. It include root, sinks and all internal steiner nodes
    //Actually are all branch edges in the tree. If this tree is one piece, it is indexes_ - root_. 
    //Otherwise branches < indexes_ - root_
    set<int> branches_;
    set<int> sinks_;
    Buffer root_buffer_;
public:
    Tree();
    Tree(int root);
    void set_indexes(const set<int>& indexes);
    set<int> get_indexes()const;
    void remove_index(int index);
    void insert_index(int index);
    void clear_indexes();
    void remove_sink(int sink_index);
    void insert_sink(int sink_index);
    void clear_sinks();
    void generate_branches();//generate branches from indexes_ and root_.It behaves like set_branches
    void set_branches(const set<int>&);
    set<int> get_branches()const;
    void remove_branch(int index);
    void insert_branch(int branch);
    void clear_branches();
    void set_root(int root);
    int get_root()const;
    Point get_root_point(const vector<Node> & nodes)const;
    set<int> get_sinks()const;
    void set_sinks(const set<int> &);
    Buffer get_root_buffer()const;

    void InitializeCap (vector<Node>&)const;//initialize 0 at branch point and Cb at sinks into nodes_
    void PropagateCap (vector<Node>&)const;
    void PropagateElmore(double delay, int node_index, const vector<Node>& nodes, map<int,double>& delay_map) const;
    //Include initialize cap, propagate cap and propagate elmore delay
    //Function returns a map of elmore delay,indexes of sinks. Only output is the map and change of nodes_
    map<int,double> CalculateElmore (vector<Node>&)const;
    map<int,double> CalculateElmore (vector<Node>&, const Solution&)const;
    double CalculateOutputSlew(const vector<Node> & nodes)const;//Need PropagateCap first

    DirectionalEdge FindFirstLShapeEndPoint(int sink_index, const vector<Node>& nodes)const;//Find the shape of the first L shape from sink to root. If step != 0, it must have at least one L shape. Return the second edge
    DirectionalEdge FindEdge(Point point, const vector<Node> &)const;//Find the edge in the tree that point is on
    void FindEdgeIndex(Point point, const vector<Node> &, int* parent_index, int * child_index)const;
    DTYPE CalculateBranchLength(int node_index, const vector<Node> & nodes)const;//Return distance between 
    DirectionalEdge GetBranchEdge (int node_index, const vector<Node> & nodes)const;
    DTYPE CalculatePathToRootLength(int sink_index, const vector<Node> & nodes) const;
    list<DirectionalEdge> FindPathToRoot(int node_index, const vector<Node> & nodes)const;
    DTYPE  CalculateTotalWirelength(const vector<Node> & nodes)const;

/*--------------  Node +- functions      ------------------*/
    //Let node to be the root of all parts(connected with node). 
    void Rootify(int node_index, vector<Node> & nodes, int real_parent_node_index);
    Node FindAndInsertNewNode(Point new_node_point, vector<Node>&);//Find a place for the point to be inserted
    Node InsertNewNode(int parent_node_index, int child_node_index, Point new_node_point, vector<Node>&);//To insert a new node between node_index and child_node_index by point new_node_point.Also update indexes
    //The break rule is always to minimize number of corners, node_vector will be changed.
    void BreakLShape(vector<Node>& node_vector);
    void StoreChildrenNode(vector<Node> & nodes)const;
    //Remove set child parent doesn't change index
    void RemoveChildNode(vector<Node>& nodes, int node_index, int child_node_index)const;
    void AddChildNode(vector<Node>& nodes, int node_index, int child_node_index)const;
    void SetParentNode(vector<Node>& nodes, int parent_node_index, int node_index)const;
    bool CleanNode(vector<Node>& nodes, int node_index);//If node is tail node or with degree 2 and not corner, clean it. Update indexes for sure
    bool CleanFloatingNode(vector<Node>& nodes, int node_index);//If node is tail node
    bool CleanMiddleNode(vector<Node>& nodes, int node_index);//Only work for tree within or totally without a block, otherwise will delete escaping point. If node is with degree 2 and not corner

};
class InsideTree:public Tree{
    int block_index_;
    set<int> root_down_outside_trees_;//outside trees which have their roots at the sinks of this inside tree
public:
    InsideTree();
    InsideTree(int root, int block_index);
    int get_block_index()const;
    DTYPE GetShorterOutsideLength()const;//Return the shorter outside length on block of two escaping points
    void insert_root_down_outside_tree(int outside_tree_index);
    set<int> get_root_down_outside_trees()const;
};
class OutsideTree:public Tree{
    set<int> root_down_inside_trees_;
public:
    OutsideTree();
    OutsideTree(int root);
    void insert_root_down_inside_tree(int inside_tree_index);
    set<int> get_root_down_inside_trees()const;
};

#endif

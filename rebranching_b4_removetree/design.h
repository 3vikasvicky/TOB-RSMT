#ifndef DESIGN_H
#define DESIGN_H
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

#include "param.h"
#include "buffering.h"
#include "tree.h"
#include "geometry.h"
#include "BinaryHeap.h"
using namespace std;
using std::tr1::tuple;
using std::tr1::tie;
using std::tr1::make_tuple;
/*****************************/
/*  User-Defined Parameters  */
/*****************************/

class Design{
    char work_path_[BUFFERSIZE], benchmark_path_[BUFFERSIZE], block_file_[BUFFERSIZE], branch_file_[BUFFERSIZE];
    DTYPE step_size_;//The linearize step length
    int root_; //the root for a whole Steiner Tree
    vector<Poly> blocks_;//have all block information stored
    vector<Poly> original_blocks_;//have all block information stored
    vector<Buffer> buffers_;

    DTYPE xmin_, xmax_, ymin_, ymax_;//use double because FLT_MAX and -FLT_MAX
    double slew_percentage_;
    double slew_spec_;//slew requirement of this Steiner Tree

    Tree tree_;//the outside blockage tree
    vector<Node> nodes_;
    set<int> sinks_;
    vector<InsideTree> inside_trees_;
    vector<OutsideTree> outside_trees_;
    map<int, int> inside_tree_map_;//root and inside tree index
    map<int, int> outside_tree_map_;


    vector<pair<int,int> > unionR;//branch index and itree(was step for reconnect function)
    vector<vector<bool> > blockMarks;
    vector<vector<DIRECTION> > directions;//direction from previos grid cross to (ixgrid, iygrid)

    Solution best_solution_;


public:
    Design (char work_path[], char benchmark_name[], double slew_percentage);
    Design ();
    void ReadBlockFile ();//Only read root and step size now
    void ReadBranchFile ();//Just read all branch information into nodes_ and set parent_
    //This will remove 0 length branch, sequence the tree, break L shape. Also store children, index, sinks information to tree
    void ReformBranch ();
    void PlotAndSave(const Tree&)const;
    void SortAndRecordSlew ();//sort slew in each inside tree, and record min and max for all
    Tree get_tree()const;
    vector<Node> get_nodes();
    void Clean();

    /*--------------  Second level functions      ------------------*/
    void InitialXYMinMax();//In Design constructor, initialize range of x and y
    void SetXYMinMax (DTYPE x,DTYPE y);//In ReadBranch, set new range of x and y according to branches
    set<int> RemoveZeroLengthBranch(vector<Node> &);//In ReformBranch, to remove 0 length branch resulted in flute
    void CompactNodes(int, set<int> valid_index, vector<Node>& nodes);
    void PlotAndSave(const vector<Node> & nodes)const;//generate line doc for gnu
    set<int> RecognizeSinks(vector<Node>&)const;//In ReformBranch, Find and store all sinks in Design
    //Change all pointers, which are pointing reverse way, to root. Only useful on the result of flute
    void InitialSequenceBranch (int root, vector<Node> &);//In ReformBranch, sequenfy the branches

    /*--------------  Buffering functions      ------------------*/
    void ReadBuffer ();
    void ContinuousBuffering(BUFFERINGMODE buffering_mode);
    map<int,vector<Solution> > BuildSinkSolutions()const;
    list<int> GeneratePostorderTraversal(int) const;
    vector<vector<Solution> > PropagateChildrenSolutions(int node_index, Point point, 
            BUFFERINGMODE buffering_mode, map<int,vector<Solution> > solution_map)const;
    vector<Solution> PropagateChildSolutions(int node_index, Point point, int child_index, 
            BUFFERINGMODE buffering_mode, vector<Solution> child_solutions, list<Point> points_list)const;
    vector<Solution> MergeSolutions(vector<vector<Solution> >, Point)const;
    void AddBufferAtSolutions(Point, vector<Solution> &)const;//Add minimum buffers at sinks at a inside tree
    void UpdateSolutions(Solution new_solution, vector<Solution> & solutions, int i, double slew_spec, Point point)const;

    map<int,vector< pair<Point, vector<Solution> > > > BuildPointSinkSolution(
            const map<int,vector<Solution> > &solution_map)const;
    void MultiToMultiChildrenPropagate(int node_index, vector<Point> points, 
            BUFFERINGMODE buffering_mode, map<int, vector< pair<Point,vector<Solution> > > >& solution_map)const;
    vector<Solution> MultiToOneChildPropagate(int node_index, Point point, int child_index, BUFFERINGMODE buffering_mode,
            const map<int, vector< pair<Point,vector<Solution> > > >& solution_map)const;
    vector<Solution> PointToPointChildPropagate(int node_index, Point point, int child_index, Point child_point,
            BUFFERINGMODE buffering_mode, const vector<Solution> &child_one_point_solutions)const;

    list<Point> FindPointsPath(Point child_point, Point point)const;
    void GenerateGrid(vector<DTYPE> & x_grids, vector<DTYPE>& y_grids, map<DTYPE,int>& x_grids_map, map<DTYPE,int>& y_grids_map, vector<vector<bool> >& block_marks, vector<vector<DIRECTION> >& direction_marks, vector<vector<double> >& length_marks, vector<vector<bool> >& outside_tree_marks)const;
    DTYPE CalculateDistanceToEnd(Point solution_point, list<Point> points_list)const;
    Point FindUpPointWithDistance(Point solution_point, list<Point> points_list, DTYPE distance)const;

    bool PointIsInBlocks(const Point & point, int* block_index_pointer) const;
    bool PointIsOnBlocks(const Point & point, int* block_index_pointer) const;
    vector<Point> FindUnblockedPoints(Point point, int block_index) const;

    void set_best_solution (const Solution & best_solution);
    /*--------------  Rebranch functions      ------------------*/
    void Improve();
    vector<Node> AddBufferNodes(const vector<Node> & nodes, const Solution & solution);
    void PropagateBufferNodes(int node_index, vector<Node>& nodes, const list<pair<Point,Buffer> > & buffer_locations);
    /*--------------  Timing/Slew functions  --------------------*/
    void InitializeCap (vector<Node>&)const;//initialize 0 at branch point and Cb at sinks into nodes_
    void PropagateCap (vector<Node>&)const;
    void PropagateElmore(double delay, int node_index, const vector<Node>& nodes, map<int,double>& delay_map) const;
    //Include initialize cap, propagate cap and propagate elmore delay
    //Function returns a map of elmore delay,indexes of sinks. Only output is the map and change of nodes_
    map<int,double> CalculateElmore (vector<Node>&)const;
};
#endif

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
#include<fstream>
#include <tr1/tuple>

#include "param.h"
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
    char work_path_[BUFFERSIZE], benchmark_path_[BUFFERSIZE], block_file_[BUFFERSIZE], branch_file_[BUFFERSIZE], benchmark_name_[BUFFERSIZE];
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


public:
    Design (char work_path[], char benchmark_name[], double slew_percentage);
    Design ();
    void ReadBlockFile ();
    void ReformBlock();//Combine just adjacent blocks
    //Add random logic blocks around existing blocks, not in use right now because of problem
    void AddPlacedLogics();
    void ReadBranchFile ();//Just read all branch information into nodes_ and set parent_
    //This will remove 0 length branch, sequence the tree, break L shape. Also store children, index, sinks information to tree
    void ReformBranch ();
    void ReadBuffer ();
    Tree RebuildTree();//tree_ will become the union of all outside tree and inside tree sinks(moved maybe). And it has correct branches
    void ContinuousBuffering(BUFFERINGMODE buffering_mode)const;
    void SplitBranch();
    void DumpTree(const Tree&)const;
    void PlotAndSave(const Tree&)const;
    void SortAndRecordSlew ();//sort slew in each inside tree, and record min and max for all
    set<int> Legitimate ();
    void MazeRouting(const set<int>& all_disconnected_nodes);
    Tree get_tree()const;
    Tree UniteInsideTrees()const;
    Tree UniteTreeAndInsideTrees()const;
    void Clean();
    void Write_BOBRSMT();

    /*--------------  Second level functions      ------------------*/
    void InitialXYMinMax();//In Design constructor, initialize range of x and y
    void SetXYMinMax (DTYPE x,DTYPE y);//In ReadBranch, set new range of x and y according to branches
    set<int> RemoveZeroLengthBranch();//In ReformBranch, to remove 0 length branch resulted in flute
    set<int> RecognizeSinks()const;//In ReformBranch, Find and store all sinks in Design
    //Change all pointers, which are pointing reverse way, to root. Only useful on the result of flute
    void InitialSequenceBranch (int root);//In ReformBranch, sequenfy the branches



    //In SplitBranch, try to use recursion algorithm to split.
    void SplitOutsideBranch(int node_index, int outside_tree_index, const set<int>&);
    void SplitInsideBranch(int node_index, int inside_tree_index, const set<int>&);
    //In Legitimate, find all pr to improve slew for one worst sink. Only consider ramp_slew deduction, assuming
    //output_slew fixed
    vector<vector<PossiblePoint> > FindAllChoices (int tree_index, set<int>& movable_sinks, 
            int worst_sink_index, double output_slew, map<int,double> slew_map)const;

    //In Legitimate, write the file for solver to read
    void WriteToSolver(const vector<vector<PossiblePoint> >& choices_all_sinks, double output_slew,
            double worst_ramp_slew)const;

    void CallSolver()const;
    //This will return all moved sinks, as well as adding index of node to disconnected_nodes if they are. disconnected_nodes will automatically remove duplicated ones since it is a set
    vector<PossiblePoint> ReadFromSolver(const vector<vector<PossiblePoint> >& all_choices, 
            set<int>& disconnected_nodes)const;

    void UpdateOneTree(InsideTree & inside_tree, const vector<PossiblePoint>& moved_sinks, 
            const set<int> &, set<int> &);

    //From MazeRouting function:

    template <class T>
    T UniteTreeVector(const vector<T>& trees)const;//Only operate indexes and branches. Not sinks and root

    /*--------------  Third level functions      ------------------*/
    //To find 5 points in ContinuousBuffering
    vector<Point> FindUnblockedPoints(Point point, int block_index) const;

    //In FindAllChoices,find step for current movable sink
    int GetStep(int inside_tree_index, int sink_index)const;
    //In FindAllChoices, add combination pr
    PossiblePoint AddCombinationChoice (int sink_index, int worst_sink_index, DTYPE distance_to_next_sink, double output_slew, map<int,double> slew_map, int inside_tree_index)const;
    //Build a fake new_inside_tree to get all values
    PossiblePoint AddOneSlideChoice(int inside_tree_index,Point new_sink_point,Point new_parent_point, int sink_index, int worst_sink_index, DTYPE distance_to_next_sink, double output_slew, map<int,double> slew_map)const;

    //In FindAllChoices, return the edge index of the block, which the terminal is on
    int GetBlockEdgeIndex (int block_index, int sink_index)const;
    int FindClosestTerminalOnBlockBoundary (int inside_tree_index, int terminal_index, DTYPE *distance_pointer)const;
    //In FindAllChoices, return the index of next sink or root according to step, and store the total distance
    int FindNextTerminalOnBlockBoundary (int inside_tree_index, int step, int sink_index, DTYPE *distance_pointer)const;
    /*--------------  Fourth level functions      ------------------*/
    void UpdateOneNode(InsideTree& inside_tree, vector<Node>& nodes, Point new_sink_point,Point new_parent_point, int sink_index)const;
    /*--------------  All over functions      ------------------*/
    //Judge if a node is in any blocks, have both version of set block index or not
    bool PointIsInBlocks(const Point & point, int* = NULL)const;
    bool PointIsOnBlocks(const Point & point, int* = NULL)const;
    bool IsInSameInsideTree(int node1_index, int node2_index, int* inside_tree_index_pointer = NULL)const;
    /*--------------  Mazerouting functions      ------------------*/
    void GenerateGrid(vector<DTYPE> & x_grids, vector<DTYPE>& y_grids, map<DTYPE,int>& x_grids_map, 
            map<DTYPE,int>& y_grids_map, vector<vector<bool> >& block_marks, 
            vector<vector<DIRECTION> >& direction_marks, vector<vector<double> >& length_marks, 
            vector<vector<bool> >& outside_tree_marks)const;
    void MarkOutsideTree(map<DTYPE,int>& x_grids_map, map<DTYPE,int>& y_grids_map, 
            vector<vector<bool> >& outside_tree_marks)const;

    //Mark all node downstream with 0 length, and put most directly connected outside tree into heap
    void MarkGridDownFromNode(int node_index, int current_tree_level, const map<DTYPE,int>& x_grids_map,
            const map<DTYPE,int>& y_grids_map, vector<vector<double> >& length_marks, 
            multimap<double, MRP>& m_heap) const;
    //If block_factor is very small, it is for MazeRouting, which will choose one with less use of block
    //If block_factor is very large, it is for FindPointsPath, which will choose block avoiding path
    MRP FindPath (double block_factor, multimap<double, MRP>& m_heap, const vector<DTYPE> & x_grids, 
            const vector<DTYPE>& y_grids, const vector<vector<bool> >& block_marks, 
            vector<vector<DIRECTION> >& direction_marks,vector<vector<double> >& length_marks, 
            const vector<vector<bool> >&)const;

    void TraceBack(const MRP& end_mrp, const vector<DTYPE> & x_grids, const vector<DTYPE>& y_grids,
            const map<DTYPE,int>& x_grids_map, const map<DTYPE,int>& y_grids_map, 
            const vector<vector<DIRECTION> >& direction_marks, vector<vector<bool> >& outside_tree_marks);

    void ClearGridMarksAndDirections(vector<vector<DIRECTION> >& direction_marks, 
            vector<vector<double> >& length_marks)const;
    /*--------------  Buffering functions      ------------------*/
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
    Point FindUpPointWithDistance(Point solution_point, list<Point> points_list, DTYPE distance)const;
    DTYPE CalculateDistanceToEnd(Point solution_point, list<Point> points_list)const;

};
#endif

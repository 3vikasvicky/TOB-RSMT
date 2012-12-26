#include "design.h"

using namespace std;
/*things to do:
  1. In Addoneslidenode, inside tree may include one edge which is at the boundary. Make sure it will not be chosen, or deliminate it
  2. ADAPTIVE mode still have problem. If one edge is from child node to one of candidates of current node, it is not straight
  3. If have time, work on block_in_length update. One situation that the edge go cross the block is not considered
  4. Remove dangling edges after all. It can be a root of an empty inside tree. It can be an EP of an inside_tree, which is also a root of an outside tree
*/
int main(int argc, char *argv[]){
    char workPath[BUFFERSIZE], benchmarkName[BUFFERSIZE];
    char benchmarkPath[BUFFERSIZE], blockFile[BUFFERSIZE], branchFile[BUFFERSIZE];

    if(argc != 4 && argc != 3) {
        printf("Usage: %s <work_dir> <benchmark_name> <slew>\n", argv[0]);
        printf("    <work_dir> is the work directory, which include benchmarks. It's optional. If work diretory is current directory, it can be omitted\n");
        printf("    <benchmark_name> is the name of benchmark.\n");
        printf("    <slew> is the slew constraint, e.g. 65.\n");
        exit(1);
    }    

    float slew = atof(argv[3]);
    Design design(argv[1], argv[2], slew);

    cout<<"\n-------------init data-----------\n";
    design.ReadBlockFile(); 
//    design.ReformBlock();//This will combine adjacent blocks
    //design.AddPlacedLogics();//This will randomly add new squares around existing blocks
    design.ReadBranchFile ();
    design.ReformBranch();
    design.ReadBuffer();
    design.SetRootBuffer(design.get_nodes());
    design.SortAndRecordSlew(design.get_nodes());//Just to generate slew_spec_ now
    /*
    cout<<"\n-------------omit block dense candidates mode buffering-----------\n";
    design.ContinuousBuffering(OMIT_DENSE, design.get_nodes());//More candidates
    cout<<"\n-------------Plot the current tree-----------\n";
    design.PlotAndSave(design.get_nodes());
    */
    cout<<"\n-------------omit block least candidates mode buffering-----------\n";
    design.ContinuousBuffering(OMIT, design.get_nodes());//Irrespective to block. It's better turned into a tree function
    cout<<"\n-------------Plot the current tree-----------\n";
    design.PlotAndSave(design.get_nodes());
    cout<<"\n-------------Before improving the timing-----------\n";
    vector<Node> buffered_nodes = design.AddBufferNodes(design.get_nodes(), design.get_best_solution());
    design.InitializeCap(buffered_nodes);
    design.PropagateCap(buffered_nodes);
    design.PrintTiming (buffered_nodes, 0);//Period set to 0

    cout<<"\n-------------Improve the timing now-----------\n";
    vector<Node> new_nodes = design.Improve();
    cout<<"\n-------------omit block least candidates mode buffering-----------\n";
    design.ContinuousBuffering(OMIT, new_nodes);//Irrespective to block. It's better turned into a tree function

    cout<<"\n-------------After improving the timing-----------\n";
    vector<Node> new_buffered_nodes = design.AddBufferNodes(new_nodes, design.get_best_solution());
    design.InitializeCap(new_buffered_nodes);
    design.PropagateCap(new_buffered_nodes);
    design.PrintTiming (new_buffered_nodes, 0);//Period set to 0
    cout<<"\n-------------Plot the current tree-----------\n";
    design.PlotAndSave(new_buffered_nodes);
   // design.PlotAndSave(design.get_tree());
/*    design.SplitBranch ();
    design.RebuildTree();
    design.PlotAndSave(design.UniteTreeAndInsideTrees());
    //design.PlotAndSave(design.UniteInsideTrees());
    design.SortAndRecordSlew();

    clock_t omit_mode_buffering_start = clock();
    design.ContinuousBuffering(OMIT);//Need slew spec from SortAndRecordSlew
    clock_t omit_mode_buffering_end = clock();
    double omit_mode_buffering_cpu_time= (omit_mode_buffering_end - omit_mode_buffering_start)/ (double)(CLOCKS_PER_SEC);
    cout<< "omit_mode_buffering_cpu_time is "<< omit_mode_buffering_cpu_time<< endl;

    clock_t adaptive_mode_buffering_start = clock();
    design.ContinuousBuffering(ADAPTIVE);//Need slew spec from SortAndRecordSlew
    clock_t adaptive_mode_buffering_end = clock();
    double adaptive_mode_buffering_cpu_time= (adaptive_mode_buffering_end - adaptive_mode_buffering_start)/ (double)(CLOCKS_PER_SEC);
    cout<< "adaptive_mode_buffering_cpu_time is "<< adaptive_mode_buffering_cpu_time<< endl;

    clock_t legitimate_start = clock();
    set<int> all_disconnected_roots = design.Legitimate();
    clock_t legitimate_end = clock();
    double legitimate_cpu_time= (legitimate_end - legitimate_start)/ (double)(CLOCKS_PER_SEC);
    cout<< "legitimate_cpu_time is "<< legitimate_cpu_time<< endl;

    design.PlotAndSave(design.get_tree());
    design.PlotAndSave(design.UniteInsideTrees());
    design.RebuildTree();//tree_ will contain moved sinks of inside_trees_
    //design.PlotAndSave(design.get_tree());
    //design.PlotAndSave(design.UniteInsideTrees());

    clock_t maze_routing_start = clock();
    design.MazeRouting(all_disconnected_roots);
    clock_t maze_routing_end = clock();
    double maze_routing_cpu_time= (maze_routing_end - maze_routing_start)/ (double)(CLOCKS_PER_SEC);
    cout<< "maze_routing_cpu_time is "<< maze_routing_cpu_time<< endl;

    design.Clean();
    design.PlotAndSave(design.get_tree());
    design.PlotAndSave(design.UniteInsideTrees());
    design.PlotAndSave(design.UniteTreeAndInsideTrees());

    clock_t bob_mode_buffering_start = clock();
    design.ContinuousBuffering(BOB);//Need slew spec from SortAndRecordSlew
    clock_t bob_mode_buffering_end = clock();
    double bob_mode_buffering_cpu_time= (bob_mode_buffering_end - bob_mode_buffering_start)/ (double)(CLOCKS_PER_SEC);
    cout<< "bob_mode_buffering_cpu_time is "<< bob_mode_buffering_cpu_time<< endl;

    cout<< "Done"<< endl;
*/
    /*design.clean();//clear unionR and set ischanging to false
    design.plotAndSaveTree(-1);

    float insideWL = design.getInsideWL();
    pair<float,float> WLpair = design.getOutsideWL();
    cout<< "OutsideWL is "<< WLpair.second<<", InsideWL is "<< WLpair.first+insideWL<< ", total is "<< WLpair.first + WLpair.second + insideWL<< endl;
    design.uniteAllTrees();
    design.plotAndSaveTree(-1);//plot treeVect[0]
    */
}

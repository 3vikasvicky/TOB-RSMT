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
    design.PrintCapCt(design.get_nodes());
    /*
    cout<<"\n-------------omit block dense candidates mode buffering-----------\n";
    design.ContinuousBuffering(OMIT_DENSE, design.get_nodes());//More candidates
    cout<<"\n-------------Plot the current tree-----------\n";
    design.PlotAndSave(design.get_nodes());
    */
    cout<<"\n-------------omit block least candidates mode buffering-----------\n";
    //design.ContinuousBuffering(OMIT, design.get_nodes());//Irrespective to block. It's better turned into a tree function
    design.ContinuousBuffering(BOB, design.get_nodes());//Irrespective to block. It's better turned into a tree function
    cout<<"\n-------------Plot the current tree-----------\n";
    //design.PlotAndSave(design.get_nodes());
    cout<<"\n-------------Before improving the timing-----------\n";
    vector<Node> buffered_nodes = design.AddBufferNodes(design.get_nodes(), design.get_best_solution());
    design.InitializeCap(buffered_nodes);
    design.PropagateCap(buffered_nodes);
    design.PrintTiming (buffered_nodes, 0);//Period set to 0
    cout<< "Yilin totoal WL is "<< (double)design.CalculateTotalWirelength(buffered_nodes)<< endl;
}

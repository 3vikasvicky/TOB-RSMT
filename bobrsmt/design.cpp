#include "design.h"

using namespace std;

/*---------------------------  Design functions   -----------------------------*/
Design::Design (char work_path[], char benchmark_name[], double slew){
    strcat(benchmark_name_, benchmark_name);
    strcpy(work_path_, work_path);

    strcpy(benchmark_path_, work_path);
    strcat(benchmark_path_, "/../benchmarks/");
    strcat(benchmark_path_, benchmark_name);

    strcpy(block_file_, benchmark_name);
    strcat(block_file_, ".pblock");

    //strcpy(branch_file_, "flute.branch");
    //strcpy(branch_file_, "OARSMT.out");
    strcpy(branch_file_, benchmark_name);
    strcat(branch_file_, ".branch");

    slew_spec_ = slew;
    InitialXYMinMax();
}
Design::Design (){
    InitialXYMinMax();
}
void Design::ReadBlockFile (){
    char file_name[BUFFERSIZE];//find the name of block file
    strcpy(file_name, benchmark_path_);
    strcat(file_name, "/");
    strcat(file_name, block_file_);
    
    FILE *fp;
    if((fp=fopen(file_name, "r"))==NULL) {
        printf("roo: Cannot open %s file", file_name);
    }
   
    char line[LINESIZE];
    *line = '\0';
    fgets(line, LINESIZE, fp);
    char temp[BUFFERSIZE];
    sscanf(line, "%s\t%*s\n", temp);
    sscanf(line, "DeltaX\t:\t%d\n", &step_size_);

    fgets(line, LINESIZE, fp);
    sscanf(line, "%s\t%*s\n", temp);
    sscanf(line, "Root\t:\t%d\n", &root_);
    tree_.set_root(root_);

    fgets(line, LINESIZE, fp);
    sscanf(line, "%s\t%*s\n", temp);
    //sscanf(line, "SlewSpec\t:\t%lf\n", &slew_spec_); This only for assigned slew

    //sink information here is actually useless. We can find all sinks in flute.branch
    int num_sinks;
    fscanf(fp, "%d\n", &num_sinks);
    for(unsigned int i=0;i<num_sinks;i++){
        Point sink;
        fscanf(fp,"%d",&sink.x);
        fscanf(fp,"%d",&sink.y);
    }

    int num_blocks;
    fscanf(fp, "%d\n", &num_blocks);
    for(unsigned int i = 0; i < num_blocks; i++){
        fgets(line, LINESIZE, fp);
        char *pch = strtok (line, " \t");
        bool reading_x = true;//indicating it's reading x coordinate
        Poly  block;
        Point branch_corner;
        while (pch != NULL){
            if (reading_x) {
                branch_corner.x = (DTYPE)(atof(pch));
                xmin_ = min(xmin_, branch_corner.x);
                xmax_ = max(xmax_, branch_corner.x);
            }
            else  {
                branch_corner.y = (DTYPE)(atof(pch));
                ymin_ = min(ymin_, branch_corner.y);
                ymax_ = max(ymax_, branch_corner.y);
                block.PushBack(branch_corner);
            }
            reading_x = !reading_x;
            pch = strtok (NULL, " \t");
        }
        blocks_.push_back(block);
    }
    original_blocks_ = blocks_;
    fclose (fp);
}

void Design::ReformBlock(){//This function keep joining overlapping edges in two adjacent blocks. It can only deal with non-hole situation now
    int startA, startC, length;//startA is the index of A, startC is the index of C. startA+length is the index of B, startC+length is the index of D
    for (unsigned int i = 0; i < blocks_.size(); ++i){//size is changing by removing element
        Poly block1 = blocks_[i];
        bool flag = true; //flag is to indicate wether block1 is combined with any other block or not. If combined, the new shape could be adjacent to previous non-adjacent block
        while (flag){
            flag = false;
            for (int j = i+1; j < blocks_.size();++j){
                Poly block2 = blocks_[j];
                if (block1.IsAdjacent(block2)){
                    //block1.Plot();
                    //block2.Plot();
                    int index_pointA, index_pointC;
                    int num_adjacent_edges = block1.FindAdjacent(block2, &index_pointA, &index_pointC);
                    int index_pointB = (index_pointA + num_adjacent_edges) % block1.Size();
                    int index_pointD = (index_pointC + num_adjacent_edges) % block2.Size();
                    vector<Point> edgeAD, edgeCB;
                    edgeAD.push_back(block1.At(index_pointA));
                    if (block1.At(index_pointA) != block2.At(index_pointD)){//if AD(CB) is longer than 0
                        edgeAD.push_back(block2.At(index_pointD));
                    }
                    edgeCB.push_back(block2.At(index_pointC));
                    if (block2.At(index_pointC) != block1.At(index_pointB)){
                        edgeCB.push_back(block1.At(index_pointB));
                    }
                    Poly polyAD (edgeAD);
                    Poly polyCB (edgeCB);
                    Poly new_block1;
                    new_block1 = block1.Cut(0, index_pointA-1) + polyAD;
                    if (index_pointD == 0)//index_pointD == 0 is special
                        new_block1 += block2.Cut(index_pointD+1, index_pointC-1);
                    else
                        new_block1 += block2.Cut(index_pointD+1, block2.Size()-1) + block2.Cut(0, index_pointC-1);
                    new_block1 += polyCB;
                    if (index_pointB != 0){
                        new_block1 += block1.Cut(index_pointB+1, block1.Size()-1);
                    }else{
                        new_block1 = new_block1.Cut(0, new_block1.Size()-2);//Remove the duplicate last one
                    }
                    blocks_[i] = new_block1;
                    block1 = blocks_[i];
                    blocks_.erase(blocks_.begin()+j);
                    --j;
                    flag = true;
                }
            }
        }
    }

}
void Design::AddPlacedLogics(){//This will randomly add new squares around existing blocks
    for (unsigned int i = 0; i < blocks_.size(); ++i){//size is changing by removing element
        Poly block1 = blocks_[i];
        Poly new_block1;
        for (int j = 0; j < block1.Size(); ++j){//for each edge in block1
            Edge current_edge = block1.GetEdge(j);
            Point p1= block1.At ((j+block1.Size()-1)%block1.Size());//p2 is the start of current edge and p3 is the end. p1 is the start point of previous edge. p1 is used to get the shape of new placed logics
            Point p2= block1.At (j);
            Point p3= block1.At ((j+1)%block1.Size());
            srand(time(0));
            double random_pos = rand()/(double)RAND_MAX;
            double random_height = rand()/(double)RAND_MAX;
            double random_length = rand()/(double)RAND_MAX;
            Point p4 = p2 + (p3-p2)*random_pos;//p4, p5, p6, p7
            Point p5 = p4 + (p2-p1)*random_height;
            Point p6 = p5 + (p3-p2)*random_length;
            Point p7 = p4 + p6 - p5;
            assert(p4!=p5 && p5!=p6 && p6!= p7);
            Poly random_block;
            random_block.PushBack(p4);
            random_block.PushBack(p5);
            random_block.PushBack(p6);
            random_block.PushBack(p7);
            new_block1.PushBack(p2);
            if (!PointIsInBlocks(p5) && !PointIsInBlocks(p6)){// && !ContainSinkInBlock(random_block)){
                new_block1.PushBack(p4);
                new_block1.PushBack(p5);
                new_block1.PushBack(p6);
                new_block1.PushBack(p7);
            }
        }
        blocks_[i] = new_block1; 
    }
}

void Design::ReadBranchFile (){
    char file_name[BUFFERSIZE];
    strcpy(file_name, benchmark_path_);
    strcat(file_name, "/");
    strcat(file_name, branch_file_);
    
    FILE *fp;
    if((fp=fopen(file_name, "r"))==NULL) {
        printf("roo: Cannot open %s file", file_name);
    }
    
    while(!feof(fp)) {
        char line[LINESIZE];
        *line = '\0';
        fgets(line, LINESIZE, fp);
        char temp[BUFFERSIZE];
        sscanf(line, "%s\t%*s\n", temp);
        
        if( strlen(line)<5 || temp[0] == '#'||strcmp(temp,"DegreeNumber")==0){//Doesn't care
            continue;
        }
        int current_index, next_index;
        Point point;
        sscanf(line, "%d\t%d\t%d%d\n", &current_index, &point.x, &point.y, &next_index);
        Node node;
        node.parent_ = next_index;
        node.index_ = current_index;
        node.point_ = point;
        assert (nodes_.size() == current_index);
        nodes_.push_back(node);
        SetXYMinMax (point.x,point.y);
    }
    fclose (fp);
}
void Design::ReformBranch (){
    set<int> index = RemoveZeroLengthBranch();
    /*--------- Here is make a sequence----*/
    InitialSequenceBranch(root_);//This is not universal for tree, it is only effective for flute result
    /*--------- Here to set all sinks according to sequenfied relations----*/
    tree_.set_indexes (index);
    tree_.generate_branches();
    sinks_ = RecognizeSinks();
    tree_.set_sinks (sinks_);
    /*--------- split L shape branchs----*/
    tree_.BreakLShape(nodes_);//This is tree function because it can be used in tree in any situation
    tree_.StoreChildrenNode(nodes_);//This is tree function because children is a concept in tree
}

void Design::ReadBuffer (){//from small to big ones
    //double ratio = (double)CalculateTotalWirelength(nodes)/140;//to test02
    double ratio = 1;
    double input_capacitance = Cb*ratio;
    double driving_resistance = Rs*ratio;
    double intrinsic_delay = Db*ratio*ratio;
    double slew_resistance = Ss*ratio;
    double intrinsic_slew = Sb*ratio*ratio;
    double cost = 1;//*ratio??
    Buffer buffer(input_capacitance, driving_resistance, intrinsic_delay, slew_resistance, intrinsic_slew, cost);
    buffers_.push_back(buffer);

    //slew_spec_ = 7200*ratio*ratio;//7200 is the golden slew for test02
}
void Design::SplitBranch(){
    OutsideTree outside_tree(root_);
    outside_trees_.push_back(outside_tree);
    outside_tree_map_.insert(pair<int,int> (outside_tree.get_root(), outside_trees_.size()-1));
    set<int> root_children = nodes_.at(root_).get_children();//Some children may on one inside tree others may on the other outside tree if one branch is in same line to a boundary
    SplitOutsideBranch(root_, outside_trees_.size()-1, root_children); 
}
void Design::DumpTree(const Tree & tree)const{
    FILE * fp;
    //string str("/home/polaris/yzhang1/rebranching/bobrsmt/output/BOBRSMT.out");
    string str("./output/BOBRSMT.out");
    if( (fp=fopen(str.c_str(),"w")) == NULL ) {
        printf("bookshelf_IO: Cannot open: %s file for write", str.c_str());
    }

    set<int> indexes = tree.get_indexes();
    fprintf (fp, "DegreeNumber : %d\n", (int)(indexes.size()));
    for (set<int>::const_iterator it = indexes.begin(); it != indexes.end();it++) {
        int node_index = *it;
        Node node = nodes_.at(node_index);
        Node parent_node = nodes_.at(node.parent_);
        if (it != indexes.begin()){
            fprintf (fp, "\n");
        }
        fprintf(fp, "%d %d %d %d", 
                node_index, (int)node.point_.x,(int)node.point_.y, parent_node.index_);
    }
    fclose (fp);
}
void Design::PlotAndSave(const Tree& tree)const{//generate line doc for gnu
    return;
    set<int> indexes = tree.get_indexes();
    set<int> branches = tree.get_branches();
    if (branches.size() == 0)
        return;//Nothing to do with an empty tree
    static int count = -1;
    count++;//Every time plotTree got called, count+1. Then we can save picture with different name

    FILE * fp = popen ("gnuplot -persist", "w");
    //Plot the current tree edge
    DTYPE plot_x_min = INT_MAX, plot_x_max = 0, plot_y_min = INT_MAX, plot_y_max = 0;
    for (set<int>::const_iterator it = branches.begin(); it != branches.end(); it++) {
        int branch_index = *it;
        Node node = nodes_.at(branch_index);
        Node parent_node = nodes_.at(node.parent_);
        fprintf(fp, "set arrow from %f,%f to %f,%f nohead\n",(double)node.point_.x,(double)node.point_.y, 
                (double)parent_node.point_.x, (double)parent_node.point_.y);


        plot_x_min = min(plot_x_min, min(node.point_.x, parent_node.point_.x));
        plot_x_max = max(plot_x_max, max(node.point_.x, parent_node.point_.x));
        plot_y_min = min(plot_y_min, min(node.point_.y, parent_node.point_.y));
        plot_y_max = max(plot_y_max, max(node.point_.y, parent_node.point_.y));
    }
    //Only plot the range around this tree. It will include all blocks in this range
    fprintf (fp, "set xrange [ %f : %f ]\n", 
            (double)plot_x_min-0.2*(plot_x_max-plot_x_min)-30, (double)plot_x_max+0.2*(plot_x_max-plot_x_min)+30.0);//30 is for test04
    fprintf (fp, "set yrange [ %f : %f ]\n",
            (double)plot_y_min-0.2*(plot_y_max-plot_y_min)-30, (double)plot_y_max+0.2*(plot_y_max-plot_y_min)+30);
    for (vector<Poly>::const_iterator it2 = blocks_.begin(); it2 != blocks_.end(); ++it2){
        Poly block = *it2;
        fprintf (fp, "set object %d polygon from ", 1 + (int) (it2-blocks_.begin()));//from 1
        for (unsigned int i=0; i<block.Size();++i){
            Point point = block.At(i);
            fprintf (fp, "%f,%f", (double)point.x, (double)point.y);
            fprintf (fp," to ");
        }
        assert (block.Size() > 0);//Block with size 0 shouldn't exist
        if (block.Size() > 0){
            Point start_point = block.At(0);
            fprintf (fp, " %f,%f fs pattern 1 bo 2 fc rgbcolor \"cyan\"\n",
                    (double)start_point.x, (double)start_point.y);//to close the Polygon
        }
    }
    //Plot node in blue and root in red
    //This one use plot function, so it has to be done at last
    for (set<int>::const_iterator it = indexes.begin(); it != indexes.end();) {
        if (it == indexes.begin())
            fprintf (fp, "plot ");
        int node_index = *it;
        Node node = nodes_.at(node_index);
        Node parent_node = nodes_.at(node.parent_);
        if (node_index == root_){
            fprintf(fp, "\"<echo '%f %f'\" notitle with points pointtype 7 pointsize 1 linecolor rgb \"red\"", 
                    (double)node.point_.x,(double)node.point_.y);
        }else if (sinks_.find(node_index) != sinks_.end()){
            fprintf(fp, "\"<echo '%f %f'\" notitle with points pointtype 4 pointsize 1 linecolor rgb \"green\"",
                    (double)node.point_.x,(double)node.point_.y);
        }else{
            fprintf(fp, "\"<echo '%f %f'\" notitle with points pointtype 4 pointsize 1 linecolor rgb \"blue\"",
                    (double)node.point_.x,(double)node.point_.y);
        }
        it++;
        if (it != indexes.end()){
            fprintf (fp, ", ");
        }else{
            fprintf (fp, "\n");
        }
    }
    //Save to file
    fprintf (fp, "set terminal png\n");  
    fprintf (fp, "set output \"%s/my_plot_%d.png\"\n", benchmark_path_, count);//gthumb Erint
    //Plot on screen
    fprintf (fp, "replot\n");
    fprintf (fp, "set terminal x11\n");
    fprintf (fp, "pause -1\n");
    pclose (fp);

}
Tree Design::RebuildTree(){
    Tree tree = UniteTreeVector(outside_trees_);
    set<int> indexes = tree.get_indexes();
    for (unsigned int i =0; i < inside_trees_.size(); i++){//This is needed since right now inside and outside tree are not fully connected
        InsideTree inside_tree = inside_trees_.at(i);
        set<int> inside_tree_sinks = inside_tree.get_sinks(); 
        for (set<int>::const_iterator it = inside_tree_sinks.begin(); it != inside_tree_sinks.end(); it++){
            int sink_index = *it;
            indexes.insert(sink_index);
        }
    }
    tree_.set_indexes(indexes);
    tree_.set_branches(tree.get_branches());
    return tree_;
}
void Design::ContinuousBuffering(BUFFERINGMODE buffering_mode)const{
    map<int,vector<Solution> > solution_map = BuildSinkSolutions();
    map<int,vector< pair<Point, vector<Solution> > > > point_solution_map = BuildPointSinkSolution(solution_map);
    list<int> postorder_traversal = GeneratePostorderTraversal(root_);
    /*cout<< "All postorder traversal nodes are:"<< endl;
    for (list<int>::const_iterator it = postorder_traversal.begin(); it != postorder_traversal.end(); it++){
        cout<< *it<< " ";
    }
    cout<< endl;*/
    for (list<int>::const_iterator it = postorder_traversal.begin(); it != postorder_traversal.end(); it++){
        int node_index = *it;
        Node node = nodes_.at(node_index);
        if (sinks_.find(node_index) != sinks_.end() && node.get_children().size() == 0){//If is sink and at end
            continue;
        }
        Point point = node.point_;
        if (buffering_mode == OMIT || buffering_mode == BOB){
            vector<vector<Solution> > all_children_propagated_solutions = 
                PropagateChildrenSolutions(node_index, point, buffering_mode, solution_map);
            vector<Solution> merged_solutions = MergeSolutions(all_children_propagated_solutions, point);
            if (buffering_mode == BOB){
                AddBufferAtSolutions(point, merged_solutions);
            }
            solution_map.insert(pair<int,vector<Solution> >(node_index,merged_solutions));
        }else if (buffering_mode == ADAPTIVE){
            int block_index;
            vector<Point> points;
            if (PointIsInBlocks(point,&block_index)){
                points = FindUnblockedPoints(point, block_index);//original, left, top, right, bottom
            }else{
                points.push_back(point);
            }
            MultiToMultiChildrenPropagate(node_index, points, buffering_mode, point_solution_map);
        }
    }
    if (buffering_mode == OMIT || buffering_mode == BOB){
        vector<Solution> root_solutions = solution_map.find(root_)->second;
        cout<< "Under "<< buffering_mode<< " mode, Root has "<< root_solutions.size()<< " solutions"<<endl;
        cout<< "They have c, w, s as:"<< endl;
        for (unsigned int i = 0; i < root_solutions.size(); i++){
            Solution root_solution = root_solutions.at(i);
            cout << root_solution.get_c()<< " "<< root_solution.get_w()<<" " << root_solution.get_s()<< endl;
        }
    }else{
        vector< pair<Point, vector<Solution> > > root_points_solutions = point_solution_map.find(root_)->second;
        vector<Solution> root_solutions = root_points_solutions.at(0).second;
        cout<< "Under "<< buffering_mode<< " mode, Root has "<< root_solutions.size()<< " solutions"<<endl;
        cout<< "They have c, w, s as:"<< endl;
        for (unsigned int i = 0; i < root_solutions.size(); i++){
            Solution root_solution = root_solutions.at(i);
            cout << root_solution.get_c()<< " "<< root_solution.get_w()<<" " << root_solution.get_s()<< endl;
        }
    
    }

}
void Design::SortAndRecordSlew (){
    double max_slew = 0, min_slew = INT_MAX;
    for (unsigned int i = 0; i < inside_trees_.size(); ++i){
        inside_trees_[i].InitializeCap(nodes_);
        inside_trees_[i].PropagateCap(nodes_);
        //double output_slew = inside_trees_[i].CalculateOutputSlew(nodes_);
        double output_slew = BETA*slew_spec_;
        map<int,double> elmore_map = inside_trees_[i].CalculateElmore(nodes_);
        map<int,double> slew_map;
        for (map<int,double>::const_iterator it = elmore_map.begin(); it != elmore_map.end(); it++){
            double ramp_slew = it->second*log(9);
            double input_slew = sqrt(ramp_slew*ramp_slew + output_slew*output_slew);
            slew_map.insert(pair<int,double>(it->first,input_slew));
            min_slew = min (min_slew, input_slew);
            max_slew = max (max_slew, input_slew);
        }
    }
    //slew_spec_ = min_slew + slew_percentage_*(max_slew-min_slew);
    //length_limit_ = ( -(RUNIT*Cb+Rs*CUNIT)+sqrt(pow((RUNIT*Cb+Rs*CUNIT),2) - 2*RUNIT*CUNIT*(Rs*Cb-slew_spec_)) ) / (RUNIT*CUNIT);
    cout<< "The min slew is "<< min_slew<< ". The max slew is "<< max_slew<< ". The slew spec is "<< slew_spec_<< endl;
}
set<int> Design::Legitimate (){
    set<int> all_disconnected_roots;
    for (unsigned int i = 0; i < inside_trees_.size(); ++i){
        set<int> all_combined_sinks;//node index and itree(needed for reconnect function)
        set<int> movable_sinks = inside_trees_[i].get_sinks();
        while (!movable_sinks.empty()){
            inside_trees_[i].InitializeCap(nodes_);
            inside_trees_[i].PropagateCap(nodes_);
            //double output_slew = inside_trees_[i].CalculateOutputSlew(nodes_);
            double output_slew = BETA*slew_spec_;
            map<int,double> elmore_map = inside_trees_[i].CalculateElmore(nodes_);
            map<int,double> slew_map;
            int worst_sink_index;
            double worst_sink_slew = 0;
            for (map<int,double>::const_iterator it = elmore_map.begin(); it != elmore_map.end(); it++){
                double ramp_slew = it->second*log(9);
                double sink_slew = sqrt(ramp_slew*ramp_slew + output_slew*output_slew);
                int sink_index = it->first;
                slew_map.insert(pair<int,double>(sink_index, sink_slew));
                if (sink_slew > worst_sink_slew){
                    worst_sink_index = it->first;
                    worst_sink_slew = sink_slew;
                }
            }
            //if the worst slew sink is better than required, go to next inside tree
            if (worst_sink_slew <= slew_spec_){
                break;
            }
            vector<vector<PossiblePoint> > all_choices = FindAllChoices(i, movable_sinks, worst_sink_index, output_slew, slew_map);
            double worst_ramp_slew = sqrt (worst_sink_slew*worst_sink_slew - output_slew*output_slew);
            WriteToSolver(all_choices, output_slew, worst_ramp_slew);
            CallSolver();
            //slided_sinks is for updating tree.
            set<int> combined_sinks;
            vector<PossiblePoint> slided_sinks = ReadFromSolver(all_choices, combined_sinks);
            UpdateOneTree(inside_trees_[i], slided_sinks, combined_sinks, all_disconnected_roots);
        }
//        PlotAndSave(inside_trees_[i]);
    }
    return all_disconnected_roots;
}
void Design::MazeRouting(const set<int> & all_disconnected_roots){
    vector<DTYPE> x_grids;
    vector<DTYPE> y_grids;
    map<int, DTYPE> x_grids_map;
    map<int, DTYPE> y_grids_map;
    vector<vector<bool> > block_marks;//It is true for only inside block node
    vector<vector<DIRECTION> > direction_marks;
    //Because it will incorporate length in block inside. The factor of length in block cannot be too big otherwise reverse update will induced by its child. It cannot be too small otherwise (DTYPE) will omit it
    vector<vector<double> > length_marks;
    vector<vector<bool> > outside_tree_marks;//Marked if it is on(include two end) of an outside tree branch
    GenerateGrid(x_grids, y_grids, x_grids_map, y_grids_map, block_marks, direction_marks, length_marks, outside_tree_marks);
    MarkOutsideTree(x_grids_map, y_grids_map,outside_tree_marks);

    for (set<int>::const_iterator it=all_disconnected_roots.begin(); it!=all_disconnected_roots.end(); ++it){
        multimap<double, MRP> map_heap;
        int node_index = *it;
        MarkGridDownFromNode(node_index,0, x_grids_map, y_grids_map, length_marks, map_heap);//Mark all node downstream with 0 length
        MRP end_mrp = FindPath (SMALLBLOCKFACTOR, map_heap, x_grids, y_grids, block_marks, direction_marks, length_marks, outside_tree_marks);
        TraceBack(end_mrp, x_grids, y_grids, x_grids_map, y_grids_map, direction_marks, outside_tree_marks);//outside_tree_marks need to be updated
        ClearGridMarksAndDirections(direction_marks, length_marks);
        //PlotAndSave(tree_);
    }
}

Tree Design::get_tree()const{
    return tree_;
}
template <class T>
T Design::UniteTreeVector(const vector<T>& trees)const{
    T tree;
    set<int> tree_indexes;
    set<int> tree_branches;
    for (unsigned int i =0; i < trees.size(); i++){//all node in outside tree
        T current_tree = trees.at(i);
        set<int> current_tree_indexes = current_tree.get_indexes(); 
        set<int> current_tree_branches = current_tree.get_branches(); 
        set_union (current_tree_indexes.begin(), current_tree_indexes.end(), tree_indexes.begin(), 
                tree_indexes.end(), inserter(tree_indexes, tree_indexes.end()));
        //back_inserter(tree_branches) == inserter(tree_branches, tree_branches.end())
        set_union (current_tree_branches.begin(), current_tree_branches.end(), tree_branches.begin(), 
                tree_branches.end(), inserter(tree_branches, tree_branches.end()));
    }
    tree.set_indexes(tree_indexes);
    tree.set_branches(tree_branches);
    return tree;
}
Tree Design::UniteInsideTrees()const{
    return UniteTreeVector(inside_trees_);
}
Tree Design::UniteTreeAndInsideTrees()const{
    vector<Tree> tree__and_inside_trees;
    tree__and_inside_trees.push_back(tree_);
    tree__and_inside_trees.push_back(UniteInsideTrees());
    return UniteTreeVector(tree__and_inside_trees);
}
void Design::Clean(){
    /*
    set<int> tree_indexes = tree_.get_indexes();
    set<int> tree_branches = tree_.get_branches();
    set<int> floating_terminals;
    do{
        for (set<int>::const_iterator it = floating_terminals.begin(); it != floating_terminals.end(); it++){
            int floating_node_index;
            tree_indexes.erase(floating_node_index);
            tree_indexes.erase(floating_node_index);
        }
        floating_terminals.clear();
        for (set<int>::const_iterator it = tree_indexes.begin(); it != tree_indexes.end(); it++){
            int node_index = *it;
            Node node = nodes_.at(node_index);
            if (sinks_.find(node_index) == sinks_.end() && node.get_children().size() == 0){
                floating_terminals.insert(node_index);
            }        
        }
    }while (!floating_terminals.empty());*/
    tree_.set_sinks(sinks_);
    bool still_in_clean = false;
    do{
        still_in_clean = false;
        set<int> tree_indexes = tree_.get_indexes();
        for (set<int>::const_iterator it = tree_indexes.begin(); it != tree_indexes.end(); it++){
            int node_index = *it;
            bool node_in_clean = tree_.CleanFloatingNode(nodes_, node_index);//cannot clean middle point because it might be an escaping point
            still_in_clean = still_in_clean || node_in_clean;
        }
    }while(still_in_clean);
}
/*--------------  Second level functions      ------------------*/
void Design::InitialXYMinMax (){
    xmin_ = INT_MAX;
    xmax_ = 0;
    ymin_ = INT_MAX;
    ymax_ = 0;
}
void Design::SetXYMinMax (DTYPE x,DTYPE y){
    xmin_ = min(xmin_, x);
    xmax_ = max(xmax_, x);
    ymin_ = min(ymin_, y);
    ymax_ = max(ymax_, y);
}
set<int> Design::RemoveZeroLengthBranch(){
    set<int> index;//which record all valid nodes_
    for (int node_index=0; node_index < nodes_.size(); node_index++){//if i!=j and point(i) == point(j) and parent(i) == j
        Node node = nodes_.at(node_index);
        int parent_index = node.parent_;
        Node parent_node = nodes_.at(parent_index);
        if (node.point_ == parent_node.point_ && node_index != parent_index){//0 length branch remove, and there is one branch in the middle point to itself, leave this one alone cause it help to make the sequence
            for (int j = 0; j < nodes_.size(); j++){
                Node check_node = nodes_.at(j);
                if (check_node.parent_ == node.index_)  
                    nodes_.at(j).parent_ = parent_index;
            }
            if (root_ == node_index){
                root_ = parent_index;
            }
        }else{
            index.insert(node_index);
        }
    }
    for (int node_index=0; node_index < nodes_.size(); node_index++){//if parent(i) == i and i is not parent of any node, remove this idle cycle
        if (node_index == root_)    
            continue;
        Node node = nodes_.at(node_index);
        int parent_index = node.parent_;
        Node parent_node = nodes_.at(parent_index);
        //remove the index who points to itself but no others point to him
        if (node_index == parent_index){//0 length branch remove, and there is one branch in the middle point to itself, leave this one alone cause it help to make the sequence
            bool used = false;
            for (int j = 0; j < nodes_.size(); j++){
                if (j == node_index)
                    continue;
                Node check_node = nodes_.at(j);
                if (check_node.parent_ == node.index_)  
                    used = true;
                    
            }
            if (!used){
                index.erase(node_index);
            }
        }
    }
    return index;
}
set<int> Design::RecognizeSinks()const{
    set<int> sink_set;
    set<int> index = tree_.get_indexes();
    for (set<int>::const_iterator it= index.begin(); it != index.end(); it++){
        sink_set.insert(*it);
    }
    for (set<int>::const_iterator it= index.begin(); it != index.end(); it++){//root and internal ones will be removed here
        Node node = nodes_.at(*it);
        sink_set.erase(node.parent_);
    }
    return sink_set;
}
map<int,vector<Solution> > Design::BuildSinkSolutions()const{
    map<int,vector<Solution> > solution_map;
    for (set<int>::const_iterator it = sinks_.begin(); it != sinks_.end(); it++){
        vector<Solution> solutions;
        Solution solution;
        int sink_index = *it;
        Node sink_node = nodes_.at(sink_index);

        solution.set_c(Cb);
        solution.set_w(0);
        solution.set_s(0);
        solution.set_point(sink_node.point_);
        solutions.push_back(solution);
        if (sink_node.get_children().size() == 0){//If is sink and at end
            solution_map.insert(pair<int,vector<Solution> > (sink_index, solutions));
        }
    }
    return solution_map;
}
map<int,vector< pair<Point, vector<Solution> > > > Design::BuildPointSinkSolution(
        const map<int,vector<Solution> > &solution_map)const{
    map<int,vector< pair<Point, vector<Solution> > > > point_solution_map;
    for (map<int,vector<Solution> >::const_iterator it = solution_map.begin(); it != solution_map.end(); it++){
        int sink_index = it->first;
        vector<Solution> solutions = it->second;
        Point point = nodes_.at(sink_index).point_;
        vector< pair<Point, vector<Solution> > > point_solutions;
        point_solutions.push_back(pair<Point, vector<Solution> >(point, solutions));
        point_solution_map.insert(pair<int,vector< pair<Point, vector<Solution> > > >(sink_index,
                    point_solutions));
    }
    return point_solution_map;
}
list<int> Design::GeneratePostorderTraversal(int root) const{
   list<int> current_list;
   set<int> children = nodes_.at(root).get_children();
   for (set<int>::const_iterator it = children.begin(); it != children.end(); it++){
        list<int> child_list = GeneratePostorderTraversal(*it);
        current_list.splice(current_list.end(), child_list);
   }
   current_list.push_back(root);
   return current_list;
}
vector<vector<Solution> > Design::PropagateChildrenSolutions(int node_index, Point point, 
        BUFFERINGMODE buffering_mode, map<int,vector<Solution> > solution_map)const{
    vector<vector<Solution> > all_children_propagated_solutions;
    set<int> children = nodes_.at(node_index).get_children();
    for (set<int>::const_iterator it2 = children.begin(); it2 != children.end(); it2++){
        int child_index = *it2;
        Point child_point = nodes_.at(child_index).point_;
        vector<Solution> child_solutions = solution_map.find(child_index)->second;//child_solutions
        list<Point> points_list;
        points_list.push_back(child_point);
        points_list.push_back(point);
        vector<Solution> new_child_solutions = PropagateChildSolutions(node_index, point, child_index, 
                buffering_mode, child_solutions, points_list);
        all_children_propagated_solutions.push_back(new_child_solutions);
    }
    return all_children_propagated_solutions;
}
vector<Solution> Design::PropagateChildSolutions(int node_index, Point point, int child_index,
        BUFFERINGMODE buffering_mode, vector<Solution> child_solutions, list<Point> points_list)const{
    vector<Solution> new_child_solutions;
    while (!child_solutions.empty()){
        Solution child_solution = child_solutions.back();
        child_solutions.pop_back();
        if (child_solution.get_point() == point){
            new_child_solutions.push_back(child_solution);
            continue;
        }
        //            DTYPE distance = point.DistanceToPoint(child_solution.get_point());//This need to be changed to some maze, for non-straigth one
        Point solution_point = child_solution.get_point();
        //DirectionalEdge d_edge (solution_point, point);
        //DTYPE distance = d_edge.Length();
        DTYPE distance = CalculateDistanceToEnd(solution_point, points_list);
        if (buffering_mode == BOB && IsInSameInsideTree(node_index, child_index)){//No buffers allowed
            Solution new_solution;
            Buffer max_buffer = buffers_.at(buffers_.size()-1);
            DTYPE length_limit = max_buffer.CalculateLengthLimit(slew_spec_ -child_solution.get_s(),
                    child_solution.get_c());
            if (length_limit >= distance){//Survived
                new_solution.set_c(child_solution.get_c() + distance*CUNIT);
                new_solution.set_w(child_solution.get_w());
                double delta_s = log(9)*RUNIT*distance*
                    (0.5*distance*CUNIT+child_solution.get_c());
                new_solution.set_s(child_solution.get_s() + delta_s);
                new_solution.set_point(point);
                UpdateSolutions(new_solution, child_solutions, -1, slew_spec_, point);
            }else{//Not survived
                continue;
            }
        }else{
            for (unsigned int i = 0; i < buffers_.size(); i++){
                Solution new_solution;
                Buffer buffer = buffers_.at(i);
                DTYPE length_limit = buffer.CalculateLengthLimit(slew_spec_ -child_solution.get_s(),
                        child_solution.get_c());
                //If BOB mode and current edge is in block, Just find all survivor solutions
                if (distance >= length_limit){//Need buffer
                    //Point new_point = d_edge.GetProportionPoint(length_limit/(double)d_edge.Length());
                    Point new_point = FindUpPointWithDistance(solution_point, points_list, length_limit);
                    if (buffering_mode == ADAPTIVE){
                        if (PointIsInBlocks(new_point)){
                            continue;
                        }
                    }
                    new_solution.set_c(buffer.get_input_capacitance());
                    new_solution.set_w(child_solution.get_w() + buffer.get_cost());
                    new_solution.set_s(0);
                    new_solution.set_point(new_point);
                    UpdateSolutions(new_solution, child_solutions, i, slew_spec_, point);
                }else{
                    new_solution.set_c(child_solution.get_c() + distance*CUNIT);
                    new_solution.set_w(child_solution.get_w());
                    double delta_s = log(9)*RUNIT*distance*
                        (0.5*distance*CUNIT+child_solution.get_c());
                    new_solution.set_s(child_solution.get_s() + delta_s);
                    new_solution.set_point(point);
                    UpdateSolutions(new_solution, child_solutions, -1, slew_spec_, point);
                }
            }
        }
    }
    return new_child_solutions;
}
vector<Solution> Design::MergeSolutions(vector<vector<Solution> > all_children_propagated_solutions, 
        Point point)const{
    vector<Solution> new_solutions;
    vector<Solution> child1_solutions;//If one child, still OK
    vector<Solution> child2_solutions;
    while (all_children_propagated_solutions.size() > 1){
        child1_solutions = all_children_propagated_solutions.back();
        all_children_propagated_solutions.pop_back();
        child2_solutions = all_children_propagated_solutions.back();
        all_children_propagated_solutions.pop_back();
        vector<Solution> new_solutions;
        for (int i = 0; i < child1_solutions.size(); i++){
            for (int j = 0; j < child2_solutions.size(); j++){
                Solution child1_solution = child1_solutions.at(i);
                Solution child2_solution = child2_solutions.at(j);
                Solution new_solution;
                new_solution.set_c(child1_solution.get_c() + child2_solution.get_c());
                new_solution.set_w(child1_solution.get_w() + child2_solution.get_w());
                new_solution.set_s(max(child1_solution.get_s(), child2_solution.get_s()));
                new_solution.set_point(point);
                UpdateSolutions(new_solution, new_solutions, -1, slew_spec_, point);
            }
        }
        all_children_propagated_solutions.push_back(new_solutions);
    }
    vector<Solution> solutions = all_children_propagated_solutions.back();
    return solutions;

}
void Design::AddBufferAtSolutions(Point point, vector<Solution> & solutions)const{
    int min_buffer_index = 0;
    Buffer min_buffer = buffers_.at(min_buffer_index);//minimum buffer is best for inserting here
    vector<Solution> & new_solutions = solutions;
    for (unsigned int i = 0; i < solutions.size(); i++){
        Solution new_solution = solutions.at(i);
        new_solution.set_c(min_buffer.get_input_capacitance());
        new_solution.set_w(new_solution.get_w() + min_buffer.get_cost());
        new_solution.set_s(0);
        UpdateSolutions(new_solution, new_solutions, min_buffer_index, slew_spec_, point);
    }
    solutions = new_solutions;
}
void Design::UpdateSolutions(Solution new_solution, vector<Solution> & solutions, int buffer_index, double slew_spec, Point point)const{
    if (buffer_index == -1){
        if (new_solution.get_s() > slew_spec){
            return;
        }
    }else{
       /* Buffer buffer = buffers_.at(buffer_index); Unnessary, it is a must safe decision
        double output_ramp = buffer.get_slew_resistance()*new_solution.get_c() + buffer.get_intrinsic_slew();
        if (sqrt(pow(new_solution.get_s(),2) + pow(output_ramp,2)) > slew_spec){
            return;
        }*/
    }
    vector<Solution> new_solutions = solutions;
    for (unsigned int i = 0; i < solutions.size(); i++){
        Solution check_solution = solutions.at(i);
        Point check_point = check_solution.get_point();
        Point new_point = new_solution.get_point();
        //check_solution dominates new_solution.Just forget new_solution and return
        if (check_point.DistanceToPoint(point) <= new_point.DistanceToPoint(point)){
            if (check_solution.Dominate(new_solution)){
                solutions = new_solutions;
                return;
            }
        }
        if (check_point.DistanceToPoint(point) >= new_point.DistanceToPoint(point)){
            if (new_solution.Dominate(check_solution)){
                //Erase check_solution if it is positionly same or behind and worse than the new_solution
                for (vector<Solution>::iterator it = new_solutions.begin(); it != new_solutions.end(); it++){
                    if (it->SameSolution(check_solution)){
                        new_solutions.erase(it);
                        break;
                    }
                }
            }
        }
    }
    new_solutions.push_back(new_solution);
    solutions = new_solutions;
}
void Design::MultiToMultiChildrenPropagate(int node_index, vector<Point> points, 
        BUFFERINGMODE buffering_mode, map<int, vector< pair<Point,vector<Solution> > > >& solution_map)const{
    vector< pair<Point,vector<Solution> > > merged_all_points_all_children_solutions;
    merged_all_points_all_children_solutions.clear();
    int total_solution = 0;
    for (unsigned int i = 0; i < points.size(); i++){
        Point point = points.at(i);
        vector<vector<Solution> > one_point_all_children_solutions;//All children solutions to this point
        set<int> children = nodes_.at(node_index).get_children();
        for (set<int>::const_iterator it2 = children.begin(); it2 != children.end(); it2++){
            int child_index = *it2;
            vector<Solution> merged_one_point_one_child_solutions = 
                MultiToOneChildPropagate(node_index, point, child_index, buffering_mode, solution_map);
            one_point_all_children_solutions.push_back(merged_one_point_one_child_solutions);
        }
        vector<Solution> merged_one_point_all_children_solutions = 
            MergeSolutions(one_point_all_children_solutions, point);
        total_solution += merged_one_point_all_children_solutions.size();
        merged_all_points_all_children_solutions.push_back(
                pair<Point,vector<Solution> > (point, merged_one_point_all_children_solutions));
    }
    solution_map.insert(pair<int, vector< pair<Point,vector<Solution> > > > (
                node_index,merged_all_points_all_children_solutions));
    if (total_solution == 0)
        cout<< node_index<< endl;
    assert(total_solution > 0);
    //cout<< "Adaptive propagating on node "<< node_index<<" with "<< total_solution<< " total solutions"<< endl;
}

//solutions from all points of one child to one point
vector<Solution> Design::MultiToOneChildPropagate(int node_index, Point point, 
        int child_index, BUFFERINGMODE buffering_mode,
        const map<int, vector< pair<Point,vector<Solution> > > >& solution_map)const{
    vector< pair<Point, vector<Solution> > > all_points_solutions = 
        solution_map.find(child_index)->second;
    vector <Solution> new_solutions;
    new_solutions.clear();
    for (unsigned int i = 0; i < all_points_solutions.size(); i++){
        pair<Point, vector<Solution> > one_point_solution_pair = all_points_solutions.at(i);
        Point child_point = one_point_solution_pair.first;
        vector<Solution> one_point_solutions = one_point_solution_pair.second;
        vector<Solution> new_one_point_solutions = PointToPointChildPropagate(node_index, point, child_index,
                child_point, buffering_mode, one_point_solutions);
        for(unsigned int j = 0; j < new_one_point_solutions.size(); j++){
            Solution new_one_point_solution =  new_one_point_solutions.at(j);
            UpdateSolutions(new_one_point_solution, new_solutions, -1, slew_spec_, point);
        }
    }
    return new_solutions;
}
vector<Solution> Design::PointToPointChildPropagate(int node_index, Point point, int child_index, Point child_point, BUFFERINGMODE buffering_mode, const vector<Solution> &child_one_point_solutions)const{
    /*  have problem two points are not on same line
    int inside_tree_index;
    if (IsInSameInsideTree(child_index, node_index, &inside_tree_index)){
        assert(inside_tree_index >= 0 && inside_tree_index < inside_trees_.size());
        int block_index = inside_trees_.at(inside_tree_index).get_block_index();
        Poly block = blocks_.at(block_index);
        if (point.IsInsidePoly(block) || child_point.IsInsidePoly(block)){//At least one in block, has to go with BOB way
            list<Point> points_list;
            points_list.push_back(child_point);
            points_list.push_back(point);
            vector<Solution> new_child_one_point_solutions = 
                PropagateChildSolutions (node_index, point, child_index,
                    BOB, child_one_point_solutions, points_list);//Use BOB way to deal with in block routing
            return new_child_one_point_solutions;
        }
    }
*/
    int first_unblocked_ancestor = node_index;
    Point first_unblocked_ancestor_point = nodes_.at(first_unblocked_ancestor).point_;
    while (PointIsInBlocks(first_unblocked_ancestor_point)){
        first_unblocked_ancestor = nodes_.at(first_unblocked_ancestor).parent_;
        first_unblocked_ancestor_point = nodes_.at(first_unblocked_ancestor).point_;
    }
    if (point.x > max(child_point.x, first_unblocked_ancestor_point.x) ||
            point.x < min(child_point.x, first_unblocked_ancestor_point.x) ||
            point.y > max(child_point.y, first_unblocked_ancestor_point.y) ||
            point.y < min(child_point.y, first_unblocked_ancestor_point.y) ){
        vector<Solution> zero_solutions;
        zero_solutions.clear();
        return zero_solutions; 
    }
    list<Point> points_list;
    points_list = FindPointsPath(child_point, point);//points_list is from child to point
    if (IsInSameInsideTree(child_index, node_index)){//A little too nice for them to let them buffering anywhere
        vector<Solution> new_child_one_point_solutions = PropagateChildSolutions (node_index, point, child_index,
                buffering_mode, child_one_point_solutions, points_list);
        return new_child_one_point_solutions;
    }else{
        vector<Solution> new_child_one_point_solutions = PropagateChildSolutions (node_index, point, child_index,
                buffering_mode, child_one_point_solutions, points_list);
        return new_child_one_point_solutions;
    }
}
void Design::SplitOutsideBranch(int node_index, int outside_tree_index, const set<int>& children){
    Node node = nodes_.at(node_index);
    Point node_point = node.point_;
    //cout<< "Spliting Outside tree at "<< node_index<<endl;
    for (set<int>::const_iterator it = children.begin(); it != children.end(); it++){//For every child
        OutsideTree outside_tree = outside_trees_.at(outside_tree_index);//This outside_tree is only legal in this local range. Otherwise it will over-write previous/later children updates

        int child_node_index = *it;
        Node child_node = nodes_.at(child_node_index);
        set<int> child_children = child_node.get_children();
        Point child_node_point = child_node.point_;
        Edge tree_edge(node_point, child_node_point);
        Point closest_intersect_point = child_node_point;//Initial as the farest end point
        int closest_intersect_block = -1;
        for (vector<Poly>::const_iterator it = blocks_.begin(); it != blocks_.end(); it++){
            Poly block = *it;
            int block_index = it - blocks_.begin();
            //Search every edge on every block for intersection
            for (int edge_index = 0; edge_index < block.Size(); edge_index++){
                Edge block_edge = block.GetEdge(edge_index);
                if (tree_edge.IsIntersecting(block_edge)){
                    Point intersect_point = block_edge.v_?Point(block_edge.c_, tree_edge.c_):
                        Point (tree_edge.c_, block_edge.c_);
                    if (node_point.DistanceToPoint(intersect_point) < 
                            node_point.DistanceToPoint(closest_intersect_point)){
                        closest_intersect_point = intersect_point;
                        closest_intersect_block = block_index;
                    }
                }
            }
        }
        //Has intersection with current block or end of edge is on block
        int child_node_on_block_index;
        bool perpendicular_hit_block = PointIsOnBlocks(child_node_point, &child_node_on_block_index);
        if (closest_intersect_block != -1 || perpendicular_hit_block){
            if (closest_intersect_block != -1){
                Node new_node = tree_.InsertNewNode(node_index, child_node_index, closest_intersect_point, nodes_);
                int new_node_index = new_node.index_;
                outside_tree.insert_index(new_node_index);//+indexes
                outside_tree.insert_branch(new_node_index);//+branches
                outside_tree.insert_sink(new_node_index);//+sinks
                InsideTree inside_tree(new_node_index, closest_intersect_block);
                int inside_tree_index = inside_trees_.size();
                inside_trees_.push_back(inside_tree);
                inside_tree_map_.insert(pair<int,int> (inside_tree.get_root(), inside_tree_index));
                outside_tree.insert_root_down_inside_tree(inside_tree_index);
                outside_trees_.at(outside_tree_index) = outside_tree;
                set<int> new_node_children;
                new_node_children.insert(child_node_index);
                SplitInsideBranch (new_node_index, inside_tree_index, new_node_children);
            }else{//If the end node is block. Divide its children into two parts, and go seperately
                outside_tree.insert_index(child_node_index);
                outside_tree.insert_branch(child_node_index);
                outside_tree.insert_sink(child_node_index);
                outside_trees_.at(outside_tree_index) = outside_tree;

                set<int> inside_child_children, outside_child_children;
                Poly child_node_on_block = blocks_.at(child_node_on_block_index);
                for (set<int>::const_iterator it = child_children.begin(); it != child_children.end(); it++){
                    int grand_child = *it;
                    Point grand_child_point = nodes_.at(grand_child).point_;
                    Point child_point = child_node.point_;
                    DirectionalEdge d_edge(child_point, grand_child_point);
                    Point between_point = child_point;
                    if (d_edge.v_){
                        if (grand_child_point.y > child_point.y){
                            between_point.y += 1;
                        }else{
                            between_point.y -= 1;
                        }
                    }else{
                        if (grand_child_point.x > child_point.x){
                            between_point.x += 1;
                        }else{
                            between_point.x -= 1;
                        }
                    }
                    if (between_point.IsInsidePoly(child_node_on_block)){
                        inside_child_children.insert(grand_child);
                    }else{
                        outside_child_children.insert(grand_child);
                    }
                }
                if (!inside_child_children.empty()){
                    InsideTree inside_tree(child_node_index, child_node_on_block_index);
                    int inside_tree_index = inside_trees_.size();
                    inside_trees_.push_back(inside_tree);
                    inside_tree_map_.insert(pair<int,int> (inside_tree.get_root(), inside_tree_index));
                    outside_tree.insert_root_down_inside_tree(inside_tree_index);
                    outside_trees_.at(outside_tree_index) = outside_tree;
                    SplitInsideBranch (child_node_index, inside_tree_index, inside_child_children);
                }
                if (!outside_child_children.empty()){
                    outside_tree.insert_index(child_node_index);
                    outside_tree.insert_branch(child_node_index);
                    outside_trees_.at(outside_tree_index) = outside_tree;//Update outside_trees_ ontime so keep recursion
                    SplitOutsideBranch (child_node_index, outside_tree_index, outside_child_children);
                }
            }
        }else if (child_children.size() == 0){// || PointIsOnBlocks(child_node_point)
        //If this tree edge is not intersecting with any block edges and child_node is a sink with no children
            outside_tree.insert_index(child_node_index);
            outside_tree.insert_branch(child_node_index);
            outside_tree.insert_sink(child_node_index);
            outside_trees_.at(outside_tree_index) = outside_tree;
        }else{//If this tree edge is not intersecting with any block edges and child_node has children
            outside_tree.insert_index(child_node_index);
            outside_tree.insert_branch(child_node_index);
            outside_trees_.at(outside_tree_index) = outside_tree;//Update outside_trees_ ontime so keep recursion
            SplitOutsideBranch (child_node_index, outside_tree_index, child_children);
        }
    }
}
void Design::SplitInsideBranch(int node_index, int inside_tree_index, const set<int>& children){
    Node node = nodes_.at(node_index);
    Point node_point = node.point_;
    //cout<< "Spliting inside tree at "<< node_index<<endl;
    for (set<int>::const_iterator it = children.begin(); it != children.end(); it++){//For every child
        InsideTree inside_tree = inside_trees_.at(inside_tree_index);
        int block_index = inside_tree.get_block_index();
        Poly block = blocks_.at(block_index);

        int child_node_index = *it;
        Node child_node = nodes_.at(child_node_index);
        set<int> child_children = child_node.get_children();
        Point child_node_point = child_node.point_;
        Edge tree_edge(node_point, child_node_point);
        Point closest_intersect_point = child_node_point;//Initial as the farest end point
        int closest_intersect_block = -1;
        //Search every edge on every block for intersection
        for (int edge_index = 0; edge_index < block.Size(); edge_index++){
            Edge block_edge = block.GetEdge(edge_index);
            if (tree_edge.IsIntersecting(block_edge)){
                Point intersect_point = block_edge.v_?Point(block_edge.c_, tree_edge.c_):
                    Point (tree_edge.c_, block_edge.c_);
                if (node_point.DistanceToPoint(intersect_point) < 
                        node_point.DistanceToPoint(closest_intersect_point)){
                    closest_intersect_point = intersect_point;
                    closest_intersect_block = block_index;
                }
            }
        }
        bool perpendicular_hit_block = PointIsOnBlocks(child_node_point);//Different from out, once hit go out
        //Has intersection with current block or the end of edge is just on block
        if (closest_intersect_block != -1 || perpendicular_hit_block){
            if (closest_intersect_block != -1){//intersection
                Node new_node = tree_.InsertNewNode(node_index, child_node_index, closest_intersect_point, nodes_);
                int new_node_index = new_node.index_;
                inside_tree.insert_index(new_node_index);
                inside_tree.insert_branch(new_node_index);
                inside_tree.insert_sink(new_node_index);
                OutsideTree outside_tree(new_node_index);
                int outside_tree_index = outside_trees_.size();
                outside_trees_.push_back(outside_tree);
                outside_tree_map_.insert(pair<int,int> (outside_tree.get_root(), outside_tree_index));
                inside_tree.insert_root_down_outside_tree(outside_tree_index);
                inside_trees_.at(inside_tree_index) = inside_tree;
                set<int> new_node_children;
                new_node_children.insert(child_node_index);
                SplitOutsideBranch (new_node_index, outside_tree_index, new_node_children);
            }else{//end of edge is just on block
                inside_tree.insert_index(child_node_index);
                inside_tree.insert_branch(child_node_index);
                inside_tree.insert_sink(child_node_index);
                inside_trees_.at(inside_tree_index) = inside_tree;

                OutsideTree outside_tree(child_node_index);
                int outside_tree_index = outside_trees_.size();
                outside_trees_.push_back(outside_tree);
                outside_tree_map_.insert(pair<int,int> (outside_tree.get_root(), outside_tree_index));
                inside_tree.insert_root_down_outside_tree(outside_tree_index);
                inside_trees_.at(inside_tree_index) = inside_tree;
                SplitOutsideBranch (child_node_index, outside_tree_index, child_children);
            }
        }else{//It is a branch/steiner node in flute tree which is just in a block
            inside_tree.insert_index(child_node_index);
            inside_tree.insert_branch(child_node_index);
            inside_trees_.at(inside_tree_index) = inside_tree;//Update inside_trees_ ontime so keep recursion
            SplitInsideBranch (child_node_index, inside_tree_index, child_children);
        }
    }
}
void Design::InitialSequenceBranch (int root) {
    int parent = root;
    int current = nodes_[root].parent_;
    while (current!=parent){//break when to the branch with n of itself. Just reverse all adverse branch
        int next = nodes_[current].parent_;
        nodes_[current].parent_ = parent;
        parent = current;
        current = next;
    }
    nodes_[root].parent_ = root;//Last set root correctly. Root is the only one with itself as parent
}

vector<vector<PossiblePoint> > Design::FindAllChoices (int inside_tree_index, set<int>& movable_sinks, 
        int worst_sink_index, double output_slew, map<int,double> slew_map)const{
    vector<vector<PossiblePoint> > choices_all_sinks;//pps will include all pp for each movable sink
    for (set<int>::iterator it= movable_sinks.begin(); it != movable_sinks.end(); it++){
        vector<PossiblePoint> possible_points;//pps will include all pp for one moving sink
        int sink_index = *it;//current moving sink
        Node sink_node = nodes_.at(sink_index);
        int block_index = inside_trees_.at(inside_tree_index).get_block_index();
        //Find step first
        int step;
        step = GetStep (inside_tree_index, sink_index);//0 stands for nothing can be done. 1 is clockwise and -1 anticlockwise
        if (step == 0){//This terminal has not obvious moving choice
            DTYPE distance_to_closest_sink = 0;
            int closest_sink_index = FindClosestTerminalOnBlockBoundary (inside_tree_index, sink_index, &distance_to_closest_sink);
            PossiblePoint possible_point = AddCombinationChoice (sink_index, worst_sink_index, distance_to_closest_sink, output_slew, slew_map, inside_tree_index);
            possible_points.push_back(possible_point);
            choices_all_sinks.push_back(possible_points);
            continue;
        }
        //Find start and end edge and initialize start point
        int start_edge_index = GetBlockEdgeIndex (block_index, sink_index);
        int current_edge_index = start_edge_index - step;
        Point start_point = sink_node.point_;
        DTYPE distance_to_next_sink = 0;
        int next_sink_index = FindNextTerminalOnBlockBoundary (inside_tree_index, step, sink_index, &distance_to_next_sink);
        int stop_edge_index = GetBlockEdgeIndex (block_index, next_sink_index);
        //Add combination
        PossiblePoint possible_point = AddCombinationChoice (sink_index, worst_sink_index, distance_to_next_sink, output_slew, slew_map, inside_tree_index);
        possible_points.push_back(possible_point);

        //This paragraph generate more pp for this moving point
        InsideTree inside_tree = inside_trees_.at(inside_tree_index);
        DirectionalEdge second_d_edge = inside_tree.FindFirstLShapeEndPoint(sink_index, nodes_);//End shape of the first L shape from sink to root. If step != 0, it must have at least one L shape
        Point L_end_point = second_d_edge.get_end_point();
        Poly block = blocks_.at(block_index);
        int block_size = block.Size();
        bool stop_sign;//Some cases no need keep going to stop edge
        do{
            current_edge_index = (current_edge_index + step + block_size)%block_size;
            stop_sign = false;
            Point end_point = step==1?block.At((current_edge_index+1)%block_size):block.At(current_edge_index);
            DirectionalEdge current_d_edge(start_point, end_point);
            //Re-assign end point if parallel or (vertical and root is on it)
            Point inside_tree_root_point = inside_tree.get_root_point(nodes_);
            if (current_d_edge.v_ == second_d_edge.v_){
                if (L_end_point.IsProjectedIn(current_d_edge)){
                    end_point = L_end_point.Projection(current_d_edge);
                    current_d_edge.set_end_point(end_point);
                    stop_sign = true;
                }
            }else{
                start_point = end_point;  
                continue;
            }
            int total_segment_number = 1;//divide 10 segments
            for (int segment_number = 0; segment_number <= total_segment_number; segment_number++){
                double segment_portion = segment_number/(double)total_segment_number;
                Point segment_point = current_d_edge.GetProportionPoint(segment_portion);
                if (segment_point != sink_node.point_ && segment_point != inside_tree_root_point){//The non-moved point will lead to 0 length edge, which brings error
                    DTYPE distance_to_original_sink = block.GetOutsideLength(sink_node.point_, segment_point, step);
                    possible_point = AddOneSlideChoice(inside_tree_index,segment_point, segment_point.Projection(second_d_edge), sink_index, worst_sink_index, distance_to_original_sink, output_slew, slew_map);
                    possible_points.push_back(possible_point);
                }
            }
            start_point = end_point;
        }while (current_edge_index != stop_edge_index && !stop_sign);
        choices_all_sinks.push_back(possible_points);
    }
    movable_sinks.erase(worst_sink_index);
    return choices_all_sinks;
}


void Design::WriteToSolver(const vector<vector<PossiblePoint> >& choices_all_sinks, double output_slew, double worst_ramp_slew)const{
    FILE * fp;
    string str(work_path_);
    str += "/tmp/problem.lp";
    if( (fp=fopen(str.c_str(),"w")) == NULL ) {
        printf("bookshelf_IO: Cannot open: %s file for write", str.c_str());
    }

    fprintf (fp, "Minimize\n");
    for (unsigned int i = 0; i < choices_all_sinks.size(); i++){//Minimize sum_i(sum_j(A_i_j*W_i_j))
        vector<PossiblePoint> possible_points_i = choices_all_sinks.at(i);
        for(int j = 0; j < possible_points_i.size(); j++){
            PossiblePoint possible_point = possible_points_i.at(j);
            if (!(i==0 && j==0)){
                fprintf (fp, " + ");
            }
            fprintf (fp, "%f A_%d_%d", (double)possible_point.get_wirelength_increase(), i, j);
        }
    }
    //(Output_slew + sum_i(sum_j(A_i_j O_i_j)))^2 + (Ramp_slew + sum_i(sum_j(A_i_j R_i_j)))^2 <= spec^2
    fprintf (fp, "\nSubject To\n");
//2 * (A_s_t O_s_t) * (A_i_j O_i_j) + 2 * (A_s_t R_s_t) * (A_i_j R_i_j) => 2 * Y_s_t_i_j * (O_s_t O_i_j + R_s_t R_i_j)
    for (int s = 0; s < choices_all_sinks.size(); s++){
        vector<PossiblePoint> possible_points_s = choices_all_sinks.at(s);
        for(int t = 0; t < possible_points_s.size(); t++){
            PossiblePoint possible_point_s_t = possible_points_s.at(t);
            for (unsigned int i = 0; i < choices_all_sinks.size(); i++){
                if (i == s){
                    continue;
                }
                vector<PossiblePoint> possible_points_i = choices_all_sinks.at(i);
                for(int j = 0; j < possible_points_i.size(); j++){
                    PossiblePoint possible_point_i_j = possible_points_i.at(j);
                    fprintf (fp, "%f Y_%d_%d_%d_%d + ",(double)2* (
                                possible_point_s_t.get_output_slew_increase()*possible_point_i_j.get_output_slew_increase() + 
                                possible_point_s_t.get_ramp_slew_increase()*possible_point_i_j.get_ramp_slew_increase()), s, t, i, j);
                }
            }
        }
    }
    //(A_i_j O_i_j) * (A_i_j O_i_j) + (A_i_j R_i_j) * (A_i_j R_i_j) => A_i_j * (O_i_j^2 + R_i_j^2)
    for (unsigned int i = 0; i < choices_all_sinks.size(); i++){
        vector<PossiblePoint> possible_points_i = choices_all_sinks.at(i);
        for(int j = 0; j < possible_points_i.size(); j++){
            PossiblePoint possible_point_i_j = possible_points_i.at(j);
            fprintf (fp, "%f A_%d_%d + ",  
                    (double)pow(possible_point_i_j.get_output_slew_increase(),2) +
                    pow(possible_point_i_j.get_ramp_slew_increase(),2), i, j);
        }
    }
    //2 * (A_i_j O_i_j) * output_slew + 2 * (A_i_j R_i_j) * ramp_slew => 
    //A_i_j * 2 * (O_i_j * output_slew + R_i_j * ramp_slew)
    for (unsigned int i = 0; i < choices_all_sinks.size(); i++){
        vector<PossiblePoint> possible_points_i = choices_all_sinks.at(i);
        for(int j = 0; j < possible_points_i.size(); j++){
            PossiblePoint possible_point_i_j = possible_points_i.at(j);
            fprintf (fp, "%f A_%d_%d", 
                    (double)2 * (possible_point_i_j.get_output_slew_increase()*output_slew +
                        possible_point_i_j.get_ramp_slew_increase()*worst_ramp_slew), i, j);
            if (! (i == choices_all_sinks.size()-1 && j == possible_points_i.size()-1)){
                fprintf (fp, " + "); 
            }
        }
    }
    fprintf (fp, " <= %f\n", (double)slew_spec_*slew_spec_ - pow(output_slew,2) - pow(worst_ramp_slew,2));

    for (unsigned int i = 0; i < choices_all_sinks.size(); i++){//sum_j(A_i_j) <= 1
        vector<PossiblePoint> possible_points_i = choices_all_sinks.at(i);
        for(int j = 0; j < possible_points_i.size(); j++){
            PossiblePoint possible_point = possible_points_i.at(j);
            if (j != 0){
                fprintf (fp, " + ");
            }
            fprintf (fp, "A_%d_%d", i, j);
        }
        fprintf (fp, " <= 1\n");
    }
    for (int s = 0; s < choices_all_sinks.size(); s++){//Y_s_t_i_j constraints
        vector<PossiblePoint> possible_points_s = choices_all_sinks.at(s);
        for(int t = 0; t < possible_points_s.size(); t++){
            PossiblePoint possible_point_s_t = possible_points_s.at(t);
            for (unsigned int i = 0; i < choices_all_sinks.size(); i++){
                if (i == s){
                    continue;
                }
                vector<PossiblePoint> possible_points_i = choices_all_sinks.at(i);
                for(int j = 0; j < possible_points_i.size(); j++){
                    PossiblePoint possible_point_i_j = possible_points_i.at(j);
                    fprintf (fp, "Y_%d_%d_%d_%d - A_%d_%d <= 0\n", s, t, i, j, s, t); 
                    fprintf (fp, "Y_%d_%d_%d_%d - A_%d_%d <= 0\n", s, t, i, j, i, j); 
                    fprintf (fp, "Y_%d_%d_%d_%d - A_%d_%d - A_%d_%d >= -1\n", s, t, i, j, s, t, i, j); 
                }
            }
        }
    }
    fprintf (fp, "Binaries\n");
    for (unsigned int i = 0; i < choices_all_sinks.size(); i++){//A_i_j
        vector<PossiblePoint> possible_points_i = choices_all_sinks.at(i);
        for(int j = 0; j < possible_points_i.size(); j++){
            PossiblePoint possible_point = possible_points_i.at(j);
            fprintf (fp, "A_%d_%d\n", i, j);
        }
    }
    for (int s = 0; s < choices_all_sinks.size(); s++){//Y_s_t_i_j
        vector<PossiblePoint> possible_points_s = choices_all_sinks.at(s);
        for(int t = 0; t < possible_points_s.size(); t++){
            PossiblePoint possible_point_s_t = possible_points_s.at(t);
            for (unsigned int i = 0; i < choices_all_sinks.size(); i++){
                if (i == s){
                    continue;
                }
                vector<PossiblePoint> possible_points_i = choices_all_sinks.at(i);
                for(int j = 0; j < possible_points_i.size(); j++){
                    PossiblePoint possible_point_i_j = possible_points_i.at(j);
                    fprintf (fp, "Y_%d_%d_%d_%d\n", s, t, i, j); 
                }
            }
        }
    }

    fclose(fp);
}
void Design::CallSolver()const{
    string script_str(work_path_);
    script_str += "/scripts/call_gurobi.csh";
    string log_str(work_path_);
    log_str += "/tmp/gurobi.log";
    string command_str = script_str + " >> " + log_str;

    system (command_str.c_str());
}

vector<PossiblePoint> Design::ReadFromSolver(const vector<vector<PossiblePoint> >& all_choices, set<int>& combined_sinks)const{
    FILE * fp;
    string str(work_path_);
    str += "/tmp/problem.sol";
    if( (fp=fopen(str.c_str(),"r")) == NULL ) {
        printf("bookshelf_IO: Cannot open: %s file for read", str.c_str());
    }
    vector<PossiblePoint> slided_sinks;
    char line[LINESIZE];
    while(!feof(fp)) {
        *line = '\0';
        fgets(line, LINESIZE, fp);
        char variable_name[BUFFERSIZE];
        sscanf(line, "%s\t%*s\n", variable_name);
        if( strlen(line)<5 || variable_name[0] == '#'){
            continue;
        }
        int binary_value;
        sscanf(line, "%s\t%d\n", variable_name, &binary_value);
        char *pch = NULL;
        pch = strtok (variable_name, "_");
        char type_name;
        type_name = *pch;
        if (type_name == 'Y'){//Not care about Y values
            continue;
        }
        pch = strtok (NULL, "_");
        int i = *pch - '0';
        pch = strtok (NULL, "_");
        int j = *pch - '0';
        if (type_name == 'A'){
            if (binary_value == 1){
                vector<PossiblePoint> possible_points_i = all_choices.at(i);
                PossiblePoint possible_point_i_j= possible_points_i.at(j);
                if (j == 0){
                    combined_sinks.insert(possible_point_i_j.get_node_index());
                }else{//If it is slide, not combination choice
                    slided_sinks.push_back(possible_point_i_j);
                }
            }
        }
    }
    fclose(fp);
    return slided_sinks;

}
void Design::UpdateOneTree(InsideTree & inside_tree, const vector<PossiblePoint>& slided_sinks, const set<int> & combined_sinks, set<int>& all_disconnected_roots){
    //Add a new node at new sink point, this is because of the possibility that sink is an actual sink
    for (vector<PossiblePoint>::const_iterator it =  slided_sinks.begin(); it != slided_sinks.end(); it++){
        Node new_node;//new sink node
        new_node.index_ = nodes_.size();
        new_node.point_ = it->get_point();
        nodes_.push_back(new_node);
        int new_node_index = new_node.index_;
        inside_tree.insert_sink(new_node_index);
        inside_tree.insert_index(new_node_index);
        inside_tree.insert_branch(new_node_index);

        int range_parent_index,range_child_index;//new parent node
        Point new_parent_point = it->get_parent_point();
        inside_tree.FindEdgeIndex(new_parent_point, nodes_, &range_parent_index,&range_child_index);
        DirectionalEdge d_edge = inside_tree.GetBranchEdge(range_child_index, nodes_);
        if (new_parent_point != d_edge.get_start_point() && new_parent_point != d_edge.get_end_point()){
            Node new_parent_node = inside_tree.InsertNewNode(range_parent_index, range_child_index, new_parent_point, nodes_);
            int new_parent_index = new_parent_node.index_;
            inside_tree.AddChildNode(nodes_, new_parent_index, new_node_index);
            inside_tree.SetParentNode(nodes_, new_parent_index, new_node_index);
        }else if (new_parent_point == d_edge.get_start_point()){
            inside_tree.AddChildNode(nodes_, range_child_index, new_node_index);
            inside_tree.SetParentNode(nodes_, range_child_index, new_node_index);
        }else{
            inside_tree.AddChildNode(nodes_, range_parent_index, new_node_index);
            inside_tree.SetParentNode(nodes_, range_parent_index, new_node_index);
        }
        int old_sink_index = it->get_node_index();//old sink remove
        inside_tree.remove_sink(old_sink_index);
        inside_tree.remove_index(old_sink_index);
        inside_tree.remove_branch(old_sink_index);

        int old_parent_index = nodes_.at(old_sink_index).parent_;//old parent clean
        inside_tree.RemoveChildNode(nodes_, old_parent_index, old_sink_index);
        inside_tree.CleanNode(nodes_, old_parent_index);//Remove tail point, remove useless middle point

        all_disconnected_roots.insert(old_sink_index);

    }
    for (set<int>::const_iterator it = combined_sinks.begin(); it != combined_sinks.end(); it++){
        int old_sink_index = *it;
        inside_tree.remove_sink(old_sink_index);
        inside_tree.remove_index(old_sink_index);
        inside_tree.remove_branch(old_sink_index);
        int parent_node_index = nodes_.at(old_sink_index).parent_;
        inside_tree.RemoveChildNode(nodes_, parent_node_index, old_sink_index);
        inside_tree.CleanNode(nodes_, parent_node_index);

        all_disconnected_roots.insert(old_sink_index);
    }
}

void Design::GenerateGrid(vector<DTYPE> & x_grids, vector<DTYPE>& y_grids, map<DTYPE,int>& x_grids_map, map<DTYPE,int>& y_grids_map, vector<vector<bool> >& block_marks, vector<vector<DIRECTION> >& direction_marks, vector<vector<double> >& length_marks, vector<vector<bool> >& outside_tree_marks)const{
    set<DTYPE> x_grids_set, y_grids_set;
    for (unsigned int i =0; i < inside_trees_.size(); i++){//This is needed since right now inside and outside tree are not fully connected
        InsideTree inside_tree = inside_trees_.at(i);
        set<int> inside_tree_sinks = inside_tree.get_sinks(); 
        for (set<int>::const_iterator it = inside_tree_sinks.begin(); it != inside_tree_sinks.end(); it++){
            int node_index = *it;
            Node node = nodes_.at(node_index);
            Point point = node.point_;
            x_grids_set.insert(point.x);
            y_grids_set.insert(point.y);
        }
    }
    for (unsigned int i =0; i < outside_trees_.size(); i++){//all node in outside tree
        OutsideTree outside_tree = outside_trees_.at(i);
        set<int> outside_tree_indexes = outside_tree.get_indexes(); 
        for (set<int>::const_iterator it = outside_tree_indexes.begin(); it != outside_tree_indexes.end(); it++){
            int node_index = *it;
            Node node = nodes_.at(node_index);
            Point point = node.point_;
            x_grids_set.insert(point.x);
            y_grids_set.insert(point.y);
        }
    }
    for (vector<Poly>::const_iterator it = blocks_.begin(); it != blocks_.end(); ++it){
        Poly block = *it;
        int block_index = it - blocks_.begin();
        for (unsigned int i=0; i<block.Size();++i){
            Edge edge = block.GetEdge(i); 
            edge.v_? x_grids_set.insert(edge.c_):y_grids_set.insert(edge.c_);
        }
    }
    int x_size = x_grids_set.size();
    int y_size = y_grids_set.size();
    for (unsigned int i = 0; i < x_size; i++){//Initialize block_marks direction_marks length_marks
        vector<bool> block_y_marks(y_size, false);
        block_marks.push_back(block_y_marks);
        vector<DIRECTION> direction_y_marks (y_size, UNTOUCHED);
        direction_marks.push_back(direction_y_marks);
        vector<double> y_grid_marks (y_size, FLT_MAX);
        length_marks.push_back(y_grid_marks);
        vector<bool> outside_tree_y_marks(y_size, false);
        outside_tree_marks.push_back(outside_tree_y_marks);
    }
    x_grids = std::vector<DTYPE> (x_grids_set.begin(), x_grids_set.end());//Generate x_grids and x_grids_map
    y_grids = std::vector<DTYPE> (y_grids_set.begin(), y_grids_set.end());
    for (unsigned int i = 0; i < x_size; i++){
        x_grids_map.insert(pair<int,int>(x_grids.at(i), i));
    }
    for (unsigned int i = 0; i < y_size; i++){
        y_grids_map.insert(pair<int,int>(y_grids.at(i), i));
    }

    for (vector<Poly>::const_iterator it = original_blocks_.begin(); it != original_blocks_.end(); ++it){//Set block_marks
        Poly block = *it;
        int block_index = it - original_blocks_.begin();
        pair<Point,Point> pair_points = block.GenerateBoundingBox();
        Point left_down_point = pair_points.first;
        Point up_right_point = pair_points.second;
        int min_x_grid = x_grids_map.find(left_down_point.x)->second;
        int max_x_grid = x_grids_map.find(up_right_point.x)->second;
        int min_y_grid = y_grids_map.find(left_down_point.y)->second;
        int max_y_grid = y_grids_map.find(up_right_point.y)->second;
        for (int i = min_x_grid+1; i < max_x_grid; i++){
            for (int j = min_y_grid+1; j < max_y_grid; j++){
                block_marks[i][j] = true;
            }
        }
    }
}

void Design::MarkOutsideTree(map<DTYPE,int>& x_grids_map, map<DTYPE,int>& y_grids_map, 
        vector<vector<bool> >& outside_tree_marks)const{
    set<int> branches = tree_.get_branches();
    for (set<int>::const_iterator it = branches.begin(); it != branches.end(); it++){
        DirectionalEdge d_edge =  tree_.GetBranchEdge(*it, nodes_);
        if (d_edge.v_){
            int min_y_grid = y_grids_map.find(d_edge.c1_)->second;
            int max_y_grid = y_grids_map.find(d_edge.c2_)->second;
            int x_grid = x_grids_map.find(d_edge.c_)->second;
            for (int i = min_y_grid; i <= max_y_grid; i++){
                outside_tree_marks[x_grid][i] = true;                
            }
        }else{
            int min_x_grid = x_grids_map.find(d_edge.c1_)->second;
            int max_x_grid = x_grids_map.find(d_edge.c2_)->second;
            int y_grid = y_grids_map.find(d_edge.c_)->second;
            for (int i = min_x_grid; i <= max_x_grid; i++){
                outside_tree_marks[i][y_grid] = true;                
            }

        }
    }
    for (unsigned int i =0; i < inside_trees_.size(); i++){//This is needed since right now inside and outside tree are not fully connected
        InsideTree inside_tree = inside_trees_.at(i);
        set<int> inside_tree_sinks = inside_tree.get_sinks(); 
        for (set<int>::const_iterator it = inside_tree_sinks.begin(); it != inside_tree_sinks.end(); it++){
            int node_index = *it;
            Node node = nodes_.at(node_index);
            Point point = node.point_;
            int x_grid = x_grids_map.find(point.x)->second;
            int y_grid = y_grids_map.find(point.y)->second;
            outside_tree_marks[x_grid][y_grid] = true;
        }
    }
}
void Design::MarkGridDownFromNode(int node_index, int current_tree_level, const map<DTYPE,int>& x_grids_map, const map<DTYPE,int>& y_grids_map, vector<vector<double> >& length_marks, multimap<double, MRP> & map_heap) const{
    Node node = nodes_.at(node_index);
    Point point = node.point_;
    int x_grid = x_grids_map.find(point.x)->second;//[] cannot be used on const map
    int y_grid = y_grids_map.find(point.y)->second;
    length_marks[x_grid][y_grid] = 0.0;
    MRP start_mrp;
    start_mrp.x_grid_ = x_grid;
    start_mrp.y_grid_ = y_grid;
    start_mrp.point_ = point; 
    start_mrp.length_in_block_ = 0;
    if (current_tree_level == 0){
        map_heap.insert(pair<double, MRP>(0, start_mrp));//Only start from the connected outside tree
    }
    //If it is a root of an inside tree, it will only mark the sinks connected with it by inside tree and its new outside children(because previous mazerouting)
    set<int> children = node.get_children();
    if (inside_tree_map_.find(node.index_) != inside_tree_map_.end()){
        int inside_tree_index = inside_tree_map_.find(node.index_)->second;
        InsideTree inside_tree = inside_trees_.at(inside_tree_index);
        set<int> inside_tree_sinks = inside_tree.get_sinks(); //might be empty
        for (set<int>::const_iterator it = inside_tree_sinks.begin(); 
                it != inside_tree_sinks.end(); it++){
            int inside_tree_sink = *it;
            MarkGridDownFromNode(inside_tree_sink, current_tree_level + 1,
                    x_grids_map, y_grids_map, length_marks, map_heap);
            while (inside_tree_sink != node_index){
                if (children.find(inside_tree_sink) != children.end()){
                    children.erase(inside_tree_sink);
                }
                inside_tree_sink = nodes_.at(inside_tree_sink).parent_;
            }
        } 
        
    }
    for (set<int>::const_iterator it = children.begin(); it != children.end(); it++){
        int child_index = *it;
        Node child_node = nodes_.at(child_index);
        Point child_point = child_node.point_;
        int child_x_grid = x_grids_map.find(child_point.x)->second;
        int child_y_grid = y_grids_map.find(child_point.y)->second;
        int x_step, y_step;
        if (x_grid == child_x_grid){
            x_step = 0;
            assert (child_y_grid != y_grid);
            y_step = child_y_grid > y_grid?1:-1;
            for (int i = y_grid; i != child_y_grid + y_step; i += y_step){
                length_marks[x_grid][i] = 0.0;
            }
        }else if (y_grid == child_y_grid){
            assert (child_x_grid != x_grid);
            x_step = child_x_grid > x_grid?1:-1;
            for (int i = x_grid; i != child_x_grid + x_step; i += x_step){
                length_marks[i][y_grid] = 0.0;
            }
        }else{
            assert(1);//Coundn't be here
        }
        MarkGridDownFromNode(child_index, current_tree_level, x_grids_map, y_grids_map, length_marks, map_heap);
    }
}
MRP Design::FindPath (double block_factor, multimap<double, MRP>& map_heap, const vector<DTYPE> & x_grids, const vector<DTYPE>& y_grids, const vector<vector<bool> >& block_marks, vector<vector<DIRECTION> >& direction_marks, vector<vector<double> >& length_marks, const vector<vector<bool> >& outside_tree_marks)const{
    double length_so_far, next_length_so_far;
    while (!map_heap.empty()){
       /* mrp = m_heap.min();
        length_so_far = m_heap.minTag();
        m_heap.extractMin();*/
        map<double,MRP>::iterator it = map_heap.begin();
        MRP mrp = it->second;
        length_so_far = it->first;
        map_heap.erase(it); 
        if (outside_tree_marks[mrp.x_grid_][mrp.y_grid_] && length_marks[mrp.x_grid_][mrp.y_grid_] > 0){
            return mrp;//Is on outside trees and it is not the initial connected nodes
        }
        for (int int_direction = RIGHT; int_direction <= DOWN; int_direction++){//UNTOUCHED, RIGHT, LEFT, UP, DOWN
            MRP next_mrp = mrp;
            DIRECTION direction;
            if ( int_direction == (int)RIGHT){
                if (mrp.x_grid_ < x_grids.size()-1){//x_grid_+1
                    next_mrp.x_grid_ = mrp.x_grid_ + 1;
                }else{
                    continue;
                }
                direction = RIGHT;
            }else if (int_direction == (int)LEFT){
                if (mrp.x_grid_ > 0){
                    next_mrp.x_grid_ = mrp.x_grid_ - 1; 
                }else{
                    continue;
                }
                direction = LEFT;
            }else if (int_direction == (int)UP){
                if (mrp.y_grid_ < y_grids.size()-1){
                    next_mrp.y_grid_ = mrp.y_grid_ + 1; 
                }else{
                    continue;
                }
                direction = UP;
            }else if (int_direction == (int)DOWN){
                if (mrp.y_grid_ > 0){
                    next_mrp.y_grid_ = mrp.y_grid_ - 1; 
                }else{
                    continue;
                }
                direction = DOWN;
            }
            next_mrp.point_ = Point (x_grids.at(next_mrp.x_grid_), y_grids.at(next_mrp.y_grid_));
            DTYPE step_length = next_mrp.point_.DistanceToPoint(mrp.point_);
            next_length_so_far = length_so_far + (double)step_length;
            // Omit the situation one edge go crossing one block. It rarely happens in large scale benchmark
            if (block_marks[next_mrp.x_grid_][next_mrp.y_grid_] || block_marks[mrp.x_grid_][mrp.y_grid_]){
                next_mrp.length_in_block_ +=  step_length;
                next_length_so_far += block_factor * step_length;
            }else{
                next_length_so_far -= block_factor * mrp.length_in_block_;//Come back to pure wire length
                next_mrp.length_in_block_ = 0;
            }
            /*-------------put in next MazeRoutingPoint, or update-------------*/
            if (length_marks[next_mrp.x_grid_][next_mrp.y_grid_] == 0){//It is connected part
                continue;
            }
            Buffer min_buffer = buffers_.at(0);//unfinished;
            //Use the minimum buffer to get the possible length in block
            DTYPE min_length_limit = min_buffer.CalculateLengthLimit(slew_spec_, min_buffer.get_input_capacitance());
            if (next_mrp.length_in_block_ < min_length_limit){
                if (length_marks[next_mrp.x_grid_][next_mrp.y_grid_] > next_length_so_far){
                    if (length_marks[next_mrp.x_grid_][next_mrp.y_grid_] == FLT_MAX){
                        //m_heap.put(next_length_so_far, next_mrp);
                        map_heap.insert(pair<double,MRP>(next_length_so_far, next_mrp));
                    }else{
                        //map_heap.decrease(pair<double,MRP>(next_length_so_far, next_mrp))
                        map<double,MRP>::iterator it;
                        for (it = map_heap.begin(); it != map_heap.end(); it++){
                            if (it->second.IsAtSameGrid(next_mrp)){
                                break;   
                            }
                        }
                        //assert(it != map_heap.end()); In mazerouting it should assert, but in FindPointsPath,
                        //This may happen because revisit brings less length_so_far by removing length_in_block
                        //Don't insert in this case, no meaning
                        if (it != map_heap.end()){
                            map_heap.erase(it);
                            map_heap.insert(pair<double,MRP>(next_length_so_far, next_mrp));
                        }else{
                            continue;
                        }
                        //m_heap.decrease(next_length_so_far, next_mrp);
                    }
                    length_marks[next_mrp.x_grid_][next_mrp.y_grid_] = next_length_so_far;
                    direction_marks[next_mrp.x_grid_][next_mrp.y_grid_] = direction;
                }
            }
        }//End of four int_direction loop
            
    }// end of while
}
void Design::TraceBack(const MRP& end_mrp, const vector<DTYPE> & x_grids, const vector<DTYPE>& y_grids, const map<DTYPE,int>& x_grids_map, const map<DTYPE,int>& y_grids_map, const vector<vector<DIRECTION> >& direction_marks, vector<vector<bool> >& outside_tree_marks){
    //--------------------handle end newBranch and endBranch-------------------
    Point end_point = end_mrp.point_;
    Node end_node = tree_.FindAndInsertNewNode(end_point, nodes_);
    //--------------------handle middle branch and start branch-------------------
    int x_grid = end_mrp.x_grid_, y_grid = end_mrp.y_grid_;

    Node node;
    node.index_ = nodes_.size();
    nodes_.push_back(node);//Push_back has to be done first, otherwise SetParentNode et functions are unusable
    tree_.SetParentNode(nodes_, end_node.index_, node.index_);
    tree_.AddChildNode(nodes_, end_node.index_, node.index_);
    node = nodes_.at(node.index_);//node is updated on parent and children
    DIRECTION direction = UNTOUCHED, previous_direction = UNTOUCHED;
    do{
        //-----------updates childPoint, point, x_grid, y_grid---------------
        previous_direction = direction;
        direction  = direction_marks[x_grid][y_grid];
        node.point_ = Point (x_grids.at(x_grid), y_grids.at(y_grid));
        outside_tree_marks[x_grid][y_grid] = true;
        if (direction == UNTOUCHED){//Last branch of the trace
            int parent_node_index = node.parent_;
            tree_.RemoveChildNode(nodes_, parent_node_index, node.index_);
            nodes_.pop_back();
            Node start_node = tree_.FindAndInsertNewNode(node.point_, nodes_);//Already push_backed
            int start_node_index = start_node.index_;
            tree_.SetParentNode(nodes_, parent_node_index, start_node_index);
            tree_.AddChildNode(nodes_, parent_node_index, start_node_index);
            tree_.insert_branch(start_node_index);
            tree_.Rootify(start_node_index, nodes_, parent_node_index);
        }else if (previous_direction != UNTOUCHED){//In the middle, not first branch either
            if (direction != previous_direction){
                nodes_.at(node.index_) = node;
                tree_.insert_index(node.index_);
                tree_.insert_branch(node.index_);

                Node new_node;
                new_node.point_ = node.point_;
                new_node.index_ = nodes_.size();
                nodes_.push_back(new_node);
                tree_.SetParentNode(nodes_, node.index_, new_node.index_);
                tree_.AddChildNode(nodes_, node.index_, new_node.index_);

                node = nodes_.at(new_node.index_);//new_node has been updated children and parent
            }
        }
        if (direction == RIGHT){
            x_grid -= 1;
        }else if (direction == LEFT){
            x_grid += 1;
        }else if (direction == UP){
            y_grid -= 1;
        }else if (direction == DOWN){
            y_grid += 1;
        }
    }while (direction != UNTOUCHED);
}
void Design::ClearGridMarksAndDirections(vector<vector<DIRECTION> >& direction_marks, vector<vector<double> >& length_marks)const{
    for (unsigned int i = 0; i < direction_marks.size(); i++){
        length_marks.at(i) = vector<double> (length_marks.at(i).size(), FLT_MAX);
        direction_marks.at(i) = vector<DIRECTION> (direction_marks.at(i).size(), UNTOUCHED);
    }
}
/*--------------  Third level functions      ------------------*/
vector<Point> Design::FindUnblockedPoints(Point point, int block_index) const{
    vector<Point> points;
    points.push_back(point);
    Poly block = blocks_.at(block_index);
    for (unsigned int i = 0; i < block.Size(); ++i){
        Edge edge = block.GetEdge(i);
        //only take one end for each edge to avoid duplicates
        if (point.IsProjectedOn(edge) && !point.PointsOnSameLine(block.At(i))){
            points.push_back(point.Projection(edge));
        }
    }
    return points;
}
int Design::GetStep(int inside_tree_index, int sink_index)const{
    InsideTree inside_tree = inside_trees_.at(inside_tree_index);
    Node node = nodes_.at(sink_index);
    Node parent_node = nodes_.at(node.parent_);
    int inside_tree_root = inside_tree.get_root();
    if (parent_node.index_ == inside_tree_root){//two escaping point inside tree
        /*int step = 1;
        DTYPE distance_to_root_clock = inside_tree.GetOutsideLength(sink_index, inside_tree_root, step, nodes_);
        step = -1;
        DTYPE distance_to_root_anti_clock = inside_tree.GetOutsideLength(sink_index, inside_tree_root, step, nodes_);
        return distance_to_root_clock>=distance_to_root_anti_clock?1:-1;
        */
        return 0;//Temporarily doing this because no way in build possible point
    }
    Edge first_edge = inside_tree.GetBranchEdge(sink_index, nodes_);//first edge is from sink to its parent
    Edge second_edge = inside_tree.GetBranchEdge(parent_node.index_, nodes_);//first edge is from sink to its parent

    if (first_edge.v_ && second_edge.c_==first_edge.c1_ && second_edge.c1_==first_edge.c_)//down,right
        return 1;
    else if (first_edge.v_ && second_edge.c_==first_edge.c2_ && second_edge.c2_==first_edge.c_)//up,left
        return 1;
    else if (!first_edge.v_ && second_edge.c_==first_edge.c2_ && second_edge.c1_==first_edge.c_)//right,up
        return 1;
    else if (!first_edge.v_ && second_edge.c_==first_edge.c1_ && second_edge.c2_==first_edge.c_)//left,down
        return 1;
    else if (!first_edge.v_ && second_edge.c_==first_edge.c1_ && second_edge.c1_==first_edge.c_)//left,up
        return -1;
    else if (!first_edge.v_ && second_edge.c_==first_edge.c2_ && second_edge.c2_==first_edge.c_)//right,down
        return -1;
    else if (first_edge.v_ && second_edge.c_==first_edge.c2_ && second_edge.c1_==first_edge.c_)//up,right
        return -1;
    else if (first_edge.v_ && second_edge.c_==first_edge.c1_ && second_edge.c2_==first_edge.c_)//down,left
        return -1;
    else{
        return 0;//The jogging or end of max segment part
    }
}
PossiblePoint Design::AddCombinationChoice(int sink_index, int worst_sink_index, DTYPE distance_to_next_sink, double output_slew, map<int,double> slew_map, int inside_tree_index)const
{
    vector<Node> new_nodes = nodes_;
    InsideTree new_inside_tree = inside_trees_.at(inside_tree_index);
    DTYPE original_wirelength = new_inside_tree.CalculateTotalWirelength(new_nodes);
    new_inside_tree.remove_sink(sink_index);
    new_inside_tree.remove_index(sink_index);
    new_inside_tree.remove_branch(sink_index);
    Node node = new_nodes.at(sink_index);
    int parent_index = node.parent_;
    new_inside_tree.RemoveChildNode(new_nodes, parent_index, sink_index);
    new_inside_tree.CleanNode(new_nodes, parent_index);
    DTYPE new_wirelength = new_inside_tree.CalculateTotalWirelength(new_nodes);

    double worst_slew = slew_map.find(worst_sink_index)->second;
    double worst_ramp_slew = sqrt(worst_slew*worst_slew - output_slew*output_slew);
    PossiblePoint possible_point;
    possible_point.set_node_index(sink_index);
    possible_point.set_point(Point(0,0));
    possible_point.set_parent_point(Point(0,0));
    possible_point.set_wirelength_increase(distance_to_next_sink + new_wirelength-original_wirelength);
    if (sink_index == worst_sink_index){
        possible_point.set_output_slew_increase(-output_slew);//Actually output_slew is not 0. But it is 0 for the calculation of worst sink slew
        possible_point.set_ramp_slew_increase(-worst_ramp_slew);
        return possible_point;
    }else{
        new_inside_tree.InitializeCap(new_nodes);
        new_inside_tree.PropagateCap(new_nodes);
        //double new_output_slew = new_inside_tree.CalculateOutputSlew(new_nodes);
        double new_output_slew = BETA*slew_spec_;
        map<int,double> new_elmore_map = new_inside_tree.CalculateElmore(new_nodes);
        double new_ramp_slew = log(9)*new_elmore_map.find(worst_sink_index)->second;
        possible_point.set_output_slew_increase(new_output_slew-output_slew);
        possible_point.set_ramp_slew_increase(new_ramp_slew - worst_ramp_slew);
        return possible_point;
    }
}
PossiblePoint Design::AddOneSlideChoice(int inside_tree_index, Point new_sink_point,Point new_parent_point, int sink_index, int worst_sink_index, DTYPE distance_to_original_sink, double output_slew, map<int,double> slew_map)const{
    //set up a dummy new tree to have slew change information
    vector<Node> new_nodes = nodes_;
    InsideTree new_inside_tree = inside_trees_.at(inside_tree_index);
    DTYPE original_wirelength = new_inside_tree.CalculateTotalWirelength(new_nodes);
    UpdateOneNode(new_inside_tree, new_nodes, new_sink_point, new_parent_point, sink_index);
    DTYPE new_wirelength = new_inside_tree.CalculateTotalWirelength(new_nodes);

    new_inside_tree.InitializeCap(new_nodes);
    new_inside_tree.PropagateCap(new_nodes);
    //double new_output_slew = new_inside_tree.CalculateOutputSlew(new_nodes);
    double new_output_slew = BETA*slew_spec_;
    map<int,double> new_elmore_map = new_inside_tree.CalculateElmore(new_nodes);
    double new_ramp_slew = log(9)*new_elmore_map.find(worst_sink_index)->second;
    int block_index = new_inside_tree.get_block_index();
    Poly block = blocks_.at(block_index);
    PossiblePoint possible_point;
    possible_point.set_node_index(sink_index);
    possible_point.set_point(new_sink_point);
    possible_point.set_parent_point(new_parent_point);
    possible_point.set_wirelength_increase(distance_to_original_sink + new_wirelength-original_wirelength);
    possible_point.set_output_slew_increase(new_output_slew-output_slew);
    double worst_slew = slew_map.find(worst_sink_index)->second;
    double worst_ramp_slew = sqrt(worst_slew*worst_slew - output_slew*output_slew);
    possible_point.set_ramp_slew_increase(new_ramp_slew - worst_ramp_slew);
    return possible_point;
}
int Design::GetBlockEdgeIndex (int block_index, int sink_index)const{//return the start point of the edge
    Poly block = blocks_.at(block_index);
    Point point = nodes_.at(sink_index).point_;
    for (unsigned int i = 0; i < block.Size(); ++i){
        Edge edge = block.GetEdge(i); 
        if (point.IsOnEdge(edge))
            return i;
    }
    assert(1);//Should not go here
    return -1;//that's not gonna happen
}
//Find edge numbers first. Find one with minimum edge number difference. If two with same gap, choose the one closer to start_point
int Design::FindClosestTerminalOnBlockBoundary (int inside_tree_index, int terminal_index, DTYPE *distance_pointer)const{
    DTYPE distance_to_next_clockwise_sink = 0;
    DTYPE distance_to_next_anticlockwise_sink = 0;
    int next_clockwise_sink_index = FindNextTerminalOnBlockBoundary (inside_tree_index, 1, terminal_index, &distance_to_next_clockwise_sink);
    int next_anticlockwise_sink_index = FindNextTerminalOnBlockBoundary (inside_tree_index, -1, terminal_index, &distance_to_next_anticlockwise_sink);
    *distance_pointer = min(distance_to_next_clockwise_sink, distance_to_next_anticlockwise_sink);
    return distance_to_next_clockwise_sink<distance_to_next_anticlockwise_sink?
        next_clockwise_sink_index:next_anticlockwise_sink_index;
}
int Design::FindNextTerminalOnBlockBoundary (int inside_tree_index, int step, int terminal_index, DTYPE *distance_pointer)const{
    Node node = nodes_.at(terminal_index);
    Point point = node.point_;
    InsideTree inside_tree = inside_trees_.at(inside_tree_index);
    int block_index = inside_tree.get_block_index();
    Poly block = blocks_.at(block_index);
    set<int> inside_tree_sinks = inside_tree.get_sinks();
    DTYPE distance_to_next_terminal = INT_MAX;
    int next_terminal_index;
    for (set<int>::const_iterator it = inside_tree_sinks.begin(); it != inside_tree_sinks.end(); it++){
        int check_terminal_index = *it;
        Node check_terminal_node = nodes_.at(check_terminal_index);
        Point check_terminal_point = check_terminal_node.point_;
        if (check_terminal_index == terminal_index){//Shouldn't check with it self, change to root to save lines
            check_terminal_index = inside_tree.get_root();
            check_terminal_point = inside_tree.get_root_point(nodes_);
        }
        DTYPE check_distance = block.GetOutsideLength(point, check_terminal_point, step);
        if (check_distance < distance_to_next_terminal){
            distance_to_next_terminal = check_distance;
            next_terminal_index = check_terminal_index;
        }
    }
    *distance_pointer = distance_to_next_terminal;
    return next_terminal_index;
}
/*--------------  Fourth level functions      ------------------*/
void Design::UpdateOneNode(InsideTree& inside_tree, vector<Node>& nodes, Point new_sink_point,Point new_parent_point, int sink_index)const{
    //Update the position of sink
    Node node = nodes.at(sink_index);
    Point old_sink_point = node.point_;
    node.point_ = new_sink_point;
    nodes.at(sink_index) = node;

    int old_parent_index = node.parent_;
    inside_tree.RemoveChildNode(nodes, old_parent_index, sink_index);

    int range_parent_index,range_child_index;
    inside_tree.FindEdgeIndex(new_parent_point, nodes, &range_parent_index,&range_child_index);
    DirectionalEdge d_edge = inside_tree.GetBranchEdge(range_child_index, nodes);
    if (new_parent_point != d_edge.get_start_point() && new_parent_point != d_edge.get_end_point()){
        Node new_parent_node = inside_tree.InsertNewNode(range_parent_index, range_child_index, new_parent_point, nodes);
        int new_parent_index = new_parent_node.index_;
        inside_tree.AddChildNode(nodes, new_parent_index, sink_index);
        inside_tree.SetParentNode(nodes, new_parent_index, sink_index);
    }else if (new_parent_point == d_edge.get_start_point()){
        inside_tree.AddChildNode(nodes, range_child_index, sink_index);
        inside_tree.SetParentNode(nodes, range_child_index, sink_index);
    }else{
        inside_tree.AddChildNode(nodes, range_parent_index, sink_index);
        inside_tree.SetParentNode(nodes, range_parent_index, sink_index);
    }
    inside_tree.CleanNode(nodes, old_parent_index);//Remove tail point, remove useless middle point
}
/*--------------  All over functions      ------------------*/
bool Design::PointIsInBlocks(const Point & point, int* block_index_pointer) const{//strictly in
    for (vector<Poly >::const_iterator it = blocks_.begin(); it != blocks_.end(); ++it){//for block corners
        Poly block = *it;
        if (point.IsInsidePoly(block)){
            if (block_index_pointer != NULL){
                *block_index_pointer = it-blocks_.begin();
            }
            return true;
        }
    }
    if (block_index_pointer != NULL){
        * block_index_pointer = -1;
    }
    return false;
}
bool Design::PointIsOnBlocks(const Point & point, int* block_index_pointer) const{//strictly in
    for (vector<Poly >::const_iterator it = blocks_.begin(); it != blocks_.end(); ++it){//for block corners
        Poly block = *it;
        if (point.IsOnPoly(block)){
            if (block_index_pointer != NULL){
                *block_index_pointer = it-blocks_.begin();
            }
            return true;
        }
    }
    if (block_index_pointer != NULL){
        * block_index_pointer = -1;
    }
    return false;
}
bool Design::IsInSameInsideTree(int node1_index, int node2_index, int* inside_tree_index_pointer)const{
    for (unsigned int i =0; i < inside_trees_.size(); i++){//This is needed since right now inside and outside tree are not fully connected
        InsideTree inside_tree = inside_trees_.at(i);
        set<int> inside_tree_indexes = inside_tree.get_indexes(); 
        if (inside_tree_indexes.find(node1_index) != inside_tree_indexes.end() &&
                inside_tree_indexes.find(node2_index) != inside_tree_indexes.end()){
            if (inside_tree_index_pointer != NULL){
                *inside_tree_index_pointer = i;
            }
            return true;
        }
    }
    if (inside_tree_index_pointer != NULL){
        *inside_tree_index_pointer = -1;
    }
    return false;
}
    /*--------------  Buffering functions      ------------------*/
list<Point> Design::FindPointsPath(Point child_point, Point point)const{//Find path from child_point to point
    list<Point> points_list;
    vector<DTYPE> x_grids;
    vector<DTYPE> y_grids;
    map<int, DTYPE> x_grids_map;
    map<int, DTYPE> y_grids_map;
    vector<vector<bool> > block_marks;//It is true for only inside block node
    vector<vector<DIRECTION> > direction_marks;
    vector<vector<double> > length_marks;
    vector<vector<bool> > outside_tree_marks;//Marked if it is on(include two end) of an outside tree branch
    GenerateGrid(x_grids, y_grids, x_grids_map, y_grids_map, block_marks, direction_marks, length_marks, outside_tree_marks);
    int x_grid = x_grids_map.find(point.x)->second;
    int y_grid = y_grids_map.find(point.y)->second;
    int child_x_grid = x_grids_map.find(child_point.x)->second;
    int child_y_grid = y_grids_map.find(child_point.y)->second;
    outside_tree_marks[x_grid][y_grid] = true;                
    length_marks[child_x_grid][child_y_grid] = 0.0;
    multimap<double, MRP> map_heap;
    MRP start_mrp;
    start_mrp.x_grid_ = child_x_grid;
    start_mrp.y_grid_ = child_y_grid;
    start_mrp.point_ = child_point; 
    map_heap.insert(pair<double, MRP>(0, start_mrp));//Only start from the connected outside tree
    double length_so_far, next_length_so_far;
    while (!map_heap.empty()){
        map<double,MRP>::iterator it = map_heap.begin();
        MRP mrp = it->second;
        length_so_far = it->first;
        map_heap.erase(it); 
        if (outside_tree_marks[mrp.x_grid_][mrp.y_grid_]){//Some times child_point overlap point
            break;//Is on outside trees and it is not the initial connected nodes
        }
        for (int int_direction = RIGHT; int_direction <= DOWN; int_direction++){
            MRP next_mrp = mrp;
            DIRECTION direction;
            if ( int_direction == (int)RIGHT){
                if (mrp.x_grid_ < x_grids.size()-1){//x_grid_+1
                    next_mrp.x_grid_ = mrp.x_grid_ + 1;
                }else{
                    continue;
                }
                direction = RIGHT;
            }else if (int_direction == (int)LEFT){
                if (mrp.x_grid_ > 0){
                    next_mrp.x_grid_ = mrp.x_grid_ - 1; 
                }else{
                    continue;
                }
                direction = LEFT;
            }else if (int_direction == (int)UP){
                if (mrp.y_grid_ < y_grids.size()-1){
                    next_mrp.y_grid_ = mrp.y_grid_ + 1; 
                }else{
                    continue;
                }
                direction = UP;
            }else if (int_direction == (int)DOWN){
                if (mrp.y_grid_ > 0){
                    next_mrp.y_grid_ = mrp.y_grid_ - 1; 
                }else{
                    continue;
                }
                direction = DOWN;
            }
            next_mrp.point_ = Point (x_grids.at(next_mrp.x_grid_), y_grids.at(next_mrp.y_grid_));
            DTYPE step_length = next_mrp.point_.DistanceToPoint(mrp.point_);
            next_length_so_far = length_so_far + (double)step_length;
            if (block_marks[next_mrp.x_grid_][next_mrp.y_grid_] || block_marks[mrp.x_grid_][mrp.y_grid_]
                    || PointIsInBlocks((mrp.point_+next_mrp.point_)/2)){
                next_length_so_far += LARGEBLOCKFACTOR * step_length;
            }
            /*-------------put in next MazeRoutingPoint, or update-------------*/
            if (length_marks[next_mrp.x_grid_][next_mrp.y_grid_] == 0){//It is connected part
                continue;
            }
            assert(next_length_so_far > 0);
            if (length_marks[next_mrp.x_grid_][next_mrp.y_grid_] > next_length_so_far){
                if (length_marks[next_mrp.x_grid_][next_mrp.y_grid_] == FLT_MAX){
                    map_heap.insert(pair<double,MRP>(next_length_so_far, next_mrp));
                }else{
                    map<double,MRP>::iterator it;
                    for (it = map_heap.begin(); it != map_heap.end(); it++){
                        if (it->second.IsAtSameGrid(next_mrp)){
                            break;   
                        }
                    }
                    assert(it != map_heap.end());
                    map_heap.erase(it);
                    map_heap.insert(pair<double,MRP>(next_length_so_far, next_mrp));
                }
                length_marks[next_mrp.x_grid_][next_mrp.y_grid_] = next_length_so_far;
                direction_marks[next_mrp.x_grid_][next_mrp.y_grid_] = direction;
            }
        }//End of four int_direction loop
    }// end of while
    DIRECTION direction = UNTOUCHED, previous_direction = UNTOUCHED;
    do{
        //-----------updates childPoint, point, x_grid, y_grid---------------
        previous_direction = direction;
        direction  = direction_marks[x_grid][y_grid];
        Point point = Point (x_grids.at(x_grid), y_grids.at(y_grid));
        points_list.push_back(point);
        if (direction == RIGHT){
            x_grid -= 1;
        }else if (direction == LEFT){
            x_grid += 1;
        }else if (direction == UP){
            y_grid -= 1;
        }else if (direction == DOWN){
            y_grid += 1;
        }
    }while (direction != UNTOUCHED);
    points_list.reverse();//After reverse, the first point is child_point, the last one is point
    return points_list;
}
DTYPE Design::CalculateDistanceToEnd(Point solution_point, list<Point> points_list)const{
    DirectionalEdge d_edge;
    if (solution_point == points_list.back()){//If solution_point is the end of points_list
        return 0;
    }
    while(true){
        Point point = points_list.front();
        points_list.pop_front();
        assert(points_list.size() > 0);//Shouldn't stay in the loop till point
        Point next_point = points_list.front();
        d_edge = DirectionalEdge(point, next_point);
        if (solution_point.IsOnEdge(d_edge) && solution_point != d_edge.get_end_point()){
            d_edge.set_start_point(solution_point);
            break;
        }
    }
    DTYPE distance_to_end = 0;
    do{
        DTYPE length = d_edge.Length();
        distance_to_end += length;
        Point point = points_list.front();
        points_list.pop_front();
        if (points_list.size() == 0){
            return distance_to_end;
        }
        Point next_point = points_list.front();
        d_edge = DirectionalEdge(point, next_point);
    }while(true);
}
Point Design::FindUpPointWithDistance(Point solution_point, list<Point> points_list, DTYPE distance)const{
    DirectionalEdge d_edge;
    while(true){
        Point point = points_list.front();
        points_list.pop_front();
        assert(point != points_list.back());//Shouldn't stay in the loop till point
        Point next_point = points_list.front();
        d_edge = DirectionalEdge(point, next_point);
        if (solution_point.IsOnEdge(d_edge) && solution_point != d_edge.get_end_point()){
            d_edge.set_start_point(solution_point);
            break;
        }
    }
    do{
        DTYPE length = d_edge.Length();
        if (distance > length){
            distance -= length;
        }else{
            return d_edge.GetProportionPoint((double)distance/(double)length);
        }
        Point point = points_list.front();
        points_list.pop_front();
        assert(point != points_list.back());//Shouldn't stay in the loop till point
        Point next_point = points_list.front();
        d_edge = DirectionalEdge(point, next_point);
    }while(true);
}

void Design::Write_BOBRSMT(){
    string benchmark_name(benchmark_name_);
	string file_name = "../benchmarks/" + benchmark_name  + "_BOBRSMT/" + benchmark_name + "_BOBRSMT.branch";
	const char * c = file_name.c_str();
  ofstream outfile(c);

  outfile<< "DegreeNumber : "<< nodes_.size()<< endl;//Because dual edges(2 directions)
  //Root to root itself, because the buffering tool needs this format...
  for(size_t i=0;i<nodes_.size();++i){
      //Use to_node to from_node which is bottom to up. This manner enables the first node of the branch to be unique
      outfile<< i <<" "<< nodes_.at(i).point_.x<<" "<<nodes_.at(i).point_.y<<" "<<nodes_.at(i).parent_<<endl;
  }
  outfile.close();
}

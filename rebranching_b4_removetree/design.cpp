#include "design.h"

using namespace std;

/*---------------------------  Design functions   -----------------------------*/
Design::Design (char work_path[], char benchmark_name[], double slew_percentage){
    strcpy(work_path_, work_path);

    strcpy(benchmark_path_, work_path);
    strcat(benchmark_path_, benchmark_name);

    strcpy(block_file_, benchmark_name);
    strcat(block_file_, ".pblock");

    strcpy(branch_file_, "flute.branch");

    slew_percentage_ = slew_percentage;
    InitialXYMinMax();
}
Design::Design (){
    InitialXYMinMax();
}

void Design::ReadBlockFile (){//Only read root and step size now
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
}
void Design::ReformBranch (){
    //index includes all valid nodes, including the center node
    set<int> index = RemoveZeroLengthBranch(nodes_);
    CompactNodes(root_, index, nodes_);//Remove all invalid nodes from nodes_, this save "index_"
    /*--------- Here is make a sequence----*/
    InitialSequenceBranch(root_, nodes_);//This is not universal for tree, it is only effective for flute result
    /*--------- Here to set all sinks according to sequenfied relations----*/
//    tree_.set_indexes (index);
//    tree_.generate_branches();
    sinks_ = RecognizeSinks(nodes_);
//    tree_.set_sinks (sinks_);
    /*--------- split L shape branchs----*/
//    tree_.BreakLShape(nodes_);//This is tree function because it can be used in tree in any situation
 //   tree_.StoreChildrenNode(nodes_);//This is tree function because children is a concept in tree
}
void Design::ReadBuffer (){
    double input_capacitance = 1;
    double driving_resistance = 1;
    double intrinsic_delay = 1;
    double slew_resistance = 1;
    double intrinsic_slew = 1;
    double cost = 1;
    Buffer buffer(input_capacitance, driving_resistance, intrinsic_delay, slew_resistance, intrinsic_slew, cost);
    buffers_.push_back(buffer); 
}
void Design::PlotAndSave(const Tree& tree)const{//generate line doc for gnu
    //return;
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
        fprintf (fp, "set object %d polygon from ", 1 + it2-blocks_.begin());//from 1
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
    assert (indexes.size() > 0);
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
        }
    }
    //Plot buffers in purple
    list<pair<Point,Buffer> > buffer_locations = best_solution_.get_buffer_locations();
    printf ("The buffer number is %d\n", buffer_locations.size());
    if (buffer_locations.size() !=0 ){
        for (list<pair<Point,Buffer> >::const_iterator buffer_location = buffer_locations.begin(); buffer_location != buffer_locations.end(); ){
            Point buffer_location_point = buffer_location->first;
            printf ("The location is at %f %f\n", (double)buffer_location_point.x,(double)buffer_location_point.y);
            fprintf (fp, ", ");
            fprintf(fp, "\"<echo '%f %f'\" notitle with points pointtype 6 pointsize 1 linecolor rgb \"purple\"",
                    (double)buffer_location_point.x,(double)buffer_location_point.y);
            buffer_location++;
        }
    }
    fprintf (fp, "\n");
    //Save to file
    fprintf (fp, "set terminal png\n");  
    fprintf (fp, "set output \"%s/my_plot_%d.png\"\n", benchmark_path_, count);//gthumb Erint
    //Plot on screen
    fprintf (fp, "replot\n");
    fprintf (fp, "set terminal x11\n");
    fprintf (fp, "pause -1\n");
    pclose (fp);
    return;

}
void Design::PlotAndSave(const vector<Node> & nodes)const{//generate line doc for gnu
    //return;
    if (nodes.size() == 0)
        return;//Nothing to do with an empty tree
    static int count = -1;
    count++;//Every time plotTree got called, count+1. Then we can save picture with different name

    FILE * fp = popen ("gnuplot -persist", "w");
    //Plot the current tree edge
    DTYPE plot_x_min = INT_MAX, plot_x_max = 0, plot_y_min = INT_MAX, plot_y_max = 0;
    for (int i = 0; i < nodes.size(); ++i) {
        Node node = nodes_.at(i);
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
        fprintf (fp, "set object %d polygon from ", 1 + it2-blocks_.begin());//from 1
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
    for (int i = 0; i < nodes.size();) {
        if (i == 0)
            fprintf (fp, "plot ");
        Node node = nodes_.at(i);
        Node parent_node = nodes_.at(node.parent_);
        if (i == 0){//root
            fprintf(fp, "\"<echo '%f %f'\" notitle with points pointtype 7 pointsize 1 linecolor rgb \"red\"", 
                    (double)node.point_.x,(double)node.point_.y);
        }else if (node.type_ == Node::SINK){
            fprintf(fp, "\"<echo '%f %f'\" notitle with points pointtype 4 pointsize 1 linecolor rgb \"green\"",
                    (double)node.point_.x,(double)node.point_.y);
        }else{
            fprintf(fp, "\"<echo '%f %f'\" notitle with points pointtype 4 pointsize 1 linecolor rgb \"blue\"",
                    (double)node.point_.x,(double)node.point_.y);
        }
        i++;
        if (i != nodes.size()){
            fprintf (fp, ", ");
        }
    }
    //Plot buffers in purple
    list<pair<Point,Buffer> > buffer_locations = best_solution_.get_buffer_locations();
    printf ("The buffer number is %d\n", buffer_locations.size());
    if (buffer_locations.size() !=0 ){
        for (list<pair<Point,Buffer> >::const_iterator buffer_location = buffer_locations.begin(); buffer_location != buffer_locations.end(); ){
            Point buffer_location_point = buffer_location->first;
            printf ("The location is at %f %f\n", (double)buffer_location_point.x,(double)buffer_location_point.y);
            fprintf (fp, ", ");
            fprintf(fp, "\"<echo '%f %f'\" notitle with points pointtype 6 pointsize 1 linecolor rgb \"purple\"",
                    (double)buffer_location_point.x,(double)buffer_location_point.y);
            buffer_location++;
        }
    }
    fprintf (fp, "\n");
    //Save to file
    fprintf (fp, "set terminal png\n");  
    fprintf (fp, "set output \"%s/my_plot_%d.png\"\n", benchmark_path_, count);//gthumb Erint
    //Plot on screen
    fprintf (fp, "replot\n");
    fprintf (fp, "set terminal x11\n");
    fprintf (fp, "pause -1\n");
    pclose (fp);
    return;

}
void Design::SortAndRecordSlew (){//Keep original name although it is giving an estimation on slew spec now
    double max_slew = 0, min_slew = INT_MAX;
    Tree tree = get_tree();
    tree.InitializeCap(nodes_);
    tree.PropagateCap(nodes_);
    double output_slew = tree.CalculateOutputSlew(nodes_);
    map<int,double> elmore_map = tree.CalculateElmore(nodes_);
    map<int,double> slew_map;
    for (map<int,double>::const_iterator it = elmore_map.begin(); it != elmore_map.end(); it++){
        double ramp_slew = it->second*log(9);
        double input_slew = sqrt(ramp_slew*ramp_slew + output_slew*output_slew);
        slew_map.insert(pair<int,double>(it->first,input_slew));
        min_slew = min (min_slew, input_slew);
        max_slew = max (max_slew, input_slew);
    }
    slew_spec_ = min_slew + slew_percentage_*(max_slew-min_slew);
//    slew_spec_ = 100;
    //length_limit_ = ( -(RUNIT*Cb+Rs*CUNIT)+sqrt(pow((RUNIT*Cb+Rs*CUNIT),2) - 2*RUNIT*CUNIT*(Rs*Cb-slew_spec_)) ) / (RUNIT*CUNIT);
    cout<< "The min slew is "<< min_slew<< ". The max slew is "<< max_slew<< ". The slew spec is "<< slew_spec_<< endl;
}

Tree Design::get_tree()const{
    return tree_;
}
vector<Node> Design::get_nodes(){
    return nodes_;
}
void Design::Clean(){
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
set<int> Design::RemoveZeroLengthBranch(vector<Node> & nodes){
    set<int> index;//which record all valid nodes
    for (int node_index=0; node_index < nodes.size(); node_index++){//if i!=j and point(i) == point(j) and parent(i) == j
        Node node = nodes.at(node_index);
        int parent_index = node.parent_;
        Node parent_node = nodes.at(parent_index);
        if (node.point_ == parent_node.point_ && node_index != parent_index){//0 length branch remove, and there is one branch in the middle point to itself, leave this one alone cause it help to make the sequence
            for (int j = 0; j < nodes.size(); j++){
                Node check_node = nodes.at(j);
                if (check_node.parent_ == node.index_)  
                    nodes.at(j).parent_ = parent_index;
            }
            if (root_ == node_index){
                root_ = parent_index;
            }
        }else{
            index.insert(node_index);
        }
    }
    for (int node_index=0; node_index < nodes.size(); node_index++){//if parent(i) == i and i is not parent of any node, remove this idle cycle
        if (node_index == root_)    
            continue;
        Node node = nodes.at(node_index);
        int parent_index = node.parent_;
        Node parent_node = nodes.at(parent_index);
        if (node_index == parent_index){//0 length branch remove, and there is one branch in the middle point to itself, leave this one alone cause it help to make the sequence
            bool used = false;
            for (int j = 0; j < nodes.size(); j++){
                if (j == node_index)
                    continue;
                Node check_node = nodes.at(j);
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
//1. Remove useless nodes 2.Swap root to 0 in nodes
void Design::CompactNodes(int root, set<int> valid_index, vector<Node>& nodes){
    map<int,int> m1;//key is 0,1,2,3...; value is 0,2,3,5,...
    map<int,int> m2;//key is 0,2,3,5,...; value is 0,1,2,3,...
    int i = 1;
    for (set<int>::iterator it = valid_index.begin(); it!=valid_index.end(); it++){
        int valid = *it;
        if (valid == root){
            m1[0] = valid;
            m2[valid] = 0;
        }else{
            m1[i] = valid;
            m2[valid] = i;
            i++;
        }
    }
    int valid_size = valid_index.size();
    assert(m1.size()==valid_size);
    assert(m2.size()==valid_size);

    vector<Node> new_nodes;
    for (int i = 0; i < valid_size; i++){
        Node new_node;
        new_node = nodes[m1[i]];
        new_node.parent_ = m2[new_node.parent_];
        set<int> children = new_node.get_children();
        set<int> new_children;
        for (set<int>::iterator it = children.begin(); it != children.end(); it++){
            new_children.insert(m2[*it]);
        }
        new_node.set_children(new_children);
        new_nodes.push_back(new_node);
    }

    nodes = new_nodes;
}
//Not only return the set of sinks, but also change all NodeType of sink nodes to SINK
set<int> Design::RecognizeSinks(vector<Node>& nodes)const{
    set<int> sink_set;
    for (int i = 0; i < nodes.size(); i++){
        sink_set.insert(i);
    }
    for (int i = 0; i < nodes.size(); i++){
        sink_set.erase(nodes[i].parent_);
    }
    for (set<int>::iterator it = sink_set.begin(); it!=sink_set.end(); it++){
        nodes[*it].type_ = Node::SINK;
    }
    return sink_set;
}

//Change all pointers, which are pointing reverse way, to root. Only useful on the result of flute
void Design::InitialSequenceBranch (int root, vector<Node> & nodes) {
    int parent = root;
    int current = nodes[root].parent_;
    while (current!=parent){//break when to the branch with n of itself. Just reverse all adverse branch
        int next = nodes[current].parent_;
        nodes[current].parent_ = parent;
        parent = current;
        current = next;
    }
    nodes[root].parent_ = root;//Last set root correctly. Root is the only one with itself as parent
    nodes[root].type_ = Node::ROOT;
}
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
/*--------------  Timing/Slew functions  --------------------*/
void Design::InitializeCap(vector<Node> & nodes)const{
    for (int node_index = 0; node_index < nodes.size(); node_index++){
        if (sinks_.find(node_index) != sinks_.end()) 
//        if (nodes.at(node_index).type_ == Node::SINK)
            nodes.at(node_index).ct_ = Cb;//Initialize capacitance of sinks
        else if (nodes.at(node_index).type_ == Node::BUFFER)
            assert(nodes.at(node_index).ct_ == nodes.at(node_index).buffer_.get_input_capacitance());
        else 
            nodes.at(node_index).ct_ = 0;//Initialize capacitance of branch point
    }
}
void Design::PropagateCap(vector<Node> & nodes)const{
    for (int node_index = 0; node_index < nodes.size(); node_index++){
        if (node_index == root_)
            continue;
        Node node = nodes.at(node_index);
        int parent_node_index = node.parent_;
        Node parent_node = nodes.at(parent_node_index);
        Edge edge(node.point_, parent_node.point_);
        DTYPE distance = edge.Length();
        double cap = distance * CUNIT;
        if (sinks_.find(node_index) != sinks_.end())//If it is sink, beside path cap, sink cap should be included 
            cap += node.ct_;
        while (node_index != root_){
            nodes.at(parent_node_index).ct_ += cap;
            node_index = parent_node_index;
            node = nodes.at(node_index);
            parent_node_index = node.parent_;
        }

    }
}

void Design::PropagateElmore(double delay, int node_index, const vector<Node>& nodes, map<int,double>& delay_map)const{
    Node node = nodes.at(node_index);
    set<int> children =  node.get_children();
    //Not a sink of the tree(might be inside tree, for which sink has children)
    if (sinks_.find(node_index) == sinks_.end()){
        for (set<int>::const_iterator it = children.begin(); it != children.end(); it++){
            int child_node_index = *it;
            Node child_node = nodes.at(child_node_index);
            Edge edge(child_node.point_, node.point_);
            DTYPE distance = edge.Length();
            double child_delay = delay + RUNIT*distance*(0.5*CUNIT*distance + child_node.ct_);
            PropagateElmore(child_delay, child_node_index, nodes, delay_map);
        }
    }else{
        delay_map.insert(pair<int,double>(node_index, delay));//Add sink elmore and index to the map
    }
}
map<int,double> Design::CalculateElmore(vector<Node>& nodes)const{//Calculate Elmore values for all EP and MP. EP and MP may have different size due to eliminating MP
    map<int,double> delay_map;
    //double root_delay = root_buffer_.driving_resistance_ * nodes.at(root_).ct_ + root_buffer_.intrinsic_delay_;
    double root_delay = 0;//The elmore calculation start from behind of the root driver
    PropagateElmore(root_delay, root_, nodes, delay_map);
    return delay_map;
}

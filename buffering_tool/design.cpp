#include "design.h"

using namespace std;

/*---------------------------  Design functions   -----------------------------*/
Design::Design (char work_path[], char benchmark_name[], double slew){
    strcpy(work_path_, work_path);

    strcpy(benchmark_path_, work_path);
    strcat(benchmark_path_, benchmark_name);

    strcpy(block_file_, benchmark_name);
    strcat(block_file_, ".pblock");

    strcpy(branch_file_, benchmark_name);
    strcat(branch_file_, ".branch");

    slew_spec_ = slew;
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

    fgets(line, LINESIZE, fp);
    sscanf(line, "%s\t%*s\n", temp);
    //sscanf(line, "SlewSpec\t:\t%lf\n", &slew_spec_);
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
    //1. Remove useless nodes 2.Swap root to 0 in nodes
    CompactNodes(root_, index, nodes_);
    /*--------- Here is make a sequence----*/
    InitialSequenceBranch(0, nodes_);//This is not universal for tree, it is only effective for flute result
    /*--------- Here to set all sinks according to sequenfied relations----*/
    //    tree_.set_indexes (index);
    //    tree_.generate_branches();
    sinks_ = RecognizeSinks(nodes_);
    //    tree_.set_sinks (sinks_);
    /*--------- split L shape branchs----*/
    BreakLShape(nodes_);//This is tree function because it can be used in tree in any situation
    //Children info valid after this function. Before this, no children info stored
    StoreChildrenNode(nodes_);//This is tree function because children is a concept in tree
}

void Design::BreakLShape(vector<Node>& nodes){
    for (int index = 0; index < nodes.size(); ++index){
        Point point = nodes[index].point_;
        int parent_node_index = nodes[index].parent_;
        Point parent_point = nodes[parent_node_index].point_;
        if (!point.PointsOnSameLine(parent_point)){//If between point and parent_point is L shape
            //Deside the shape of L
            int support_vertical_first = 0;
            int support_horizontal_first = 0;
            for (int child_branch_index = 0; child_branch_index < nodes.size(); ++child_branch_index){
                if (nodes[child_branch_index].parent_ == index){
                    Point child_point = nodes[child_branch_index].point_;
                    if (child_point.PointsOnSameHorizontalLine(point)){
                        support_horizontal_first++;
                    }else if (child_point.PointsOnSameVerticalLine(point)){
                        support_vertical_first++;
                    }
                }
            }
            int grandparent_point_index = nodes[parent_node_index].parent_;
            Point grandparent_point = nodes[grandparent_point_index].point_;
            if (parent_point.PointsOnSameHorizontalLine(grandparent_point)){
                support_horizontal_first++;
            }else if (parent_point.PointsOnSameVerticalLine(grandparent_point)){
                support_vertical_first++;
            }
            //Insert new Node into nodes 
            Point corner_point;
            if (support_vertical_first >= support_horizontal_first){
                corner_point.x = point.x;
                corner_point.y = parent_point.y;
            }else{
                corner_point.y = point.y;
                corner_point.x = parent_point.x;
            }
            Node corner_node;
            corner_node.point_ = corner_point;
            corner_node.index_ = nodes.size();
            corner_node.parent_ = parent_node_index;
            nodes.push_back(corner_node);
            nodes[index].parent_ = corner_node.index_;
        }
    }
}

void Design::StoreChildrenNode(vector<Node> & nodes){
    for (int child = 1; child < nodes.size(); ++child){
        Point point = nodes.at(child).point_;
        int parent_node_child = nodes.at(child).parent_;
        nodes.at(parent_node_child).children_.insert(child);
    }
}
//Read all buffers and put a largest at the root
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
void Design::SetRootBuffer(vector<Node> & nodes){
    nodes[0].buffer_ = buffers_.at(buffers_.size()-1);
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
    for (int i = 1; i < nodes.size(); ++i) {
        Node node = nodes.at(i);
        Node parent_node = nodes.at(node.parent_);
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
        Node node = nodes.at(i);
        Node parent_node = nodes.at(node.parent_);
        if (node.type_ == Node::ROOT){//root
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
void Design::SortAndRecordSlew (vector<Node>& nodes){//Keep original name although it is giving an estimation on slew spec now
    double max_slew = 0, min_slew = INT_MAX;
    InitializeCap(nodes);
    PropagateCap(nodes);
#ifdef PRINT
    for (int i = 0; i < nodes.size(); i++){
        printf ("Cap at Node%d(%d, %d) is %f\n", 
                i, nodes.at(i).point_.x, nodes.at(i).point_.y, nodes.at(i).ct_);
    }
#endif
    map<int,double> slew_map, elmore_map;
    CalculateElmore(nodes, slew_map, elmore_map);
    slew_map.clear();
    for (map<int,double>::const_iterator it = elmore_map.begin(); it != elmore_map.end(); it++){
#ifdef PRINT
        printf ("Delay at Node%d(%d, %d) is %f\n", 
                it->first, nodes.at(it->first).point_.x, nodes.at(it->first).point_.y, it->second);
#endif
        double ramp_slew = it->second*log(9);
        double input_slew = sqrt(ramp_slew*ramp_slew);//should be the output from prev stage + output_slew*output_slew);
        slew_map.insert(pair<int,double>(it->first,input_slew));
        min_slew = min (min_slew, input_slew);
        max_slew = max (max_slew, input_slew);
    }
    //slew_spec_ now is read in from the pblock file
    //slew_spec_ = min_slew + slew_percentage_*(max_slew-min_slew);
    //    slew_spec_ = 1200;
    //length_limit_ = ( -(RUNIT*Cb+Rs*CUNIT)+sqrt(pow((RUNIT*Cb+Rs*CUNIT),2) - 2*RUNIT*CUNIT*(Rs*Cb-slew_spec_)) ) / (RUNIT*CUNIT);
    //cout<< "The min slew is "<< min_slew<< ". The max slew is "<< max_slew<< ". The slew spec is "<< slew_spec_<< endl;
#ifdef PRINT
#endif
}

Tree Design::get_tree()const{
    return tree_;
}
vector<Node>& Design::get_nodes(){
    return nodes_;
}
Solution Design::get_best_solution(){
    return best_solution_;
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
        new_node.index_ = new_nodes.size();
        new_node.parent_ = m2[new_node.parent_];
        //all children work no use now, cause no children info yet
        set<int> children = new_node.get_children();
        set<int> new_children;
        for (set<int>::iterator it = children.begin(); it != children.end(); it++){
            new_children.insert(m2[*it]);
        }
        new_node.set_children(new_children);
        new_nodes.push_back(new_node);
    }
    new_nodes[0].type_ = Node::ROOT;
    nodes = new_nodes;
}
//Not only return the set of sinks, but also change all NodeType of sink nodes to SINK
set<int> Design::RecognizeSinks(vector<Node>& nodes)const{
    set<int> sink_set;

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
    sscanf(line, "DeltaX\t:\t%d\n", temp);

    fgets(line, LINESIZE, fp);
    sscanf(line, "%s\t%*s\n", temp);
    sscanf(line, "Root\t:\t%d\n", temp);

    fgets(line, LINESIZE, fp);
    sscanf(line, "%s\t%*s\n", temp);

    int num_sinks;
    fscanf(fp, "%d\n", &num_sinks);
    for(unsigned int i=0;i<num_sinks;i++){
        Point sink;
        fscanf(fp,"%d",&sink.x);
        fscanf(fp,"%d",&sink.y);
        for (int j = 0; j < nodes.size(); j++){
            if(sink.x==nodes[j].point_.x && sink.y==nodes[j].point_.y && nodes[j].type_!=Node::ROOT){
                nodes[j].type_ = Node::SINK;
                sink_set.insert(j);
            }
        }
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
    //    nodes[root].type_ = Node::ROOT;
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

DTYPE  Design::CalculateTotalWirelength(const vector<Node> nodes)const{
    DTYPE total_wirelength = 0;
    for (int i = 1; i<nodes.size(); ++i){//Except the root at 0
        Node node = nodes.at(i);
        Node parent_node = nodes.at(node.parent_);
        Edge edge(node.point_, parent_node.point_);
        total_wirelength += edge.Length();
    }
    return total_wirelength;
}

/*--------------  Timing/Slew functions  --------------------*/
void Design::InitializeCap(vector<Node> & nodes)const{
    for (int node_index = 0; node_index < nodes.size(); node_index++){
        //if (sinks_.find(node_index) != sinks_.end()) 
        if (nodes.at(node_index).type_ == Node::SINK)
            nodes.at(node_index).ct_ = Cb;//Add capacitance of sinks to 0
        else if (nodes.at(node_index).type_ == Node::BUFFER)
            nodes.at(node_index).ct_ = nodes.at(node_index).buffer_.get_input_capacitance();
        else 
            nodes.at(node_index).ct_ = 0;//Initialize capacitance of branch point
    }
}
void Design::PropagateCap(vector<Node> & nodes)const{
    for (int i = 1; i < nodes.size(); i++){
        Node node = nodes.at(i);
        int parent_node_index = node.parent_;
        Node parent_node = nodes.at(parent_node_index);
        Edge edge(node.point_, parent_node.point_);
        DTYPE distance = edge.Length();
        double cap = distance * CUNIT;
        if (node.type_ == Node::SINK || node.type_ == Node::BUFFER)//If it is sink, beside path cap, sink cap should be included
            //be careful not use ct_ because ct_ including downstream cap
            cap += Cb;
        int node_index = i;
        //Add the cap of one edge and the node itself to all its ancestor
        //This is to caculate the downstream cap for each node, when observed a little downstream. So for buffers, ct_ is a big number.
        do{
            nodes.at(parent_node_index).ct_ += cap;
            node_index = parent_node_index;
            node = nodes.at(node_index);
            parent_node_index = node.parent_;
        }while (node.type_ != Node::BUFFER && node.type_ != Node::ROOT);
    }
}
void Design::PrintCapCt(vector<Node>& nodes)const{
    for (int i = 0; i < nodes.size(); i++){
        Node node = nodes.at(i);
        printf ("Downstream Cap below Node%d(%d, %d) is %.3e\n",
                node.index_, node.point_.x, node.point_.y, node.ct_);
    }

}

void Design::PropagateElmore(double delay, int node_index, const vector<Node>& nodes, map<int,double>& slew_map, map<int,double>& delay_map)const{
    Node node = nodes.at(node_index);
    set<int> children =  node.get_children();

    if (!children.empty()){//Keep propagating if has children. Sink could have children
        if(node.type_ == Node::SINK){
            delay_map.insert(pair<int,double>(node_index, delay));
        }
        if (node.type_ == Node::BUFFER){
            slew_map.insert(pair<int,double>(node_index, delay));
            delay += node.buffer_.get_intrinsic_delay();
        }
        for (set<int>::const_iterator it = children.begin(); it != children.end(); it++){
            int child_node_index = *it;
            Node child_node = nodes.at(child_node_index);
            Edge edge(child_node.point_, node.point_);
            DTYPE distance = edge.Length();
            double child_in_cap = child_node.type_==Node::BUFFER?//To get the downstream cap from a little upstream
                child_node.buffer_.get_input_capacitance():child_node.ct_;
            double child_delay = delay + RUNIT*distance*(0.5*CUNIT*distance + child_in_cap);
            PropagateElmore(child_delay, child_node_index, nodes, slew_map, delay_map);
        }
    }else{
        //assert (node.type_ == Node::SINK);
        delay_map.insert(pair<int,double>(node_index, delay));//Add sink elmore and index to the map
    }
}
void Design::CalculateElmore(const vector<Node>& nodes, map<int,double>&slew_map, map<int,double>&delay_map)const{//Calculate Elmore values for all EP and MP. EP and MP may have different size due to eliminating MP
    //double root_delay = root_buffer_.driving_resistance_ * nodes.at(root_).ct_ + root_buffer_.intrinsic_delay_;
    //double root_delay = 0;//The elmore calculation start from behind of the root driver
    double root_delay = nodes[0].buffer_.get_driving_resistance()*nodes[0].ct_ + nodes[0].buffer_.get_intrinsic_delay();//Only use intrinsic delay
    PropagateElmore(root_delay, 0, nodes, slew_map, delay_map);
    return;
}

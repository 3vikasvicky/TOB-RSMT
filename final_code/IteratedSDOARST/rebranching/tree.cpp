#include "tree.h"

using namespace std;

//--------------  Node functions   ------------------
Node::Node(NodeType node_type){
    children_.clear();
    type_ = node_type;
}
set<int> Node::get_children()const{
    return children_;
}
void Node::set_children(const set<int> & children){
    children_ = children;
}
void Node::add_child(int child){
    children_.insert(child);
}
void Node::remove_child(int child){
    children_.erase(child);
}
/*---------------------------  Tree functions(new)   ------------------------------*/
Tree::Tree(){}
Tree::Tree(int root):root_(root){
    clear_indexes();
    clear_branches();
    insert_index(root);
}
void Tree::set_indexes(const set<int>& indexes){
    indexes_ = indexes;
}
set<int> Tree::get_indexes()const{
    return indexes_;
}
void Tree::remove_index(int index){
    indexes_.erase(index);
}
void Tree::insert_index(int index){
    indexes_.insert(index);
}
void Tree::clear_indexes(){
    indexes_.clear();
}
void Tree::clear_sinks(){
    sinks_.clear();
}
void Tree::remove_sink(int sink_index){
    sinks_.erase(sink_index);
}
void Tree::insert_sink(int sink_index){
    sinks_.insert(sink_index);
}
set<int> Tree::get_sinks()const{
    return sinks_;
}
void Tree::set_sinks(const set<int> & sinks){
    sinks_ = sinks;
}
void Tree::generate_branches(){
    branches_ = indexes_;
    branches_.erase(root_);
}
void Tree::set_branches(const set<int>& branches){
    branches_ = branches;
}
set<int> Tree::get_branches()const{
    return branches_;
}
void Tree::remove_branch(int branch){
    branches_.erase(branch);
}
void Tree::insert_branch(int branch){
    branches_.insert(branch);
}
void Tree::clear_branches(){
    branches_.clear();
}
void Tree::set_root(int root){
    root_ = root;
}
int Tree::get_root()const{
    return root_;
}
Point Tree::get_root_point(const vector<Node> & nodes)const{
    return nodes.at(root_).point_;
}

void Tree::BreakLShape(vector<Node>& nodes){
    for (set<int>::const_iterator it = branches_.begin(); it != branches_.end(); it++){
        int index = *it;
        Point point = nodes[index].point_;
        int parent_node_index = nodes[index].parent_;
        Point parent_point = nodes[parent_node_index].point_;
        if (!point.PointsOnSameLine(parent_point)){//If between point and parent_point is L shape
            //Deside the shape of L
            int support_vertical_first = 0;
            int support_horizontal_first = 0;
            for (set<int>::const_iterator it2 = branches_.begin(); it2 != branches_.end(); it2++){
                int child_branch_index = *it2;
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
            //Insert new Node into nodes and insert index into indexes_
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
            insert_index(corner_node.index_);
            insert_branch(corner_node.index_);
            nodes[index].parent_ = corner_node.index_;
        }
    }

    for (set<int>::const_iterator it = indexes_.begin(); it != indexes_.end();++it){
        int node_index = *it;
        if (node_index == get_root())
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

void Tree::StoreChildrenNode(vector<Node> & nodes)const{
    for (set<int>::const_iterator it = indexes_.begin(); it != indexes_.end(); it++){
        int index = *it;
        Point point = nodes.at(index).point_;
        int parent_node_index = nodes.at(index).parent_;
        if (parent_node_index != index){//If it is not root and root
            nodes.at(parent_node_index).children_.insert(index);
        }
    }
}
void Tree::InitializeCap(vector<Node> & nodes)const{
    for (set<int>::iterator it = indexes_.begin(); it != indexes_.end(); ++it){
        int node_index = *it;
        if (sinks_.find(node_index) != sinks_.end())  
            nodes.at(node_index).ct_ = Cb;//Initialize capacitance of sinks
        else 
            nodes.at(node_index).ct_ = 0;//Initialize capacitance of branch point
    }
}
void Tree::PropagateCap(vector<Node> & nodes)const{
    for (set<int>::const_iterator it = indexes_.begin(); it != indexes_.end();++it){
        int node_index = *it;
        if (node_index == get_root())
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

Buffer Tree::get_root_buffer()const{
    return root_buffer_;
}
void Tree::PropagateElmore(double delay, int node_index, const vector<Node>& nodes, map<int,double>& delay_map)const{
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
map<int,double> Tree::CalculateElmore(vector<Node>& nodes)const{//Calculate Elmore values for all EP and MP. EP and MP may have different size due to eliminating MP
    map<int,double> delay_map;
    //double root_delay = root_buffer_.driving_resistance_ * nodes.at(root_).ct_ + root_buffer_.intrinsic_delay_;
    double root_delay = 0;//The elmore calculation start from behind of the root driver
    PropagateElmore(root_delay, root_, nodes, delay_map);
    return delay_map;
}
map<int,double> Tree::CalculateElmore(vector<Node>& nodes, const Solution & buffering_solution)const{//Calculate Elmore values with a buffering solution
    map<int,double> delay_map;
    //double root_delay = root_buffer_.driving_resistance_ * nodes.at(root_).ct_ + root_buffer_.intrinsic_delay_;
    double root_delay = 0;//The elmore calculation start from behind of the root driver
    PropagateElmore(root_delay, root_, nodes, delay_map);
    return delay_map;
}
double Tree::CalculateOutputSlew(const vector<Node> & nodes)const{
    Node root_node = nodes.at(root_);
    double output_slew = root_buffer_.slew_resistance_ *  root_node.ct_ + root_buffer_.intrinsic_slew_;
    return output_slew;
}

DirectionalEdge Tree::FindFirstLShapeEndPoint(int sink_index, const vector<Node>& nodes)const{
    list<DirectionalEdge> path_to_root = FindPathToRoot(sink_index, nodes);
    list<DirectionalEdge>::const_iterator it2 = path_to_root.begin();
    it2++;
    DirectionalEdge second_d_edge = *it2;
    it2++;
    while (it2 != path_to_root.end()){
        DirectionalEdge next_dedge = *it2;
        if (second_d_edge.IsSameLine(next_dedge)){
            second_d_edge.set_end_point(next_dedge.get_end_point());
        }
        it2++;
    }
    return second_d_edge;
}
DirectionalEdge Tree::FindEdge(Point point, const vector<Node>& nodes)const{
    int parent_node_index;
    int child_node_index;
    FindEdgeIndex(point, nodes, &parent_node_index, &child_node_index);
    return GetBranchEdge(child_node_index, nodes);
}
void Tree::FindEdgeIndex(Point point, const vector<Node>& nodes, int* parent_node_index_pointer, int * child_node_index_pointer)const{
    for (set<int>::iterator it = branches_.begin(); it != branches_.end(); ++it){
        int branch_index = *it;
        DirectionalEdge d_edge = GetBranchEdge(branch_index, nodes);
        if (point.IsOnEdge(d_edge)){
            *child_node_index_pointer = branch_index;
            *parent_node_index_pointer = nodes.at(branch_index).parent_;
            return;
        }
    }
    assert(1);//Should not be here
}
DirectionalEdge Tree::GetBranchEdge(int sink_index, const vector<Node> & nodes) const{//Assume no degree 2 steiner node
    Node node = nodes.at(sink_index);
    Node parent_node = nodes.at(node.parent_);
    DirectionalEdge d_edge (node.point_, parent_node.point_);
    return d_edge;
}
DTYPE Tree::CalculateBranchLength(int sink_index, const vector<Node> & nodes) const{//Assume no degree 2 steiner node
    DirectionalEdge d_edge = GetBranchEdge(sink_index, nodes);
    return d_edge.Length();
}
DTYPE Tree::CalculatePathToRootLength(int sink_index, const vector<Node> & nodes) const{
    DTYPE path_to_root = 0;
    while (sink_index != root_){
        path_to_root += CalculateBranchLength(sink_index, nodes);
        Node node = nodes.at(sink_index);
        sink_index = node.parent_;
    }
    return path_to_root;
}
list<DirectionalEdge> Tree::FindPathToRoot(int node_index, const vector<Node> & nodes)const{
    list<DirectionalEdge> path;
    while (node_index != root_){
        Node node = nodes.at(node_index);
        int parent_node_index = node.parent_;
        Node parent_node = nodes.at(parent_node_index);
        DirectionalEdge directional_edge (node.point_, parent_node.point_);
        path.push_back(directional_edge);
        node_index = parent_node_index;
    }
    return path;
}
DTYPE  Tree::CalculateTotalWirelength(const vector<Node> & nodes)const{
    DTYPE total_wirelength = 0;
    for (set<int>::iterator it = branches_.begin(); it != branches_.end(); ++it){
        int branch_index = *it;
        total_wirelength += CalculateBranchLength(branch_index, nodes);
    }
    return total_wirelength;
}

/*--------------  Node +- functions      ------------------*/
void Tree::Rootify(int node_index, vector<Node> & nodes, int real_parent_node_index){
    for (set<int>::const_iterator it = indexes_.begin(); it != indexes_.end(); it++){
        int parent_node_index = *it;
        if (parent_node_index == real_parent_node_index){
            continue;
        }
        Node parent_node = nodes.at(parent_node_index);
        if (parent_node.children_.find(node_index) != parent_node.children_.end()){
            AddChildNode(nodes, node_index, parent_node_index);
            SetParentNode(nodes, node_index, parent_node_index);
            RemoveChildNode(nodes, parent_node_index, node_index);
            if (branches_.find(parent_node_index) == branches_.end()){//Add new branch if needed
                branches_.insert(parent_node_index);
            }
            if (node_index == parent_node_index){//Remove 0 length branch now
                branches_.erase(node_index);
            }
            Rootify(parent_node_index, nodes, node_index);
            return;//Only one parent for each child
        }
    }
    assert(1);
}
Node Tree::FindAndInsertNewNode(Point new_node_point, vector<Node>& nodes){
    for (set<int>::const_iterator it = indexes_.begin(); it != indexes_.end(); it++){
        int node_index = *it;
        Node node = nodes.at(node_index);
        Point point = node.point_;
        if (point == new_node_point){
            return node;
        }
    }
    for (set<int>::const_iterator it = branches_.begin(); it != branches_.end(); it++){
        int node_index = *it;
        int parent_node_index = nodes.at(node_index).parent_;
        DirectionalEdge d_edge =  GetBranchEdge(node_index, nodes);
        if (new_node_point.IsOnEdge(d_edge)){
            /*if (new_node_point == d_edge.get_start_point()){
                return nodes.at(node_index);
            }else if (new_node_point == d_edge.get_end_point()){ 
                return nodes.at(parent_node_index);
            }else{
            }*/
                return InsertNewNode(parent_node_index, node_index, new_node_point, nodes);
        }
    }
}
Node Tree::InsertNewNode(int parent_node_index, int child_node_index, Point new_node_point, vector<Node>& nodes){
    Node new_node;
    new_node.index_ = nodes.size();
    new_node.point_ = new_node_point;
    new_node.ct_ = 0;
    nodes.push_back(new_node);
    int new_node_index = new_node.index_;
    SetParentNode(nodes, parent_node_index, new_node_index);
    AddChildNode(nodes, new_node_index, child_node_index);
    new_node = nodes.at(new_node_index);//It was updated

    RemoveChildNode(nodes, parent_node_index, child_node_index);
    AddChildNode(nodes, parent_node_index, new_node_index);

    SetParentNode(nodes, new_node_index, child_node_index);

    insert_index(new_node_index);
    insert_branch(new_node_index);
    return new_node;
}
void Tree::RemoveChildNode(vector<Node>& nodes, int node_index, int child_node_index)const{
    Node node = nodes.at(node_index);
    set<int> children = node.get_children();
    children.erase(child_node_index);
    node.set_children(children);
    nodes.at(node_index) = node;
}
void Tree::AddChildNode(vector<Node>& nodes, int node_index, int child_node_index)const{
    Node node = nodes.at(node_index);
    set<int> children = node.get_children();
    children.insert(child_node_index);
    node.set_children(children);
    nodes.at(node_index) = node;
}
void Tree::SetParentNode(vector<Node>& nodes, int parent_node_index, int node_index)const{
    Node node = nodes.at(node_index);
    node.parent_ = parent_node_index;
    nodes.at(node_index) = node;
}
bool Tree::CleanNode(vector<Node>& nodes, int node_index){
    bool is_floating_node = CleanFloatingNode(nodes, node_index);
    bool is_middle_node = CleanMiddleNode(nodes, node_index);
    if (is_floating_node || is_middle_node){
        return true;
    }else{
        return false;
    }
}
bool Tree::CleanMiddleNode(vector<Node>& nodes, int node_index){
    Node node = nodes.at(node_index);
    set<int> children = node.get_children();
    int parent_node_index = node.parent_;
    //Non-escaping point Tail point
    if (children.size() == 1 && node_index != root_){//Redundant node
        set<int>::iterator it = children.begin();
        int child_node_index = *it;
        DirectionalEdge child_d_edge = GetBranchEdge (child_node_index, nodes);
        DirectionalEdge parent_d_edge = GetBranchEdge (node_index, nodes);
        if (child_d_edge.Direction() == parent_d_edge.Direction()){
            RemoveChildNode(nodes, parent_node_index, node_index);
            AddChildNode(nodes, parent_node_index, child_node_index);
            SetParentNode(nodes, parent_node_index, child_node_index);
            remove_index(node_index);
            remove_branch(node_index);
            return true;
        }
    }
    return false;
}
bool Tree::CleanFloatingNode(vector<Node>& nodes, int node_index){
    Node node = nodes.at(node_index);
    set<int> children = node.get_children();
    int parent_node_index = node.parent_;
    //Non-escaping point Tail point
    if (children.size() == 0 && sinks_.find(node_index) == sinks_.end() && node_index != root_){
        RemoveChildNode(nodes, parent_node_index, node_index);
        remove_index(node_index);
        remove_branch(node_index);
        CleanNode(nodes, parent_node_index);
        return true;
    }
    return false;
}
/*void Tree::RemoveNode(vector<Node>& nodes, int node_index)const{
    Node node = nodes.at(node_index);
    int parent_node_index = node.parent_;
    Node parent_node = nodes.at(parent_node_index);
    set<int> children = node.get_children();
    set<int> parent_children = parent_node.get_children();
    parent_children.erase(node_index);
    for (set<int>::const_iterator it = children.begin(); it != children.end(); it++){
        int child_node_index = *it;
        parent_children.insert(child_node_index);
        nodes.at(child_node_index).parent_ = parent_node_index;
    }
    nodes.at(parent_node_index).set_children(parent_children);
}*/
/*---------------------------  InsideTree functions(new)   ------------------------------*/
InsideTree::InsideTree(){}
InsideTree::InsideTree(int root, int block_index):Tree(root){
    block_index_ = block_index;
}
int InsideTree::get_block_index()const{
    return block_index_;
}
void InsideTree::insert_root_down_outside_tree(int outside_tree_index){
    root_down_outside_trees_.insert(outside_tree_index);
}
set<int> InsideTree::get_root_down_outside_trees()const{
    return root_down_outside_trees_;
}
/*---------------------------  OutsideTree functions(new)   ------------------------------*/
OutsideTree::OutsideTree(){}
OutsideTree::OutsideTree(int root):Tree(root){
}
void OutsideTree::insert_root_down_inside_tree(int inside_tree_index){
    root_down_inside_trees_.insert(inside_tree_index);
}
set<int> OutsideTree::get_root_down_inside_trees()const{
    return root_down_inside_trees_;
}

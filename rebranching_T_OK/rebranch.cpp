#include "design.h"

//---------------------------  Recomputer Buffered Delay functions   ------------------------------
vector<Node> Design::AddBufferNodes(const vector<Node> & nodes, const Solution & solution){
    vector<Node> new_nodes = nodes;
    list<pair<Point,Buffer> > buffer_locations = solution.get_buffer_locations();
    PropagateBufferNodes (0, new_nodes, buffer_locations);
    return new_nodes;
}
//PropagateBufferNodes put every buffer on the tree. 
//Due to the ancestor buffers are more at the beginning of buffer_locations list, we can use the for loop to interate all buffers
void Design::PropagateBufferNodes(int node_index, vector<Node>& nodes, const list<pair<Point,Buffer> > & buffer_locations){
    if (sinks_.find(node_index) != sinks_.end()){
        return;
    }
    Node node = nodes.at(node_index);
    set<int> children =  node.get_children();
    for (set<int>::const_iterator it = children.begin(); it != children.end(); it++){
        int start_index = node_index;//For each child it will change. But at the beginning of each child, it is the node index
        int child_node_index = *it;
        Node child_node = nodes.at(child_node_index);
        Point start_point = node.point_;
        Point end_point = child_node.point_;

        Edge edge(start_point, end_point);
        for (list<pair<Point,Buffer> >::const_iterator lit = buffer_locations.begin(); 
                lit != buffer_locations.end(); ++lit){
            Point buffer_point = lit->first;
            Buffer buffer = lit->second;
            if (buffer_point.IsOnEdge(edge) && buffer_point!=start_point){//Don't count buffer twice
                Edge after_edge (buffer_point, end_point);
                Node buffer_node;
                buffer_node.point_ = buffer_point;
                buffer_node.index_ = nodes.size();
                buffer_node.parent_ = start_index;
                buffer_node.add_child(child_node_index);
                buffer_node.type_ = Node::BUFFER;
                buffer_node.buffer_ = buffer;
                nodes.push_back(buffer_node);
                nodes.at(start_index).remove_child(child_node_index);
                nodes.at(start_index).add_child(buffer_node.index_);
                nodes.at(child_node_index).parent_ = buffer_node.index_;

                start_index = buffer_node.index_;
                start_point = buffer_point;
                edge = Edge (start_point, end_point);
            }
        }
        PropagateBufferNodes(child_node_index, nodes, buffer_locations);
    }
}

//---------------------------  Rebranch functions   ------------------------------
void Design::Improve(){
    vector<Node> new_nodes = AddBufferNodes(nodes_, best_solution_);
    InitializeCap(new_nodes);
    PropagateCap(new_nodes);

#ifdef PRINT
    for (int i = 0; i < nodes.size(); i++){
        printf ("Cap at Node%d(%d, %d) is %f\n", 
                i, nodes.at(i).point_.x, nodes.at(i).point_.y, nodes.at(i).ct_);
    }
#endif
    map<int,double> slew_map, elmore_map;//slew_map for Buffers and elmore_map for sinks
    CalculateElmore(new_nodes, slew_map, elmore_map);
    double initial_score,initial_tns,intial_wns;
    double period = elmore_map.begin()->second;//Randomly pick the first element in the map
    Evaluate (initial_tns,intial_wns, period, elmore_map);


    int worst_sink;//start from one node, then for all negative slack sink
    double worst_delay = 0;
    for (map<int,double>::const_iterator it = elmore_map.begin(); it != elmore_map.end(); it++){
#ifdef PRINT
#endif
        printf ("Delay at Node%d(%d, %d) is %f\n", 
                it->first, new_nodes.at(it->first).point_.x, new_nodes.at(it->first).point_.y, it->second);
        if (it->second > worst_delay){
            worst_delay = it->second;
            worst_sink = it->first;
        }
    }
    set<int> break_nodes = SeekOppotunity(worst_sink, new_nodes);
}

double Design::Evaluate(double& tns, double& wns, double period, const map<int,double>& delay_map){
    tns = 0;
    wns = 0;
    for (map<int,double>::const_iterator it = delay_map.begin(); it != delay_map.end(); it++){
        if (it->second-period < wns){
            wns = it->second - period;
            tns += wns;
        }
    }
    return -tns*wns;//This can be changed if better score comes to mind
}
set<int> Design::SeekOppotunity (int worst_sink, const vector<Node>& nodes){
    set<int> break_nodes;
    int current = worst_sink;
    int parent = nodes.at(current).parent_;
    while (parent != 0){//0 is the root
        Node current_node = nodes.at(current);
        Node parent_node = nodes.at(parent);
        int grandparent = parent_node.parent_;
        if (current_node.type_ == Node::BUFFER && parent_node.get_children().size()>1){
            break_nodes.insert(current); 
        }
        current = parent;
        parent = nodes.at(current).parent_;
    }
    return break_nodes;
}

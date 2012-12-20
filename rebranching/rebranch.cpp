#include "design.h"

//---------------------------  Recomputer Buffered Delay functions   ------------------------------
vector<Node> Design::AddBufferNodes(const vector<Node> & nodes, const Solution & solution){
    vector<Node> new_nodes = nodes;
    list<pair<Point,Buffer> > buffer_locations = solution.get_buffer_locations();
    PropagateBufferNodes (0, new_nodes, buffer_locations);//0 is the root
    return new_nodes;
}
//PropagateBufferNodes put every buffer on the tree. 
//Due to the ancestor buffers are more at the beginning of buffer_locations list, we can use the for loop to interate all buffers
void Design::PropagateBufferNodes(int node_index, vector<Node>& nodes, const list<pair<Point,Buffer> > & buffer_locations){
    if (nodes.at(node_index).get_children().empty()){
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
vector<Node> Design::Improve(){
    vector<Node> buffered_nodes = AddBufferNodes(nodes_, best_solution_);

    double tns = 0, wns = 0, period = 0;//temporarily set to 0 then we treat slack as negative delay
    InitializeCap(buffered_nodes);
    //PrintCapCt(buffered_nodes);
    PropagateCap(buffered_nodes);
    int worst_sink = Evaluate (buffered_nodes, period, tns, wns);//start from one node, then for all negative slack sink
//    set<int> break_nodes = SeekOppotunity(worst_sink, buffered_nodes);
    vector<Node> new_nodes = Rebranch (worst_sink, buffered_nodes);
    return new_nodes;
}

int Design::Evaluate(const vector<Node> nodes, double period, double& tns, double&wns){

#ifdef PRINT
    for (int i = 0; i < nodes.size(); i++){
        printf ("Cap at Node%d(%d, %d) is %f\n", 
                i, nodes.at(i).point_.x, nodes.at(i).point_.y, nodes.at(i).ct_);
    }
#endif

    tns = 0;
    wns = 0;
    int worst_sink;
    map<int,double> slew_map, elmore_map;//slew_map for Buffers and elmore_map for sinks
    CalculateElmore(nodes, slew_map, elmore_map);
    for (map<int,double>::iterator it = elmore_map.begin(); it != elmore_map.end(); it++){
        if (period - it->second < wns){
            worst_sink = it->first;
            wns = period - it->second;
            tns += wns;
        }
    }
    return worst_sink;//This can be changed if better score comes to mind
}
double Design::PrintTiming(const vector<Node> nodes, double period){
    map<int,double> slew_map, elmore_map;//slew_map for Buffers and elmore_map for sinks
    CalculateElmore(nodes, slew_map, elmore_map);

    double tns = 0, wns = 0;
    int worst_sink;
    for (map<int,double>::const_iterator it = elmore_map.begin(); it != elmore_map.end(); it++){
        printf ("Delay at Node%d(%d, %d) is %.3e\n", 
                it->first, nodes.at(it->first).point_.x, nodes.at(it->first).point_.y, it->second);
        if (period - it->second < wns){
            worst_sink = it->first;
            wns = period - it->second;
        }
        tns += wns;
    }
    printf ("The total negative slack is %.3e ps\n", tns);
    printf ("The worst negative slack is at node %d with %.3e ps\n", worst_sink, wns);
    
}
/*
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
}*/
//Search in buffered_nodes but change new_nodes
vector<Node> Design::Rebranch (int worst_sink, const vector<Node> & buffered_nodes){
    vector<Node> new_nodes = buffered_nodes;
    int current = worst_sink;
    int parent = new_nodes.at(current).parent_;
    while (parent != 0){//0 is the root
        Node current_node = new_nodes.at(current);
        Node parent_node = new_nodes.at(parent);
        int grandparent = parent_node.parent_;
        if (current_node.type_ == Node::BUFFER && parent_node.get_children().size()>1){
            Node upstream_buffer_node = FindFirstUpstreamBuffer(parent_node.index_, new_nodes);
            //Even upstream_buffer_node is the root, we still try rebranch at the root
            Node new_steiner_node;
            new_steiner_node.index_ = new_nodes.size();
            Node direction_node = new_nodes.at(upstream_buffer_node.parent_);
            new_steiner_node.point_ = upstream_buffer_node.point_;
            if (direction_node.point_.x > upstream_buffer_node.point_.x){
                new_steiner_node.point_.x += 1;
            }else if (direction_node.point_.x < upstream_buffer_node.point_.x){
                new_steiner_node.point_.x -= 1;
            }
            if (direction_node.point_.y > upstream_buffer_node.point_.y){
                new_steiner_node.point_.y += 1;
            }else if (direction_node.point_.y < upstream_buffer_node.point_.y){
                new_steiner_node.point_.y -= 1;
            }
            new_steiner_node.parent_ = direction_node.index_;
            new_nodes.at(direction_node.index_).remove_child(upstream_buffer_node.index_);
            new_nodes.at(direction_node.index_).add_child(new_steiner_node.index_);
            new_steiner_node.add_child(upstream_buffer_node.index_);
           // new_steiner_node.add_child(current_node.index_);
            
           // current_node.parent_ = new_steiner_node.index_;
            upstream_buffer_node.parent_ = new_steiner_node.index_;

            new_nodes.push_back(new_steiner_node);

            ChangeParent(current_node.index_, parent_node.index_, new_steiner_node.index_, new_nodes);
            InsertCornerNode(current_node.index_,new_steiner_node.index_, new_nodes);
            parent = new_steiner_node.index_;
        }
        current = parent;
        parent = new_nodes.at(current).parent_;
    }
  /* 
    //remove buffers in new_nodes
    for (int i = 0; i < new_nodes.size(); i++){
        if (new_nodes.at(i).type_ == Node::BUFFER){
            new_nodes.at(i).type_ = Node::NORMAL;
        }
    }*/
    return new_nodes; 
}
//If no upstream buffer, root is returned
Node Design::FindFirstUpstreamBuffer(int start, const vector<Node> &nodes){
    Node node = nodes.at(start);
    if (node.parent_ == 0){//If node itself is the root
        return node;
    }
    Node current_node = nodes.at(node.parent_);
    while (current_node.type_ != Node::ROOT){
        if (current_node.type_ == Node::BUFFER){
            return current_node;
        }
        current_node = nodes.at(current_node.parent_);
    }
    return current_node;//Return the root if no buffer upstream
    
}
void Design::ChangeParent(int current, int current_parent, int new_parent, vector<Node> &nodes){
    nodes.at(new_parent).add_child(current);
    nodes.at(current_parent).remove_child(current);
    nodes.at(current).parent_ = new_parent;
}
void Design::InsertCornerNode(int current, int parent, vector<Node> &nodes){
    Node corner_node;
    corner_node.add_child(current);
    corner_node.parent_ = parent;
    corner_node.index_ = nodes.size();
    //This to make the coordinate of the corner point 
    corner_node.point_.x = nodes.at(current).point_.x;
    corner_node.point_.y = nodes.at(parent).point_.y;
    nodes.push_back(corner_node);

    nodes.at(parent).remove_child(current);
    nodes.at(parent).add_child(corner_node.index_);

    nodes.at(current).parent_ = corner_node.index_;
}

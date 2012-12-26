#include "design.h"

//---------------------------  Recomputer Buffered Delay functions   ------------------------------
vector<Node> Design::AddBufferNodes(const vector<Node> & nodes, const Solution & solution){
    vector<Node> new_nodes = nodes;
    list<pair<Point,Buffer> > buffer_locations = solution.get_buffer_locations();
    PropagateBufferNodes (root_, new_nodes, buffer_locations);
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
    int start_index = node_index;
    for (set<int>::const_iterator it = children.begin(); it != children.end(); it++){
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
//    vector<Node> nobuffer_nodes = nodes_;
//    nodes_ = new_nodes;
    InitializeCap(new_nodes);
    PropagateCap(new_nodes);
}

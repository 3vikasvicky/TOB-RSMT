#include "design.h"
#include "buffering.h"
//#define INFO

//---------------------------  Buffer functions   ------------------------------
Buffer::Buffer(){
    input_capacitance_ = 1;
    driving_resistance_ = 1;//driving resistance
    intrinsic_delay_ = 1;//instrinsic buffer delay
    slew_resistance_ = 1;//slew resistance of the buffer
    intrinsic_slew_ = 1;//instrisic slew of the buffer
    cost_ = 1;
}
Buffer::Buffer (double input_capacitance, double driving_resistance, double intrinsic_delay, double slew_resistance, double intrinsic_slew, double cost):
    input_capacitance_(input_capacitance), driving_resistance_(driving_resistance), intrinsic_delay_
    (intrinsic_delay), slew_resistance_(slew_resistance), intrinsic_slew_(intrinsic_slew), cost_(cost){}

double Buffer::get_input_capacitance()const{
    return input_capacitance_;
}
void Buffer::set_input_capacitance(double input_capacitance){
    input_capacitance_ = input_capacitance;
}
double Buffer::get_driving_resistance()const{
    return driving_resistance_;
}
void Buffer::set_driving_resistance(double driving_resistance){
    driving_resistance_ = driving_resistance;
}
double Buffer::get_intrinsic_delay()const{
    return intrinsic_delay_;
}
void Buffer::set_intrinsic_delay(double intrinsic_delay){
    intrinsic_delay_ = intrinsic_delay;
}
double Buffer::get_slew_resistance()const{
    return slew_resistance_;
}
void Buffer::set_slew_resistance(double slew_resistance){
    slew_resistance_ = slew_resistance;
}
double Buffer::get_intrinsic_slew()const{
    return intrinsic_slew_;
}
void Buffer::set_intrinsic_slew(double intrinsic_slew){
    intrinsic_slew_ = intrinsic_slew;
}
double Buffer::get_cost()const{
    return cost_;
}
void Buffer::set_cost(double cost){
    cost_ = cost;
}
DTYPE Buffer::CalculateLengthLimit(double slew, double capacitance) const{
    double output_slew = 0;//Initially set to 0, get an upbound of limit_length
    double elmore_delay = sqrt(slew*slew - output_slew*output_slew)/log(9);
    double a = 0.5*RUNIT*CUNIT;
    // The elmore calculation start right behind the buffer. + driving_resistance_ * CUNIT
    double b = RUNIT * capacitance;
    //Delay of buffer should not be included for ramp slew
    double c = -elmore_delay;// + driving_resistance_ * capacitance 
    DTYPE current_length = (DTYPE)(( -b + sqrt (b*b-4*a*c) ) / ( 2 * a ));//It's the upbound of length
    double up_bound = current_length, low_bound = 0;
    do {
        output_slew = slew_resistance_ * (current_length*CUNIT + capacitance) + intrinsic_slew_;
        elmore_delay = RUNIT*current_length*(0.5*CUNIT*current_length + capacitance);
        double current_slew = sqrt(pow(output_slew,2) + pow(log(9)*elmore_delay,2));
        if (current_slew > slew){
            up_bound = current_length;
        }else {
            if (slew-current_slew < 0.01*slew){
                return current_length;//good enough
            }
            low_bound = current_length;
        }
        current_length = 0.5*(low_bound+up_bound);
    }while (up_bound-low_bound<=1);//stop when up_bound is neighbor with low_bound or slew is within 1%

    return current_length;
}

/*---------------------------  Solution functions   ------------------------------*/
double Solution::get_c()const{
    return c_;
}
void Solution::set_c(double c){
    c_ = c;
}
double Solution::get_w()const{
    return w_;
}
void Solution::set_w(double w){
    w_ = w;
}
double Solution::get_s()const{
    return s_;
}
void Solution::set_s(double s){
    s_ = s;
}
Point Solution::get_point()const{
    return point_;
}
void Solution::set_point(Point point){
    point_ = point;
}
list<pair<Point,Buffer> > Solution::get_buffer_locations()const{
    return buffer_locations_;
}
void Solution::set_buffer_locations(list<pair<Point,Buffer> > buffer_locations){
    buffer_locations_ = buffer_locations;
}
void Solution::add_buffer_location(pair<Point,Buffer> point){
    buffer_locations_.push_front(point);
}

bool Solution::Dominate(const Solution & that)const{
    return get_c() <= that.get_c() && get_w() <= that.get_w() && get_s() <= that.get_s();
}
bool Solution::SameSolution (const Solution & that) const{
    return get_point() == that.get_point() && get_c() == that.get_c() && 
        get_w() == that.get_w() && get_s() == that.get_s();
}

/*---------------------------  Buffering functions   ------------------------------*/
void Design::ContinuousBuffering(BUFFERINGMODE buffering_mode, const vector<Node>& nodes){
    map<int,vector<Solution> > solution_map = BuildSinkSolutions(nodes);
    list<int> postorder_traversal = GeneratePostorderTraversal(0,nodes);//0 is the root
    /*cout<< "All postorder traversal nodes are:"<< endl;
    for (list<int>::const_iterator it = postorder_traversal.begin(); it != postorder_traversal.end(); it++){
        cout<< *it<< " ";
    }
    cout<< endl;*/
    for (list<int>::const_iterator it = postorder_traversal.begin(); it != postorder_traversal.end(); it++){
        int node_index = *it;
        Node node = nodes.at(node_index);
        if (node.type_ == Node::SINK){//If is sink and at end
            assert (node.get_children().size() == 0);
            continue;
        }
        Point point = node.point_;
        vector<vector<Solution> > all_children_propagated_solutions = 
            PropagateChildrenSolutions(node_index, point, buffering_mode, solution_map, nodes);
        vector<Solution> merged_solutions = MergeSolutions(all_children_propagated_solutions, point);
        solution_map.insert(pair<int,vector<Solution> >(node_index,merged_solutions));
    }
    vector<Solution> root_solutions = solution_map.find(0)->second;
    assert (root_solutions.size() > 0);
    Solution best_solution = root_solutions.at(0);
    cout<< endl<< "Under "<< buffering_mode<< " mode, Root has "<< root_solutions.size()<< " solutions"<<endl;
    cout<< "They have c, w, s as:"<< endl;
    for (unsigned int i = 0; i < root_solutions.size(); i++){
        Solution root_solution = root_solutions.at(i);
        cout << root_solution.get_c()<< " "<< root_solution.get_w()<<" " << root_solution.get_s()<< endl;
        if (best_solution.get_w() >= root_solution.get_w()){
            best_solution = root_solution;//best_solution use least amount of buffers
        }
    }
    cout<< "Best solution uses "<< best_solution.get_w()<<" buffers"<< endl;
    set_best_solution(best_solution);
}

map<int,vector<Solution> > Design::BuildSinkSolutions(const vector<Node> & nodes)const{
    map<int,vector<Solution> > solution_map;
    for (set<int>::const_iterator it = sinks_.begin(); it != sinks_.end(); it++){
        vector<Solution> solutions;
        Solution solution;
        int sink_index = *it;
        Node sink_node = nodes.at(sink_index);

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
list<int> Design::GeneratePostorderTraversal(int root, const vector<Node> & nodes) const{
   list<int> current_list;
   set<int> children = nodes.at(root).get_children();
   for (set<int>::const_iterator it = children.begin(); it != children.end(); it++){
        list<int> child_list = GeneratePostorderTraversal(*it, nodes);
        current_list.splice(current_list.end(), child_list);
   }
   current_list.push_back(root);
   return current_list;
}
vector<vector<Solution> > Design::PropagateChildrenSolutions(int node_index, Point point, 
        BUFFERINGMODE buffering_mode, map<int,vector<Solution> > solution_map, const vector<Node> & nodes)const{
    vector<vector<Solution> > all_children_propagated_solutions;
    set<int> children = nodes.at(node_index).get_children();
    for (set<int>::const_iterator it2 = children.begin(); it2 != children.end(); it2++){
        int child_index = *it2;
        Point child_point = nodes.at(child_index).point_;
        vector<Solution> child_solutions = solution_map.find(child_index)->second;//child_solutions
        list<Point> points_list;
        points_list.push_back(child_point);
        points_list.push_back(point);
        vector<Solution> new_child_solutions = PropagateChildSolutions(node_index, point, child_index, 
                buffering_mode, child_solutions, points_list,nodes);
        all_children_propagated_solutions.push_back(new_child_solutions);
    }
    return all_children_propagated_solutions;
}
vector<Solution> Design::PropagateChildSolutions(int node_index, Point point, int child_index,
        BUFFERINGMODE buffering_mode, vector<Solution> child_solutions, list<Point> points_list, const vector<Node> & nodes)const{
    vector<Solution> new_child_solutions;
    while (!child_solutions.empty()){
        Solution child_solution = child_solutions.back();
        child_solutions.pop_back();
        if (child_solution.get_point() == point){//reached point, 1 is for buffer
            new_child_solutions.push_back(child_solution);
            continue;
        }
        //            DTYPE distance = point.DistanceToPoint(child_solution.get_point());//This need to be changed to some maze, for non-straigth one
        Point solution_point = child_solution.get_point();
        list<pair<Point,Buffer> > child_solution_buffer_locations = child_solution.get_buffer_locations();
        DirectionalEdge d_edge (solution_point, point);
        //DTYPE distance = d_edge.Length();
        DTYPE distance = CalculateDistanceToEnd(solution_point, points_list);
        if (buffering_mode == OMIT_DENSE ){//Add each buffer to a location one STEP away from solution_point if the distance is larger than STEP. Right now STEP = 5, but it should be %1*total_wirelength
            DTYPE total_tree_length = CalculateTotalWirelength(nodes);
            //#buffers: 0.001 is 4 vs 3, 0.0001 3 vs 3 in RT5
            DTYPE step = DTYPE(0.0001 * total_tree_length) > 0? 0.0001 * total_tree_length: 1;
            Point new_point = point;
            if (distance >= step){//If the distance to branching point is larger then step
                new_point = FindUpPointWithDistance(solution_point, points_list, step);
            }
            Solution new_solution;
            for (unsigned int i = 0; i < buffers_.size(); i++){
                Buffer buffer = buffers_.at(i);
#ifdef INFO
                printf ("child solution_point is at %f, %f, new_point is at %f, %f, # of total_solutions now is %d\n", (double)solution_point.x, (double)solution_point.y, (double)point.x, (double)point.y, child_solutions.size());
#endif
                //new_solution with a buffer put
                new_solution.set_c(buffer.get_input_capacitance());
                new_solution.set_w(child_solution.get_w() + buffer.get_cost());
                new_solution.set_s(0);
                new_solution.set_point(new_point);
                new_solution.set_buffer_locations(child_solution_buffer_locations);
                Point buffer_point = new_point;
                //Buffer solution always put a little behind the point(new solution is at the point but the buffer not)
                if (new_point == point){//already the end, distance <= step
                    switch (d_edge.Direction()){//Put buffer a little from the branch point
                        case RIGHT: 
                            buffer_point.x -= 1;
                            break;
                        case LEFT:
                            buffer_point.x += 1;
                            break;
                        case UP:
                            buffer_point.y -= 1;
                            break;
                        case DOWN:
                            buffer_point.y += 1;
                            break;
                    }
                }
                new_solution.add_buffer_location(pair<Point, Buffer> (buffer_point,buffer));
                //cout << "Debug new_solution is at "<< new_point.x<< ", "<< new_point.y<< " "<< new_solution.get_c()<< " "<< new_solution.get_w()<<" " << new_solution.get_s()<< endl;
                assert (new_solution.get_w() == new_solution.get_buffer_locations().size());
                UpdateSolutions(new_solution, child_solutions, i, slew_spec_, point);

                //new_solution without a buffer put
                new_solution.set_c(child_solution.get_c() + step*CUNIT);
                new_solution.set_w(child_solution.get_w());
                double delta_s = log(9)*RUNIT*step*
                    (0.5*step*CUNIT+child_solution.get_c());
                new_solution.set_s(child_solution.get_s() + delta_s);
                new_solution.set_point(new_point);
                new_solution.set_buffer_locations(child_solution_buffer_locations);
                UpdateSolutions(new_solution, child_solutions, -1, slew_spec_, point);
            }
        }else{//Only OMIT, only buffer needed places
            for (unsigned int i = 0; i < buffers_.size(); i++){
                Solution new_solution;
                Buffer buffer = buffers_.at(i);
                DTYPE length_limit = buffer.CalculateLengthLimit(slew_spec_ -child_solution.get_s(),
                        child_solution.get_c());
                //If BOB mode and current edge is in block, Just find all survivor solutions
#ifdef INFO
                printf ("At OMIT mode, node is at %f, %f, distance to from child to the node is %f, length_limit is %f\n", (double)point.x, (double)point.y, (double)distance, (double)length_limit);
#endif
                Point new_point = point;
                if (distance >= length_limit){//Need buffer
                    new_point = FindUpPointWithDistance(solution_point, points_list, length_limit);
                }
                new_solution.set_c(buffer.get_input_capacitance());
                new_solution.set_w(child_solution.get_w() + buffer.get_cost());
                new_solution.set_s(0);
                new_solution.set_point(new_point);
                new_solution.set_buffer_locations(child_solution_buffer_locations);
                Point buffer_point = new_point;
                if (new_point == point){//already the end, distance <= length limit
                    switch (d_edge.Direction()){
                        case RIGHT: 
                            buffer_point.x -= 1;
                            break;
                        case LEFT:
                            buffer_point.x += 1;
                            break;
                        case UP:
                            buffer_point.y -= 1;
                            break;
                        case DOWN:
                            buffer_point.y += 1;
                            break;
                    }
                }
                new_solution.add_buffer_location(pair<Point, Buffer> (buffer_point,buffer));
                UpdateSolutions(new_solution, child_solutions, i, slew_spec_, point);

                new_solution.set_c(child_solution.get_c() + distance*CUNIT);
                new_solution.set_w(child_solution.get_w());
                double delta_s = log(9)*RUNIT*distance*
                    (0.5*distance*CUNIT+child_solution.get_c());
                new_solution.set_s(child_solution.get_s() + delta_s);
                new_solution.set_point(point);
                new_solution.set_buffer_locations(child_solution_buffer_locations);
                assert (new_solution.get_w() == new_solution.get_buffer_locations().size());
                UpdateSolutions(new_solution, child_solutions, -1, slew_spec_, point);
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
                list<pair<Point,Buffer> > child1_solution_buffer_locations = 
                    child1_solution.get_buffer_locations();
                list<pair<Point,Buffer> > child2_solution_buffer_locations = 
                    child2_solution.get_buffer_locations();
                child1_solution_buffer_locations.splice(child1_solution_buffer_locations.begin(), child2_solution_buffer_locations);
                new_solution.set_buffer_locations(child1_solution_buffer_locations);
                UpdateSolutions(new_solution, new_solutions, -1, slew_spec_, point);
            }
        }
        all_children_propagated_solutions.push_back(new_solutions);
    }
    vector<Solution> solutions = all_children_propagated_solutions.back();
    return solutions;

}
void Design::UpdateSolutions(Solution new_solution, vector<Solution> & solutions, int buffer_index, double slew_spec, Point point)const{
    Buffer buffer;
    if (buffer_index == -1){
        //if (new_solution.get_s() > slew_spec){
         //   return;
       // }
        buffer = buffers_.at(buffers_.size()-1);// nessary, consider output slew as well. Use the largest buffer for no buffer situation
    }else{
        buffer = buffers_.at(buffer_index);// nessary
        /* Buffer buffer = buffers_.at(buffer_index); Unnessary, it is a must safe decision
           double output_ramp = buffer.get_slew_resistance()*new_solution.get_c() + buffer.get_intrinsic_slew();
           if (sqrt(pow(new_solution.get_s(),2) + pow(output_ramp,2)) > slew_spec){
           return;
           }*/
    }
    double output_ramp = buffer.get_slew_resistance()*new_solution.get_c() + buffer.get_intrinsic_slew();
    assert (pow(output_ramp,2) == output_ramp*output_ramp);
    if (sqrt(pow(new_solution.get_s(),2) + pow(output_ramp,2)) > slew_spec){
        return;
    }
    vector<Solution> new_solutions = solutions;
    for (unsigned int i = 0; i < solutions.size(); i++){
        Solution check_solution = solutions.at(i);
        Point check_point = check_solution.get_point();
        Point new_point = new_solution.get_point();
        //check_solution dominates new_solution.Just forget new_solution and return
        if (check_point.DistanceToPoint(point) <= new_point.DistanceToPoint(point)){
            if (check_solution.Dominate(new_solution)){
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


/*--------------  Buffering functions      ------------------*/

DTYPE Design::CalculateDistanceToEnd(Point solution_point, list<Point> points_list)const{
    DirectionalEdge d_edge;
    list<Point> copy_points_list = points_list;
    if (solution_point == points_list.back()){//If solution_point is the end of points_list
        return 0;
    }
    while(true){
        Point point = points_list.front();
        points_list.pop_front();
        if (points_list.size() <=0){
            cout<< "Debug something wrong: the solution_point is not on any edge"<< endl;
        }
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
void Design::set_best_solution (const Solution & best_solution){
    best_solution_ = best_solution;
}

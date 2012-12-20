#include "geometry.h"

using namespace std;
/*--------------  Poly functions   ------------------*/
//verified
Poly::Poly(){}
Poly::Poly(const vector<Point>& that):points_(that){}
void Poly::PushBack (const Point& point){
    points_.push_back(point);
}
Point Poly::At(int i)const{//point index
    assert (i>=0 && i<Size());
    return points_.at(i);
}
int Poly::Size() const {
    return points_.size();
}
Edge Poly::GetEdge(int index_edge)const{
    Point point1 = points_.at(index_edge);
    Point point2 = points_.at((index_edge+1)%Size());
    return Edge(point1, point2);
}
bool Poly::IsAdjacent (const Poly& that) const{
    for (unsigned int i = 0; i < Size(); i++){
        Edge edge1 = GetEdge(i);
        for (unsigned int j = 0; j < that.Size(); j++){
            Edge edge2 = that.GetEdge(j);
            if (edge1.Overlap(edge2))   return true;
        }
    }
    return false;
}
int Poly::FindAdjacent (const Poly& that, int *index_pointA, int *index_pointC) const{
    vector<int> points1, points2;
    int num_adjacent_edges = 0;
    for (unsigned int i = 0; i < Size(); i++){
        Edge edge1 = GetEdge(i);
        for (unsigned int j = 0; j < that.Size(); j++){
            Edge edge2 = that.GetEdge(j);
            if (edge1.Overlap(edge2)){
                if(num_adjacent_edges == 0){
                    *index_pointA = i;
                    *index_pointC = j;
                }
                num_adjacent_edges++;
            }   
        }
    }
    *index_pointC -= num_adjacent_edges - 1;//Because for block2, the recorded one is the last edge index, but we need to return the first edge index
    return num_adjacent_edges;
}
Poly Poly::Cut(int point1, int point2){
    Poly that;
    for (int i = point1; i <= point2; i++){
        that.PushBack(At(i));
    }
    return that;
}
Poly Poly::operator + (const Poly& that)const{
    Poly result = *this;
    for (unsigned int i = 0; i < that.Size(); i++){
        result.PushBack(that.At(i));
    }
    return result;
}
Poly Poly::operator += (const Poly& that){
    *this = *this + that;
    return *this;
}
Poly Poly::operator = (const Poly& that){
    points_ = that.points_;
    return *this;
}
pair<Point,Point> Poly::GenerateBoundingBox()const{
    Point left_down_point(INT_MAX,INT_MAX), up_right_point(-INT_MAX,-INT_MAX);
    for (unsigned int point_index = 0; point_index < Size(); point_index++){
        Point point = At(point_index);
        left_down_point.x = min(left_down_point.x, point.x);
        left_down_point.y = min(left_down_point.y, point.y);
        up_right_point.x = max(up_right_point.x, point.x);
        up_right_point.y = max(up_right_point.y, point.y);
    }
    return pair<Point,Point> (left_down_point, up_right_point);
}

DTYPE Poly::GetOutsideLength(Point point1, Point point2, int step)const{
    unsigned int block_size = Size();
    int start_edge_index = FindEdgeIndex(point1);
    int stop_edge_index = FindEdgeIndex(point2);
    DTYPE outside_length = 0;
    if (start_edge_index == stop_edge_index){
        return point1.DistanceToPoint(point2);
    }

    if (step==1){//Deal with first edge
        outside_length += point1.DistanceToPoint(At((start_edge_index+1)%block_size));
    }else{
        outside_length += point1.DistanceToPoint(At(start_edge_index));
    }
    for (unsigned int current_edge_index = (start_edge_index + step +block_size)%block_size; current_edge_index != stop_edge_index; current_edge_index=(current_edge_index+step+block_size)%block_size){
        Edge current_block_edge = GetEdge(current_edge_index); 
        outside_length += current_block_edge.Length();
    }
    if (step==1){//Deal with last edge
        outside_length += point2.DistanceToPoint(At(stop_edge_index));
    }else{
        outside_length += point2.DistanceToPoint(At((stop_edge_index+1)%block_size));
    }
    return outside_length;
}
int Poly::FindEdgeIndex(Point point)const{
    unsigned int block_size = Size();
    for (unsigned int current_edge_index = 0; current_edge_index < block_size; current_edge_index++){
        Edge current_block_edge = GetEdge(current_edge_index);
        if(point.IsOnEdge(current_block_edge)){
            return current_edge_index;
        }
    }
    assert(1);//Shouldn't be here since sink(root) must be able to find a edge index
}
//--------------  Edge functions   ------------------
Edge::Edge(){}
Edge::Edge(const Point& p1, const Point& p2){
    if (p1.DistanceToPoint(p2) <= EPSILON){
        cout<< p1.x<<" "<<p1.y<<endl;
    }
    if (p1.DistanceToPoint(p2) <= EPSILON){
        cout << "Zero length edge"<< endl;
    }
    //assert (p1.DistanceToPoint(p2) > EPSILON);
    if (!p1.PointsOnSameLine(p2)){
        cout<< p1.x<<" "<<p1.y<<endl;
        cout<< p2.x<<" "<<p2.y<<endl;
    }
    assert(p1.PointsOnSameLine(p2));
    v_ = (p1.PointsOnSameVerticalLine(p2));
    if (v_){
        c_ = p1.x; c1_=min(p1.y, p2.y); c2_=max(p1.y, p2.y);
    }
    else{
        c_ = p1.y; c1_=min(p1.x, p2.x); c2_=max(p1.x, p2.x);
    }
}
bool Edge::IsSameLine(const Edge that)const{
    return v_ == that.v_ && abs(c_ - that.c_)<EPSILON;
}
bool Edge::Overlap(const Edge that)const{
    return (IsSameLine(that) && max(c2_, that.c2_)-min(c1_,that.c1_) < Length() + that.Length());//*this and that overlaps
}
DTYPE Edge::Length()const{
    return c2_ - c1_;
}
bool Edge::IsIntersecting(const Edge& that) const{//Return true if it's a pure intersection between vertical and horizontaledge
    if (v_ == that.v_) return false;
    else return (c_ > that.c1_ && c_ < that.c2_ && c1_ < that.c_ && c2_ > that.c_);
}
//--------------  DirectionalEdge functions   ------------------
DirectionalEdge::DirectionalEdge(const Point& start_point, const Point& end_point)
    :Edge(start_point,end_point), start_point_(start_point), end_point_(end_point){}

DirectionalEdge::DirectionalEdge()
    :Edge(){}

    DIRECTION DirectionalEdge::Direction() const{
        if (v_ == true) 
            return start_point_.y>end_point_.y?DOWN:UP;
        else 
            return start_point_.x>end_point_.x?LEFT:RIGHT;
    }
Point DirectionalEdge::get_start_point() const{
    return start_point_;
}
Point DirectionalEdge::get_end_point() const{
    return end_point_;
}
Point DirectionalEdge::set_start_point(const Point& start_point){
    DirectionalEdge new_edge (start_point, end_point_);
    *this = new_edge;
}
Point DirectionalEdge::set_end_point(const Point& end_point){
    DirectionalEdge new_edge (start_point_, end_point);
    *this = new_edge;
}
Point DirectionalEdge::GetProportionPoint(double segment_portion)const{
    Point point = start_point_ + (end_point_-start_point_)*segment_portion;
    return point;
}
//--------------  Point functions   ------------------
Point::Point(DTYPE a, DTYPE b):x(a), y(b){}
bool Point::IsInsidePoly(const Poly& block) const{
    int left = 0,  up= 0, right = 0, down = 0;
    for (unsigned int i = 0; i < block.Size(); ++i){
        Edge edge = block.GetEdge(i);
        if (IsOnEdge(edge))  return false; //on the edge
        if (edge.v_ && edge.c1_<=y && y<=edge.c2_){
            left += (edge.c_ < x)?1:0;
            right += (edge.c_ > x)?1:0;
        }
        else if (!edge.v_ && edge.c1_<=x && x<=edge.c2_){
            down += (edge.c_ < y)?1:0;
            up += (edge.c_ > y)?1:0;
        }
    }//There is more than or eq 1 edge on four sides 
    return (left>=1 && right>=1 && down>=1 && up>=1);
}
bool Point::IsOnEdge (const Edge& edge) const{
    if (edge.v_ == true)
        return (abs(x-edge.c_)<EPSILON) && (edge.c1_<=y) && (y<=edge.c2_);
    else return (abs(y-edge.c_)<EPSILON) && (edge.c1_<=x) && (x<=edge.c2_);
}
bool Point::IsOnPoly(const Poly& block) const{
    pair<Point,Point> point_pair = block.GenerateBoundingBox();
    Point left_down_point = point_pair.first;
    Point up_right_point = point_pair.second;
    if (x < left_down_point.x || x > up_right_point.x || y < left_down_point.y || y > up_right_point.y)
        return false;//reduce computation
    for (unsigned int edge_index = 0; edge_index < block.Size(); edge_index++){
        Edge edge = block.GetEdge(edge_index);
        if (IsOnEdge(edge))
            return true;
    }
    return false;
}
DTYPE Point::DistanceToPoint (const Point& other)const{
    return abs(x-other.x)+abs(y-other.y);
}
DTYPE Point::DistanceToEdge (const Edge& edge) const{
    if (edge.v_){
        if (y<edge.c1_) return abs(x-edge.c_)+abs(edge.c1_-y);
        else if (y>edge.c2_) return abs(x-edge.c_)+abs(y-edge.c2_);
        else return abs(x-edge.c_);
    }else{
        if (x<edge.c1_) return abs(y-edge.c_)+abs(edge.c1_-x);
        else if (x>edge.c2_) return abs(y-edge.c_)+abs(x-edge.c2_);
        else return abs(y-edge.c_);
    }
}
bool Point::operator == (const Point & that)const{
    return (abs(x-that.x)<EPSILON && abs(y-that.y)<EPSILON);
}
bool Point::operator != (const Point & that)const{
    return ! this->operator == (that);
}
Point Point::operator + (const Point & that)const{
    return Point(x+that.x, y+that.y);
}
Point Point::operator - (const Point & that)const{
    return Point(x-that.x, y-that.y);
}
Point Point::operator * (double mul)const{
    DTYPE round_x = x>0?x*mul+0.5:x*mul-0.5;
    DTYPE round_y = y>0?y*mul+0.5:y*mul-0.5;
    return Point(round_x, round_y);//Because the DTYPE here is int, it will round 0.9999 to 0 which make a step=1 in buffering to be step=0, which make infinite loop
}
Point Point::operator / (double div)const{
    assert (div!=0); 
    DTYPE round_x = x>0?x/div+0.5:x/div-0.5;
    DTYPE round_y = y>0?y/div+0.5:y/div-0.5;
    return Point(round_x, round_y);//Because the DTYPE here is int, it will round 0.9999 to 0 which make a step=1 in buffering to be step=0, which make infinite loop
}
Point Point::Projection (const Edge& e) const{
    if (e.v_){
        assert (y<=e.c2_ && y>=e.c1_);
        return (Point(e.c_,y));
    }else {
        assert (x<=e.c2_ && x>=e.c1_);
        return (Point(x,e.c_));
    }
}

bool Point::IsProjectedIn (const Edge& e) const{
    if (e.v_){
        return (y<e.c2_ && y>e.c1_);
    }else {
        return (x<e.c2_ && x>e.c1_);
    }
}
bool Point::IsProjectedOn (const Edge& e) const{
    if (e.v_){
        return (y<=e.c2_ && y>=e.c1_);
    }else {
        return (x<=e.c2_ && x>=e.c1_);
    }
}
/*
Point Poly::p1_edge(int edge)const{//edge index. get the first 
    return points_.at(edge);
}
Point Poly::p2_edge(int edge)const{
    return points_.at((edge+1)%size());
}
DTYPE Point::distance (const Edge& e) const{//return the shortest Manhatan distance to e
    if (e.v_){
        if (y<e.c1_) return abs(x-e.c_)+abs(e.c1_-y);
        else if (y>e.c2_) return abs(x-e.c_)+abs(y-e.c2_);
        else return abs(x-e.c_);
    }else{
        if (x<e.c1_) return abs(y-e.c_)+abs(e.c1_-x);
        else if (x>e.c2_) return abs(y-e.c_)+abs(x-e.c2_);
        else return abs(y-e.c_);
    }
}
Point Point::findClosestPoint (const Edge& e) const{
    if (isProjectedIn(e))
        return projection(e);
    else
        return distance(e.point1())<distance(e.point2())? 
            e.point1(): e.point2();
}
bool Point::IsOnPoly (const Poly& poly) const{
    int i;
    for (i = 0; i < poly.size(); ++i){
        Edge edge = poly.edge(i);
        if (IsOnEdge(edge))
            return true;
    }
    return false;
}
bool Point::isOutside (const Poly& poly) const{
    return !(IsOnPoly(poly) || poly.contain(*this));
}


//--------------  Edge functions   ------------------

inline Point Edge::point1() const
{
    if (v_ == true) return Point(c_, c1_);
    else return Point(c1_, c_);
}
inline Point Edge::point2() const
{
    if (v_ == true) return Point(c_, c2_);
    else return Point(c2_, c_);
}



vector<Point>::iterator Poly::begin(){return points_.begin();}
vector<Point>::iterator Poly::end(){return points_.end();}
Edge Poly::PreviousEdge(int i)const{
    return edge((i+size()-1)%size());
}
void Poly::print() const{
    int iEdge;
    for (iEdge=0; iEdge < size(); iEdge++){
        p1_edge(iEdge).print();
        printf (" to ");
        p2_edge(iEdge).print();
        if (iEdge+1<size()) printf (" , ");
    }
    printf ("\n");
}
bool Poly::contain(const Edge& e){//this function is not in used now
    Point p1 = e.point1();
    Point p2 = e.point2();
    bool b1, b2;
    b1 = contain(p1);
    b2 = contain(p2);
    return contain(p1) && contain(p2);
}
void Poly::Plot()const{//use gnu to plot a polyhedron
    FILE * fp = popen ("gnuplot -persist", "w");
    fprintf (fp, "set object 1 polygon from ");
    double x_min = INT_MAX, x_max = 0, y_min = INT_MAX, y_max = 0;
    for (unsigned int i=0; i<size();++i){
        fprintf (fp, "%f,%f to ", (double)at(i).x, (double)at(i).y);
        x_min = min(at(i).x, x_min);
        x_max = max(at(i).x, x_max);
        y_min = min(at(i).y, y_min);
        y_max = max(at(i).y, y_max);
    }
    fprintf (fp, " %f,%f fs pattern 1 bo 2 fc rgbcolor \"cyan\"\n",  (double)at(0).x, (double)at(0).y);//to close the Polygon
    fprintf (fp, "set xrange [ %f : %f ]\n", x_min-0.2*(x_max-x_min), x_max+0.2*(x_max-x_min));
    fprintf (fp, "set yrange [ %f : %f ]\n", y_min-0.2*(y_max-y_min), y_max+0.2*(y_max-y_min));
    fprintf (fp, "plot x with dots\n");//plot something
    fprintf (fp, "set terminal x11\n");
    fprintf (fp, "pause -1\n");
    pclose (fp);

}
*/

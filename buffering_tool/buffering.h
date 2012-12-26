#ifndef BUFFERING_H
#define BUFFERING_H
#include "geometry.h"

class Buffer{
    //Db_ = Rb_'* C(u) + Kb_'
public:
    double input_capacitance_;
    double driving_resistance_;//driving resistance
    double intrinsic_delay_;//instrinsic buffer delay
    //Sb,out = Rb_* C(u) + Kb_
    double slew_resistance_;//slew resistance of the buffer
    double intrinsic_slew_;//instrisic slew of the buffer
    double cost_;
    Buffer();
    Buffer (double input_capacitance, double driving_resistance, double intrinsic_delay, double slew_resistance, double intrinsic_slew, double);
    double get_input_capacitance()const;
    void set_input_capacitance(double input_capacitance);
    double get_driving_resistance()const;
    void set_driving_resistance(double drving_resistance);
    double get_intrinsic_delay()const;
    void set_intrinsic_delay(double intrinsic_delay);
    double get_slew_resistance()const;
    void set_slew_resistance(double);
    double get_intrinsic_slew()const;
    void set_intrinsic_slew(double);
    double get_cost()const;
    void set_cost(double cost);
    DTYPE CalculateLengthLimit(double slew, double capacitance) const;
};

class Solution{
    double c_;//Downstream capacitance at current node
    double w_;//Total area cost
    double s_;//Accumulated slew degradation
    Point point_;
    list< pair<Point,Buffer> > buffer_locations_;
public:
    double get_c()const;
    void set_c(double c);
    double get_w()const;
    void set_w(double w);
    double get_s()const;
    void set_s(double s);
    Point get_point()const;
    void set_point(Point point);
    list<pair<Point,Buffer> > get_buffer_locations()const;
    void set_buffer_locations(list<pair<Point,Buffer> > buffer_locations);
    void add_buffer_location(pair<Point,Buffer> point);
    bool Dominate(const Solution & that)const;
    bool SameSolution (const Solution & that) const;
};

#endif

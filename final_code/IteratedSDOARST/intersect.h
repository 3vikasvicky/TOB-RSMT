////////////////////////////////////////////////////////////////////////////////
// 
// 2D Line Segment Intersection example
// Implementation of the theory provided by Paul Bourke
// 
// Written by Damian Coventry
// Tuesday, 9 January 2007
// 
////////////////////////////////////////////////////////////////////////////////


#ifndef INTERSECT_H //Avoid duplicate definition
#define INTERSECT_H

#include <iostream>

class Vector
{
public:
    float x_, y_;

    Vector(float f = 0.0f)
        : x_(f), y_(f) {}

    Vector(float x, float y)
        : x_(x), y_(y) {}
};

class LineSegment
{
public:
    Vector begin_;
    Vector end_;

    LineSegment(const Vector& begin, const Vector& end)
        : begin_(begin), end_(end) {}

    enum IntersectResult { PARALLEL, COINCIDENT, NOT_INTERESECTING, INTERESECTING };

    IntersectResult Intersect(const LineSegment& other_line, Vector& intersection)
    {
        float denom = ((other_line.end_.y_ - other_line.begin_.y_)*(end_.x_ - begin_.x_)) -
                      ((other_line.end_.x_ - other_line.begin_.x_)*(end_.y_ - begin_.y_));

        float nume_a = ((other_line.end_.x_ - other_line.begin_.x_)*(begin_.y_ - other_line.begin_.y_)) -
                       ((other_line.end_.y_ - other_line.begin_.y_)*(begin_.x_ - other_line.begin_.x_));

        float nume_b = ((end_.x_ - begin_.x_)*(begin_.y_ - other_line.begin_.y_)) -
                       ((end_.y_ - begin_.y_)*(begin_.x_ - other_line.begin_.x_));

        if(denom == 0.0f)
        {
            if(nume_a == 0.0f && nume_b == 0.0f)
            {
                return COINCIDENT;
            }
            return PARALLEL;
        }

        float ua = nume_a / denom;
        float ub = nume_b / denom;

        if(ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f)
        {
            // Get the intersection point.
            intersection.x_ = begin_.x_ + ua*(end_.x_ - begin_.x_);
            intersection.y_ = begin_.y_ + ua*(end_.y_ - begin_.y_);

            return INTERESECTING;
        }

        return NOT_INTERESECTING;
    }
};


//bool DetectLineIntersection(SGEdge* E1,SGEdge* E2);

#endif
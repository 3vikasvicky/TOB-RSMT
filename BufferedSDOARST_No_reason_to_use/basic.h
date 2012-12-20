#ifndef BASIC_H //Avoid duplicate definition
#define BASIC_H

#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class Pin;
class Obstacle;
class Point;

//The following seven structs are wrapper types (make interfaces easy to use correctly)
struct X {
	explicit X(int x) : val(x) {}
	int val;
};
struct Y {
	explicit Y(int y) : val(y) {}
	int val;
};
struct X1 {
	explicit X1(int x1) : val(x1) {}
	int val;
};
struct Y1 {
	explicit Y1(int y1) : val(y1) {}
	int val;
};
struct X2 {
	explicit X2(int x2) : val(x2) {}
	int val;
};
struct Y2 {
	explicit Y2(int y2) : val(y2) {}
	int val;
};

const int MaxNumLayer=10;	//this program can processes 10 layers at most (0~9)

struct Layer {
	explicit Layer(int layer) : val(layer) {
		if(layer<0) {cout<<"Layer can not be negative! "; exit(0); }
		else if(layer>=MaxNumLayer) {cout<<"Layer Index exceeds 10 !"; exit(0);}
	}
	int val;
};

class Pin {
public:
	Pin(const X& x,const Y& y,const Layer& layer,double required_time = 0,double loading = 0) : x(x.val) , y(y.val) , layer(layer.val) , required_time(required_time), loading(loading)  {}

	int getX() const {return x;} //Read only
	int getY() const {return y;}
	int getLayer() const {return layer;}
	double getRequiredTime() const { return required_time; }
	void setRequiredTime(double val) { required_time=val; }
	double getLoading() const { return loading; }
private:
	int x;
	int y;
	int layer;
	double required_time;
	double loading;
};

class Obstacle {
public:
	Obstacle(const X1& x1,const Y1& y1,const X2& x2,const Y2& y2,const Layer& layer) : 
																					   x1(x1.val) , y1(y1.val) , 
																					   x2(x2.val) , y2(y2.val) ,
																					   layer(layer.val)  {}
	int getX1() const {return x1;} //Read only
	int getY1() const {return y1;}
	int getX2() const {return x2;} //Read only
	int getY2() const {return y2;}
	int getLayer() const {return layer;}
private:
	int x1;
	int y1;
	int x2;
	int y2;
	int layer;
};

class Point{
public:
	Point(int X,int Y,int Layer) : x(X) , y(Y) , layer(Layer)  {}
	int getX() const {return x;} //Read only
	int getY() const {return y;}
	int getLayer() const {return layer;}
	void setX(int val) { x=val; } 
	void setY(int val) { y=val;}
	void setLayer(int val) { layer=val;}


private:
	int x;
	int y;
	int layer;
};


#endif


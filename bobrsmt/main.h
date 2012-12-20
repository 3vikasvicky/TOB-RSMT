#ifndef MAIN_H
#define MAIN_H
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <algorithm>
#include <iostream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cassert>
#include <climits>
#include <cfloat>
#include <sstream>
#include <cmath>
#include <ctime>
#include <tr1/tuple>

#include "design.h"
#include "tree.h"
#include "geometry.h"
using namespace std;
using std::tr1::tuple;
using std::tr1::tie;
using std::tr1::make_tuple;
/*****************************/
/*  User-Defined Parameters  */
/*****************************/
#ifndef DTYPE   // Data type for distance
#define DTYPE float
#endif
#define BUFFERSIZE 1000
#define LINESIZE 2000 
#define CUNIT 1
#define RUNIT 1
#define Rs 0
#define Db 0
#define Cb 0
#define SLEWSPEC 2400.0
//ALPHA is in objective
#define ALPHA 0.0001
//EPSILON is for float comparison
#define EPSILON 0.01
//FACTOR is how to add lengthInBlock into lengthSoFar, because when we compare same lengthSoFar, we like the one with less lengthInBlock
#define FACTOR 0.00001
//#define DEBUG
#define INFO
enum DIRECTION {UNREACHED, RIGHT, LEFT, UP, DOWN};
#endif

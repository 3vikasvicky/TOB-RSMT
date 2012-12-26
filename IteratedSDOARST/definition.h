#ifndef DEFINITION_H //Avoid duplicate definition
#define DEFINITION_H

#define PRIORITY_BASED // by homer
#define BUF_DRIVEN
#define TEST_HOMER

//------------------------------------
//#define XP_MODE     //If you want to run this program on XP platform, enable this definition and disable #define LINUX_MODE
#define LINUX_MODE    //If you want to run this program on linux-based platform, enable this definition and disable #define XP_MODE
//------------------------------------

//#define CLOCKS_PER_SEC  1000
#define CLK_TCK  CLOCKS_PER_SEC


//------------------------------------
//If you want to see the detail process of each step, just enable the following definition you want
//then you will see some debug informations such as value of variables, processing flow and correction result (DEBUG_CHECK_CORRECTION)
//------------------------------------
#define DEBUG_ANALYSIS_RUN_TIME

//#define DEBUG_SLACK_INFO
//#define DEBUG_DELAY_INFO
//#define DEBUG_REDIRECT_FLOW

//#define DEBUG_FILEIO	
//#define DEBUG_OASG_CHECK_COVERAGE
//#define DEBUG_OASG_CHECK_BLOCKED	
//#define DEBUG_OASG_SWEEP_SCAN
//#define DEBUG_OASG_ADDING_A
//#define DEBUG_OASG_ADDING_EDGE
//#define DEBUG_OASG_ADDING_EDGE_CHECK	
//#define DEBUG_OASG_CHECK_ASET
//#define DEBUG_OASG_CHECK_NEIGHBOR
//#define DEBUG_OASG_DETECT_INTERSECTION
//#define DEBUG_OASG_DETECT_INTERSECTION_DETAIL
//#define DEBUG_OASG_CHECK_CONNECTION
//#define DEBUG_OASG_CHECK_CONNECTION_STRICTLY


//#define DEBUG_RTREE_INSERT_OBSTACLE
//#define DEBUG_RTREE_SEARCH_OBSTACLE
//#define DEBUG_PRIM
//#define DEBUG_PRIM_CONSIDER_OBSTACLE


#define DEBUG_NEW_COST_FUNCTION	//use new cost function: 7/8/2008
//#define DEBUG_OARST_CRITICAL_PATH
//#define DEBUG_OARST_ROUTING_INFO
//#define DEBUG_OARST_RESTORE_USED_NODES
//#define DEBUG_OARST_MAZE_ROUTE	
//#define DEBUG_OARST_BACK_TRACE
//#define DEBUG_OARST_COMPUTE_DISTANCE_FROM_DRIVER
//#define DEBUG_OARST_MULTI_SOURCE
//#define DEBUG_OARST_MULTI_SOURCE_DETAIL_INFO
//#define DEBUG_OARST_REDUCE_LENGTH	
//#define DEBUG_OARST_BRANCH_MOVING_DISCONNECT_WNS_SINK
//#define DEBUG_OARST_BRANCH_MOVING_CONNECT_WNS_SINK


//#define DEBUG_OARG_DO_RECTILINEAR	
//#define DEBUG_OARG_DO_RECTILINEAR_SEARCH_SLANT_EDGES
//#define DEBUG_OARG_PICK_SE0
//#define DEBUG_OARG_PICK_RE0
//#define DEBUG_OARG_DO_RECTILINEAR_ADD_RGEDGE
//#define DEBUG_OARG_DO_RECTILINEAR_ADD_RGNODE

//#define DEBUG_OARG_ELIMINATE_OVERLAP_EDGE
//#define DEBUG_OARG_MERGE_TWO_EGEDGE
//#define DEBUG_OARG_ERASE_EDGE
//#define DEBUG_OARG_ERASE_NODE
//#define DEBUG_OARG_U_SHAPE_PUSH_EDGE

//#define DEBUG_OARG_CHECK_CONNECTION
#define DEBUG_OARG_CHECK_CONNECTION_STRICTLY

//#define DEBUG_ELMORE_DOWNSTREAM_CAPACITANCE
//#define DEBUG_ELMORE_SINK_DELAY

#define DEBUG_FILEIO_WRITE_TIMING_CONSTRAIN

//---------------------------------------------
//#define DEBUG_CHECK_CORRECTION
//---------------------------------------------




#define SPEED_UP

const double _ZERO = 0;

const int _CONTINUE=50;
const int _STOP=51;
//---------------------------------------------
const int _VERTICAL=100;
const int _HORIZONTAL=101;
//---------------------------------------------
const int _PIN=5;
const int _OBSTACLE_LEFT=6;
const int _OBSTACLE_RIGHT=7;
const int _OBSTACLE_LEFT_LOWER=8;
const int _OBSTACLE_LEFT_UPPER=9;
const int _OBSTACLE_RIGHT_LOWER=10;
const int _OBSTACLE_RIGHT_UPPER=11;
const int _TURNING=8; //ref Graduate Institute of Electronics Engineering 
					  //    College of Electrical Engineering and Computer Science National Taiwan University Master Thesis P29
const int _OBSTACLE=9;

//---------------------------------------------
const int _UNSET = -1;	
const double _INFINITE = 1000000000;
const double _NINFINITE = -1000000000;

//---------------------------------------------
const int _R1=1;
const int _R2=2;
const int _R3=3;
const int _R4=4;

//---------------------------------------------
const int _QUADRANT1=1;
const int _QUADRANT2=2;
const int _QUADRANT3=3;
const int _QUADRANT4=4;

const int _QUADRANT12=5;
const int _QUADRANT21=5;
const int _QUADRANT23=6;
const int _QUADRANT32=6;
const int _QUADRANT34=7;
const int _QUADRANT43=7;
const int _QUADRANT41=8;
const int _QUADRANT14=8;

//---------------------------------------------
const int _E_E0_CASE0=0; //only one slant edge, and no following slant egdes exist.
const int _E_E0_CASE1=1; //one slant edge and one H/V edge
const int _E_E0_CASE2=2; //two slant edge : adjacent quadrants
const int _E_E0_CASE3=3; //two slant edge : adjacent quadrants
const int _E_E0_CASE4=4; //two slant edge : the same quadrants

//---------------------------------------------
const int TEND_MST=0;
const int TEND_SPT=1;
#endif

#ifndef PARAM_H
#define PARAM_H
/*****************************/
/*  User-Defined Parameters  */
/*****************************/
#ifndef DTYPE   // Data type for distance
//#define DTYPE double
#define DTYPE int
#endif
#define BUFFERSIZE 1000
#define LINESIZE 2000 
#define CUNIT 1
#define RUNIT 1
#define Rs 0
#define Db 0
//Cb should be equal to min buffer cap, or a little bigger
#define Cb 2
//EPSILON is for double comparison
#define EPSILON 0.0001
//ALITTLEPORTION is to get a point really close to start point of a directional edge
#define ALITTLEPORTION 0.000001
//BLOCKFACTOR is how to add lengthInBlock into lengthSoFar, because when we compare same lengthSoFar, we like the one with less lengthInBlock. Because lengthSoFar is int type, if BLOCKFACTOR is too small, it will probably not working. It also cannot be too big, otherwise the closer node at the boundary of block may have more length than the one after it, and will be reverse updated
//If block_factor is very small, it is for MazeRouting, which will choose one with less use of block
//If block_factor is very large, it is for FindPointsPath, which will choose block avoiding path
#define SMALLBLOCKFACTOR 0.000001
#define LARGEBLOCKFACTOR 100
//#define DEBUG
//#define INFO
#endif

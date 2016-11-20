/*
 * HandPositions.h
 *
 *  Created on: 1 Nov 2015
 *      Author: osboxes
 */

#ifndef HANDPOSITIONS_H_
#define HANDPOSITIONS_H_
#include <iostream>
#include <map>
#include "Params.h"
//maps where all offsets for a given division node are stored
typedef std::map<std::string, float*> MapOffset;
typedef std::pair<std::string, float*> PairOffset;

//LRT node could be split or division node
typedef struct {
	float coord [3];
} Node;
struct LRTNode{
LRTNode* nextLeft;
LRTNode* nextRight;
MapOffset toLeft;
MapOffset toRight;
bool isSplit;
float u[3];
float v[3];
};

typedef std::map<long long int, LRTNode&> MapLRTNodes;
typedef std::pair<long long int, LRTNode&> MapLRTPair;
//used in split node, before the optimal value is found
struct RandomValues{
	float u[3];
	float v[3];
} ;
//structure for an image with path to the image and x,y,z coordinates for all LTM nodes of an image
typedef struct {
	std::string pathToImage;
	Node* nodes;
} HandPose;
//structure for LRT
typedef struct {
	struct {
		int left;
		int right;
		int weight;
	} ;
} HandStructure;
#endif /* HANDPOSITIONS_H_ */

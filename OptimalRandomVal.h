/*
 * SplitNode.h
 *
 *  Created on: 6 Nov 2015
 *      Author: osboxes
 */

#ifndef OPTIMALRANDOMVAL_H_
#define OPTIMALRANDOMVAL_H_
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <set>
#include <string>
#include "HandPoseStructs.h"

//stores the ID of the images we use at a given LRT node
typedef std::set<int> SetPoses;
typedef std::pair<int, HandPose> PairPoses;

class OptimalRandomVal {
public:
	OptimalRandomVal();
	virtual ~OptimalRandomVal();
	/**Divides the set S into subsets Sl and Sr a given amount of times and stores the random values for the best division
	 * @param pathToDepth Absolute path to depth
	 * @param S The set which is divided
	 * @param leftSOptimum Set for storing  left subset for best division result
	 * @param rightSOptimum Set for storing  right subset for best division result
	 * @param optimum structure where the best random values are stored
	 * @param hp All positions of nodes of all training samples
	 * @param hs the IDs of the LTM children of current node
	 * @param nodeID ID of current LTM node*/
	void applySplit(std::string pathToDepth, SetPoses& S, SetPoses& leftSOptimum,
			SetPoses& rightSOptimum, LRTNode* optimum, HandPose* hp, HandStructure& hs, int nodeID,
			 float& infoGain, int stage);
	/**Calculates difference of intensities of two points
	 * @param image The image on which the test is applied
	 * @param intensityAtRoot Intensity at root, for normalisation
	 * @param currNode currNode Node around which we apply the tests
	 * @param r The random offsets from currNode where we apply the test
	 * @return difference of intensity */
	float applyTest(cv::Mat& image, float intensityAtRoot, float* currNode, RandomValues& r);
	/**Calculates the information gain for a given split of a set
	 * @param hp the positions of all nodes of all images of the LTM tree
	 * @param S given set of images
	 * @param hs the IDs of the LTM children of current node
	 * @param leftS Left subset
	 * @param rightS Right subset
	 * @param nodeID ID of the LTM current node
	 * @return The calculated information gain*/
	float calcInfoGain(HandPose* hp, SetPoses& S, HandStructure& hs, SetPoses& leftS,
			SetPoses& rightS, int nodeID);
private:
	/**Calculates the variances for a 3x1 offsets for a fiven set
	 * @param S the set
	 * @param hp All positions of nodes of all training samples
	 * @param nodeID ID of current LTM node
	 * @param nodeID2 ID of left or right child of the current LTm node
	 * @param variance Structure for soring the variance */
	void variance(SetPoses& S, HandPose* hp, int nodeID, int nodeID2, Node& variance);
	/**Sums the variances of left and right offsets from a given node and a given set
	 * @param S the set
	 * @param hp All positions of nodes of all training samples
	 * @param hs IDs of left and right child of the current node
	 * nodeID ID of current LTM node
	 * sum 3x1 vectore, where the sum is stored*/
	void sumTraceF(SetPoses& S, HandPose* hp, HandStructure& hs, int nodeID, float* sum);
	/**Generates random values for the split function
	 * @param r stores the generated values
	 * @param stage current depth of the LTM tree*/
	void generateRandVal(float* r, int stage);
};

#endif /* OPTIMALRANDOMVAL_H_ */

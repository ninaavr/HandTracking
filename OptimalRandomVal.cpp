/*
 * SplitNode.cpp
 *
 *  Created on: 6 Nov 2015
 *      Author: osboxes
 */

#include "OptimalRandomVal.h"
using namespace cv;
using namespace std;

OptimalRandomVal::OptimalRandomVal() {
	// TODO Auto-generated constructor stub

}

OptimalRandomVal::~OptimalRandomVal() {
	// TODO Auto-generated destructor stub
}

void OptimalRandomVal::applySplit(string pathToDepth, SetPoses& S,
		SetPoses& leftSOptimum, SetPoses& rightSOptimum, LRTNode* optimum,
		HandPose* hp, HandStructure& hs, int nodeID, float& infoGain,
		int stage) {
	//optimum are is set to 0 if S is empty
	for (int i = 0; i < 3; ++i) {
		optimum->u[i] = 0;
		optimum->v[i] = 0;
	}
	SetPoses leftS[testNums];
	SetPoses rightS[testNums];
	RandomValues r[testNums];
	//generates all random offsets
	for (int j = 0; j < testNums; ++j) {
		generateRandVal(r[j].u, stage);
		generateRandVal(r[j].v, stage);
	}
	infoGain = 0;
	//iterates over all images
	for (SetPoses::iterator i = S.begin(); i != S.end(); ++i) {
		string pathImage = hp[*i].pathToImage;
		Mat image = imread((pathToDepth + pathImage).c_str(), IMREAD_GRAYSCALE);
		if (!image.data) {
			cout << "image not found" << endl << "image = " << pathToDepth << endl;
		} else {
			//apply all test functions on a image and store the solutions
			for (int j = 0; j < testNums; ++j) {
				if (image.data) {
					Node root = hp[*i].nodes[2 * numNodes - 2];
					float intensityAtRoot = root.coord[2];
					float f = applyTest(image, intensityAtRoot,
							hp[*i].nodes[nodeID].coord, r[j]);
					//cout << "test result:   "<<f <<endl;
					(f < tauIntensity) ? leftS[j].insert(*i) : rightS[j].insert(*i);
				}
			}
		}
		image.release();
	}
	//defines which test values are optimal and stores them
	for (int i = 0; i < testNums; ++i) {
		float currentInfoGain = calcInfoGain(hp, S, hs, leftS[i], rightS[i],
				nodeID);
		//cout << "InfoGain:   " << currentInfoGain << endl;
		if (infoGain < currentInfoGain) {
			infoGain = currentInfoGain;
			for (int j = 0; j < 3; ++j) {
				optimum->u[j] = r[i].u[j];
				optimum->v[j] = r[i].v[j];
			}
			leftSOptimum = leftS[i];
			rightSOptimum = rightS[i];
		}
		leftS[i].clear();
		rightS[i].clear();
	}
	//for (SetPoses::iterator i = leftSOptimum.begin(); i != leftSOptimum.end(); ++i)
	//cout << "left   " <<  *i << endl;
	//for (SetPoses::iterator i = rightSOptimum.begin(); i != rightSOptimum.end(); ++i)
	//cout << "right   " <<  *i << endl;
}

float OptimalRandomVal::applyTest(Mat& image, float intensityAtRoot,
		float* currNode, RandomValues& r) {
	float rOffset1[3];
	float rOffset2[3];
	for (int i = 0; i < 3; ++i) {
		rOffset1[i] = (currNode[i] + r.u[i] / intensityAtRoot);
		rOffset2[i] = (currNode[i] + r.v[i] / intensityAtRoot);
		//cout << "offset 1"<< rOffset1[i] << endl;
		//cout << "offset 2"<< rOffset2[i] << endl;
	}
	float result = image.at<uchar>((int) rOffset1[0] % imageW,
			(int) rOffset1[1] % imageH)
			- image.at<uchar>((int) rOffset2[0] % imageW,
					(int) rOffset2[1] % imageH);
	return abs(result);
}

void OptimalRandomVal::generateRandVal(float* r, int stage) {
	//take smaller offsets by the fingers
	int offset = randOffsetMax;
	if (stage > maxLTMDepth / 4)
		offset /= 2;
	if (stage > maxLTMDepth / 2)
		offset /= 2;
	for (int i = 0; i < 3; ++i) {
		r[i] = (float) (rand() % offset);
		int pn;
		pn = rand() % 2;
		if (pn == 1)
			r[i] *= -1;
		r[i] *= meanDepth;
	}
}

float OptimalRandomVal::calcInfoGain(HandPose* hp, SetPoses& S,
		HandStructure& hs, SetPoses& leftS, SetPoses& rightS, int nodeID) {
	if (leftS.size() == 0 || rightS.size() == 0)
		return 0;
	//calculates variance for current set
	float sumVal[3];
	sumTraceF(S, hp, hs, nodeID, sumVal);
	float sumValLeft[3];
	//calculates variance for left set
	sumTraceF(leftS, hp, hs, nodeID, sumValLeft);
	float sumValRight[3];
	//calculates variance for right set
	sumTraceF(rightS, hp, hs, nodeID, sumValRight);
	float nl = (float) leftS.size();
	float nr = (float) rightS.size();
	float n = (float) S.size();
	float norm = 0;
	for (int i = 0; i < 3; ++i) {
		sumVal[i] = sumVal[i] - sumValLeft[i] * nl / n
				- sumValRight[i] * nr / n;
		norm += sumVal[i] * sumVal[i];
	}
	return sqrt(norm);
}

void OptimalRandomVal::variance(SetPoses& S, HandPose* hp, int nodeID,
		int nodeID2, Node& variance) {
	int n = S.size();
	if ((n == 0) || (n == 1))
		memset(variance.coord, 0, 3 * sizeof(int));
	else {
		SetPoses::iterator i;
		float meanVal[3];
		memset(meanVal, 0, 3 * sizeof(float));
		//sum all offsets for the mean value
		for (i = S.begin(); i != S.end(); ++i) {
			for (int j = 0; j < 3; ++j) {
				meanVal[j] += abs(
						(hp[*i].nodes[nodeID].coord[j]
								- hp[*i].nodes[nodeID2].coord[j]));
				//	cout << abs((hp[*i].nodes[nodeID].coord[j] - hp[*i].nodes[nodeID2].coord[j])) << endl;
			}
		}
		for (int j = 0; j < 3; ++j)
			meanVal[j] /= n;

		float diffSum[3];
		float diffSumSq[3];
		memset(diffSumSq, 0, 3 * sizeof(float));
		//sums square differences between each offset and mean value
		for (i = S.begin(); i != S.end(); ++i) {
			for (int j = 0; j < 3; ++j) {
				diffSum[j] = abs(
						hp[*i].nodes[nodeID].coord[j]
								- hp[*i].nodes[nodeID2].coord[j]) - meanVal[j];
				diffSumSq[j] += diffSum[j] * diffSum[j];
			}
		}
		for (int j = 0; j < 3; ++j) {
			diffSumSq[j] /= (n - 1);
			variance.coord[j] = sqrt(diffSumSq[j]);
			//cout << variance.coord[j] << endl;
		}
	}
}

void OptimalRandomVal::sumTraceF(SetPoses& S, HandPose* hp, HandStructure& hs,
		int nodeID, float* sum) {
	SetPoses::iterator i;
	Node var;
	Node var2;
	//calculates variance for left node's offset
	variance(S, hp, nodeID, hs.left, var);
	//calculates variance for right node's offset
	variance(S, hp, nodeID, hs.right, var2);
	memset(sum, 0, 3 * sizeof(float));
	for (int j = 0; j < 3; ++j) {
		sum[j] = var.coord[j] + var2.coord[j];
		//cout <<sum[j] <<endl;
	}
}

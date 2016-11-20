/*
 * Training.cpp
 *
 *  Created on: 6 Nov 2015
 *      Author: osboxes
 */

#include "Training.h"
using namespace std;
using namespace boost;

Training::Training() {
	// TODO Auto-generated constructor stub

}

Training::~Training() {
	// TODO Auto-generated destructor stub
}

void Training::trainTree(string pathToHandTrackingDirectory) {
	//set random for choosing arbitrary images and choosing arbitrary pixels
	srand(time(NULL));

	//allocates memory and loads the data for hand structure(LTM)
	//and hand pose(data for each training sample)
	HandPositionsLoading handPositionsLoading = HandPositionsLoading();
	hs = new HandStructure[2 * numNodes - 1];
	handPositionsLoading.loadStructure(hs,
			(pathToHandTrackingDirectory + "InputData/").c_str());
	hp = new HandPose[numImages];
	for (int i = 0; i < numImages; ++i) {
		hp[i].nodes = new Node[2 * numNodes - 1];
	}
	handPositionsLoading.loadPositions(hp, hs,(pathToHandTrackingDirectory + "InputData/").c_str());

	pathToDepth = pathToHandTrackingDirectory;
	pathToDepth += "InputData/Depth/";
	int currentNodeID = numNodes * 2 - 2;
	SetPoses S;
	OptimalRandomVal optimalRand = OptimalRandomVal();
	LRTNode* startNode = new LRTNode;
	MapLRTNodes flatStructure;
	addTrainingSamples(S, 0);
	long long int id = 0;
	recursiveLRT(flatStructure, optimalRand, startNode, S, currentNodeID, 1, id);
	printOutput(flatStructure, pathToHandTrackingDirectory + "OutputData/" + "LatentRegressionTree.txt");
}

void Training::recursiveLRT(MapLRTNodes& LRT, OptimalRandomVal& optimalRand,
		LRTNode* node, SetPoses& S, int nodeID, int stage, long long int LRTNodeID) {
	SetPoses Sleft;
	SetPoses Sright;
	float infoGain;
	optimalRand.applySplit(pathToDepth.c_str(), S, Sleft, Sright, node, hp, hs[nodeID], nodeID, infoGain, stage);
	bool infoGainIsSuff;
	if (stage <= 3)
		infoGainIsSuff = infoGain < InfoGainThreshold;
	else
		infoGainIsSuff = infoGain < 4*InfoGainThreshold;
	if (infoGainIsSuff && ((hs[nodeID].left == -1) && (hs[nodeID].right == -1))) {
		//cout << "node  " << nodeID << endl;
		//cout << "leaf node  "<<LRTNodeID;
		//for (SetPoses::iterator i = S.begin(); i != S.end(); ++i)
		//	cout << "   " << *i <<'\t';
		//cout << endl;
		LRTNode* divisionNode = new LRTNode;
		loadDivisionNode(S, *divisionNode, nodeID);
		node = divisionNode;
		LRT.insert(MapLRTPair(LRTNodeID, *divisionNode));
		return;
	}

	if (infoGainIsSuff) {
		//cout << "division node  "<<LRTNodeID ;
		//for (SetPoses::iterator i = S.begin(); i != S.end(); ++i)
		//	cout << '\t'<< *i ;
		//cout << endl;
		LRTNode* divisionNode = new LRTNode;
		loadDivisionNode(S, *divisionNode, nodeID);
		node = divisionNode;
		node->nextLeft = new LRTNode;
		node->nextRight = new LRTNode;
			LRT.insert(MapLRTPair(LRTNodeID, *divisionNode));
		addTrainingSamples(S, stage);
		//cout << "stage   " << stage << endl;
		recursiveLRT(LRT, optimalRand, node->nextLeft, S, hs[nodeID].left,
				stage + 1, 2*LRTNodeID+1);
		recursiveLRT(LRT, optimalRand, node->nextRight, S, hs[nodeID].right,
				stage + 1, 2*LRTNodeID+2);
	} else {
		//cout << "split node   "<<LRTNodeID;
		//for (SetPoses::iterator i = S.begin(); i != S.end(); ++i)
		//	cout << '\t'<< *i ;
		//cout << endl;
		node->nextLeft = new LRTNode;
		node->nextRight = new LRTNode;
		node->isSplit = true;
		LRT.insert(MapLRTPair(LRTNodeID, *node));
		recursiveLRT(LRT, optimalRand, node->nextLeft, Sleft, nodeID, stage, 2*LRTNodeID+1);
		recursiveLRT(LRT, optimalRand, node->nextRight, Sright, nodeID, stage, 2*LRTNodeID+2);
	}
}


void Training::addTrainingSamples(SetPoses& S, size_t levelLTM) {
	if ((levelLTM == maxLTMDepth) || (!randomChoosingImages)) {
		for (int i = 0; i < numImages; ++i) {
			S.insert(i);
		}
	} else {
		size_t imagesForIter = (levelLTM+1)* (numImages / maxLTMDepth);
		//cout << "imagesForIter    "<<imagesForIter <<endl;
		while (imagesForIter > S.size()) {
			size_t t = (rand() % numImages);
			S.insert(t);
			//cout << "rand image:"<<'\t'<<t<<endl;
			//cout << "Size:"<<'\t'<< S.size()<<endl;
		}
	}
}

void Training::loadDivisionNode(SetPoses& S, LRTNode& div, int nodeID) {
	div.isSplit = false;
	int id = 0;
	for (SetPoses::iterator i = S.begin(); i != S.end(); ++i) {
		float* leftOffset = new float[3];
		float* rightOffset = new float[3];
		for (int j = 0; j < 3; ++j) {
			leftOffset[j] = abs(
					hp[*i].nodes[nodeID].coord[j]
							- hp[*i].nodes[hs[nodeID].left].coord[j]);
			rightOffset[j] = abs(
					hp[*i].nodes[nodeID].coord[j]
							- hp[*i].nodes[hs[nodeID].right].coord[j]);
		}
		div.toLeft.insert(
				PairOffset(lexical_cast<std::string>(id), leftOffset));
		div.toRight.insert(
				PairOffset(lexical_cast<std::string>(id), rightOffset));
		++id;
	}
}

void Training::printOutput(MapLRTNodes& LRT, string filename){
	ofstream fileForStoringTree((filename).c_str(	));
	//int j = 0;
	for(MapLRTNodes::iterator i=LRT.begin(); i!=LRT.end(); ++i){
		//stores information for split nodes
		if (i->second.isSplit){
			fileForStoringTree << "split "<< i->first << " ";
			for (int k = 0; k < 3; ++k){
				fileForStoringTree << i->second.u[k];;
				fileForStoringTree <<" ";
			}
			for (int k = 0; k < 3; ++k){
				fileForStoringTree << i->second.v[k];;
				fileForStoringTree <<" ";
			}
		}
		//stores information for division nodes
		else {fileForStoringTree << "division " << i->first << " toLeft ";
			for(MapOffset::iterator k = i->second.toLeft.begin(); k != i->second.toLeft.end(); ++k){
				for (int h = 0; h < 3; ++h){
					fileForStoringTree << k->second[h];
					fileForStoringTree <<" ";
				}
			}
			fileForStoringTree << " toRight ";
			for(MapOffset::iterator k = i->second.toRight.begin(); k != i->second.toRight.end(); ++k){
				for (int h = 0; h < 3; ++h){
					fileForStoringTree << k->second[h];
					fileForStoringTree <<" ";
				}
			}
		}
		fileForStoringTree<<endl;
	}
	fileForStoringTree.close();
}

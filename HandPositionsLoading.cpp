/*
 * LoadHandPositions.cpp
 *
 *  Created on: 2 Nov 2015
 *      Author: osboxes
 */

#include "HandPositionsLoading.h"
using namespace std;

HandPositionsLoading::HandPositionsLoading() {
}

HandPositionsLoading::~HandPositionsLoading() {
	// TODO Auto-generated destructor stub
}

void HandPositionsLoading::loadPositions(HandPose* handPoses,
		HandStructure* handStructure, string pathToInutData) {
	readPositions(handPoses, pathToInutData);
	calculateRestPositions(handPoses, handStructure);
}
void HandPositionsLoading::readPositions(HandPose* handPoses,
		string pathToInutData) {
	ifstream NodesCoordinates;
	NodesCoordinates.open((pathToInutData + "labels.txt").c_str());
	if (NodesCoordinates.is_open()) {
		string line;
		int numCurrentImage = 0;
		while (getline(NodesCoordinates, line)) {

			//reads the directory of the current image
			int pos = line.find(' ');
			string pathToCurrentImage;
			pathToCurrentImage = line.substr(0, pos);
			//cout << pathToCurrentImage;

			handPoses[numCurrentImage].pathToImage = pathToCurrentImage;
			//reads the values for the nodes of the current image
			std::istringstream currLine(line);
			currLine.ignore(100, ' ');
			for (int i = 0; i < numNodes; ++i) {
				for (int j = 0; j < 3; ++j)
					currLine >> handPoses[numCurrentImage].nodes[i].coord[j];
				/*for (int j = 0; j < 3; ++j)
				  cout << "coord:  " << handPoses[numCurrentImage].nodes[i].coord[j] << '\t';
				  cout << endl;*/
			}
			++numCurrentImage;
		}
	} else {
		cout << endl << "Error: labels.txt not found." << endl
				<< "labels.txt should be in HandTracking/InputData directory"
				<< endl;
		exit(-1);
	}
}
void HandPositionsLoading::calculateRestPositions(HandPose* handPoses,
		HandStructure* h) {
	calcWeights(h, 2*numNodes-2);
	for (int i = 0; i < numImages; ++i) {
		//we can only compute a position if both of the childern's positions are known
		for(int currWeight = 2; currWeight <=  maxLTMDepth; ++currWeight){
			for (int j = numNodes; j < 2 * numNodes; ++j) {
				if (h[j].weight == currWeight){
					for (int k = 0; k < 3; ++k){
					float leftNode = handPoses[i].nodes[h[j].left].coord[k];
					//cout << "iteration:     << j <<endl;
					leftNode =  leftNode*(float)h[h[j].left].weight/(float)h[j].weight;
					//cout << "left weight:   "<<leftNode <<endl;
					//cout << "right Node ID" << h[j].right <<endl;
					float rightNode = handPoses[i].nodes[h[j].right].coord[k];
					//cout << "rightNode coord:  "<< rightNode <<endl;
					rightNode = rightNode*(float)h[h[j].right].weight/(float)h[j].weight;
					//cout << rightNode <<endl;
					//cout << "right weight :   "<<rightNode <<endl;
					handPoses[i].nodes[j].coord[k] = leftNode + rightNode;
					}
				}
			}
		}
	}
	/*for(int i =0;i<numImages;++i){
		for(int j = 0;j<2*numNodes-1;++j){
			for(int k=0;k<3;++k)
				cout << j <<  "   coord:  " << handPoses[i].nodes[j].coord[k] << '\t';
		cout << endl;
		}
	}*/
}

void HandPositionsLoading::loadStructure(HandStructure* hs, string pathToInutData) {
	ifstream structure;
	structure.open((pathToInutData + "LatentTreeModel").c_str());
	if (structure.fail()) {
		cout << endl << "Error: LatentTreeModel not found.";
		cout << endl
				<< "LatentTreeModel should be in HandTracking/InputData directory"
				<< endl;
		exit(-1);
	}
	string line;
	istringstream currentLine(line);
	for (int i = 0; i < 2 * numNodes - 1; ++i) {
		hs[i].left = -1;
		hs[i].right = -1;
	}
	bool leftsTurn = true;
	while (getline(structure, line)) {
		int i = -1, j = -1;
		//reads two numbers from a line
		sscanf(line.c_str(), "%d %*s %d", &i, &j);
		if (i != -1 && j != -1) {
			if (leftsTurn) {
				hs[i].left = j;
				leftsTurn = false;
			} else {
				hs[i].right = j;
				leftsTurn = true;
			}
		}
	}
	/*for (int i = 0; i < 2 * numNodes - 1; ++i) {
		cout << i << " -> " << hs[i].left << endl;
		cout << i << " -> " << hs[i].right << endl;
	}*/
}

int HandPositionsLoading::calcWeights(HandStructure* hs,  int currNode){
if((hs[currNode].left ==-1)&&(hs[currNode].right ==-1)){
		hs[currNode].weight = 1;
		return 1;
	}
	int left = calcWeights(hs, hs[currNode].left);
	int right = calcWeights(hs, hs[currNode].right);
	hs[currNode].weight = left + right;
	return hs[currNode].weight;
}

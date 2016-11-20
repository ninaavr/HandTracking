/*
 * LoadHandPositions.h
 *
 *  Created on: 2 Nov 2015
 *      Author: osboxes
 */

#ifndef LOADHANDPOSITIONS_H_
#define LOADHANDPOSITIONS_H_
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include "HandPoseStructs.h"

class HandPositionsLoading {
public:
	HandPositionsLoading();
	virtual ~HandPositionsLoading();
	/**Reads the positions of the observable vertices from a text file and stores all vertices in array of structs
	 * @param handPoses The array of structs
	 * @param handStructure structure of LTM tree
	 * @param pathToInutData Absolute path to InputData*/
	void loadPositions(HandPose* handPoses,HandStructure* handStructure, std::string pathToInutData);
	/**Reads the LTM structure from a dot file
	 * @param hs Where the LTM structure is stored
	 * @param pathToInutData Absolute path to InputData*/
	void loadStructure(HandStructure* hs, std::string pathToInutData);
private:
	/**Reads the positions of the observable nodes from a text file "labels.txt"
	 * @param handPoses Array of structs where the offsets are stored
	 * @param pathToInutData Absolute path to directory InputData*/
	void readPositions(HandPose* handPoses, std::string pathToInutData);
	/**Calculates the positions of the latent vertices as the mean position of all its observable vertices
	 * @param handPoses Coordinates of the observable vertices
	 * @param h LTM structure*/
	void calculateRestPositions(HandPose* handPoses, HandStructure* h);
	/**Calculates how many observable nodes stay "behind" the left or right LTM child
	 * @param hs LTM structure
	 * @param currNode ID of current LTM node*/
	int calcWeights(HandStructure* hs, int currNode);
};

#endif /* LOADHANDPOSITIONS_H_ */

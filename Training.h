/*
 * Training.h
 *
 *  Created on: 6 Nov 2015
 *      Author: osboxes
 */

#ifndef TRAINING_H_
#include <fstream>
#include <stdlib.h>
#include <list>
#include <utility>
#include <iostream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional/optional.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include "OptimalRandomVal.h"
#include "HandPositionsLoading.h"

/**Trains a latent regression tree and stores it in a text file*/
class Training {
public:
	Training();
	virtual ~Training();
	/**Adds the first subset of training samples and and calls recursiveLRT
	 * pathToHandTrackingDirectory absolute path to the folder HandTracking*/
	void trainTree(std::string pathToHandTrackingDirectory);
	/**Stores the LRT from std mapping of the LRT nodes in a txt file
	 * @param LRT std mapping of the LRT nodes
	 * @param name of the text file*/
	void printOutput(MapLRTNodes& LRT, std::string filename);
private:
	/**Iterates over the nodes of the LRT and stores the values needed for split or division node until reaching a leaf
	 * @param LRT Stores the data to a flat structure with ID as of balanced tree, so that we can store the tree to a file later
	 * @param optimalRand Object that calls the iterations for one split node
	 * @param node Current split or division node
	 * @param S Current set of images
	 * @param nodeID Current node in LTM tree
	 * @param stage Current depth in LTM tree
	 * @param LRTNodeID nodeID in the LRT tree */
	void recursiveLRT(MapLRTNodes& LRT, OptimalRandomVal& optimalRand, LRTNode* node, SetPoses& S, int nodeID, int stage, long long int LRTNodeID);
	/**Adds more training samples at the beginning or if a new stage is entered, if randomChoosingImages set to false
	 * adds all training samples at the beginning
	 * @param S Set where the images are added
	 * @param levelLTM Current stage*/
	void addTrainingSamples(SetPoses& S,  size_t levelLTM);
	/**When a division node is reached, stores all offsets from LTM parent to its children of the current set
	 * @param S The current set
	 * @param div The structure where the offsets are stored
	 * @param nodeID The ID of the LTM node */
	void loadDivisionNode(SetPoses& S, LRTNode& div, int nodeID);
	//information of the positions of all LTM nodes of training samples
	HandPose* hp;
	//structure of the hand (Latent Tree structure)
	HandStructure* hs;
	std::string pathToDepth;
};

#endif /* TRAINING_H_ */

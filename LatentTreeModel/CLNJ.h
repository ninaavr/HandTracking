/*
 * CLNJ.h
 *
 *  Created on: 23 Sep 2015
 *      Author: osboxes
 */

#ifndef CLNJ_H_
#define CLNJ_H_
#include <iostream>
#include <fstream>
#include <string>
#include "NeighbourJoining.h"
#include "MinimalSpanningTree.h"
#include "../HandPoseStructs.h"
#include "stdlib.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace std;
using namespace boost;
using namespace Eigen;

class CLNJ {
public:
	CLNJ();
	virtual ~CLNJ();
	/**Builds LTM tree with CLNJ algorithm
	 * @param _D Given distance matrix to all nodes with place for latent nodes
	 * @param ms Tree where LTM is stored*/
	void calcCLNJ(MatrixXf& _D, MinimalSpanningTree& ms);
	/**Prints the vertices and the edges of a graph in a dot file
	 * @param filename Name of the dot file*/
	void loadTreeToStructure(const MinimalSpanningTree& ms, HandStructure* handStructure);
	void printStructure(HandStructure* handStructure, string filename);
	/**Distance Matrix of all nodes, inclusive latent nodes*/
	MatrixXf* D;
private:
	int reservedIDs;
	int selectedNodes;
	MinimalSpanningTree::Vertex root;
	/** By given internal node, defines its direct neighbours and stores their IDs in a vector
	 * @param ms Minimal spanning tree, defined by the distance matrix
	 * @param nodesID vector for the IDs
	 * @param root current selected internal node */
	void selectSubTree(MinimalSpanningTree& ms, MatrixXi& nodesID,
			BuildNJTree::Vertex& root);
	/**Calculates NJ algorithm and adds the result in the tree
	 * @param LTM(Latent tree model) where the subtree is added
	 * @param nodesID IDs of selected nodes of the current SubTree*/
	void calcNJ(MinimalSpanningTree& ms, MatrixXi& nodesID);
	/**Connects two latent nodes trough a new node, to get binary tree
	 * @param the LTM(Latent tree model)*/
	void finishTree(MinimalSpanningTree& ms);
};

#endif /* CLNJ_H_ */

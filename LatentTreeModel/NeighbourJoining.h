/*
 * NeighbourJoining.h
 *
 *  Created on: 12 Jul 2015
 *      Author: osboxes
 */

#ifndef NEIGHBOURJOINING_H_
#define NEIGHBOURJOINING_H_

#include <new>
#include <iostream>
#include <Eigen/Dense>
#include "BuildNJTree.h"
#include "ClosestPair.h"


//defines number of nodes in the given distance matrix
#define latentNodes 16

using namespace Eigen;
using namespace std;

//typedef Matrix<float, matrixLength, matrixLength> DistanceMatrix;

/** Computes neighbour joining algorithm for growing LTM - Latent Tree Model by given distance matrix
 * of 16 observable points in a hand position */
class NeighbourJoining {
public:
	NeighbourJoining();
	virtual ~NeighbourJoining();
	/**Builds LTM with neighbour joining algorithm by given distance matrix
	 *@param D The given distance matrix
	 *@param LTM Tree where the result is stored
	 *@param numOccupiedNodes Gives number of nodes in the graph, that aren't in the input matrix,
	 so that the new latent nodes don't receive already given indices*/
	void calcNJ(MatrixXf& D, MatrixXi& rowsID,  BuildNJTree& LTM, int numOccupiedNodes = 0);
	/** Prints the values of a matrix that are considered at the current iteration
	 * @param M The printed matrix */
	void printMatrix(const MatrixXf& M);
private:
	/**Loads current distance matrix, gives IDs for the latent nodes
	 * @param D The given distance matrix
	 * @param currentD Matrix with distances of subtree nodes
	 * @param numOccupiedNodes Number of nodes in the graph, that aren't in the input matrix,
	 so that the new latent nodes don't receive already given indices*/
	void configure(const MatrixXf& D, MatrixXf& currentD, MatrixXi& rowsID, int numOccupiedNodes);
	/**Calculates Q matrix, which minimum we use to define sibling nodes for the LTM
	 * @param currentD The current distance matrix,needed for calculation of Q
	 * @param Q The calculated matrix*/
	void calcQ(const MatrixXf& currentD, MatrixXf& Q);
	/**Finds the closest pair, i.e. the pair with minimal value in a matrix
	 * @param Q The matrix in which the minimal pair is searched
	 * @param rowsID Array with the IDs of the nodes
	 * @param p Closest pair */
	void findMinQ(const MatrixXf& Q, const MatrixXi& rowsID, Pair& p);
	/**Calculates the values to the new node in the matrix and removes the closest pair nodes
	 * @param currentD The new distance matrix without the closest pair and with the new node
	 * @param  rowsID Array with the IDs of the nodes
	 * @param p Closest pair*/
	void calcNewD(MatrixXf& currentD, MatrixXi& rowsID, const Pair& p);
	/**Updates the distance matrix with all nodes, i.e. adds the distances from the new added node to all other nodes
	 * @param D The whole distance matrix
	 * @param currentD Distance matrix of current NJ iteration of the nodes of the subtree
	 * @param p minimal pair
	 * @param newNode index of the new node*/
	void updateD(MatrixXf& D,const MatrixXf& currentD, const Pair& p, int newNode);
	//number of nodes in the current distance matrix, not yet added to the tree
	int numCurrentNodes;
	//number of observable(given) nodes
	int numObservableNodes;
	//current closest nodes
	Pair pair;
};

#endif /* NEIGHBOURJOINING_H_ */

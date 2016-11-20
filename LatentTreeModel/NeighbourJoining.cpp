/*
 * NeighbourJoining.cpp
 *
 *  Created on: 12 Jul 2015
 *      Author: osboxes
 */
#include "NeighbourJoining.h"

using namespace Eigen;
using namespace std;

NeighbourJoining::NeighbourJoining() {
}

NeighbourJoining::~NeighbourJoining() {
}

void NeighbourJoining::configure(const MatrixXf& D, MatrixXf& currentD, MatrixXi& rowsID, int numOccupiedNodes) {
	numObservableNodes = rowsID.rows();
	numCurrentNodes = numObservableNodes;

	//allocates memory for the latent nodes
	rowsID.conservativeResize((2 * numObservableNodes - 2), 1);

	//gives latent nodes IDs
	for (int i = numObservableNodes; i < 2 * numObservableNodes - 2; ++i) {
		rowsID(i) = i + numOccupiedNodes;
	}
	sort(rowsID.data(), rowsID.data() + rowsID.size());
	//cout << "rowsID after sort:" << rowsID.transpose();	cout << endl;

	//copies distances of selected Nodes
	int offseti = 0;	int offsetj = 0;
	for (int i = 0; i < numObservableNodes; ++i) {
		while (rowsID(i) != i + offseti)
			++offseti;
		offsetj = 0;
		for (int j = 0; j < numObservableNodes; ++j) {
			while (rowsID(j) != j + offsetj)
				++offsetj;
			currentD(i, j) = D(i + offseti, j + offsetj);
			currentD(j, i) = currentD(i, j);
		}
	}
	//cout << "copied matrix: " << endl;		printMatrix(currentD);
}

void NeighbourJoining::calcNJ(MatrixXf& D, MatrixXi& rowsID, BuildNJTree& LTM,
		int numOccupiedNodes) {
	MatrixXf currentD(rowsID.rows() + 1, rowsID.rows() + 1);
	configure(D, currentD, rowsID, numOccupiedNodes);

	MatrixXf Q(currentD.rows(), currentD.cols());
	LTM.configure(rowsID, numOccupiedNodes);
	//BuildNJTree* LTM = new BuildNJTree(D.rows() + numOccupiedNodes, rowsID);

	//grows LTM till there are at least two nodes left
	while (numCurrentNodes > 2) {
		calcQ(currentD, Q);
		findMinQ(Q, rowsID, pair);
		updateD(D, currentD, pair,	2 * numObservableNodes - numCurrentNodes + numOccupiedNodes);
		calcNewD(currentD, rowsID, pair);
		LTM.addClosestPair(pair);
		--numCurrentNodes;
	}
	LTM.connectLast2Nodes();
}

void NeighbourJoining::calcQ(const MatrixXf& currentD, MatrixXf& Q) {
	//calculates sum of the rows of the distance matrix
	Matrix<float, latentNodes, 1> sumRows;
	//MatrixXf sumRows (numObservableNodes+1,1) ;
	sumRows.head(numCurrentNodes) = currentD.topLeftCorner(numCurrentNodes,
			numCurrentNodes).colwise().sum();
	//cout << sumRows.head(numCurrentNodes); cout << endl;
	//Q = (n-2) * currentD
	Q.topLeftCorner(numCurrentNodes, numCurrentNodes) = (numCurrentNodes - 2)
			* currentD.topLeftCorner(numCurrentNodes, numCurrentNodes);
	//each row in Q - sumRows and each column in Q - sumRows
	Q.topLeftCorner(numCurrentNodes, numCurrentNodes).colwise() -= sumRows.head(
			numCurrentNodes);
	Q.topLeftCorner(numCurrentNodes, numCurrentNodes).rowwise() -= sumRows.head(
			numCurrentNodes).transpose();

	Q.diagonal().setZero();
	//cout << "Q Matrix:" << endl;		printMatrix(Q);
}

void NeighbourJoining::findMinQ(const MatrixXf& Q, const MatrixXi& rowsID, Pair& p) {
	Q.topLeftCorner(numCurrentNodes, numCurrentNodes).minCoeff(&pair.i,
			&pair.j);
	pair.iID = rowsID(pair.i);
	pair.jID = rowsID(pair.j);
	//cout << " closestPair:  " << pair.i << ", " << pair.j << endl;
	//cout << " closestPairID:  " << pair.iID << ", " << pair.jID << endl;
}

void NeighbourJoining::calcNewD(MatrixXf& currentD, MatrixXi& rowsID, const Pair& p) {
	//calculates distances to new node
	int j = 0;
	for (int i = 0; i < numCurrentNodes - 1; ++i) {
		if (i == p.i)
			j++;
		currentD(numCurrentNodes, i) = (currentD(p.i, i + j) + currentD(p.j, i + j) - currentD(p.i, p.j)) / 2;
		currentD(i, numCurrentNodes) = currentD(numCurrentNodes, i);
	}
	//cout << "distances to new node: " << currentD.row(numCurrentNodes).head(numCurrentNodes-1) <<endl;

	//swaps rows and columns so that the closest pair nodes go right and at the bottom of the matrix
	currentD.row(p.i).head(numCurrentNodes - 1).swap(
			currentD.row(numCurrentNodes - 1).head(numCurrentNodes - 1));

	currentD.col(p.i).head(numCurrentNodes - 1).swap(
			currentD.col(numCurrentNodes - 1).head(numCurrentNodes - 1));

	currentD.row(p.j).head(numCurrentNodes - 1).swap(
			currentD.row(numCurrentNodes).head(numCurrentNodes - 1));

	currentD.col(p.j).head(numCurrentNodes - 1).swap(
			currentD.col(numCurrentNodes).head(numCurrentNodes - 1));

	currentD.diagonal().setZero();
	//cout << "new Matrix:" << endl;  	printMatrix(currentD);

	//adjusts node IDs to new matrix indices
	int newNode = 2 * numObservableNodes - numCurrentNodes;
	rowsID.row(p.i).swap(rowsID.row(numCurrentNodes - 1));
	rowsID.row(p.j).swap(rowsID.row(newNode));

	//cout << "rowsID:   " << rowsID.transpose(); cout << endl;
}

void NeighbourJoining::updateD(MatrixXf& D, const MatrixXf& currentD, const Pair& p,
		int newNode) {
	//calculates distance form all nodes to the new node
	D.row(newNode).head(newNode) = (((D.row(p.iID) + D.row(p.jID))
			- MatrixXf::Constant(1, D.rows(), 1) * D(p.iID, p.jID)) / 2).head(
			newNode);
	D.col(newNode).head(newNode) = D.row(newNode).head(newNode);

	//calculates distances from child nodes to new node
	D(newNode, p.iID) = abs(currentD.row(p.i).head(numCurrentNodes).sum()
			- currentD.row(p.j).head(numCurrentNodes).sum());
	D(newNode, p.iID) /= (2 * (numCurrentNodes - 2));
	D(newNode, p.iID) = D(p.iID, p.jID)/2;
	D(p.iID, newNode) = D(newNode, p.iID);
	D(newNode, p.jID) = D(p.jID, newNode) = D(p.iID, p.jID) - D(p.iID, newNode);
	//cout << "updated D: " << endl << D << endl;
}

void NeighbourJoining::printMatrix(const MatrixXf& M) {
	cout << M.topLeftCorner(numCurrentNodes, numCurrentNodes);
	cout << endl;
}

/*
 * BuildNJTree.h
 *
 *  Created on: 12 Sep 2015
 *      Author: osboxes
 */
#ifndef BUILDNJTREE_H_
#define BUILDNJTREE_H_

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/config.hpp>
#include <Eigen/Dense>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "ClosestPair.h"

using namespace boost;
using namespace std;

/** Stores the result of the NeighbourJoining algorithm */
class BuildNJTree {
public:
	//Definitions and typedefs for boost graph
	typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_name_t, int>,
			no_property> Graph;
	typedef graph_traits<Graph>::vertex_descriptor Vertex;
	typedef graph_traits<Graph>::vertex_iterator VertexIterator;
	typedef graph_traits<Graph>::edge_iterator EdgeIterator;
	//typedef property_map<Graph, vertex_index_t>::type IndexMap;
	typedef property_map<Graph, vertex_name_t>::type NameMap;
	/**Constructs graph with vertices and gives them IDs
	 * @param _numJoints Number of vertices*/
	BuildNJTree();
	virtual ~BuildNJTree();
	/**Initializes a graph for the given number IDs and stores the given IDs in the graph
	 * @param IDs The IDs of the nodes for which to allocate memory
	 * @param  numOccupiedNodes The number of the rest given IDs in the whole graph*/
	void configure(Eigen::MatrixXi& IDs, int numOccupiedNodes = 0);
	/**Adds new node and edges to the two given nodes in the graph
	 *@param p IDs of the given nodes */
	void addClosestPair(const Pair& p);
	/**The last step of building NJ subtree is to connect the remained two nodes to each other*/
	void connectLast2Nodes();
	/**Prints the vertices and the edges of the tree*/
	void printTree();
	/**Prints the tree in a dot file*/
	void print(std::string filename) ;
	Graph graph;
	//Needed to set the vertex indices
	NameMap name;


private:
	//number of nodes from which to build LTM
	int numJoints;
	//defines which ID to give by adding a new node to the graph
	int newNodesIndex;
};

#endif /* BUILDNJTREE_H_ */


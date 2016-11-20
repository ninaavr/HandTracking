/*
 * MinimalSpanningTree.h
 *
 *  Created on: 16 Sep 2015
 *      Author: osboxes
 */

#ifndef MINIMALSPANNINGTREE_H_
#define MINIMALSPANNINGTREE_H_

#include <boost/config.hpp>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/graph/graphviz.hpp>
#include <Eigen/Dense>
#include "NeighbourJoining.h"

using namespace std;
using namespace boost;

class MinimalSpanningTree {
public:
	//Definitions and typedefs for boost graph
	typedef boost::adjacency_list_traits<vecS, vecS, undirectedS>::vertex_descriptor Vertex;

	typedef boost::property<boost::edge_weight_t, float> EdgeProperty;

	typedef boost::property<boost::vertex_distance_t, float,
			boost::property<boost::vertex_predecessor_t, Vertex> > VertexProperty;

	typedef boost::adjacency_list<vecS, vecS, boost::undirectedS,
			VertexProperty, EdgeProperty> Graph;

	typedef vector<graph_traits<Graph>::vertex_descriptor> Vertices;
	typedef graph_traits<Graph>::vertex_iterator VertexIterator;
	typedef graph_traits<Graph>::edge_descriptor Edge;
	//typedef vector<graph_traits<Graph>::edge_descriptor> Edges;
	typedef graph_traits<Graph>::edge_iterator EdgeIterator;
	typedef property_map<Graph, vertex_index_t>::type NameMap;
	typedef property_map<Graph, vertex_distance_t>::type VertexDistanceMap;
	typedef property_map<Graph, vertex_predecessor_t>::type PredecessorMap;
	typedef property_map<Graph, edge_weight_t>::type WeightMap;

	MinimalSpanningTree();
	virtual ~MinimalSpanningTree();
	/**Stores the minimum spanning tree of a graph given in a distance matrix
	 * @param D The given distance matrix*/
	void findMinSpanningTree(MatrixXf& D);
	/**Prints the vertices and the edges of the tree*/
	void printTree();
	/**Prints the vertices and the edges of a graph in a dot file
	 * (with weights for edges, used for Minimal Spanning Trees)
	 * @param filename Name of the dot file*/
	void print(std::string filename);
	/**Prints the vertices and the edges of a graph in a dot file
	 * (as a tree with directed edges - used for LTM (Latent Tree Model))
	 * @param filename Name of the dot file*/
	void printSpanningTree(std::string filename);

	//needed for minimum spanning tree algorithm
	Graph graph;
	//NameMap name;
	VertexDistanceMap distance;
	PredecessorMap predecessor;
	WeightMap weight;
};

#endif /* MINIMALSPANNINGTREE_H_ */

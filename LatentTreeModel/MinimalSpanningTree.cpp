/*
 * MinimalSpanningTree.cpp
 *
 *  Created on: 16 Sep 2015
 *      Author: osboxes
 */

#include "MinimalSpanningTree.h"

MinimalSpanningTree::MinimalSpanningTree() {
	//name = get(vertex_name, graph);
	distance = get(vertex_distance, graph);
	weight = get(edge_weight, graph);
	predecessor = get(vertex_predecessor, graph);
	VertexIterator vi, vend;
	std::size_t c = 0;
	for (tie(vi, vend) = vertices(graph); vi != vend; ++vi, ++c) {
		distance[*vi] = 0;
	}
}

MinimalSpanningTree::~MinimalSpanningTree() {
	// TODO Auto-generated destructor stub
}

void MinimalSpanningTree::findMinSpanningTree(MatrixXf& D) {
	//initialisation
	size_t DLength = (D.rows() + 2) / 2;
	graph = Graph(DLength - 1);
	Edge e;
	bool b;

	//loads the distance matrix in a graph
	for (size_t i = 0; i < DLength; ++i) {
		for (size_t j = 0; j < i; ++j) {
			tie(e, b) = add_edge(i, j, graph);
			get(weight, e) = D(i, j);
		}
	}

	//performs Prim's algorithm
	Vertices p(num_vertices(graph));
	prim_minimum_spanning_tree(graph, &p[0]);
	//clears the graph
	graph.clear();

	//loads just spanning tree in the graph
	for (size_t i = 0; i != p.size(); ++i) {
		if (p[i] != i) {
			tie(e, b) = add_edge(i, p[i], graph);
			get(weight, e) = D(i, p[i]);
		}
	}
}

void MinimalSpanningTree::printTree() {
	VertexIterator vi, vend;
	EdgeIterator ei, eend;
	cout << "vertices:" << endl;
	for (tie(vi, vend) = vertices(graph); vi != vend; ++vi) {
		cout << "id:" << *vi << '\t';
	}
	cout << endl;
	cout << "edges:" << endl;
	for (tie(ei, eend) = edges(graph); ei != eend; ++ei) {
		cout << *ei << " = " << get(weight, *ei) << '\t';
	}
	cout << endl;
}

void MinimalSpanningTree::print(std::string filename) {
	std::ofstream dot_file(filename.c_str());

	dot_file << "digraph D {\n" << "  ratio=\"fill\"\n" << "  size=\"3,3\"\n"
			<< "  edge[arrowhead=\"vee\"]\n" << "  node[shape=\"circle\"]\n";

	EdgeIterator ei, ei_end;
	for (tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei) {
		dot_file << source(*ei, graph) << " -> " << target(*ei, graph)
				<< "; \n";
	}
	dot_file << "}";
}

void MinimalSpanningTree::printSpanningTree(std::string filename) {
	std::ofstream dot_file(filename.c_str());
	dot_file << "graph SpanningTree {\n" << "  ratio=\"fill\"\n"
			<< "  rankdir=LR\n" << "  size=\"3,3\"\n"
			<< "  node[shape=\"circle\"]\n";

	EdgeIterator ei, eend;
	for (boost::tie(ei, eend) = boost::edges(graph); ei != eend; ++ei) {
		dot_file << source(*ei, graph) << " -- " << target(*ei, graph)
				<< "[label=\"" << get(weight, *ei) << "\"" << "]";
	}
	dot_file << "}";
}

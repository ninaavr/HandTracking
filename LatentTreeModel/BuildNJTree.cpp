/*
 * BuildNJTree.cpp
 *
 *  Created on: 12 Sep 2015
 *      Author: osboxes
 */

#include "BuildNJTree.h"

BuildNJTree::BuildNJTree() {
}

BuildNJTree::~BuildNJTree() {
}

void BuildNJTree::configure(Eigen::MatrixXi& IDs, int numOccupiedNodes) {
	numJoints = (IDs.size()+2)/2;
	graph = Graph(numJoints);
	name = get(vertex_name, graph);
	newNodesIndex = numJoints + numOccupiedNodes;
	int i = 0;
	VertexIterator vi, vend;
	for (tie(vi, vend) = vertices(graph); vi != vend; ++vi, ++i) {
		name[*vi] = IDs(i);
	}
}

void BuildNJTree::addClosestPair(const Pair& p) {
	Vertex v = add_vertex(newNodesIndex, graph);
	VertexIterator vi, vend;
	for (tie(vi, vend) = vertices(graph); vi != vend; ++vi) {
		if (name[*vi] == p.iID) {
			add_edge(v, *vi, graph);
		}
		if (name[*vi] == p.jID) {
			add_edge(v, *vi, graph);
		}
	}
	++newNodesIndex;
}

void BuildNJTree::connectLast2Nodes() {
	//define nodes which we connect to each other
	Vertex v, u;
	VertexIterator vi, vend;
	bool firstFound = false;
	for (tie(vi, vend) = vertices(graph); vi != vend; ++vi) {
		//if there is a leaf node left, we connect it to the rest of the graph
		if ((degree(*vi, graph) == 0) || (degree(*vi, graph) == 2)) {
			if (!firstFound) {
				v = *vi;
				firstFound = true;
			} else
				u = *vi;
		}
	}
	add_edge(u, v, graph);
}

void BuildNJTree::printTree() {
	VertexIterator vi, vend;
	EdgeIterator ei, eend;
	cout << "vertices:" << endl;
	for (tie(vi, vend) = vertices(graph); vi != vend; ++vi) {
		cout << get(name, *vi) << '\t';
	}
	cout << endl;
	cout << "edges:" << endl;
	for (tie(ei, eend) = edges(graph); ei != eend; ++ei) {
		cout << "(" << get(name, source(*ei, graph)) << ", "
				<< get(name, target(*ei, graph)) << ")" << '\t';
	}
	cout << endl;
}

void BuildNJTree::print(std::string filename) {
	std::ofstream dot_file(filename.c_str());

	dot_file << "digraph D {\n" << "  ratio=\"fill\"\n" << "  size=\"3,3\"\n"
			<< "  edge[arrowhead=\"vee\"]\n" << "  node[shape=\"circle\"]\n";

	EdgeIterator ei, ei_end;
	for (tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei) {
		dot_file << get(name, source(*ei, graph)) << " -> "
				<< get(name, target(*ei, graph)) << "; \n";
	}
	dot_file << "}";
}


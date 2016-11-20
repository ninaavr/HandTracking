/*
 * CLNJ.cpp
 *
 *  Created on: 23 Sep 2015
 *      Author: osboxes
 */

#include "CLNJ.h"

CLNJ::CLNJ() {
}

CLNJ::~CLNJ() {

}

void CLNJ::calcCLNJ(MatrixXf& _D, MinimalSpanningTree& ms) {
	D = &_D;
	ms.findMinSpanningTree(*D);
	//at the beginning unavailable are all nodes IDs in the graph
	reservedIDs = (D->rows() + 1) / 2;
	size_t numNodes1 = (D->rows() + 2) / 2;
	MinimalSpanningTree::VertexIterator vi, vend;
	for (tie(vi, vend) = vertices(ms.graph); vi != vend; ++vi) {
		//selects internal, observable nodes
		if ((degree(*vi, ms.graph) > 1) && (*vi < numNodes1)) {
			BuildNJTree::Vertex root = *vi;
			selectedNodes = degree(root, ms.graph) + 1;
			MatrixXi nodesID(selectedNodes, 1);
			selectSubTree(ms, nodesID, root);
			calcNJ(ms, nodesID);
			//adds the new latent nodes
			reservedIDs += selectedNodes - 2;
		}
	}
	finishTree(ms);
}

void CLNJ::selectSubTree(MinimalSpanningTree& ms, MatrixXi& nodesID,
		BuildNJTree::Vertex& root) {
	//first node is the selected internal node
	nodesID(0) = root;
	int currentNumNodes = 1;

	//deletes subtree from the whole tree
	//and stores direct neighbours of the selected internal node in nodesID
	MinimalSpanningTree::EdgeIterator ei, enext, eend;
	tie(ei, eend) = edges(ms.graph);
	for (enext = ei; ei != eend; ei = enext) {
		++enext;
		BuildNJTree::Vertex u = source(*ei, ms.graph);
		BuildNJTree::Vertex v = target(*ei, ms.graph);
		if ((u == root) || (v == root)) {
			remove_edge(*ei, ms.graph);
			(u == root) ? nodesID(currentNumNodes) = v : nodesID(currentNumNodes) = u;
			//cout << nodesID.transpose(); cout << endl;
			++currentNumNodes;
		}
	}
}

void CLNJ::calcNJ(MinimalSpanningTree& ms, MatrixXi& nodesID) {
	//builds subtree
	BuildNJTree NJTree = BuildNJTree();
	NeighbourJoining nj = NeighbourJoining();
	nj.calcNJ(*D, nodesID, NJTree, reservedIDs - selectedNodes);

	//loads the subtree(NJTree) in the main tree
	BuildNJTree::EdgeIterator ei, eend;
	for (tie(ei, eend) = edges(NJTree.graph); ei != eend; ++ei) {
		add_edge(NJTree.name[source(*ei, NJTree.graph)],
				NJTree.name[target(*ei, NJTree.graph)],
				MinimalSpanningTree::EdgeProperty(1), ms.graph);
	}
	NJTree.graph.clear();
}

void CLNJ::finishTree(MinimalSpanningTree& ms) {
	MinimalSpanningTree::EdgeIterator ei, enext, eend;
	tie(ei, eend) = edges(ms.graph);
	enext = ei;
	enext++;
	while (enext != eend) {
		ei++;
		enext++;
	}
	root = add_vertex(ms.graph);
	add_edge(root, source(*ei, ms.graph), MinimalSpanningTree::EdgeProperty(1),
			ms.graph);
	add_edge(root, target(*ei, ms.graph), MinimalSpanningTree::EdgeProperty(1),
			ms.graph);
	remove_edge(*ei, ms.graph);
}

void CLNJ::loadTreeToStructure(const MinimalSpanningTree& ms, HandStructure* handStructure) {
	//sets initial values
	for (int i = 0; i < 2 * numNodes - 1; ++i) {
		handStructure[i].left = -1;
		handStructure[i].right = -1;
	}
	dijkstra_shortest_paths(ms.graph, root,
			boost::predecessor_map(ms.predecessor).distance_map(ms.distance));
	MinimalSpanningTree::EdgeIterator ei, eend;
	for (tie(ei, eend) = edges(ms.graph); ei != eend; ++ei) {
		MinimalSpanningTree::Vertex parent, child;
		if (get(ms.distance, source(*ei, ms.graph))
				< get(ms.distance, target(*ei, ms.graph))) {
			parent = source(*ei, ms.graph);
			child = target(*ei, ms.graph);
		} else {
			parent = target(*ei, ms.graph);
			child = source(*ei, ms.graph);
		}
		(handStructure[parent].left == -1) ?
				handStructure[parent].left = child : handStructure[parent].right = child;
	}
}

void CLNJ::printStructure(HandStructure* handStructure, string filename) {
	std::ofstream dot_file(filename.c_str());
	dot_file << "digraph D {\n" << "  ratio=\"fill\"\n" << "  size=\"3,3\"\n"
			<< "  edge[arrowhead=\"vee\"]\n" << "  node[shape=\"circle\"]\n";
	for (int i = 0; i < numNodes * 2 - 1; ++i) {
		if (handStructure[i].left != -1)
			dot_file << i << " -> " << handStructure[i].left << "; \n";
		if (handStructure[i].right != -1)
			dot_file << i << " -> " << handStructure[i].right << "; \n";
	}
	dot_file << "}";
}


/*
 * ReadHandPoses.cpp
 *
 *  Created on: 25 Oct 2015
 *      Author: osboxes
 */

#include "MeanGeodesicDistances.h"

using namespace cv;
using namespace std;
using namespace boost;
using namespace Eigen;

MeanGeodesicDistances::MeanGeodesicDistances() {
	// TODO Auto-generated constructor stub
}

MeanGeodesicDistances::~MeanGeodesicDistances() {
	// TODO Auto-generated destructor stub
}

void MeanGeodesicDistances::initGraph(int numberNodes) {
	graph.clear();
	graph = Graph(numberNodes);
	index = get(vertex_index, graph);
	distance = get(vertex_distance, graph);
	predecessor = get(vertex_predecessor, graph);
	weight = get(edge_weight, graph);
	VertexIterator vi, vend;
	std::size_t c = 0;
	for (tie(vi, vend) = vertices(graph); vi != vend; ++vi, ++c) {
		index[*vi] = c;
		distance[*vi] = 0;
	}
}

void MeanGeodesicDistances::calcGeodesicDistances(std::string pathToInutData,
		MatrixXf& distanceMatrix) {
	distanceMatrix = MatrixXf::Constant(2 * numNodes - 1, 2 * numNodes - 1, 0);
	//counts number of valid nodes
	MatrixXf numValidImages(numNodes, numNodes);
	numValidImages = MatrixXf::Constant(numNodes, numNodes, 0);
	ifstream NodesCoordinates;
	NodesCoordinates.open((pathToInutData + "labels.txt").c_str());
	if (NodesCoordinates.fail()) {
		cout << endl << "Error: labels.txt not found.";
		cout << endl;
		exit(-1);
	}
	Node* nodes = new Node[16 * numImages];
	for (int i = 0; i < numImages; ++i) {
		initGraph();
		string pathToCurrentImage;
		readLine(NodesCoordinates, i, nodes, pathToCurrentImage);
		pathToCurrentImage = pathToInutData + "Depth/" + pathToCurrentImage;
		//cout << pathToCurrentImage<<endl;
		bool imageFound = addEdgesToTree(pathToCurrentImage, nodes, i * 16);
		if (imageFound)
			calcMinimalPaths(distanceMatrix, numValidImages);
		//cout << distanceMatrix.topLeftCorner(numNodes, numNodes); cout << endl;
	}
	NodesCoordinates.close();

	//calculates mean distances of the sums
	for (int i = 0; i < numNodes; ++i) {
		for (int j = 0; j < numNodes; ++j) {
			distanceMatrix(i, j) = distanceMatrix(i, j) / numValidImages(i, j);
		}
	}
	//cout << numValidImages<< endl;
	distanceMatrix.diagonal().setZero();
	//	cout << distanceMatrix.topLeftCorner(numNodes, numNodes);		cout << endl;
}

bool MeanGeodesicDistances::addEdgesToTree(const string& pathToImage,
		const Node* nodes, int offset) {
	Mat image = imread(pathToImage.c_str(), IMREAD_GRAYSCALE);
	if (!image.data) {
		cout << "image not found" << endl << "image = " << pathToImage << endl;
		image.release();
		return false;
	} else {
		VertexIterator vi1, vi2, vend, vibegin;
		int i = 0, j = 0;
		for (tie(vi1, vend) = vertices(graph); vi1 != vend; ++vi1, ++i) {
			j = i;
			for (vi2 = vi1; vi2 != vend; ++vi2, ++j) {
				//cout << "iterate over all nodes:   "<< get (name, *vi1) << '\t' << get (name, *vi2)<< endl;
				//cout << nodes[i].coord[2] << '\t' << nodes[j].coord[2] <<endl;
				if (isValidPath(image, nodes[i], nodes[j]) && (vi2 != vi1)) {
					float dist = calcEuclideanDistance(nodes[i], nodes[j]);
					//cout << get(name, *vi1) << ", " << get(name, *vi2) << "   "<< dist << endl;
					add_edge(*vi1, *vi2, EdgeProperty(dist), graph);
				}
			}
		}
		image.release();
		return true;
	}
}

void MeanGeodesicDistances::calcMinimalPaths(MatrixXf& distanceMatrix,
		MatrixXf& numvalidImg) {
	VertexIterator vi1, vi2, vend;
	for (tie(vi1, vend) = vertices(graph); vi1 != vend; ++vi1) {
		distance[*vi1] = 0;
	}
	for (tie(vi1, vend) = vertices(graph); vi1 != vend; ++vi1) {
		dijkstra_shortest_paths(graph, *vi1,
				predecessor_map(predecessor).distance_map(distance));
		vi2 = vi1;
		int j = get(index, *vi1);
		//cout << "degree of vertex 2:   " << degree(*vi2, graph) << endl;
		//cout << "degree of vertex 1:   " << degree(*vi1, graph) << endl;
		for (int i = j; i < numNodes; ++i, ++vi2) {
			if(get(distance, *vi2)<feasibleDistances){
				//cout << "distance    " << get(distance, *vi2) << endl;
				distanceMatrix(i, j) += get(distance, *vi2);
				distanceMatrix(j, i) = distanceMatrix(i, j);
				numvalidImg(i, j) += 1;
				numvalidImg(j, i) = numvalidImg(i, j);
			}
			//cout <<" i   "<< i << '\t'<<"j    "<<j << '\t'<<"validImgs     "<< numvalidImg(i, j)<< endl;
		}
	}
	//cout<< 	"nodes    " << get(name, *vi1) <<  ":  "<<get(name, *vi2) << endl;
	//cout << "indices    " << i << " , " << j << endl;
	//cout << "distance "<< get(distance, *vi2) << endl;
}

void MeanGeodesicDistances::readLine(ifstream& file, int numCurrentImage,
		Node* nodes, string& pathToCurrentImage) {
	if (file.is_open()) {
		string line;
		if (getline(file, line)) {

			//reads the directory of the current image
			int pos = line.find(' ');
			pathToCurrentImage = line.substr(0, pos);
			//cout << pathToCurrentImage;

			//reads the values for the nodes of the current image
			istringstream currentLine(line);
			currentLine.ignore(100, ' ');
			for (int i = 0; i < numNodes; ++i) {
				for (int j = 0; j < 3; ++j)
					currentLine	>> nodes[numCurrentImage * numNodes + i].coord[j];
				//	cout << "x:  " << nodes[numCurrentImage * numNodes + i].x
				//	 << "  y:  " << nodes[numCurrentImage * numNodes + i].y
				//	 << "  z:  " << nodes[numCurrentImage * numNodes + i].z << endl;
			}
		}
	}
}

bool MeanGeodesicDistances::isValidPath(const cv::Mat& image, const Node& node1,
		const Node& node2) {
	Point p1 = Point(node1.coord[0], node1.coord[1]);
	Point p2 = Point(node2.coord[0], node2.coord[1]);
	LineIterator line = LineIterator(image, p1, p2);

	//the points could be rounded wrong and can get values on the background
	float f;
	for (int i = 0; i < line.count; ++i, ++line) {
		f = image.at<uchar>(line.pos());
		if (f > backgroudIntensity - balance) {
			return false;
		}
	}
	//cout << "line intensity : " << f <<endl;
	return true;
}

float MeanGeodesicDistances::calcEuclideanDistance(const Node node1,
		const Node node2) {
	return abs(node1.coord[0] - node2.coord[0])
			+ abs(node1.coord[2] - node2.coord[2])
			+ abs(node1.coord[2] - node2.coord[2]);
}

void MeanGeodesicDistances::print(std::string filename) {
	std::ofstream dot_file(filename.c_str());
	dot_file << "graph SpanningTree {\n" << "  ratio=\"fill\"\n"
			<< "  rankdir=LR\n" << "  size=\"3,3\"\n"
			<< "  node[shape=\"circle\"]\n";
	WeightMap weight;
	EdgeIterator ei, eend;
	for (boost::tie(ei, eend) = boost::edges(graph); ei != eend; ++ei) {
		dot_file << get(index, source(*ei, graph)) << " -- "
				<< get(index, target(*ei, graph)) << "[label=\""
				<< get(weight, *ei) << "\"" << "]";
	}
	dot_file << "}";
}

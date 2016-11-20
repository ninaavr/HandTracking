/*
 * ReadHandPoses.h
 *
 *  Created on: 25 Oct 2015
 *      Author: osboxes
 */
#ifndef READHANDPOSES_H_
#define READHANDPOSES_H_
#include <boost/config.hpp>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include "../HandPoseStructs.h"
#define backgroudIntensity 255

#define numNodes 16
#define balance 128
#define feasibleDistances 600

class MeanGeodesicDistances {
public:
	//Definitions and typedefs for boost graph
	typedef boost::adjacency_list_traits<boost::listS, boost::listS,
			boost::undirectedS>::vertex_descriptor Vertex;

	typedef boost::property<boost::edge_weight_t, float> EdgeProperty;

	typedef boost::property<boost::vertex_index_t, int,
			boost::property<boost::vertex_name_t, int,
					boost::property<boost::vertex_distance_t, float,
							boost::property<boost::vertex_predecessor_t, Vertex> > > > VertexProperty;

	typedef boost::adjacency_list<boost::listS, boost::listS,
			boost::undirectedS, VertexProperty, EdgeProperty> Graph;

	typedef boost::graph_traits<Graph>::edge_descriptor Edge;

	typedef boost::graph_traits<Graph>::edge_iterator EdgeIterator;

	typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;

	typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;

	typedef boost::property_map<Graph, boost::vertex_name_t>::type NameMap;

	typedef boost::property_map<Graph, boost::vertex_distance_t>::type DistanceMap;

	typedef boost::property_map<Graph, boost::vertex_predecessor_t>::type PredecessorMap;

	typedef boost::property_map<Graph, boost::edge_weight_t>::type WeightMap;

	MeanGeodesicDistances();
	virtual ~MeanGeodesicDistances();
	/**Reads the training samples and stores their mean distances
	 * @param pathToInutData Absolute path to Handtracking folder (inclusive)
	 * @param distanceMatrix Distance matrix where the mean distances between each couple of nodes are stored*/
	void calcGeodesicDistances(std::string pathToInutData, Eigen::MatrixXf& distanceMatrix);
	/**Prints a dot file of the graph
	 * @param filename Name of the file*/
	void print(std::string filename);
	//graph and its settings for Dijkstra algorithm
	Graph graph;
	IndexMap index;
	//NameMap name;
	DistanceMap distance;
	PredecessorMap predecessor;
	WeightMap weight;
private:
	/** Makes vertices for the graph and sets its properties
	 * @param numberNodes Number of the vertices*/
	void initGraph(int numberNodes=16);
	/** Loads edges to the graph if the line between them doesn't go trough the background
	 * @param pathToImage absolute path to current image
	 * @param nodes array with the positions of all joints on the images
	 * @param offset The index where the nodes' coordinates for the current image start
	 * @return Returns true if the current image was found*/
	bool addEdgesToTree(const std::string& pathToImage, const Node* nodes, int offset);
	/**Finds the shortest paths in the graph for every node to all the others and stores them in the distance matrix
	 * @param distanceMatrix The matrix where the shortest paths are stored*/
	void calcMinimalPaths(Eigen::MatrixXf& distanceMatrix, Eigen::MatrixXf& numvalidImg);
	/**Reads a line from the file with the coordinates of the nodes
	 * @param file The file from which the coordinates are read
	 * @param numCurrentImage The number of the current image
	 * @param nodes Array of structure Node that stores the nodes coordinates
	 * @param pathToCurrentImage Path from InputData/Depth/ to the image is read and stored here*/
	void readLine(std::ifstream& file, int numCurrentImage, Node* nodes,  std::string& pathToCurrentImage);
	/** Returns false if the path between two pixels goes through the background
	 * @param image Image for which the path is checked
	 * @param node1 Structure that consists of the x, y and z coordinates of a point
	 * @param node2 Structure that consists of the x, y and z coordinates of the other point
	 * @result Returns true id the path is valid*/
	bool isValidPath(const cv::Mat& image, const Node& node1, const Node& node2);
	/**Calculates the Euclidean distance between two points(pixels)
	 * @param node1 Structure with x, y, z coordinates of one point
	 * @param node2 Structure with x, y, z coordinates of other point
	 * @result The Euclidean distance of the two points*/
	float calcEuclideanDistance(const Node node1, const Node node2);
};

#endif /* READHANDPOSES_H_ */

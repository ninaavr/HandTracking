#include "LatentTreeModel/BuildNJTree.h"
#include "LatentTreeModel/NeighbourJoining.h"
#include "LatentTreeModel/MinimalSpanningTree.h"
#include "LatentTreeModel/CLNJ.h"
#include "HandPositionsLoading.h"
#include "LatentTreeModel/MeanGeodesicDistances.h"
#include "Training.h"

using namespace Eigen;
using namespace std;

int main() {
	string pathToHandTrackingDirectory = dir;
#if 0
	/*----calculates the mean distance between each two nodes of all training samples----*/
	//mean distances for all images
	MatrixXf distanceMatrix(numNodes, numNodes);
	distanceMatrix = MatrixXf::Constant(numNodes, numNodes, 0);
	MeanGeodesicDistances x = MeanGeodesicDistances();
	x.calcGeodesicDistances(pathToHandTrackingDirectory + "InputData/",
			distanceMatrix);
	ofstream meanDistancesOutput(
			(pathToHandTrackingDirectory + "OutputData/MeanDistances.txt").c_str());
	for (int i = 0; i < numNodes; ++i) {
		for (int j = 0; j < numNodes; ++j) {
			meanDistancesOutput << distanceMatrix(i, j) << '\t';
		}
		meanDistancesOutput << '\n';
	}
	meanDistancesOutput.close();
#endif
#if 0

	/*----calculates LTM with a distance matrix, loaded in a file from InputData directory
	 * and to load LTM training samples-----*/
	//reads data from file with mean distances
	ifstream meanDistancesInput;
	meanDistancesInput.open((pathToHandTrackingDirectory + "InputData/MeanDistances.txt").c_str());

	if (meanDistancesInput.fail()) {
		cout << endl << "Error: MeanDistances.txt not found.";
		cout << endl
		<< "MeanDistances.txt should be in HandTracking/InputData directory"
		<< endl;
		exit(-1);
	}
	int l = 2 * numNodes - 1;
	MatrixXf meanDistancesMatrix(l, l);
	meanDistancesMatrix = MatrixXf::Constant(l, l, 0);
	for (int i = 0; i < numNodes; ++i) {
		for (int j = 0; j < numNodes; ++j) {
			meanDistancesInput >> meanDistancesMatrix(i, j);
		}
	}
	meanDistancesInput.close();
	//cout << meanDistancesMatrix.topLeftCorner(numNodes, numNodes); cout << endl;

	MinimalSpanningTree ms = MinimalSpanningTree();
	CLNJ clnj = CLNJ();
	clnj.calcCLNJ(meanDistancesMatrix, ms);
	HandStructure* handStructure = new HandStructure[2 * numNodes - 1];
	clnj.loadTreeToStructure(ms, handStructure);
	clnj.printStructure(handStructure, (pathToHandTrackingDirectory + "OutputData/LatentTreeModel").c_str());
#endif
#if 1
	/**----build a LRT by given Latent Tree Model in a dot file------**/
	Training t = Training();
	t.trainTree(pathToHandTrackingDirectory);
#endif
	cout << endl << "Program terminated." << endl;
	return 0;
}

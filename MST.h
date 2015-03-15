#include "common.h"
#include "Minmatching/PerfectMatching.h"
#include <vector>
#include <unordered_map>
#include <utility>

#pragma once

class MST {
public:
	float** adjacentMatrix;
    float ** mstAdjMatrix;
	int* parent; //Array to store constructed MST
	int* key; //Key values used to pick minimum weight edge in cut
	bool* mstSet; //To represent set of vertices not yet included in MST
	int N; //the size of pointset

    int * edges;
    int * weights;

    bool * visited;

	MST(float** adjacentMatrix, int size);
	~MST();

	//deliverable a
	void makeTree();
	int printMST();

	//deliverable b
	int makeTSP2();
    void printTSP2();

    void dfs(int u);

	//deliverable c
    void findOddDegreeVertices();
	int makeTSP1_5();
    void eulerTour(int u, float ** adjMatrix);
    void printTSP1_5();
	

    std::vector<int> mstResults;
    std::vector<int> TSP2Results;
    std::vector<int> TSP1p5Results;

	float calMean(int option);
	float calStd(int option, int mean);

private:
    std::vector<int> odds;
    std::vector<int> prelist;
    std::vector<int> tour;
    std::unordered_map<int, int> matched;
    PerfectMatching * pm;
	void minimumMatching();
	int combine(int node_num);
	int minKey(int key[], bool mstSet[]);

};

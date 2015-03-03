#include "common.h"

#pragma once

class MST {
public:
	int** adjacentMatrix;
	int* parent; //Array to store constructed MST
	int* key; //Key values used to pick minimum weight edge in cut
	bool* mstSet; //To represent set of vertices not yet included in MST
	int N; //the size of pointset

	MST(int** adjacentMatrix, int size);
	~MST();

	//deliverable a
	void makeTree();
	void printMST();

	//deliverable b
	void makeTSP2();

	//deliverable c
	void makeTSP1_5();
	
	float calMean(int option);
	float calStd(int option, int mean);

private:
	void minimumMatching();
	void combine();
	int minKey(int key[], bool mstSet[]);

};

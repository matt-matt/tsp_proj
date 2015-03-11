#include "common.h"
#include "point.h"
#include "MST.h"

/*
This project is a starter code and wrappers for CSE101W15 Implementation project.

point.h - uniform random pointset generator

MST.h - minimum spanning tree

PerfectMatching.h - interface to min cost perfect matching code 

-------------------------------------
PerfectMatching is from the paper:

Vladimir Kolmogorov. "Blossom V: A new implementation of a minimum cost perfect matching algorithm."
In Mathematical Programming Computation (MPC), July 2009, 1(1):43-67.

sourcecode : pub.ist.ac.at/~vnk/software/blossom5-v2.05.src.tar.gz

*/


int main() {
	set< pair<int,int> > generatedPointset;
	float** adjacentMatrix;
	int W, H, N;
	Point pointset;

	W = 16811;
	H = 17722;
	N = 9596;

	cout<<"W: "<<W<<" H: "<<H<<" N:"<<N<<endl;

	pointset.generatePoint(W, H, N); //max(W,H,N) should be < 20000 because of memory limitation
	//pointset.printPointset();

	generatedPointset = pointset.getPointset();
	adjacentMatrix = pointset.getAdjacentMatrix();

	//Deliverable A: From pointset and adjacentMatrix, you should construct MST with Prim or Kruskal

	MST mst(adjacentMatrix, N);
	mst.makeTree();
    cout<<"Total cost of MST: "<<mst.printMST()<<endl;
	

	//Deliverable B: Find TSP2 path from the constructed MST
    
	cout<<"Length of Eulerian circuit, no matching: "<<mst.makeTSP2()<<endl;


	//Deliverable C: Find TSP1.5 path from the constructed MST
    cout<<"Length of Eulerian circuit with matching: "<<mst.makeTSP1_5()<<endl;
			
	return 0;
}

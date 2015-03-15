#include "common.h"
#include "point.h"
#include "MST.h"
#include "Minmatching/timer.h"

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
    cout<<"Starting A...";
    cout.flush();
    double start_time = get_time();
	mst.makeTree();
    printf("MST found in %.3f seconds.\n", get_time() - start_time);
    cout<<"Total cost of MST: "<<mst.printMST()<<endl;
	

	//Deliverable B: Find TSP2 path from the constructed MST
    cout<<"Starting B...";
    cout.flush();
    start_time = get_time();
	cout<<"Length of Eulerian circuit, no matching: "<<mst.makeTSP2()<<endl;
    printf("Eulerian curcuit found in %.3f seconds.\n", get_time() - start_time);



	//Deliverable C: Find TSP1.5 path from the constructed MST
    cout<<"Starting C...";
    cout.flush();
    start_time = get_time();
    cout<<"Length of Eulerian circuit with matching: "<<mst.makeTSP1_5()<<endl;
    printf("Eulerian curcuit with matching found in %.3f seconds.\n", get_time() - start_time);

			
	return 0;
}

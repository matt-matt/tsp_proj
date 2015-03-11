#include "MST.h"
#include "Minmatching/PerfectMatching.h"
#include <cmath>

MST::MST(float** input, int size) {
	adjacentMatrix = input;
    mstAdjMatrix = new float * [size];
    for (int i = 0; i < size; i++) {
        mstAdjMatrix[i] = new float[size];
    }
	key = new int[size];   
    mstSet = new bool[size];  
	parent = new int[size];

    visited = new bool[size];

	N = size;
}

MST::~MST() {
    delete[] key;
    delete[] mstSet;
    for (int i = 0; i < N; i++) {
        delete[] mstAdjMatrix[i];
    }
    delete[] mstAdjMatrix;
    delete[] parent;
    delete[] visited;
    delete pm;
}

//use Prim's algorithm or Kruskal algorithm. Copied from 'http://www.geeksforgeeks.org/greedy-algorithms-set-5-prims-minimum-spanning-tree-mst-2/'
void MST::makeTree() { 
     // Initialize all keys as INFINITE
     for (int i = 0; i < N; i++)
        key[i] = INT_MAX, mstSet[i] = false;
 
     // Always include first 1st vertex in MST.
     key[0] = 0;     // Make key 0 so that this vertex is picked as first vertex
     parent[0] = -1; // First node is always root of MST 
 
     // The MST will have V vertices
     for (int count = 0; count < N-1; count++)
     {
        // Pick thd minimum key vertex from the set of vertices
        // not yet included in MST
        int u = minKey(key, mstSet);
 
        // Add the picked vertex to the MST Set
        mstSet[u] = true;
 
        // Update key value and parent index of the adjacent vertices of
        // the picked vertex. Consider only those vertices which are not yet
        // included in MST
        for (int v = 0; v < N; v++)
           // mstSet[v] is false for vertices not yet included in MST
           // Update the key only if adjacentMatrix[u][v] is smaller than key[v]
          if (adjacentMatrix[u][v] && mstSet[v] == false && adjacentMatrix[u][v]
              <  key[v]) {
             parent[v]  = u, key[v] = adjacentMatrix[u][v];
          }
     }

    //Construct an adjacency matrix specifically for the MST
    for (int i = 1; i < N; i++) {
        mstAdjMatrix[i][parent[i]] = adjacentMatrix[i][parent[i]];
        mstAdjMatrix[parent[i]][i] = adjacentMatrix[parent[i]][i];
    }

}

// A utility function to find the vertex with minimum key value, from
// the set of vertices not yet included in MST
int MST::minKey(int key[], bool mstSet[])
{
   // Initialize min value
   int min = INT_MAX, min_index;
 
   for (int v = 0; v < N; v++)
     if (mstSet[v] == false && key[v] < min)
         min = key[v], min_index = v;
 
   return min_index;
}

// A utility function to print the constructed MST stored in parent[]
int MST::printMST() {
	cout<<endl;
	cout<<"Minimum spanning tree from the adjacency matrix"<<endl;
	cout<<"Edge   Weight"<<endl;
    int sum = 0;
	for (int i = 1; i < N; i++) {
		cout<<parent[i]<<" - "<<i<<"  "<<adjacentMatrix[i][parent[i]]<<endl;
        sum += adjacentMatrix[i][parent[i]];
	}
    
    return sum;

}

int MST::makeTSP2() {
	//make a Eulerian tour by DFS

	//add shortcuts if a vertex has no detours.

	//calculate heuristic TSP cost

    //Assemble list of nodes in ascending order of pre values from DFS
    //Append the DFS root node with it post value to complete cycle
    int root = rand() % (N - 1) + 1;
    prelist.push_back(std::make_pair(dfs(root, 1), root));
    //printTSP2();
    int sum = 0;
    for (int i = 1; i <= N; i++)    {
        int to = prelist[i].second;
        int from = prelist[i - 1].second;
        sum += adjacentMatrix[to][from];
    }
    return sum;
}

int MST::dfs(int u, int count) {
    visited[u] = true;
    prelist.push_back(std::make_pair(count, u));
    for (int v = 0; v < N; v++)    {
        if (!visited[v] && (mstAdjMatrix[u][v] && mstAdjMatrix[v][u]))  {
            count = dfs(v, ++count);
        }
    }
    return ++count;

}

void MST::printTSP2()   {
    cout<<endl;
	cout<<"Eulerien tour from shortcutted DFS of MST"<<endl;
	cout<<"Edge   Weight"<<endl;
    float sum = 0;
	for (int i = 1; i <= N; i++) {
        int to = prelist[i].second;
        int from = prelist[i - 1].second;
		cout<<from<<" - "<<to<<"  "<<adjacentMatrix[to][from]<<endl;
        sum += adjacentMatrix[to][from];
	}
    cout<<sum<<endl;
}

void MST::findOddDegreeVertices()   {
    for (int i = 0; i < N; i++) {
        int degree = 0;
        for (int j = 0; j < N; j++) {
            if (mstAdjMatrix[i][j] && mstAdjMatrix[j][i])   {
                degree++;
            }
        }
        if (degree % 2) {
            odds.push_back(i);
        }
    }
}

void LoadInput(int& node_num, int& edge_num, int*& edges, int*& weights,
vector<int> odds, float ** adjacencyMatrix) {
	int e = 0;

	edges = new int[2*edge_num];
	weights = new int[edge_num];

	for(int i = 0; i < odds.size(); ++i) {
		for(int j = i+1 ; j< odds.size(); ++j) {
    	    edges[2*e] = i;
		    edges[2*e+1] = j;
		    weights[e] = adjacencyMatrix[odds[i]][odds[j]];
		    e++;
		}
	}

	if (e != edge_num) { 
		cout<<"the number of edge is wrong"<<endl;

		exit(1); 
	}
}

void PrintMatching(int node_num, PerfectMatching* pm, std::vector<int> odds) {
	int i, j;

	for (i=0; i<node_num; i++) {
		j = pm->GetMatch(i);
		if (i < j) printf("%d %d\n", odds[i], odds[j]);
	}
}


void MST::minimumMatching() { //if you choose O(n^2)
	//find minimum-weight matching for the MST. 
	
	//you should carefully choose a matching algorithm to optimize the TSP cost.
    //Find the perfect minimum-weight matching 
	struct PerfectMatching::Options options;
	int i, e, node_num = odds.size(), edge_num = (node_num * (node_num - 1)) / 2;
	pm = new PerfectMatching(node_num, edge_num);

	LoadInput(node_num, edge_num, edges, weights, odds, adjacentMatrix);

	for (e=0; e<edge_num; e++) {
		pm->AddEdge(edges[2*e], edges[2*e+1], weights[e]);
	}

	pm->options = options;
	pm->Solve();

	double cost = ComputePerfectMatchingCost(node_num, edge_num, edges, weights, pm);
	printf("Total cost of the perfect min-weight matching = %.1f\n", cost);


}

void MST::printTSP1_5() {
    cout<<endl;
	cout<<"Eulerien tour from shortcutted DFS of MST with perfect matching of odd degree vertices"<<endl;
	cout<<"Edge   Weight"<<endl;
    float sum = 0;
	for (int i = 1; i < N; i++) {
        int to = tour[i];
        int from = tour[i - 1];
		cout<<from<<" - "<<to<<"  "<<adjacentMatrix[to][from]<<endl;
        sum += adjacentMatrix[from][to];
	}
    cout<<tour[N - 1]<<" - "<<tour[0]<<"  "<<adjacentMatrix[tour[N -
    1]][tour[0]]<<endl;
    sum += adjacentMatrix[tour[N - 1]][tour[0]];
    cout<<sum<<endl;
}

int MST::makeTSP1_5() {
	
    findOddDegreeVertices();

    
	//construct minimum-weight-matching for the given MST
	minimumMatching();

	//make all edges has even degree by combining mimimum-weight matching and MST
    return combine(odds.size());
}

int MST::eulerTour(int u, float ** adjMatrix)    {
    for (int v = 0; v < N; v++) {
         auto searchu = matched.find(u);
         auto searchv = matched.find(v);
         if (searchu != matched.end() && searchu -> second == v)   {
             matched.erase(searchu);
             //cout<<"Eliminating "<<u<<" - "<<v<<endl;
             eulerTour(v, adjMatrix);
         }
         else if (searchv != matched.end() && searchv -> second == u)  {
             matched.erase(searchv);
             //cout<<"Eliminating "<<v<<" - "<<u<<endl;
             eulerTour(v, adjMatrix);
        }

        if (adjMatrix[u][v] && adjMatrix[v][u]) {
            adjMatrix[u][v] = 0;
            adjMatrix[v][u] = 0;
            //cout<<"Eliminating "<<u<<" - "<<v<<endl;
            eulerTour(v, adjMatrix);
        }
   }
   if (u % 1000 == 0)   {
       cout<<".";
       cout.flush();
   }
   tour.push_back(u);
}
        

int MST::combine(int node_num) {
	//combine minimum-weight matching with the MST to get a multigraph which has vertices with even degree

    int i, j;

	for (i=0; i<node_num; i++) {
		j = pm->GetMatch(i);
		if (i < j)  {
            //printf("%d %d\n", odds[i], odds[j]);
            //mstAdjMatrix[odds[i]][odds[j]] = adjacentMatrix[odds[i]][odds[j]];
            //mstAdjMatrix[odds[j]][odds[i]] = adjacentMatrix[odds[j]][odds[i]];
            //cout<<mstAdjMatrix[odds[j]][odds[i]]<<endl;
            matched.insert(std::make_pair(odds[i], odds[j]));
        }
	}
    eulerTour(0, mstAdjMatrix);
    std::unordered_map<int, int> dupcheck;
    
    for (int i = 0; i < N; i++) {
        auto search = dupcheck.find(tour[i]);
        if (search != dupcheck.end())   {
            tour.erase(tour.begin() + i);
        }
        else    {
            dupcheck.insert(std::make_pair(tour[i], tour[i]));
        }
    }
    //printTSP1_5();
    int sum = 0;
    for (int i = 1; i <= N; i++)    {
        int to = tour[i];
        int from = tour[i - 1];
        sum += adjacentMatrix[to][from];
    }

    for (int i = 0; i < N; i++) {
        visited[i] = false;
    }

    return sum;


}

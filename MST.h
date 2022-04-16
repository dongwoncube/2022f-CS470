#pragma once

#include "./Matching/Graph.h"
#include "./Matching/BinaryHeap.h"
#include "./Matching/Globals.h"
#include <stdio.h>

//Prim's algorithm using binary heap
pair< list<int>, double > Prim(const Graph & G, const vector<double> & cost)
{
	BinaryHeap B;

	vector<int> father(G.GetNumVertices(), -1);
	list<int> mst;

	double obj = 0;

	//S E 1-2 3-4 5-6 ..... 
	int N = G.GetNumVertices();
	vector<vector<double>> adj(N, vector<double>(N));
	for(int i=0;i<N;i++) {
		for(int j=0;j<N;j++) {
			adj[i][j] = 1e30;
		}
		for(list<int>::const_iterator it = G.AdjList(i).begin(); it != G.AdjList(i).end(); it++) {
			int j = *it;
			adj[i][j] = cost[G.GetEdgeIndex(i,j)];
		}
	}
    auto adj0 = adj;
    vector<pair<double, int>> _cost(N);
    vector<bool> selected(N);
    vector<int> match(N, -1);
    selected[0] = true;

    for (int i=0;i+1<N;i+=2) {
        selected[i+1] = true;
        match[i] = i+1;
        match[i+1] = i;
        for (int j = 0; j < N; ++j) {
            adj[i][j] = adj[j][i] = min(adj[j][i], adj[j][i+1]);
        }
		mst.push_back(G.GetEdgeIndex(i, i+1));
		obj += cost[G.GetEdgeIndex(i, i+1)];
        //msst.push_back(i);
    }
    for (int i = 0; i < N; ++i) _cost[i].first = adj[0][i];

    for (int i = N/2 + 1; i < N; ++i) {
        pair<double, int> min_cost(1e30, 0);
        int min_idx;
        for (int j = 1; j < N; ++j) {
            if (selected[j] == false && min_cost > _cost[j]) {
                min_cost = _cost[j];
                min_idx = j;
            }
        }
        selected[min_idx] = true;
        for (int j = 1; j < N; ++j) _cost[j] = min(_cost[j], make_pair(adj[min_idx][j], min_idx));
        if (adj0[min_cost.second][min_idx] != min_cost.first) min_idx = match[min_idx];
		mst.push_back(G.GetEdgeIndex(min_cost.second, min_idx));
		obj += cost[G.GetEdgeIndex(min_cost.second, min_idx)];
    }

	/*
	//Put 0 in the heap
	B.Insert(0, 0);

	while(B.Size() > 0)
	{
		//Select the vertex that is closest to the growing tree
		int u = B.DeleteMin();
		int w = father[u];

		//Add {w,u} to the tree
		if(w != -1)
		{
			int i = G.GetEdgeIndex(w, u);
			mst.push_back(i);
			obj += cost[i];
		}

		//This is to indicate that u is already in the tree
		father[u] = -2;

		//Update the heap with vertices adjacent to u
		for(list<int>::const_iterator it = G.AdjList(u).begin(); it != G.AdjList(u).end(); it++)
		{
			int v = *it;
			
			//if v is already in the tree
			if(father[v] == -2)
				continue;

			double c = cost[G.GetEdgeIndex(u,v)];

			//v has not been reached by anyone else
			if(father[v] == -1)
			{
				father[v] = u;	
				B.Insert(c, v);
			}
			//we found a cheaper connection to v
			else if( LESS(c, cost[G.GetEdgeIndex(father[v], v)]) )
			{
				father[v] = u;
				B.ChangeKey(c, v);
			}
		}
	}
	*/

	if((int)mst.size() < G.GetNumVertices()-1)
		throw "Error: graph does not have a spanning tree";

	return pair< list<int>, double >(mst, obj);
}




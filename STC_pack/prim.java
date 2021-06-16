package STC_pack;

public class prim {
	private int V;
	private int parent[];
	private int graph[][];

	public void setV(int V_set){
		// Function to set vertices
		this.V = V_set;
	}

	int minKey(int key[], Boolean mstSet[])
	{
		// Initialize min value
		int min = Integer.MAX_VALUE, min_index = -1;

		// Select lowest weight edge from edges going out of "supernode"
		for (int v = 0; v < this.V; v++)
			if (mstSet[v] == false && key[v] < min) {
				min = key[v];
				min_index = v;
			}

		return min_index;
	}

	public int[] primMST(int graph_input[][])
	{
		// function to cunstruct MST
		// initializes private graph variable
		this.graph = new int[this.V][this.V]; 
		this.graph = graph_input;

		// initializes private parent variable
		this.parent = new int[this.V];

		// Key values used to pick minimum weight edge in cut
		int key[] = new int[this.V];
		
		// To represent set of vertices included in MST
		Boolean mstSet[] = new Boolean[this.V];

		// set first key to 0 and first parent to -1 (Because it is the root)
		
		this.parent[0] = -1;
		// Set all other keys to infinity
		for (int i = 0; i < this.V; i++) {
			key[i] = Integer.MAX_VALUE;
			mstSet[i] = false;
		}
		key[0] = 0;
		
		for (int count = 0; count < this.V - 1; count++) {
			// get index of minimum key that is not already in MST - finding the edge with the lowest weight
			int u = minKey(key, mstSet); 

			// Set that index to true in mstSet to indicate that it is already in MST
			mstSet[u] = true; // this node is now inculded in the MST

			// Update key and parent values for indices where mstSet is FALSE
			for (int v = 0; v < this.V; v++)
				if (graph[u][v] != 0 && mstSet[v] == false && graph[u][v] < key[v]) {
					// graph = 0 means that those vertices are not connected in any way so those are ignored
					// graph < key means that the selected node, u, has an edge connected to one of the available nodes of a lower weight than the previous option 
					// e.g. 
							// node 0 connects to node 3 with weight 6 - stored in key values under index 3
							// node 1 is now selected anf connects to node 3 with weight 6 (u=1)
							// at index 3 of key values, weight is 6 (node 0 - 3). The new suggested connection (node 1 - 8) has weight of 8
							// therefore graph > key and the value stays 6 in key values and the parent stays node 0
					this.parent[v] = u;
					key[v] = graph[u][v];
				}
		}
		return(parent);
	}
}
// Based on code by Aakash Hasija
// Code can be found at: https://www.geeksforgeeks.org/prims-minimum-spanning-tree-mst-greedy-algo-5/

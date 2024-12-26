#include <limits.h>
#include <vector>
#include <queue>
using namespace std;

#define V 16 // Number of vertices in the graph

// Function prototypes
int minDistance(int dist[]);
void dijkstra(int graph[V][V], int src, int dest);

// A utility function to find the vertex with minimum distance value
int minDistance(int dist[]) {
    int min = INT_MAX, min_index;
    for (int v = 0; v < V; v++)
        if (dist[v] <= min)
            min = dist[v], min_index = v;
    return min_index;
}

// Function that implements modified Dijkstra's algorithm allowing revisiting nodes
void dijkstra(int graph[V][V], int src, int dest) {
    int dist[V]; // The output array. dist[i] will hold the shortest distance from src to i

    // Initialize all distances as INFINITE
    for (int i = 0; i < V; i++) {
        dist[i] = INT_MAX;
    }

    // Distance of source vertex from itself is always 0
    dist[src] = 0;

    // Min-priority queue to pick the node with the smallest distance
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, src}); // {distance, node}

    // Loop until the priority queue is empty
    while (!pq.empty()) {
        int u = pq.top().second;
        int currentDist = pq.top().first;
        pq.pop();

        // Print the current node
        switch (u) {
            case 0: Serial.println("Moved from Start"); break;
            case 1: Serial.println("Moved to T1"); break;
            case 2: Serial.println("Moved to J1"); break;
            case 3: Serial.println("Moved to Yellow Warehouse"); break;
            case 4: Serial.println("Moved to T2"); break;
            case 5: Serial.println("Moved to J2"); break;
            case 6: Serial.println("Moved to Green Warehouse"); break;
            case 7: Serial.println("Moved to T3"); break;
            case 8: Serial.println("Moved to T4"); break;
            case 9: Serial.println("Moved to J3"); break;
            case 10: Serial.println("Moved to Red Warehouse"); break;
            case 11: Serial.println("Moved to J4"); break;
            case 12: Serial.println("Moved to Blue Warehouse"); break;
            case 13: Serial.println("Moved to T5"); break;
            case 14: Serial.println("Moved to T6"); break;
            case 15: Serial.println("Moved to End"); break;
        }

        // Traverse all adjacent vertices of the picked vertex u
        for (int v = 0; v < V; v++) {
            if (graph[u][v] && dist[u] != INT_MAX && currentDist + graph[u][v] < dist[v]) {
                dist[v] = currentDist + graph[u][v];
                pq.push({dist[v], v}); // Add the neighbor with the updated distance
                Serial.print("Updated distance of node ");
                Serial.print(v);
                Serial.print(" to ");
                Serial.println(dist[v]);

                // Check for obstacles and print corresponding messages
                if (v == 3) {
                    Serial.println("Detected yellow obstacle");
                } else if (v == 6) {
                    Serial.println("Detected green obstacle");
                } else if (v == 10) {
                    Serial.println("Detected red obstacle");
                } else if (v == 12) {
                    Serial.println("Detected blue obstacle");
                }
            }
        }
    }

    // Print the constructed distance array
    Serial.println("Vertex Distance from Source");
    for (int i = 0; i < V; i++) {
        Serial.print(i);
        Serial.print(" \t\t ");
        Serial.println(dist[i]);
    }
}

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Example graph represented as an adjacency matrix
    int graph[V][V] = {
       // S, T1,J1, Y,T2,J2,Gr,T3,T4,J3,Re,J4,Bl,T5,T6,End
          {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Start
          {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // T1
          {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // J1
          {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Yellow Warehouse
          {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // T2
          {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // J2
          {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Green Warehouse
          {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, // T3
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}, // T4
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0}, // J3
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}, // Red Warehouse
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0}, // J4
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0}, // Blue Warehouse
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}, // T5
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, // T6
          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // End
      };

    Serial.println("Starting Modified Dijkstra's Algorithm");

    dijkstra(graph, 0, 15); // Example: from Start (node 0) to End (node 15)

    delay(5000); // Delay to avoid flooding the Serial Monitor
}
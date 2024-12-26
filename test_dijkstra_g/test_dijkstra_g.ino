//Dijsktra graph


#include <limits.h>
#include <vector>
using namespace std;

#define V 20 // Number of vertices in the graph

// Function prototypes
int minDistance(int dist[], bool sptSet[]);
void dijkstra(int graph[V][V], int src, int dest);

// A utility function to find the vertex with minimum distance value
int minDistance(int dist[], bool sptSet[]) {
    int min = INT_MAX, min_index;
    for (int v = 0; v < V; v++)
        if (!sptSet[v] && dist[v] <= min)
            min = dist[v], min_index = v;
    return min_index;
}

// Function that implements Dijkstra's single source shortest path algorithm
void dijkstra(int graph[V][V], int src, int dest) {
    int dist[V]; // The output array. dist[i] will hold the shortest distance from src to i
    bool sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest path tree

    // Initialize all distances as INFINITE and sptSet[] as false
    for (int i = 0; i < V; i++) {
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }

    // Distance of source vertex from itself is always 0
    dist[src] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
        int u = minDistance(dist, sptSet);
        sptSet[u] = true;

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

        // Update dist value of the adjacent vertices of the picked vertex.
        for (int v = 0; v < V; v++) {
            if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
                /*Serial.print("Updated distance of node ");
                Serial.print(v);
                Serial.print(" to ");
                Serial.println(dist[v]);*/

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
       //  S, T1,J1, Y,J1',T2,J2,Gr,J2',T3,T4,J3,Re,J3',J4,Bl,J4',T5,T6,End
          {0,  1, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0}, // Start
          {0,  0, 1, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0}, // T1
          {0,  0, 0, 1, 0, 1, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0}, // J1
          {0,  0, 0, 0, 1, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0}, // Yellow Warehouse
          {0,  0, 0, 0, 0, 1, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0}, //J1'
          {0,  0, 0, 0, 0, 0, 1, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0}, // T2
          {0,  0, 0, 0, 0, 0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0}, // J2
          {0,  0, 0, 0, 0, 0, 0, 0, 1,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0}, // Green Warehouse
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0}, //J2'
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 1, 0, 0,  0, 0, 0, 0,  0, 0, 0}, // T3
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 1, 0,  0, 0, 0, 0,  0, 0, 0}, // T4
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 1,  0, 1, 0, 0,  0, 0, 0}, // J3
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0}, // Red Warehouse
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 1, 0, 0,  0, 0, 0}, // J3'
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 1, 0,  1, 0, 0}, // J4
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 1,  0, 0, 0}, // Blue Warehouse
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0}, //J4'
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 1, 0}, // T5
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 1}, // T6
          {0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0},  // End
      };

    Serial.println("Starting Dijkstra's Algorithm");

    dijkstra(graph, 0, 15); // Example: from Start (node 0) to End (node 15)

    delay(5000); // Delay to avoid flooding the Serial Monitor
}
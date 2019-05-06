# graph-algorithms
Python implementation of graph optimisation algorithms (Kruskal, Dijkstra, Floyd-Warshall, and Bellman-Ford) using a personal graph data structure. Some of the algorithms include implementation of path reconstruction and negative-weight cycle recognition. 

The included *exercise.py* file demonstrates how to instantiate a graph, call the algorithms, and properly catch negative-weight cycles.

# Documentation

**Kruskal(graph)**
*Given a graph, returns it minimum spanning tree (which is returned as a graph).*

**Dijkstra(graph, source, target)**
*Given a graph without any negative-weight cycles, returns the minimum distance and the shortest path between the source and the target.*
If the target is not given, it returns the minimum distance from the source vertex to every other vertex.
If the target is given, it returns the minimum distance to the target and the shortest path taken.

**Floyd(graph, source, target)**
*Given a graph with possible negative-weight cycles, returns the minimum distance and the shortest path between the source and the target.*
If neither the target nor the source is given, it returns all the minimum distances between every vertex.
If both are given, it returns the minimum distance from the source to the target and the shortest path taken.

**Bellman_Ford(graph, source)**
*Given a graph with possible negative-weight cycles, returns the minimum distance and the shortest path between the source and the target.*

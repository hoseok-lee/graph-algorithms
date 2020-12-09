from structures import *
from heapq import *


class NegativeWeightCycle(RuntimeError):
    pass


#
# KRUSKAL'S ALGORITHM
#           Calculates the minimum spanning tree (MST)
#
# Input:    Graph
# Output:   Graph
# Notes:    Implemented with a disjoint set
#           The most standard MST algorithm
#
def Kruskal(graph):
    A = Graph()

    # Initialize disjoint set for vertices
    djs_vertices = DisjointSet()
    for vertex in graph.vertices:
        djs_vertices.make_set(vertex)
        A.add_vertex(vertex.value)

    # Initialize priority queue for edges
    pq_edges = []
    for edge in graph.edges:
        heappush(pq_edges, edge)

    # For each minimal edge, add it to the minimum spanning tree, as long as it doesn't create a cycle
    for edge in [heappop(pq_edges) for i in range(len(pq_edges))]:
        if djs_vertices.find(edge.source) != djs_vertices.find(edge.sink):
            A.add_edge(edge.weight, edge.source.index, edge.sink.index)
            djs_vertices.union(edge.source, edge.sink)

    return A


#
# DIJKSTRA'S ALGORITHM
#           Calculates the minimum path between an arbitrary vertex and every other vertex in the graph
#           If given a target vertex, calculates the minimum path and its cost
#
# Input:    Graph, Vertex (source vertex), (optional Vertex, target vertex)
# Output:   List(Float) or [Float, List(Vertex)]
# Notes:    Minimum path calculation for graphs with possible negative-weight cycles
#           Implemented with a priority queue
#           The most standard minimum path algorithm
#
def Dijkstra(graph, source, target=None):
    class heap_vertex:
        def __init__(self, vertex, distance):
            self.vertex = vertex
            self.distance = distance
            dist[vertex.index] = distance

        def __lt__(self, other):
            return self.distance < other.distance

    Q = []
    dist = [graph.inf]*graph.num_v
    prev = [None]*graph.num_v

    # Set the distance to source to 0
    heappush(Q, heap_vertex(graph.vertices[source], 0))

    while Q:
        # Determine the vertex with the shortest distance, runs on O(n)
        u = heappop(Q).vertex

        # Begin indexing through all neighbour nodes
        for v_index, adjacency in enumerate(graph.adj_list[u.index]):
            if adjacency != 0:
                # for multigraphs, use the shortest edge from u to v
                alt = dist[u.index] + min([edge.weight for edge in graph.find_edges(u.index, v_index)])

                # If shorter path has been found
                if alt < dist[v_index]:
                    heappush(Q, heap_vertex(graph.vertices[v_index], alt))
                    prev[v_index] = u

    # Reconstruct path
    def PathReconstruction(target):
        S = []

        u = target
        while u and u != graph.vertices[source]:
            # Since the list stores the PREVIOUS vertex, to reconstruct the path in order from source -> target
            # Add the latest "previous" vertex to the front until the source is reached
            # Path reconstruction of Dijkstra goes from target to source, Floyd-Warshall goes from source to target
            S.insert(0, u)
            u = prev[u.index]

        # Add the last vertex
        S.insert(0, u)

        return S

    # Return the path length and the path
    if target:
        return dist[target], PathReconstruction(graph.vertices[target])

    # Return the distance to each vertex
    return dist


#
# FLOYD-WARSHALL'S ALGORITHM
#           Calculates the minimum path between an arbitrary vertex and every other vertex in the graph
#           If given a target vertex, calculates the minimum path and its cost
#
# Input:    Graph, (optional Vertex, source vertex), (optional Vertex, target vertex)
# Output:   List(Float) or [Float, List(Vertex)]
# Notes:    Identical function to Dijkstra's algorithm, but detects negative-weight cycles
#           Negative-weight cycles causes Dijkstra's algorithm to loop infinitely, but Floyd-Warshall's algorithm
#           identifies and throws an error
#
def Floyd(graph, source=None, target=None):
    dist = [[graph.inf for i in range(graph.num_v)] for j in range(graph.num_v)]
    next = [[None for i in range(graph.num_v)] for j in range(graph.num_v)]

    for edge in graph.edges:
        dist[edge.source.index][edge.sink.index] = edge.weight
        next[edge.source.index][edge.sink.index] = edge.sink
        if not graph.directed:
            dist[edge.sink.index][edge.source.index] = edge.weight
            next[edge.sink.index][edge.source.index] = edge.sink

    for vertex in graph.vertices:
        dist[vertex.index][vertex.index] = 0

    for k in range(graph.num_v):
        for i in range(graph.num_v):
            for j in range(graph.num_v):
                if dist[i][j] > dist[i][k] + dist[k][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    next[i][j] = next[i][k]

    # Check for negative-weight cycles
    # If there are negative-weight cycles, cycles (paths going from vertex i to vertex i) will be negative
    for i in range(graph.num_v):
        if dist[i][i] < 0:
            raise NegativeWeightCycle()

    # Reconstruct path
    def PathReconstruction(u, v):
        if not next:
            return []

        path = [u]
        while u != v:
            u = next[u.index][v.index]
            path.append(u)

        return path

    if (source is not None) and (target is not None):
        return dist[source][target], PathReconstruction(graph.vertices[source], graph.vertices[target])

    # Else just return the distances
    return dist


#
# BELLMAN-FORD'S ALGORITHM
#           Calculates the minimum path between an arbitrary vertex and every other vertex in the graph and returns the
#           cost to each node and its immediate predecessor (in the minimum path)
#
# Input:    Graph, Vertex (source vertex)
# Output:   [List(Vertex), List(Vertex)]
# Notes:    For graphs with edges of undetermined sign
#           Slower than Dijkstra's algorithm, but more versatile
#
def Bellman_Ford(graph, source):
    # Initialize graph
    distance = [graph.inf for i in range(graph.num_v)]
    predecessor = [None for i in range(graph.num_v)]

    # Set the distance to the source is 0
    distance[source] = 0

    # Relax edges repeatedly until the distances to all vertices have converged
    for i in range(graph.num_v - 1):
        for edge in graph.edges:
            if distance[edge.source.index] + edge.weight < distance[edge.sink.index]:
                distance[edge.sink.index] = distance[edge.source.index] + edge.weight
                predecessor[edge.sink.index] = edge.source

            if not graph.directed and distance[edge.sink.index] + edge.weight < distance[edge.source.index]:
                distance[edge.source.index] = distance[edge.sink.index] + edge.weight
                predecessor[edge.source.index] = edge.sink

    # Check for negative-weight cycles
    # If minimum weights can still be found, there are negative-weight cycles
    for edge in graph.edges:
        if (distance[edge.source.index] + edge.weight < distance[edge.sink.index]) or \
                (not graph.directed and distance[edge.sink.index] + edge.weight < distance[edge.source.index]):
            raise NegativeWeightCycle()

    return distance, predecessor


#
# PRIM'S ALGORITHM
#           Calculates the minimum spanning tree (MST)
#
# Input:    Graph
# Output:   Graph
# Notes:    Identical function to Kruskal's algorithm
#
def Prim(graph):
    class heap_vertex:
        def __init__(self, vertex, distance):
            self.vertex = vertex
            self.distance = distance

        def __lt__(self, other):
            return self.distance < other.distance

    F = Graph()
    C = []  # Cost of a vertex
    E = []  # Minimum edge
    Q = []  # Unvisited vertices

    for v in graph.vertices:
        # Set up the vertex and edge list (and its cost)
        heappush(Q, heap_vertex(v, graph.inf))
        C.append(graph.inf)
        E.append(None)

    while Q:
        v = heappop(Q)
        F.add_vertex(v.vertex.value)

        if E[v.vertex.index] is not None:
            edge = E[v.vertex.index]
            F.add_edge(edge.weight, F.find_vertex(edge.source.value).index, F.find_vertex(edge.sink.value).index)

        # Loop over neighbours of v
        for w, adjacency in enumerate(graph.adj_list[v.vertex.index]):
            # For every neighbour w of v, calculate the new minimum cost of w and the minimum-weight edge of w
            # While w still belongs to Q (and not in F)
            min_edge = None
            min_weight = graph.inf

            if adjacency == 1:
                for edge in graph.find_edges(v.vertex.index, w):
                    if edge.weight < min_weight:
                        min_edge = edge
                        min_weight = edge.weight

                if (min_weight < C[w]) and (w in [hv.vertex.index for hv in Q]):
                    E[w] = min_edge
                    C[w] = min_weight
                    for hv in Q:
                        if hv.vertex.index == w:
                            hv.distance = min_weight
                            break

        # Update the priority queue
        heapify(Q)

    return F


#
# BORUVKA'S ALGORITHM
#           Calculates the minimum spanning tree (MST)
#
# Input:    Graph
# Output:   Graph
# Notes:    Identical function to Kruskal's algorithm
#
def Boruvka(graph):
    pass
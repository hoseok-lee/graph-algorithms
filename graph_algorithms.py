from structures import *
from heapq import *


class NegativeWeightCycle(RuntimeError):
    pass


# using disjoint set
def Kruskal(graph):
    A = Graph()

    # initialize disjoint set for vertices
    djs_vertices = DisjointSet()
    for vertex in graph.vertices:
        djs_vertices.make_set(vertex)
        A.add_vertex(vertex.value)

    # initialize priority queue for edges
    pq_edges = []
    for edge in graph.edges:
        heappush(pq_edges, edge)

    # for each minimal edge, add it to the MST as long as it doesn't create a cycle
    for edge in [heappop(pq_edges) for i in range(len(pq_edges))]:
        if djs_vertices.find(edge.source) != djs_vertices.find(edge.sink):
            A.add_edge(edge.weight, edge.source.index, edge.sink.index)
            djs_vertices.union(edge.source, edge.sink)

    return A


# using priority queue
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

    # set the distance to source to 0
    heappush(Q, heap_vertex(graph.vertices[source], 0))

    while Q:
        # determine the vertex with the shortest distance, runs on O(n)
        u = heappop(Q).vertex

        # begin indexing through all neighbour nodes
        for v_index, adjacency in enumerate(graph.adj_list[u.index]):
            if adjacency != 0:
                # for multigraphs, use the shortest edge from u to v
                alt = dist[u.index] + min([edge.weight for edge in graph.find_edges(u.index, v_index)])

                # if shorter path has been found
                if alt < dist[v_index]:
                    heappush(Q, heap_vertex(graph.vertices[v_index], alt))
                    prev[v_index] = u

    # reconstruct path
    def PathReconstruction(target):
        S = []

        u = target
        while u and u != graph.vertices[source]:
            # since the list stores the PREVIOUS vertex, to reconstruct the path in order from source -> target
            # add the latest "previous" vertex to the front until the source is reached
            # path reconstruction of Dijkstra goes from target to source
            # Floyd-Warshall goes from source to target
            S.insert(0, u)
            u = prev[u.index]

        # add the last vertex
        S.insert(0, u)

        return S

    # return the path length and the path
    if target:
        return dist[target], PathReconstruction(graph.vertices[target])

    # return the distance to each vertex
    return dist


# for graphs with negative-weight edges
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

    # check for negative-weight cycles
    # if there are negative-weight cycles, cycles (paths going from vertex i to vertex i) will be negative
    # (by ultimate definition)
    for i in range(graph.num_v):
        if dist[i][i] < 0:
            raise NegativeWeightCycle()

    # reconstruct path
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

    # else just return the distances
    return dist


# for graphs with edges of undetermined sign
# slower than Dijkstra's, but more versatile
def Bellman_Ford(graph, source):
    # initialize graph
    distance = [graph.inf for i in range(graph.num_v)]
    predecessor = [None for i in range(graph.num_v)]

    # set the distance to the source is 0
    distance[source] = 0

    # relax edges repeatedly until the distances to all vertices have converged
    for i in range(graph.num_v - 1):
        for edge in graph.edges:
            if distance[edge.source.index] + edge.weight < distance[edge.sink.index]:
                distance[edge.sink.index] = distance[edge.source.index] + edge.weight
                predecessor[edge.sink.index] = edge.source

            if not graph.directed and distance[edge.sink.index] + edge.weight < distance[edge.source.index]:
                distance[edge.source.index] = distance[edge.sink.index] + edge.weight
                predecessor[edge.source.index] = edge.sink

    # check for negative-weight cycles
    # if minimum weights can still be found, there are negative-weight cycles
    for edge in graph.edges:
        if (distance[edge.source.index] + edge.weight < distance[edge.sink.index]) or \
                (not graph.directed and distance[edge.sink.index] + edge.weight < distance[edge.source.index]):
            raise NegativeWeightCycle()

    return distance, predecessor

'''
def Prim(graph):
    class heap_vertex:
        def __init__(self, vertex, distance):
            self.vertex = vertex
            self.distance = distance
            C.append(distance)
            E.append(None)

        def __lt__(self, other):
            return self.distance < other.distance

    F = Graph()
    C = []  # cost of a vertex
    E = []  # minimum edge
    Q = []

    for v in graph.vertices:
        # set up the vertex list
        heappush(Q, heap_vertex(v, graph.inf))

    print(", ".join([str(edge) for edge in E]))

    while Q:
        v = heappop(Q)
        F.add_vertex(v.vertex.value)

        # if v is unconnected
        min_cost_edge = E[v.vertex.index]


        # loop over neighbours of v
        for w, adjacency in enumerate(graph.adj_list[v.vertex.index]):
            # for every neighbour w of v, calculate the new minimum cost of w
            # and the minimum-weight edge of w
            if adjacency == 1:
                min_edge = None
                min_weight = graph.inf
                for edge in graph.find_edges(v.vertex.index, w):
                    if edge.weight < min_weight:
                        min_edge = edge
                        min_weight = edge.weight

                if (C[w] and min_weight) and (w in [hv.vertex.index for hv in Q]):
                    E[w] = min_edge
                    C[w] = min_weight'''

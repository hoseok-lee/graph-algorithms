class DisjointSet:
    def __init__(self):
        self.set = []

    def make_set(self, item):
        if self.find(item) == -1:
            self.set.append([item])

    def find(self, item):
        index = -1

        for i, subset in enumerate(self.set):
            if item in subset:
                index = i
                break

        return index

    def union(self, x, y):
        x_root = self.find(x)
        y_root = self.find(y)

        if x_root != y_root:
            self.set[x_root].extend(self.set[y_root])
            del self.set[y_root]


class Graph:
    class Vertex:
        def __init__(self, value, index):
            self.value = value
            self.index = index

        def __eq__(self, other):
            return self.index == other.index

        def __lt__(self, other):
            return self.index < other.index

        def __hash__(self):
            return hash(str(self))

        def __str__(self):
            return str(self.value)

    class Edge:
        def __init__(self, weight, source, sink):
            self.weight = weight
            self.source = source
            self.sink = sink

        # to deal with the heap queue sort
        def __lt__(self, other):
            return self.weight < other.weight

        def __str__(self):
            return "(" + str(self.weight) + ", " + str(self.source.value) + ", " + str(self.sink.value) + ")"

    def __init__(self, directed=False):
        # all vertices MUST be unique
        self.vertices = []
        self.num_v = 0
        # in a multi-graph, there may be multiple edges of same weight
        # that lead from the same source to the same sink
        # cosmetically identical, yet two distinct edges
        self.edges = []
        self.num_e = 0

        # continually generated adjacency list
        self.adj_list = []
        self.edg_list = []

        # determines if it is a digraph
        self.directed = directed

        # for graph algorithms, determines the maximum weight any path within the graph can have
        # this is more ideal than setting the maximum to some integer large enough to replace infinity practically
        self.inf = 0

    def add_edge(self, weight, source, sink):
        # instead of assigning an edge the index or value of a vertex
        # assign the actual vertex

        # add edge
        self.edges.append(self.Edge(weight, self.vertices[source], self.vertices[sink]))
        self.num_e += 1

        # update matrices
        # adjacency and edges
        self.adj_list[source][sink] += 1
        self.edg_list[source][sink].append(self.edges[self.num_e - 1])
        if not self.directed and sink != source:
            self.adj_list[sink][source] += 1
            self.edg_list[sink][source].append(self.edges[self.num_e - 1])

        self.inf += abs(weight)

    def add_vertex(self, value, auto_index=True):
        if auto_index:
            self.vertices.append(self.Vertex(value, len(self.vertices)))
        else:
            self.vertices.append(value)

        # append the matrices
        self.adj_list.append([0]*self.num_v)
        self.edg_list.append([]*self.num_v)
        for i in range(self.num_v):
            self.edg_list[self.num_v].append([])

        self.num_v += 1

        # update the matrices
        for i in range(self.num_v):
            self.adj_list[i].append(0)
            # for the exception that it is a multi-graph
            self.edg_list[i].append([])

    def find_vertex(self, value):
        for vertex in self.vertices:
            if vertex.value == value:
                return vertex

        return None

    def find_edges(self, source, sink):
        return self.edg_list[source][sink]

    def in_graph(self, value_weight, source=None, sink=None):
        if source and sink:
            return value_weight in [edge.weight for edge in self.edg_list[source][sink]]

        return self.find_vertex(value_weight) is not None

    def __str__(self):
        out = "graph"
        out += ("\n\tvertices = " + ", ".join([str(vertex) for vertex in self.vertices]))
        out += ("\n\tedges = " + ", ".join([str(edge) for edge in self.edges]))
        out += "\n\tadjacency list = "
        for vertex in self.adj_list:
            out += ("\n\t\t" + "  ".join(str(adjacency) for adjacency in vertex))

        return out

#!/usr/bin/python

from graph_algorithms import *
from structures import *


g = Graph()

for i in ['A', 'B', 'C', 'D']:
    g.add_vertex(i)

g.add_edge(2, 0, 2)
g.add_edge(4, 1, 0)
g.add_edge(1, 3, 1)
g.add_edge(3, 1, 2)
g.add_edge(2, 2, 3)

print(g)

try:
    distances = Floyd(g)

    print(distances)
except NegativeWeightCycle:
    print("run-time error: negative-weight cycle found")

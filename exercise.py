#!/usr/bin/python

from graph_algorithms import *
from structures import *


g = Graph()

for i in ['A', 'B', 'C', 'D']:
    g.add_vertex(i)

g.add_edge(2, 0, 1)
g.add_edge(1, 0, 3)
g.add_edge(2, 1, 3)
g.add_edge(3, 2, 3)

try:
    Bellman_Ford(g, 0)
except NegativeWeightCycle:
    print("run-time error: negative-weight cycle found")

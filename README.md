# GameMapNav

In this project, I find optimal paths in video game maps using pathfinding algorithms dijkstra's alogorithm and A*.

## Problem Setting
* The project works with grid based video game maps. 
* Each action in the cardinal directions(north, south, east, west) cost 1.0 and any action in the diagonal directions cost 1.5. 
* A search problem consists of a video game map, a start and a goal location.
* The goal is to find an optimal path from the start till the goal location.

## Implementation
* The program finds optimal path for a number of start and goal locations.
* It uses both Dijkstra's algorithm and A* for each of these problems.
* It plots a graph to compare the number of nodes expanded and running time for the two algorithms.

## Tech Stack
* Python
* Libraries used: time, getopt, sys, heapq, copy, numpy, random, matplotlib

## Algorithms Used
* Dijkstra's algorithm
* A* (Heuristic used: Octile Distance)

## How to run 
* To change the map, change line 27 in main.py to the file path for the map.
* To get the plots run with "--plots"
* To understand usage run with "-help"
* To change start and goals edit "test-instances/testinstances"

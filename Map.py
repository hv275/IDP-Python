# import the two custom classes from the other file
# graph is a basic implementation of a graph
# node represents a node during the A* algorithm
from Graph import Graph
import math
from node import Node


# note, I am currently marking regions by their bottom left vertice
# vertice names refer to the that coordinate
# hence can easily pinpoint the bot
# maybe send it to that corner before beginning the path?
class gridmap():

    def __init__(self, x, y, square_side):
        """constructor
            takes in length and width of the arena as well as side lengths
            square size is also used to regulate map block sizes"""
        self.map = Graph()
        # creation of the graph
        # tuples are used as indices
        # loops through the length sides and adds connected noted using the graph class methods
        for i in range(math.floor(x / square_side)):
            for j in range(math.floor(y / square_side)):
                self.map.add_vertex((i * square_side, j * square_side))
                if i != 0:
                    self.map.add_edge(((i * square_side, j * square_side), ((i - 1) * square_side, j * square_side)))
                if j != 0:
                    self.map.add_edge(((i * square_side, j * square_side), (i * square_side, (j - 1) * square_side)))

    def alastair(self, start, target, heuristic):
        """method that implements A* pathfinding algorithm
            takes in the start and end notes as an argument and returns the path between then
            returned path takes a form of list of nodes to pass through
            heuristic is either 'man' or 'acf' and determines the heuristic to be used in
            (name is an easter egg)"""

        # brief comments are supplied
        # it is basically a classic A* pathfinder

        # checks if the node needs to be added to the open list
        def add_to_open(open, neighbor):
            for node in open:
                if (neighbor == node and neighbor.f >= node.f):
                    return False
            return True

        # create the start and end node objects
        startNode = Node(start, None)
        targetNode = Node(target, None)

        # define the two implemented heuristics
        h = lambda x: math.sqrt((target[1] - x[1]) ** 2 + (target[0] - x[0]) ** 2)
        g = lambda x: math.sqrt((start[1] - x[1]) ** 2 + (start[0] - x[0]) ** 2)

        h_man = lambda x: target[1] - x[1] + target[0] - x[0]
        g_man = lambda x: start[1] - x[1] + start[0] - x[0]

        # add the start node to open list and begin a search
        open_list = [startNode]
        closed_list = []

        # if there are open nodes keep looping
        while len(open_list) > 0:
            open_list.sort()
            # pop the first node out of open list
            current = open_list.pop(0)
            closed_list.append(current)

            # if the currentl node is the target node, generate a path
            if current == targetNode:
                path = []
                while current != startNode:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]

            # find neighbours on the map and return them in a sorted manner
            (x, y) = current.position
            neighbors = self.map.dict[(x, y)]
            neighbors.sort()

            #go through the immediate neighbours
            for next in neighbors:
                # Create a neighbor node
                neighbor = Node(next, current)
                # Check if the neighbor is in the closed list
                if neighbor in closed_list:
                    continue
                # Generate heuristics
                if heuristic == "man":
                    neighbor.g = g_man(neighbor.position)
                    neighbor.h = h_man(neighbor.position)
                elif heuristic == "acf":
                    neighbor.g = g(neighbor.position)
                    neighbor.h = h(neighbor.position)
                neighbor.f = neighbor.g + neighbor.h
                # Check if neighbor is in open list and if it has a lower f value
                if add_to_open(open_list, neighbor):
                    # Everything is green, add neighbor to open list
                    open_list.append(neighbor)
                # Return None, no path is found
        print("no path found")
        return None

    def directions(self, beg, end, heurisitic):
        """method to convert the path produced by the pathfinder to directions that can be used by the robot"""
        # get the path from A*
        nodes = self.alastair(beg, end, heurisitic)
        # subtract the last node from the current
        directions = []
        for node in enumerate(nodes):
            if node[0] == 0:
                directions.append(tuple(map(lambda i, j: i - j, node[1], beg)))
            else:
                directions.append(tuple(map(lambda i, j: i - j, node[1], nodes[node[0] - 1])))
        return directions

    def disp(self):
        """just a print map in a method"""
        print(self.map)

#used for testing
if __name__ == "__main__":
    gridmapmap = gridmap(22,22,1)
    print(gridmapmap.alastair((11, 7), (3, 3)))
    print(gridmapmap.directions((11, 7), (3, 3)))

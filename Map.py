# a class to implement a map using the graph class
from Graph import Graph
import math
from node import Node

# note, I am currently marking regions by their top left vertice
# vertice names refer to the that coordinate
# hence can easily pinpoint the bot
# maybe send it to that corner before beginning the path?
class gridmap():
    # todo consider if centre squares are better
    def __init__(self, x, y, square_side):
        """self initalises to have x by grid with"""
        self.map = Graph()
        #creation of the graph
        #tuples are used as indices, it is dodgy, don't question it
        for i in range(math.floor(x / square_side)):
            for j in range(math.floor(y / square_side)):
                self.map.add_vertex((i*square_side,j*square_side))
                if i != 0:
                    self.map.add_edge(((i*square_side,j*square_side), ((i-1)*square_side,j*square_side)))
                if j != 0:
                    self.map.add_edge(((i*square_side,j*square_side), (i*square_side,(j-1)*square_side)))

    def alastair(self, start, target):
        #name explanation - algorithm is called A*, I spelt it as astar and that almost looks like alastair
        #cost of exploring all nodes is 1
        #this function is some shady code, proceed at your own peril

        #little fucntion to add things to open
        def add_to_open(open, neighbor):
            for node in open:
                if (neighbor == node and neighbor.f >= node.f):
                    return False
            return True
        startNode = Node(start,None)
        targetNode = Node(target,None)

        h = lambda x: math.sqrt((target[1] - x[1]) ** 2 + (target[0] - x[0]) ** 2)
        g = lambda x: math.sqrt((start[1] - x[1]) ** 2 + (start[0] - x[0]) ** 2)

        open_list = [startNode]
        closed_list =[]
        while len(open_list) > 0:
            open_list.sort()
            current = open_list.pop(0)
            closed_list.append(current)

            if current == targetNode:
                path = []
                while current != startNode:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]

            (x,y) = current.position
            neighbors = self.map.dict[(x,y)]
            neighbors.sort()

            for next in neighbors:
                # Create a neighbor node
                neighbor = Node(next, current)
                # Check if the neighbor is in the closed list
                if neighbor in closed_list:
                    continue
                # Generate heuristics
                neighbor.g = g(neighbor.position)
                neighbor.h = h(neighbor.position)
                neighbor.f = neighbor.g + neighbor.h
                # Check if neighbor is in open list and if it has a lower f value
                if add_to_open(open_list, neighbor):
                    # Everything is green, add neighbor to open list
                    open_list.append(neighbor)
                print(next)
                # Return None, no path is found
        return None

    def directions(self,beg,end):
        nodes = self.alastair(beg,end)
        directions = []
        for node in enumerate(nodes):
            if node[0] == 0:
                pass
            else:
                directions.append(tuple(map(lambda i, j: i - j, node[1], nodes[node[0]-1])))
        return directions






    def disp(self):
        print(self.map)


if __name__ == "__main__":
    gridmapmap = gridmap(10, 10, 1)
    print(gridmapmap.directions((0,0),(4,4)))

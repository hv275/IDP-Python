#a class to implement a map using the graph class
from Graph import Graph
import math

#note, I am currently marking regions by their top left vertice
class gridmap():

    def __init__(self,x,y,square_side):
        self.map = Graph()
        for i in range(math.floor(y/square_side)):
            for j in range(math.floor(x/square_side)):
                self.map.add_vertex(f"{j},{i}")
                if i != 0:
                    self.map.add_edge((f"{i},{j}",f"{i-1},{j}"))
                if j != 0:
                    self.map.add_edge(((f"{i},{j}",f"{i},{j-1}")))

    def astar_find(self,cur,target):





    def disp(self):
        print(self.map)






if __name__ == "__main__":
    map = gridmap(5,5,1)
    map.disp()




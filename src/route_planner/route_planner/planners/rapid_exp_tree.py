import math
import random

class RRTGraph:
    def __init__(self, start, end, map):
        (x,y) = start
        self.start = start
        self.end = end
        self.map = map

        self.endVicinityRadius = 20 # pixels

        # the path
        self.reachEndFlag = False
        self.path = []

        # initialize the graph itself
        self.nodex=[]
        self.nodey=[]
        self.nodeparent=[]

        self.nodex.append(x)
        self.nodey.append(y)
        self.nodeparent.append(0)

    def add_node(self, x, y, id):
        self.nodex.insert(id,x)
        self.nodey.insert(id,y)

    def remove_node(self, id):
        self.nodex.pop(id)
        self.nodey.pop(id)

    def add_edge(self, parent, child):
        self.parent.insert(child,parent)

    def remove_edge(self, id):
        self.parent.pop(id)

    def num_of_nodes(self):
        return len(self.nodex)

    def distance(self, id1, id2):
        (x1, y1) = (self.nodex[id1], self.nodey[id1])
        (x2, y2) = (self.nodex[id2], self.nodey[id2])

        dx = float(x1) - float(x2)
        dy = float(y1) - float(y2)

        return (dx**2 + dy**2)**(0.5)

    def random_sample(self):
        x = int(random.uniform(map_minx, map_maxx))
        y = int(random.uniform(map_miny, map_maxy))
        return x, y

    def connect(self, id1, id2):
        (x1,y1) = (self.x[id1], self.y[id1])
        (x2,y2) = (self.x[id2], self.y[id2])
        if not self.isConnectionValid():
            self.remove_node(id2)
            raise Exception("can't connect")
        else:
            self.add_edge(id1, id2)
            return True

    def step(self):
        pass

    def nearest(self):
        pass

    def isPointValid(self, x, y):
        # TODO: check the coordinate of the point againts the map
        # if white, return True, else False
        # TODO: make a util.py with this and use this method across the workspace to avoid copypase
        return False

    def isNodeInFreeSpace(self): #check if the node is in the whitespace
        pss

    def isConnectionValid(self): # check if the connection between two points crosses any obsticles
        discretizationPower = 255 # how much points th check the line
        for i in range (0, discretizationPower)
            u = i /() discretizationPower - 1)
            x = x1*u + x2*(1 - u)
            x = x1*u + x2*(1 - u)
            if self.isPointValid(x ,y):
                return True
            else:
                return False

    def path_to_end(self):
        pass

    def get_path_coordinates(self):
        pass

    def bias(self):
        pass

    def expand(self):
        pass

    def const(self):
        pass

if __name__== "__main__":
    graph = RRTGraph(start, end, map)
    (x,y) = graph.random_sample()
    id=graph.num_of_nodes()
    graph.add_node(id,x,y)
    graph.isNodeInFreeSpace()

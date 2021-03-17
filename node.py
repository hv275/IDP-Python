#node class to make a* easie
class Node:
    # Initialize the class with using the node position and the position of it's parent node
    def __init__(self, position: (), parent: ()):
        self.position = position
        self.parent = parent
        # define node cost
        self.g = 0  # Distance to start node
        self.h = 0  # Distance to goal node
        self.f = 0  # Total cost

    # overwrite the comparison operator for easy node comparison
    def __eq__(self, other):
        return self.position == other.position

    # overwhire the less than operator so sorting is easier
    def __lt__(self, other):
        return self.f < other.f

    # overwrite the pring operator for easy printing
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))

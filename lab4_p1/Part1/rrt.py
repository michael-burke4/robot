from time import sleep
from cmap import *
from gui import *
from utils import *

MAX_NODES = 20000

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, a nodes is an object that has its own
# coordinate and parent if necessary. You could access its coordinate by node.x
# or node[0] for the x coordinate, and node.y or node[1] for the y coordinate
################################################################################

def step_from_to(node0, node1, limit=75):
    ############################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    if get_dist(node0, node1) < limit:
        return node1
    
    x = node1.x - node0.x
    y = node1.y - node0.y
    theta = np.arctan2(y, x)
    new_y = np.sin(theta) * limit
    new_x = np.cos(theta) * limit
    return Node([node0.x + new_x, node0.y + new_y], None)
    ############################################################################


def node_generator(cmap):
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    pass
    ############################################################################
    valid = False
    while not valid:
        randw = np.random.rand() * cmap.width
        randh = np.random.rand() * cmap.height
        rand_node = Node([randw, randh], None)
        if cmap.is_inbound(rand_node) and not cmap.is_inside_obstacles(rand_node):
            valid = True
    return rand_node

def get_closest_node(from_node, nodes):
    closest_dist = np.inf
    closest = None
    for node in nodes:
        dist = get_dist(from_node, node)
        if dist < closest_dist:
            closest = node
            closest_dist = dist
    return closest
        

def RRT(cmap, start):
    cmap.add_node(start)

    map_width, map_height = cmap.get_size()

    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()
        nearest_node = get_closest_node(rand_node, cmap.get_nodes())
        rand_node = step_from_to(nearest_node, rand_node, 75)
        ########################################################################
        # sleep(.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
    else:
        print("Please try again :-(")

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RRTThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            sleep(1)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global grid, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/map2.json", node_generator)
    visualizer = Visualizer(cmap)
    robot = RRTThread()
    robot.start()
    visualizer.start()
    stopevent.set()

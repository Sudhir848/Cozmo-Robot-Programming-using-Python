from time import sleep
from cmap import *
from gui import *
from utils import *
import numpy as np

MAX_NODES = 20000

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, a nodes is an object that has its own
# coordinate and parent if necessary. You could access its coordinate by node.x
# or node[0] for the x coordinate, and node.y or node[1] for the y coordinate
################################################################################

def step_from_to(node0, node1, limit=75):
    if get_dist(node0, node1) < limit:
        return node1
    else:
        theta = np.arctan2(node1.y - node0.y, node1.x - node0.x)
        new_x = node0.x + limit * np.cos(theta)
        new_y = node0.y + limit * np.sin(theta)
        return Node((new_x, new_y), parent=node0)

def node_generator(cmap):
    while True:
        rand_x = np.random.uniform(0, cmap.width)
        rand_y = np.random.uniform(0, cmap.height)
        rand_node = Node((rand_x, rand_y))
        if cmap.is_inbound(rand_node) and not cmap.is_inside_obstacles(rand_node):
            return rand_node

def RRT(cmap, start):
    cmap.add_node(start)

    while not cmap.is_solved() and cmap.get_num_nodes() < MAX_NODES:
        rand_node = cmap.get_random_valid_node()
        nearest_node = None
        min_dist = float('inf')
        for node in cmap.get_nodes():
            dist = get_dist(node, rand_node)
            if dist < min_dist:
                nearest_node = node
                min_dist = dist

        if nearest_node:
            new_node = step_from_to(nearest_node, rand_node, limit=75)
            if not cmap.is_collision_with_obstacles((nearest_node, new_node)):
                cmap.add_path(nearest_node, new_node)

        sleep(0.01)

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
            sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global grid, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/map1.json", node_generator)
    visualizer = Visualizer(cmap)
    robot = RRTThread()
    robot.start()
    visualizer.start()
    stopevent.set()

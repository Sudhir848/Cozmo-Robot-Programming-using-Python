from time import sleep
from cmap import *
from gui import *
from utils import *
import cozmo
import numpy as np
import asyncio
from time import sleep
from cozmo.objects import LightCube

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

get_path = [] # List to store the final path from start to goal
observed_cubes = [] # Keeps track of observed cubes to avoid re-processing
goal = [] # Stores the goal node

def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size() # get a valid random node
    while not cmap.is_solved() and cmap.get_num_nodes() < MAX_NODES:
        rand_node = cmap.get_random_valid_node()
        nearest_node = None
        distance = np.sqrt(map_width ** 2 + map_height ** 2) # Initial large distance
        for node in cmap.get_nodes():
            if get_dist(rand_node, node) < distance:  # Finds the nearest node to rand_node
                distance = get_dist(rand_node, node)
                nearest_node = node
        rand_node = step_from_to(nearest_node, rand_node)  # Moves towards rand_node within a limit
        sleep(0.01)
        cmap.add_path(nearest_node, rand_node) # Adds the new step to the map/path
        if cmap.is_solved():  # Checks if the goal has been reached
            current_node = cmap.get_nodes()[-1]
            get_path.append(current_node)
            while current_node.parent is not None:
                current_node = current_node.parent
                get_path.append(current_node)
            get_path.reverse()
            if get_path:
                break

async def CozmoPlanning(robot: cozmo.robot.Robot):
    global cmap, stopevent
    update_cmap = False
    observed_cube_total = 0
    next_node_count = 0

    # Start in the center of the map
    center_node = Node((cmap.width / 2, cmap.height / 2))
    cmap.set_start(center_node)

    # Updating cmap
    def update_map(cmap, center, objects):
        print("Updating Map")
        cube_padding = 50 # Padding to account for cube size
        cozmo_padding = 85 # Additional padding for Cozmo's clearance
        update_cmap = True  
        if len(objects) > 0:
            obj = objects[len(objects) - 1] # Processes the most recently observed object
            object_pos = Node((center.x - obj.pose.position.x, center.y - obj.pose.position.y))
            object_angle = obj.pose.rotation.angle_z.degrees
            if robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == obj.object_id:
                 # If the object is the goal cube
                goal_node = Node((object_pos.x, object_pos.y))
                cmap.add_goal(goal_node)
                goal.append(goal_node)
                update_cmap = False
            else:
                # If the object is an obstacle
                # Calculates obstacle corners based on cube and Cozmo padding
                obstacle_nodes = []
                obstacle_nodes.append(Node((object_pos.x - ((cube_padding + cozmo_padding) / 2) *
                                            np.cos(np.deg2rad(object_angle - 45)), 
                                            object_pos.y - ((cube_padding + cozmo_padding) / 2) * 
                                            np.sin(np.deg2rad(object_angle - 45)))))
                obstacle_nodes.append(Node((object_pos.x - ((cube_padding + cozmo_padding) / 2) * 
                                            np.cos(np.deg2rad(object_angle - 45 + 90)), 
                                            object_pos.y - ((cube_padding + cozmo_padding) / 2) * 
                                            np.sin(np.deg2rad(object_angle - 45 + 90)))))
                obstacle_nodes.append(Node((object_pos.x - ((cube_padding + cozmo_padding) / 2) *
                                            np.cos(np.deg2rad(object_angle - 45 + 180)), 
                                            object_pos.y - ((cube_padding + cozmo_padding) / 2) * 
                                            np.sin(np.deg2rad(object_angle - 45 + 180)))))
                obstacle_nodes.append(Node((object_pos.x - ((cube_padding + cozmo_padding) / 2) * 
                                            np.cos(np.deg2rad(object_angle - 45 + 270)), 
                                            object_pos.y - ((cube_padding + cozmo_padding) / 2) * 
                                            np.sin(np.deg2rad(object_angle - 45 + 270)))))
                cmap.add_obstacle(obstacle_nodes)
                update_cmap = False
        return update_cmap

    # Handling observed cubes
    def handle_cube(evt, **kwargs):
        cube = evt.obj
        if isinstance(cube, LightCube):
            if cube not in observed_cubes:
                observed_cubes.append(cube)
                print(f"Cube {cube.cube_id} observed")

    robot.add_event_handler(cozmo.objects.EvtObjectAppeared, handle_cube)
    while not stopevent.is_set():
        await asyncio.sleep(0.1)
        if len(observed_cubes) > observed_cube_total:
            update_cmap = True
            observed_cube_total = len(observed_cubes)
        if update_cmap:
            cmap.reset()
            next_node_count = 0
            update_cmap = update_map(cmap, center_node, observed_cubes)

        # RRT Path
        if not cmap.is_solved():
            if len(cmap.get_goals()) > 0:
                robot_pos = Node((center_node.x - robot.pose.position.x, center_node.y - robot.pose.position.y))
                cmap.set_start(robot_pos)
                RRT(cmap, cmap.get_start())
            if len(cmap.get_goals()) <= 0:
                await robot.turn_in_place(angle=cozmo.util.degrees(30)).wait_for_completed()
        if cmap.is_solved():
            if next_node_count == 0:
                next_node_count = 1

        if update_cmap is False and next_node_count >= 1:
            robot_pos = Node((center_node.x - robot.pose.position.x, center_node.y - robot.pose.position.y))
            if get_dist(goal[0], robot_pos) > 72:
                if next_node_count <= len(get_path):
                    current_node = get_path[next_node_count - 1]
                    next_node = get_path[next_node_count]
                    goal_angle = np.degrees(np.arctan2(next_node.y - current_node.y, next_node.x - current_node.x))
                    if goal_angle < 0:
                        goal_angle += 180
                    elif goal_angle > 0:
                        goal_angle -= 180
                    angle_difference = goal_angle - robot.pose_angle.degrees
                    angle_robot = cozmo.util.degrees(angle_difference)
                    await robot.turn_in_place(angle=angle_robot,speed=cozmo.util.radians(2)).wait_for_completed()
                    move_distance = get_dist(next_node, current_node)
                    await robot.drive_straight(distance=cozmo.util.distance_mm(move_distance), speed=cozmo.util.speed_mmps(30)).wait_for_completed()
                    next_node_count += 1
                    await asyncio.sleep(1)
            else:
                print("Cozmo reached the goal")
                stopevent.set()

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.robot.Robot.drive_off_charger_on_connect = False
        cozmo.run_program(CozmoPlanning, use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
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
    global cmap, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/emptygrid.json", node_generator)
    robot_thread = RobotThread()
    robot_thread.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
    
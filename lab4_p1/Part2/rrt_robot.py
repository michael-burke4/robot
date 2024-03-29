import cozmo

from cozmo.util import radians, distance_mm, speed_mmps
from time import sleep
from cmap import *
from gui import *
from utils import *
from cozmo.util import degrees
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
import asyncio
import copy

MAX_NODES = 20000
START_OFFSET = [50, 35]
GOAL_CUBE = 2
OBSTACLE_1 = 0
OBSTACLE_2 = 1

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, all nodes are Node object, each of which has its own
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
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    valid = False
    while not valid:
        randw = np.random.rand() * cmap.width
        randh = np.random.rand() * cmap.height
        rand_node = Node([randw, randh], None)
        if cmap.is_inbound(rand_node) and not cmap.is_inside_obstacles(rand_node):
            valid = True
    ############################################################################
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
        # sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
    else:
        print("Please try again :-(")


def transform(rob_glob_pose, node):
    rob_global_angle = rob_glob_pose.rotation.angle_z.radians
    cos_t = np.cos(rob_global_angle)
    sin_t = np.sin(rob_global_angle)
    #x = rob_glob_pose.position.x + START_OFFSET[0]
    #y = rob_glob_pose.position.y + START_OFFSET[1]
    x = rob_glob_pose.position.x
    y = rob_glob_pose.position.y
    T_world_to_robot = np.matrix( [[cos_t,  sin_t, -(cos_t * x + sin_t * y)],
                                   [-sin_t, cos_t, -(-sin_t * x + cos_t * y)],
                                   [    0,      0, 1]])
    node_glob_p = np.matrix([[node[0]],
                            [node[1]],
                            [1]])
    return np.matmul(T_world_to_robot, node_glob_p)

def dist_from_node(robot, node):
    pos = robot.pose.position
    #return np.sqrt(((pos.x + START_OFFSET[0] - node[0]) ** 2) + ((pos.y + START_OFFSET[1] - node[1]) ** 2))
    return np.sqrt(((pos.x - node[0]) ** 2) + ((pos.y - node[1]) ** 2))

def search_for_goal(robot):
    while not cubes[1]:
        check_for_cubes(robot)
        if not cubes[1]:
            robot.turn_in_place(radians(np.pi/8)).wait_for_completed()
            sleep(.5)

# implementation details provided by this stackexchange post
# https://gamedev.stackexchange.com/questions/86755/how-to-calculate-corner-positions-marks-of-a-rotated-tilted-rectangle
def rotate_about_center(center, point, angle):
    tempx = point.x - center.x
    tempy = point.y - center.y
    rx = tempx * np.cos(angle) - tempy * np.sin(angle)
    ry = tempx * np.sin(angle) + tempy * np.cos(angle)
    return Node([rx + center.x, ry + center.y])


def check_for_cubes(robot):
        # Current obstacle adding code BAD, doens't care about rotation, location ends up way off in sim
        # "working prototype"
        objs = robot.world.visible_objects
        sleep(.2)
        objs = robot.world.visible_objects
        obstacle_size = 100
        for obj in objs:
            print("OBJECT ID:", obj.object_id)
            px = obj.pose.position.x + START_OFFSET[0]
            py = obj.pose.position.y + START_OFFSET[1]
            if obj.object_id == 0 and cubes[0] == False:
                tmp = cubes[0]  # dumb ordering for manipulating this tmp variable and cubes[0] to avoid adding duplicate obstacles
                cubes[0] = True # have to set cubes[0] true before adding obstacle.
                angle = obj.pose.rotation.angle_z.radians
                if not tmp:
                    cmap.add_obstacle([
                        rotate_about_center(Node([px, py]), Node([px+obstacle_size/2, py+obstacle_size/2]), angle),
                        rotate_about_center(Node([px, py]), Node([px+obstacle_size/2, py-obstacle_size/2]), angle),
                        rotate_about_center(Node([px, py]), Node([px-obstacle_size/2, py-obstacle_size/2]), angle),
                        rotate_about_center(Node([px, py]), Node([px-obstacle_size/2, py+obstacle_size/2]), angle),
                    ])
            elif obj.object_id == 1 and cubes[1] == False:
                cmap.clear_goals()
                cmap.add_goal(Node([px, py]))
                cubes[1] = True
            elif obj.object_id == 2 or obj.object_id == 3 and cubes[2] == False:
                tmp = cubes[2]
                cubes[2] = True
                angle = obj.pose.rotation.angle_z.radians
                if not tmp:
                    cmap.add_obstacle([
                        rotate_about_center(Node([px, py]), Node([px+obstacle_size/2, py+obstacle_size/2]), angle),
                        rotate_about_center(Node([px, py]), Node([px+obstacle_size/2, py-obstacle_size/2]), angle),
                        rotate_about_center(Node([px, py]), Node([px-obstacle_size/2, py-obstacle_size/2]), angle),
                        rotate_about_center(Node([px, py]), Node([px-obstacle_size/2, py+obstacle_size/2]), angle),
                    ])
def go_to_node(robot, node):
    node_ = [node[0], node[1]] # ensuring the node is actually a list for transform()
    node_rel_pose = transform(robot.pose, node_)
    rel_angle = np.arctan2(node_rel_pose[1], node_rel_pose[0])
    robot.turn_in_place(radians(rel_angle)).wait_for_completed()
    robot.drive_straight(distance_mm(dist_from_node(robot, node_)), speed_mmps(30), should_play_anim=False).wait_for_completed()

def turn_to_node(robot, node):
    node_ = [node[0], node[1]] # ensuring the node is actually a list for transform()
    node_rel_pose = transform(robot.pose, node_)
    rel_angle = np.arctan2(node_rel_pose[1], node_rel_pose[0])
    robot.turn_in_place(radians(rel_angle)).wait_for_completed()

def drive_at_node(robot, node):
    node_ = Node([node[0], node[1]])
    robot.drive_straight(distance_mm(dist_from_node(robot, node_)), speed_mmps(30), should_play_anim=False).wait_for_completed()
    
# ASSUMES THERE IS ONLY ONE GOAL!
def extract_winning_path(map):
    if not map.is_solved():
        return -1
    path = []
    goal = map.get_goals()[0]
    path.insert(0, goal)
    parent = goal.parent
    while parent != None:
        path.insert(0, parent)
        parent = parent.parent
    return path

def different(old, new):
    return (old[0] != new[0]) or (old[1] != new[1]) or (old[2] != new[2])

def follow_trail(robot, cmap):
    path = extract_winning_path(cmap)
    for current in path:
        if current == path[len(path)-1]:
            return True
        prev = copy.deepcopy(cubes)
        sleep(.5)
        check_for_cubes(robot)
        if different(prev, cubes):
            return False
        turn_to_node(robot, Node([current[0] - START_OFFSET[0], current[1] - START_OFFSET[1]]))
        sleep(.5)
        check_for_cubes(robot)
        if different(prev, cubes):
            return False
        drive_at_node(robot, Node([current[0] - START_OFFSET[0], current[1] - START_OFFSET[1]]))

def CozmoPlanning(robot: cozmo.robot.Robot):
    global cmap, stopevent, cubes
    cubes = [False, False, False]

    # whole section is for finding the goal cube
    found_goal = False
    while not found_goal:
        sleep(.5)
        check_for_cubes(robot)
        if cubes[1]:
            found_goal = True
            break
        cmap.clear_goals() 
        cmap.clear_node_paths()
        cmap.reset()
        cmap.set_start(Node([robot.pose.position.x + START_OFFSET[0], robot.pose.position.y + START_OFFSET[1]])) # have to offset or else the cmap gets angry at start
        rand_goal = cmap.get_random_valid_node()
        cmap.add_goal(rand_goal) # search by adding a new random goal
        RRT(cmap, cmap.get_start())
        follow_trail(robot, cmap) # follow the path until (1) goal is seen, (2) new obstacle is seen, or (3) current path completes
        if cubes[1]: # if we found the goal cube
            found_goal = True
            break             # then we go on to the next phase

    done = False
    while not done: # should be here when the goal cube has been found, now we just have to get there.
        cmap.reset()
        cmap.set_start(Node([robot.pose.position.x + START_OFFSET[0], robot.pose.position.y + START_OFFSET[1]])) # have to offset or else the cmap gets angry at start
        RRT(cmap, cmap.get_start())
        if follow_trail(robot, cmap): # terminates program execution upon reaching the goal.
            return

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
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
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

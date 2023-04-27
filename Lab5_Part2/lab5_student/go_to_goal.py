#!/usr/bin/env python3
# Michael Burke and Griffin Bjerke

import cv2
import cozmo
import numpy as np
from numpy.linalg import inv
import threading
import time

from ar_markers.hamming.detect import detect_markers

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *

# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

#marker size in inches
marker_size = 3.5

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"


async def image_processing(robot):

    global camK, marker_size

    event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # convert camera image to opencv format
    opencv_image = np.asarray(event.image)
    
    # detect markers
    markers = detect_markers(opencv_image, marker_size, camK)
    
    # show markers
    for marker in markers:
        marker.highlite_marker(opencv_image, draw_frame=True, camK=camK)
        #print("ID =", marker.id);
        #print(marker.contours);
    #cv2.imshow("Markers", opencv_image)

    return markers

#calculate marker pose
def cvt_2Dmarker_measurements(ar_markers):
    
    marker2d_list = []
    
    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0,0,1], [0,-1,0], [1,0,0]])
        R_2_2p = np.matrix([[0,-1,0], [0,0,-1], [1,0,0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        #print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2,0], R_2p_1p[0,0])
        
        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        # print('x =', x, 'y =', y,'theta =', yaw)
        
        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x,y,math.degrees(yaw)))

    return marker2d_list


#compute robot odometry based on past and current pose
def compute_odometry(curr_pose, cvt_inch=True):
    global last_pose
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees

    if cvt_inch:
        last_x, last_y = last_x / 25.6, last_y / 25.6
        curr_x, curr_y = curr_x / 25.6, curr_y / 25.6

    return [[last_x, last_y, last_h],[curr_x, curr_y, curr_h]]

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)

async def robust_find_markers(robot):
    markers = []
    for i in range(10):
        seen = await image_processing(robot)
        for mkr in seen:
            if len(markers) == 0:
                markers.append(mkr)
            else:
                add = True
                for noted in markers:
                    if np.abs((mkr.center[0] - noted.center[0]) + (mkr.center[1] - noted.center[1])) < 5:
                        add = False
                if add:
                    markers.append(mkr)
    return markers

def filter_update(pf, markers, robot):
    marker_poses = cvt_2Dmarker_measurements(markers)
    x, y, h, conf = pf.update(compute_odometry(robot.pose), marker_poses)
    print("robot's zone:", x, y, h)
    gui.show_particles(pf.particles)
    gui.show_mean(x, y, h, conf)
    gui.updated.set()
    return (x, y, h, conf)

def get_rel_angle(robot_est_pose, goal_pose):
    return np.degrees(np.arctan2(goal_pose[1] - robot_est_pose[1], goal_pose[0] - robot_est_pose[0]))

def check_kidnapped(robot):
    return robot.pose.position.x == 0 and robot.pose.position.y == 0

async def run(robot: cozmo.robot.Robot):
    global last_pose
    global grid, gui

    # start streaming
    robot.camera.image_stream_enabled = True

    #start particle filter
    pf = ParticleFilter(grid)

    ############################################################################
    ######################### YOUR CODE HERE####################################
    # move bot a bit at start to avoid thinking kidnapped
    await robot.drive_wheels(20, 20, None, None, 1.5)
    time.sleep(2)
    state = "localizing"
    localization_steps = 0
    while True:
        if check_kidnapped(robot):
            state = "kidnapped"
        if state == "kidnapped":
            print("the bot has been moved")
            robot.move_lift(1.0)
            time.sleep(.5)
            robot.move_lift(-100.0)
            time.sleep(.5)
            robot.move_lift(1.0)
            time.sleep(.5)
            robot.move_lift(-100.0)
            time.sleep(.5)
            time.sleep(2)
            await robot.drive_wheels(20, 20, None, None, 1.5)
            time.sleep(2)
            pf.particles = Particle.create_random(PARTICLE_COUNT, grid)
            localization_steps = 0
            state = "localizing"
        elif state == "localizing":
            last_pose = robot.pose
            if check_kidnapped(robot):
                state = "kidnapped"
                continue
            await robot.drive_wheels(-20, 20, None, None, 2)
            if check_kidnapped(robot):
                state = "kidnapped"
                continue
            markers = await robust_find_markers(robot)
            localization_steps += 1
            (_, _, _, conf) = filter_update(pf, markers, robot)
            if conf:
                localization_steps = 0
                state = "approaching goal"
                last_pose = robot.pose
            if localization_steps > 8:
                pf.particles = Particle.create_random(PARTICLE_COUNT, grid)
                localization_steps = 0
                print("localization went wrong: trying again")

        elif state == "approaching goal":
            if check_kidnapped(robot):
                state = "kidnapped"
                continue
            markers = await robust_find_markers(robot)
            (x, y, h, conf) = filter_update(pf, markers, robot)
            if not conf:
                localization_steps = 0
                state = "localizing"
            else:
                if check_kidnapped(robot):
                    state = "kidnapped"
                    continue
                rel_ang = get_rel_angle([x,y,h], goal)
                print("RELATIVE ANGLE:", h - rel_ang)
                dist = np.sqrt(((goal[0] - x) ** 2) + ((goal[1] - y) ** 2))
                print("DISTANCE:", dist)
                last_pose = robot.pose
                if dist < 1:
                    continue
                if np.abs(h - rel_ang) < 20 or 360 - np.abs(h - rel_ang) < 20:
                    await robot.drive_wheels(20, 20, None, None, 2)
                    if check_kidnapped(robot):
                        state = "kidnapped"
                        continue
                else:
                    await robot.drive_wheels(-15, 15, None, None, 2)
                    if check_kidnapped(robot):
                        state = "kidnapped"
                        continue

    ############################################################################


class CozmoThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)
    gui.start()

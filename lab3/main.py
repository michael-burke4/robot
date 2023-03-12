#!/usr/bin/env python3


# this stackoverflow link shows how to disable warnings. https://stackoverflow.com/a/50519680
# the cozmo sdk code is out of date so I kept getting deprecation warnings. Nothing was fixing
# it until I found this answer.
import warnings
def warn(*args, **kwargs):
    pass
warnings.warn = warn
import time
import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
from statemachine import State, StateMachine
import numpy as np

def transform(rob_glob_pose, box_glob_pose):
    rob_global_angle = rob_glob_pose.rotation.angle_z.radians
    cos_t = np.cos(rob_global_angle)
    sin_t = np.sin(rob_global_angle)
    x = rob_glob_pose.position.x
    y = rob_glob_pose.position.y
    T_world_to_robot = np.matrix( [[cos_t,  sin_t, -(cos_t * x + sin_t * y)],
                                   [-sin_t, cos_t, -(-sin_t * x + cos_t * y)],
                                   [    0,      0, 1]])
    box_glob_p = np.matrix([[box_glob_pose.position.x],
                            [box_glob_pose.position.y],
                            [1]])
    return np.matmul(T_world_to_robot, box_glob_p)

# returns 0 if centered, -1 if to the left, 1 if to the right.

class CozmoMachine(StateMachine):
    searching = State('Searching', initial=True)
    centering = State('Centering')
    approaching = State('Approaching')
    waiting = State('Waiting')

    center = searching.to(centering)
    lose_track = centering.to(searching)
    begin_approach = centering.to(approaching)
    wait = approaching.to(waiting)
    lose_center = approaching.to(centering)

    def on_enter_searching(self):
        print("searching!")
        #self.robot.say_text("Searching").wait_for_completed()
        # found = False
        wheelspeed = 10
        turning_cw = True
        centered = False
        self.robot.drive_wheels(wheelspeed, -wheelspeed)
        while not centered:
            time.sleep(.05)
            objs = self.robot.world.visible_objects
            try:
                for obj in objs:
                    if obj.object_type == CustomObjectTypes.CustomType01:
                        box_rel_pose = transform(self.robot.pose, obj.pose)
                        rel_angle = np.arctan(box_rel_pose[1] / box_rel_pose[0])
                        # print(rel_angle)
                        if (rel_angle < -.1):
                            if not turning_cw:
                                self.robot.stop_all_motors()
                                self.robot.drive_wheels(wheelspeed, -wheelspeed)
                                turning_cw = True
                        elif (rel_angle > .1):
                            if turning_cw:
                                self.robot.stop_all_motors()
                                self.robot.drive_wheels(-wheelspeed, wheelspeed)
                                turning_cw = False
                        else:
                            self.robot.stop_all_motors()
                            time.sleep(5)
                            print("done sleepin!")
                            return
                            
            except:
                print("SHOULDN'T GET HERE OFTEN!")
                pass
        # self.center()

    def on_enter_centering(self):
        print("Centering!")
        #self.robot.say_text("Centering").wait_for_completed()
        centered = False
        still_see = True
        self.robot.drive_wheels(10, -10)
        while (not centered) and still_see:
            time.sleep(.1)
            objs = self.robot.world.visible_objects
            still_see = False
            try:
                for obj in objs:
                    if obj.object_type == CustomObjectTypes.CustomType01:
                        still_see = True
                        center_status = check_center(obj, self.robot)
                        if center_status == 0:
                            centered = True
                            print("GOT HERE!")
                            self.robot.stop_all_motors()
                            while True:
                                time.sleep(.1)
                                pass
                        self.robot.drive_wheels(center_status * 10, center_status * -10)
            except:
                print("SHOULDNT REALLY  GET HERE")
                pass
        if not still_see:
            self.robot.stop_all_motors()
            print("LOSING TRACK!")
            self.lose_track()
        if centered:
            self.begin_approach()

    def on_enter_approaching(self):
        self.robot.stop_all_motors()
        self.robot.say_text("approaching").wait_for_completed()
        print("Approaching...")
        self.robot.drive_wheels(10, 10)
        while True:
            pass

    def on_enter_waiting(self):
        print("waiting!")
        while True:
            pass

    def __init__(self, robot):
        self.robot = robot
        super().__init__()






def run(robot: cozmo.robot.Robot):

    circle_cube = robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                              CustomObjectMarkers.Circles2,
                                              44,
                                              31, 31, True)
    diamond_cube = robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                              CustomObjectMarkers.Diamonds2,
                                              44,
                                              31, 31, True)
    
    machine = CozmoMachine(robot)

cozmo.run_program(run, use_3d_viewer=True, use_viewer = True, force_viewer_on_top = True)
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

class CozmoMachine(StateMachine):
    searching = State('Searching', initial=True)
    approaching = State('Approaching')
    waiting = State('Waiting')

    approach = searching.to(approaching)
    lose_track = approaching.to(searching)
    enter_wait = approaching.to(waiting)

    def on_enter_searching(self):
        print("searching!")
        self.robot.say_text("Searching").wait_for_completed()
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
                            centered = True
            except:
                print("Error occurred - hopefully just lost track of an obj mid-loop?")
                pass
        self.approach()

    def on_enter_approaching(self):
        self.robot.say_text("approaching").wait_for_completed()
        print("Approaching...")
        self.robot.drive_wheels(10, 10)
        time.sleep(1)
        seen = True
        close = False
        while seen and not close:
            time.sleep(.05)
            objs = self.robot.world.visible_objects
            seen = False
            try:
                for obj in objs:
                    if obj.object_type == CustomObjectTypes.CustomType01:
                        seen = True
                        box_rel_pose = transform(self.robot.pose, obj.pose)
                        dist = np.sqrt(box_rel_pose[0] ** 2 + box_rel_pose[1] ** 2)
                        rel_angle = np.arctan(box_rel_pose[1] / box_rel_pose[0])
                        print(rel_angle)
                        if (np.abs(rel_angle) > .2):
                            seen = False
                        # print(dist)
                        if dist <= 125:
                            close = True
            except:
                print("Error occurred - hopefully just lost track of an obj mid-loop?")
        if close:
            self.robot.stop_all_motors()
            self.enter_wait()
        else:
            self.robot.stop_all_motors()
            self.lose_track()

    def on_enter_waiting(self):
        print("waiting!")
        self.robot.say_text("Waiting!").wait_for_completed()
        while True:
            time.sleep(.05)
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
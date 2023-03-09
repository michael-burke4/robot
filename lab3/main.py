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

# returns 0 if centered, -1 if to the left, 1 if to the right.
def check_center(box, robot):
    b_pos = box.pose.position
    b_angle = np.arctan(b_pos.y / b_pos.x)
    rob_angle = robot.pose.rotation.angle_z.radians
    diff = rob_angle - b_angle
    if np.abs(diff) < .1:
        return 0
    elif diff > 0:
        return 1
    else:
        return -1

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
    # lose_track = approaching.to(searching)
    # wait = approaching.to(waiting)
    # search_again = waiting.to(searching)

    def on_enter_searching(self):
        print("searching!")
        self.robot.say_text("Searching").wait_for_completed()
        self.robot.drive_wheels(l_wheel_speed=10, r_wheel_speed=-10)
        found = False
        while not found:
        #     print(self.robot.pose)
            time.sleep(.1)
            objs = self.robot.world.visible_objects
            try:
                for obj in objs:
                    if obj.object_type == CustomObjectTypes.CustomType01:
                        self.robot.stop_all_motors()
                        found = True
            except:
                pass
        self.center()

    def on_enter_centering(self):
        print("Centering!")
        self.robot.say_text("Centering").wait_for_completed()
        centered = False
        still_see = True
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
                            self.robot.stop_all_motors()
                        else:
                            self.robot.drive_wheels(10 * center_status, -10 * center_status)
            except:
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
                                              30, 30, True)
    diamond_cube = robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                              CustomObjectMarkers.Diamonds2,
                                              44,
                                              30, 30, True)
    
    machine = CozmoMachine(robot)

cozmo.run_program(run, use_3d_viewer=True, use_viewer = True, force_viewer_on_top = True)
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

class CozmoMachine(StateMachine):
    searching = State('Searching', initial=True)
    approaching = State('Approaching')
    waiting = State('Waiting')

    find = searching.to(approaching)
    lose = approaching.to(searching)
    wait = approaching.to(waiting)
    search_again = waiting.to(searching)

    def on_enter_searching(self):
        print("searching!")
        self.robot.drive_wheels(l_wheel_speed=10, r_wheel_speed=-10)
        while True:
            time.sleep(.1)
            objs = self.robot.world.visible_objects
            for obj in objs:
                if obj.object_type == CustomObjectTypes.CustomType01:
                    self.find()
    def on_enter_approaching(self):
        self.robot.stop_all_motors()
        while True:
            time.sleep(.1)
            print("I FOUND THE CUBE!")
    def on_enter_waiting(self):
        print("waiting!")

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
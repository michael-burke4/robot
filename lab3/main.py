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
    def on_enter_approaching(self):
        print("approaching!")
    def on_enter_waiting(self):
        print("waiting!")



# def state0(box, robot):
#     bpos = box.pose.position
#     rpos = robot.pose.position
#     b_angle = np.arctan(bpos.y / bpos.x)
#     r_angle = robot.pose.rotation.angle_z.radians
#     print(r_angle, b_angle)


machine = CozmoMachine()
def run(robot: cozmo.robot.Robot):
    seen = False
    last = False
    # robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_appeared)
    # robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, handle_object_disappeared)

    circle_cube = robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                              CustomObjectMarkers.Circles2,
                                              44,
                                              30, 30, True)
    diamond_cube = robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                              CustomObjectMarkers.Diamonds2,
                                              44,
                                              30, 30, True)
    while True:
        global machine
        time.sleep(.2)
        last = seen
        looped = False
        objects = robot.world.visible_objects
        for obj in objects:
            looped = True
            if obj.object_type == CustomObjectTypes.CustomType01:
                seen = True
                break
            seen = False
        if looped == False:
            seen = False
        if last != seen:
            if seen == True:
                machine.find()
            if seen == False:
                machine.lose()
        # diamond_angle = 1000
        # found_diamond = False
        # for obj in objects:
        #     if obj.object_type == CustomObjectTypes.CustomType01:
        #         state0(obj, robot)
        #         return

       #  robot_angle = robot.pose.rotation.angle_z.radians
       #  print("going...", robot_angle, diamond_angle)
       #  robot.drive_wheels(10, -10)
       #  if (np.abs(robot_angle - diamond_angle) < .15):
       #      print("nice")
       #      if not flip:
       #          robot.stop_all_motors()
       #          time.sleep(3)
       #      flip = True
       #  if flip:
       #      print("DIST:", dist)
       #      robot.drive_wheels((dist - 75)/4, (dist - 75)/4)
       #      if dist < 120:
       #          robot.stop_all_motors()
       #          break


cozmo.run_program(run, use_3d_viewer=True, use_viewer = True, force_viewer_on_top = True)

# def handle_object_appeared(evt, **kw):
#     global seen
#     seen = True
    # machine.cycle()
    # if isinstance(evt.obj, CustomObject):
    #     pass
    #     # print("Cozmo is seeing a %s" % str(evt.obj.object_type))


# def handle_object_disappeared(evt, **kw):
#     global seen
#     seen = False
    # machine.cycle()
    # if isinstance(evt.obj, CustomObject):
    #     pass
    #     print("Cozmo stopped seeing a %s" % str(evt.obj.object_type))
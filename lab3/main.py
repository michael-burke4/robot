#!/usr/bin/env python3

# This code follows instruction provided by a cozmo SDK example provided publicly on github, available at
# https://github.com/anki/cozmo-python-sdk/blob/master/examples/tutorials/04_cubes_and_objects/09_custom_objects.py
# to register custom blocks.

# this stackoverflow link shows how to disable warnings. https://stackoverflow.com/a/50519680
# the cozmo sdk code is out of date so I kept getting deprecation warnings. Nothing was fixing
# it until I found this answer.
import warnings
def warn(*args, **kwargs):
    pass
warnings.warn = warn

import sys
import os
import time
import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
from statemachine import State, StateMachine
from PIL import Image, ImageDraw
import numpy as np

def get_face_images():
    current_directory = os.path.dirname(os.path.realpath(__file__))
    img_0 = os.path.join(current_directory, "faces", "approach.png")
    img_1 = os.path.join(current_directory, "faces", "chasing.png")
    img_2 = os.path.join(current_directory, "faces", "L.png")
    img_3 = os.path.join(current_directory, "faces", "R.png")
    img_4 = os.path.join(current_directory, "faces", "searching.png")
    img_5 = os.path.join(current_directory, "faces", "waiting.png")
    image_settings = [(img_0, Image.NEAREST),
                      (img_1, Image.NEAREST),
                      (img_2, Image.NEAREST),
                      (img_3, Image.NEAREST),
                      (img_4, Image.NEAREST),
                      (img_5, Image.NEAREST),]
    face_images = []
    for image_name, resampling_mode in image_settings:
        image = Image.open(image_name)

        # resize to fit on Cozmo's face screen
        resized_image = image.resize(cozmo.oled_face.dimensions(), resampling_mode)

        # convert the image to the format used by the oled screen
        face_image = cozmo.oled_face.convert_image_to_screen_data(resized_image,
                                                                 invert_image=True)
        face_images.append(face_image)
        
    return face_images


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
    hunting_left = State("Hunting Left")
    hunting_right = State("Hunting Right")
    chasing = State("Chasing")

    approach = searching.to(approaching)
    lose_track = approaching.to(searching)
    begin_hunt = approaching.to(hunting_left)
    start_chase_l = hunting_left.to(chasing)
    start_chase_r = hunting_right.to(chasing)
    lose_l = chasing.to(hunting_left)
    lose_r = chasing.to(hunting_right)

    def on_enter_searching(self):
        print("searching!")
        self.robot.say_text("Searching").wait_for_completed()
        self.robot.display_oled_face_image(self.faces[4], duration_ms=500).wait_for_completed()
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
        print("leaving searching...")
        self.approach()

    def on_enter_approaching(self):
        self.robot.say_text("approaching").wait_for_completed()
        self.robot.display_oled_face_image(self.faces[0], duration_ms=500).wait_for_completed()
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
                        # print(rel_angle)
                        if (np.abs(rel_angle) > .2):
                            seen = False
                        # print(dist)
                        if dist <= 125:
                            close = True
            except:
                print("Error occurred - hopefully just lost track of an obj mid-loop?")
        print("leaving approaching...")
        if close:
            self.robot.stop_all_motors()
            time.sleep(1)
            self.begin_hunt()
        else:
            self.robot.stop_all_motors()
            self.lose_track()

    def on_enter_hunting_left(self):
        print("hunting left")
        self.robot.display_oled_face_image(self.faces[2], duration_ms=5000)
        self.robot.drive_wheels(-10, 10)
        self.turned_left = True
        seen = False
        while not seen:
            time.sleep(.05)
            objs = self.robot.world.visible_objects
            try:
                for obj in objs:
                    if obj.object_type == CustomObjectTypes.CustomType00:
                        seen = True
            except:
                print("Shouldn't get here often, likely lost track of an object while looping over objs")
        self.robot.stop_all_motors()
        print("leaving hunt left")
        self.start_chase_l()

    def on_enter_hunting_right(self):
        print("hunting right")
        self.robot.display_oled_face_image(self.faces[3], duration_ms=5000)
        self.robot.drive_wheels(10, -10)
        self.turned_left = False
        seen = False
        while not seen:
            time.sleep(.05)
            objs = self.robot.world.visible_objects
            try:
                for obj in objs:
                    if obj.object_type == CustomObjectTypes.CustomType00:
                        seen = True
            except:
                print("Shouldn't get here often, likely lost track of an object while looping over objs")
        print("leaving hunt right")
        self.start_chase_r()

    def on_enter_chasing(self):
        print("Chasing!!")
        self.robot.display_oled_face_image(self.faces[1], duration_ms=5000)
        seen = False
        while not seen or True:
            time.sleep(.05)
            objs = self.robot.world.visible_objects
            try:
                for obj in objs:
                    if obj.object_type == CustomObjectTypes.CustomType00:
                        seen = True
            except:
                print("Shouldn't get here often, likely lost track of an object while looping over objs")
        print("leaving chasing")
    #def on_enter_waiting(self):
        #print("waiting!")
        #self.robot.say_text("Waiting!").wait_for_completed()
        #while True:
        #   time.sleep(.05)
        #   pass

    def __init__(self, robot):
        self.robot = robot
        self.faces = get_face_images()
        self.turned_left = True
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
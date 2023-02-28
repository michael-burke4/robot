#!/usr/bin/env python3
#!c:/Python35/python3.exe -u
import asyncio
import sys
import cv2
import numpy as np
import cozmo
import time
import os
from glob import glob

from find_cube import *

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
def nothing(x):
    pass

YELLOW_LOWER = np.array([9, 115, 151])
YELLOW_UPPER = np.array([179, 215, 255])

GREEN_LOWER = np.array([0,23,8])
GREEN_UPPER = np.array([78, 255, 60])

RED_LOWER = np.array([0,135,107])
RED_UPPER = np.array([4, 228, 255])

CAMERA_CENTER = 165.42


# Define a decorator as a subclass of Annotator; displays the keypoint
class BoxAnnotator(cozmo.annotate.Annotator):

    cube = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BoxAnnotator.cube is not None:

            #double size of bounding box to match size of rendered image
            # BoxAnnotator.cube = np.multiply(BoxAnnotator.cube,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BoxAnnotator.cube[0]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[1]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[2], BoxAnnotator.cube[2])
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BoxAnnotator.cube = None



async def run(robot: cozmo.robot.Robot):

    robot.world.image_annotator.annotation_enabled = False
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True

    gain, exposure, mode = 390,3,1

    consecutive_lost = 0
    try:
        while True:
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            if event.image is not None:
                image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)
                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure,fixed_gain)

                # find the cube
                # cube = find_cube(image, YELLOW_LOWER, YELLOW_UPPER)
                cube = find_cube(image, RED_LOWER, RED_UPPER)
                BoxAnnotator.cube = cube
                if cube == None:
                    if consecutive_lost > 30:
                        await robot.drive_wheels(-20, 20, None, None, 2)
                    else:
                        consecutive_lost += 1
                else:
                    speeds = calc_speed(cube)
                    print("SIZE", cube.size)
                    consecutive_lost = 0
                    await robot.drive_wheels(speeds[0], speeds[1], None, None, 1.4)
                    # robot.drive_wheel_motors(speeds[0], speeds[1])
                # robot.drive_wheel_motors(speeds[0], speeds[1])

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    #cv2.destroyAllWindows()

def calc_speed(cube):
    if (cube.size > 90):
        return (0, 0)
    x = cube.pt[0]
    center_diff = CAMERA_CENTER - x
    diff_ratio = center_diff / CAMERA_CENTER
    base_speed = 20
    r_speed = base_speed - (diff_ratio * base_speed)
    l_speed = base_speed + (diff_ratio * base_speed)
    if (r_speed - l_speed) < 3:
        r_speed *= 1.5
        l_speed *= 1.5
    return (r_speed, l_speed)
    

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)

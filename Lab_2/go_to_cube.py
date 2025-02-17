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

RED_LOWER = np.array([30,15,15 ])
RED_UPPER = np.array([205,200,82])

# Define a decorator as a subclass of Annotator; displays the keypoint
class BoxAnnotator(cozmo.annotate.Annotator):

    cube = None
    
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        #bounds = (0, 0, image.width, image.height)

        if BoxAnnotator.cube is not None:

            #double size of bounding box to match size of rendered image
            #BoxAnnotator.cube = np.multiply(BoxAnnotator.cube,2)

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

    try:

        while True:
            cube_found = False
            while not cube_found:
                event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
                if event.image is not None:
                    image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)
                    #find the cube
                    cube = find_cube(image, RED_LOWER, RED_UPPER)
                    if cube is not None:
                        x, y, radius = cube
                        BoxAnnotator.cube = cube

                        # Align Cozmo with the cube
                        image_center_x = image.shape[1] / 2
        
                        # Adjust robot's position to face the cube
                        if x < image_center_x - 10:  # If cube is to the left
                            await robot.turn_in_place(cozmo.util.degrees(10)).wait_for_completed()
                        elif x > image_center_x + 10:  # If cube is to the right
                            await robot.turn_in_place(cozmo.util.degrees(-10)).wait_for_completed()

                        ################################################################
                        # Todo: Add Motion Here
                        ################################################################
                        
                        print(radius)
                        # Drive towards the cube
                        if radius < 45.846618890762329:
                            await robot.drive_straight(cozmo.util.distance_mm(50), cozmo.util.speed_mmps(50)).wait_for_completed()
                        else:
                            print("Stopping.")
                            time.sleep(2)  # Wait for some time before restarting the search
                            #cube_found = False  # Restart the search
                    else:
                        await robot.turn_in_place(cozmo.util.degrees(-10)).wait_for_completed()


    except KeyboardInterrupt:
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
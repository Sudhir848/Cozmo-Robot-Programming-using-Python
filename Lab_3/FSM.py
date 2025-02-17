import asyncio
import cozmo
import cv2
import numpy as np
from transitions import Machine
from cozmo.util import degrees, distance_mm, speed_mmps
from PIL import Image, ImageDraw, ImageFont

class CozmoFSM:
    def __init__(self, robot):
        self.robot = robot
        self.states = ['searching', 'moving_to_cube', 'reading_ar_marker', 'restart_search']
        self.machine = Machine(model=self, states=self.states, initial='searching')
        self.machine.add_transition('found_cube', 'searching', 'moving_to_cube')
        self.machine.add_transition('reached_cube', 'moving_to_cube', 'reading_ar_marker')
        self.machine.add_transition('completed_reading', 'reading_ar_marker', 'restart_search')
        self.machine.add_transition('restart_search', 'restart_search', 'searching')
        self.last_direction = None
        self.last_seen_x = None
        
    async def beep(self):
        print("Beep! Transition state changed.")
        await self.robot.say_text("beep", duration_scalar=0.3, use_cozmo_voice=False, in_parallel=True).wait_for_completed()

    async def update_status(self, text):
        print(f"Entering state: {text}")
        await self.display_text_on_face(text)
        print(f"Exiting state: {text}")

    async def display_text_on_face(self, text):
        img = Image.new('RGB', (128, 64), color=(0, 0, 0))
        d = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype("arial.ttf", 20)
        except IOError:
            font = ImageFont.load_default()
        d.text((10, 20), text, fill=(255, 255, 255), font=font)
        face_data = cozmo.oled_face.convert_image_to_screen_data(img)
        await self.robot.display_oled_face_image(face_data, 5000.0).wait_for_completed()

    async def read_ar_marker(self):
        self.machine.trigger('completed_reading')
        await self.update_status("Reading AR Marker")
        await asyncio.sleep(2)
        
    async def search_for_cube(self):
        await self.update_status("Searching")
        self.robot.camera.image_stream_enabled = True
        
        while True:
            event = await self.robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
            opencv_image = np.array(event.image)
            gray_image = cv2.cvtColor(opencv_image, cv2.COLOR_RGB2GRAY)
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in [3, 4]:
                        x, y = np.mean(corners[i][0], axis=0)
                        self.last_seen_x = x
                        self.last_direction = 'left' if x < opencv_image.shape[1] / 2 else 'right'
                        await self.align_with_cube(x, opencv_image.shape[1])
                        return
            else:
                print("No ArUco marker found. Continuing to search...")
                if self.last_seen_x:
                    turn_degrees = 20 if self.last_seen_x < opencv_image.shape[1] / 2 else -20
                else:
                    turn_degrees = 20 if self.last_direction != 'right' else -20
                await self.robot.turn_in_place(degrees(turn_degrees)).wait_for_completed()

    async def align_with_cube(self, x, image_width):
        image_center_x = image_width / 2
        # Adjust the robot's position to face the cube
        if x < image_center_x - 10:  # If cube is to the left
            await self.robot.turn_in_place(degrees(10)).wait_for_completed()
            self.last_direction = 'left'
        elif x > image_center_x + 10:  # If cube is to the right
            await self.robot.turn_in_place(degrees(-10)).wait_for_completed()
            self.last_direction = 'right'
        await self.found_cube()

    async def found_cube(self):
        await self.beep() 
        self.to_moving_to_cube()
        await self.move_to_cube()
        await self.restart_search()
        await self.beep() 

    async def move_to_cube(self):
        await self.update_status("Moving")
        print("Moving to cube...")
        moving = True
        while moving:
            await self.robot.drive_wheels(30, 30, duration = 1)
            moving, adjusted = await self.check_and_adjust()
            if not adjusted:
                print("Stopping")
                break
        # Stopping the robot.
        self.robot.stop_all_motors()
        self.reached_cube()
        await self.beep()

    async def check_and_adjust(self):
        event = await self.robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=5)
        opencv_image = np.array(event.image)
        gray_image = cv2.cvtColor(opencv_image, cv2.COLOR_RGB2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in [3, 4]:
                    x, y = np.mean(corners[i][0], axis=0)
                    if self.last_seen_x is not None and abs(x - self.last_seen_x) > 20:
                        await self.align_with_cube(x, opencv_image.shape[1])
                        return True, True
                    self.last_seen_x = x
                    return True, False
        return False, False

    async def restart_search(self):
        await self.update_status("Restarting Search")
        await self.beep()
        await asyncio.sleep(2)
        await self.search_for_cube()
        
    async def run(self):
        await self.search_for_cube()

async def cozmo_program(robot: cozmo.robot.Robot):
    fsm = CozmoFSM(robot)
    await fsm.run()

cozmo.run_program(cozmo_program, use_viewer=True, force_viewer_on_top=True)
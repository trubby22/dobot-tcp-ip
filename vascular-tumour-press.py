from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
import re
from time import sleep
import numpy as np
import cv2
import IPython

SENSOR_DOME_TIP_INITIAL_POSE = np.array([-310, -25, 93, 180, 0, 0], dtype=float)

class Controller:
    def __init__(self):
        ip = '192.168.5.1'
        dashboard_port = 29_999
        feedback_port = 30_004
        self.d = DobotApiDashboard(ip, dashboard_port)
        self.f = DobotApiFeedBack(ip, feedback_port)
        self.robot_mode = None
        self.current_command_id = None
        self.init_camera()
        self.press_ix = 0

        print(self.d.RequestControl())
        print(self.d.EnableRobot())
        print(self.d.SetCollisionLevel(0))

        threading.Thread(target=self.get_feed_small, daemon=True).start()
    
    def parse_result_id(self, response):
        if "Not Tcp" in response:
            print("Control Mode Is Not Tcp")
            return [1]
        return [int(num) for num in re.findall(r'-?\d+', response)] or [2]

    def __del__(self):
        del self.d
        del self.f
        self.release_camera()
    
    def get_feed_small(self):
        while True:
            feedback = self.f.feedBackData()
            if feedback is None:
                continue
            if hex((feedback['TestValue'][0])) != '0x123456789abcdef':
                raise Exception("TestValue not as expected")
            self.robot_mode = feedback['RobotMode'][0]
            self.current_command_id = feedback['CurrentCommandId'][0]
    
    def run_point(self, response):
        command_id = self.parse_result_id(response)[1]
        while True:
            if self.robot_mode == 5 and self.current_command_id == command_id:
                break
            sleep(0.1)
        return response
    
    def press_and_take_photo(self):
        xr_offset = np.random.uniform(-15, 15)
        yr_offset = np.random.uniform(-15, 15)
        zr_offset = np.random.uniform(-15, 15)
        press_depth = 10 + np.random.uniform(8, 12)
        x, y, z, xr, yr, zr = SENSOR_DOME_TIP_INITIAL_POSE.copy()
        trajectory = np.array([
            [x, y, z, xr+xr_offset, yr+yr_offset, zr+zr_offset],
            [x, y, z-press_depth, xr+xr_offset, yr+yr_offset, zr+zr_offset],
            [x, y, z, xr+xr_offset, yr+yr_offset, zr+zr_offset],
        ], dtype=float)
        for i in range(trajectory.shape[0]):
            x, y, z, xr, yr, zr = trajectory[i]
            self.run_point(self.d.MovL(x, y, z, xr, yr, zr, coordinateMode=0, v=10))
            if i == 1:
                sleep(1.0)
                self.capture_and_save_photo()
                sleep(1.0)
        print(f'press {self.press_ix} completed successfully!')
        self.press_ix += 1
    
    def init_camera(self):
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            exit()
        
        print("\nConfiguring camera settings...")
        
        # Try to set resolution first
        print("\nAttempting to set resolution...")
        resolutions_to_try = [
            (1920, 1080),
            (1280, 720),
            (640, 480)
        ]
        
        resolution_set = False
        for width, height in resolutions_to_try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            if abs(actual_width - width) < 1 and abs(actual_height - height) < 1:
                print(f"Successfully set resolution to {width}x{height}")
                resolution_set = True
                break
            else:
                print(f"Failed to set resolution {width}x{height}")
        
        if not resolution_set:
            print(f"Warning: Using default resolution: {actual_width}x{actual_height}")

        # Dictionary of settings with their target values and acceptable tolerance
        settings = {
            'Brightness': {'prop': cv2.CAP_PROP_BRIGHTNESS, 'target': -64, 'tolerance': 1},
            'Contrast': {'prop': cv2.CAP_PROP_CONTRAST, 'target': 48, 'tolerance': 1},
            'Saturation': {'prop': cv2.CAP_PROP_SATURATION, 'target': 0, 'tolerance': 1},
            'Hue': {'prop': cv2.CAP_PROP_HUE, 'target': 0, 'tolerance': 1},
            'Auto White Balance': {'prop': cv2.CAP_PROP_AUTO_WB, 'target': 0, 'tolerance': 0},
            'Gamma': {'prop': cv2.CAP_PROP_GAMMA, 'target': 100, 'tolerance': 1},
            'Gain': {'prop': cv2.CAP_PROP_GAIN, 'target': 0, 'tolerance': 1},
            'White Balance Temperature': {'prop': cv2.CAP_PROP_WB_TEMPERATURE, 'target': 5000, 'tolerance': 50},
            'Sharpness': {'prop': cv2.CAP_PROP_SHARPNESS, 'target': 3, 'tolerance': 1},
            'Backlight': {'prop': cv2.CAP_PROP_BACKLIGHT, 'target': 0, 'tolerance': 0},
            'Auto Exposure': {'prop': cv2.CAP_PROP_AUTO_EXPOSURE, 'target': 1, 'tolerance': 0},
            'Exposure': {'prop': cv2.CAP_PROP_EXPOSURE, 'target': 100, 'tolerance': 1}
        }

        unsupported_settings = []
        misconfigured_settings = []

        # Try to set each property and verify
        for setting_name, setting_info in settings.items():
            # Try to set the property
            ret = self.cap.set(setting_info['prop'], setting_info['target'])
            
            # Read back the actual value
            actual_value = self.cap.get(setting_info['prop'])
            
            # Check if the setting was successful
            if actual_value == 0 and setting_info['target'] != 0:
                unsupported_settings.append(setting_name)
            elif abs(actual_value - setting_info['target']) > setting_info['tolerance']:
                misconfigured_settings.append({
                    'name': setting_name,
                    'target': setting_info['target'],
                    'actual': actual_value
                })

        # Report results
        if not unsupported_settings and not misconfigured_settings:
            print("\nAll camera settings were successfully applied!")
        else:
            if unsupported_settings:
                print("\nWarning: The following settings appear to be unsupported by your camera:")
                for setting in unsupported_settings:
                    print(f"- {setting}")
            
            if misconfigured_settings:
                print("\nWarning: The following settings could not be set to their target values:")
                for setting in misconfigured_settings:
                    print(f"- {setting['name']}: Target={setting['target']}, Actual={setting['actual']}")
            
            print("\nThe camera will continue to operate with the best possible settings.")

    def capture_and_save_photo(self):
        ret, frame = self.cap.read()
        if ret:
            cv2.imwrite(f"/home/psb120/Documents/TCP-IP-Python-V4/experiment-capture/press-{self.press_ix}.jpg", frame)
        else:
            print(f"Error: Could not capture frame for press {self.press_ix}.")

    def release_camera(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    c = Controller()
    IPython.embed()

if __name__ == '__main__':
    main()

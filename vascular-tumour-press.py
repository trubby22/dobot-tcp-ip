from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
import re
from time import sleep
import numpy as np
import cv2

SENSOR_DOME_TIP_INITIAL_POSE = np.array([0, 0, 0, 180, 0, 0], dtype=float)

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

        print(self.d.RequestControl())
        print(self.d.EnableRobot())

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
    
    def press_and_take_photo(self, press_ix):
        xr_offset = np.random.uniform(-15, 15)
        yr_offset = np.random.uniform(-15, 15)
        zr_offset = np.random.uniform(-15, 15)
        press_depth = np.random.uniform(6, 16)
        x, y, z, xr, yr, zr = SENSOR_DOME_TIP_INITIAL_POSE.copy()
        trajectory = np.array([
            [x, y, z, xr+xr_offset, yr+yr_offset, zr+zr_offset],
            [x, y, z-press_depth, xr+xr_offset, yr+yr_offset, zr+zr_offset],
            [x, y, z, xr+xr_offset, yr+yr_offset, zr+zr_offset],
        ], dtype=float)
        for i in range(trajectory.shape[0]):
            x, y, z, xr, yr, zr = trajectory[i]
            self.run_point(self.d.MovL(x, y, z, xr, yr, zr, coordinateMode=0, v=2))
            if i == 1:
                self.capture_and_save_photo(press_ix)
    
    def init_camera(self):
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            exit()
        
        guvcview_settings = [
            (0x00980900, -64),
            (0x00980901, 48),
            (0x00980902, 0),
            (0x00980903, 0),
            (0x0098090c, 1),
            (0x00980910, 100),
            (0x00980913, 0),
            (0x00980918, 1),
            (0x0098091a, 5000),
            (0x0098091b, 3),
            (0x0098091c, 0),
            (0x009a0901, 1),
            (0x009a0902, 100),
            (0x009a0903, 0),
        ]

        for id, val in guvcview_settings:
            self.cap.set(id, val)

    def capture_and_save_photo(self, i):
        ret, frame = self.cap.read()
        if ret:
            cv2.imwrite(f"/home/psb120/Documents/TCP-IP-Python-V4/experiment-capture/press-{i}.jpg", frame)
            print(f"Photo for press {i} taken successfully!")
        else:
            print(f"Error: Could not capture frame for press {i}.")

    def release_camera(self):
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    controller = Controller()
    i = 0
    while True:
        controller.capture_and_save_photo(i)
        print(f'press {i} complete!')
        i += 1

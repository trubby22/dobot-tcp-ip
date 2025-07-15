from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
import re
from time import sleep
import numpy as np
import IPython

class SystemId:
    def __init__(self):
        ip = '192.168.5.1'
        dashboard_port = 29_999
        feedback_port = 30_004
        self.d = DobotApiDashboard(ip, dashboard_port)
        self.f = DobotApiFeedBack(ip, feedback_port)
        self.feedback = dict()

        self.set_trajectories()

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
    
    def get_feed_small(self):
        while True:
            feedback = self.f.feedBackData()
            if feedback is None:
                continue
            if hex((feedback['TestValue'][0])) != '0x123456789abcdef':
                raise Exception("TestValue not as expected")
            self.feedback['RobotMode'] = feedback['RobotMode'][0]
            self.feedback['CurrentCommandId'] = feedback['CurrentCommandId'][0]
    
    def run_point(self, response):
        command_id = self.parse_result_id(response)[1]
        while True:
            if self.feedback['RobotMode'] == 5 and self.feedback['CurrentCommandId'] == command_id:
                break
            sleep(0.1)
    
    
    def set_trajectories(self):
        self.home_pos_np = np.array([-310, -25, 63, 180, 0, 0])
        x, y, z, xr, yr, zr = self.home_pos_np
        press_depth = 6
        slide_length = 25
        twist_x = 20
        twist_z = 20
        self.trajectories_np = np.array([
            [
                [x, y, z, xr, yr, zr],
                [x, y, z-press_depth, xr, yr, zr],
            ],
            [
                [x, y, z, xr, yr, zr],
                [x, y, z-press_depth, xr, yr, zr],
                [x-slide_length, y, z-press_depth, xr, yr, zr],
            ],
            [
                [x, y, z, xr, yr, zr],
                [x, y, z-press_depth, xr, yr, zr],
                [x, y, z-press_depth, xr+twist_x, yr, zr],
            ],
            [
                [x, y, z, xr, yr, zr],
                [x, y, z-press_depth, xr, yr, zr],
                [x, y, z-press_depth, xr, yr, zr+twist_z],
            ],
        ])
    
    def execute_trajectory(self, trajectory_ix):
        for i in range(self.trajectories_np[trajectory_ix].shape[0]):
            self.run_point(
                self.dMovL(*self.trajectories_np[trajectory_ix][i].tolist(), coordinateMode=0, speed=2, v=2)
            )

    def run_home_pos(self):
        self.run_point(self.d.MovJ(*self.home_pos_np.tolist(), coordinateMode=0, speed=2, v=2))

if __name__ == '__main__':
    s = SystemId()
    IPython.embed()

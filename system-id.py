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
        self.print_force = False

        self.home_pos_np = np.array([-260, -25, 82, 180, 0, 0])
        self.set_trajectories()

        print(self.d.RequestControl())
        print(self.d.EnableRobot())
        print(self.d.SetCollisionLevel(0))
        print(self.d.SetBackDistance(0))

        threading.Thread(target=self.get_feed_small, daemon=True).start()
    
    def parse_result_id(self, response):
        if "Not Tcp" in response:
            print("Control Mode Is Not Tcp")
            return [1]
        return [int(num) for num in re.findall(r'-?\d+', response)] or [2]

    def parse_pose(self, pose_str):
        pose_part = pose_str.split('{')[1].split('}')[0]
        return [float(x) for x in pose_part.split(',')]

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
            if self.print_force:
                print(feedback['TCPForce'])
                
    
    def run_point(self, response):
        command_id = self.parse_result_id(response)[1]
        while True:
            if self.feedback['RobotMode'] == 5 and self.feedback['CurrentCommandId'] == command_id:
                break
            sleep(0.1)
    
    def run_point_servo(self, dx, v):
        if abs(dx) < 1e-6:
            print(f'dx cannot be 0')
        pose = self.parse_pose(self.d.GetPose())
        init_x = pose[0]
        target_x = init_x + dx
        while True:
            direction = np.sign(target_x - pose[0])
            pose[0] += direction * v * 10 * 0.030
            # pose[2] += (v / 5) * 10 * 0.030
            self.d.ServoP(*pose)
            sleep(0.030)
            pose = self.parse_pose(self.d.GetPose())
            if dx > 0:
                succ = pose[0] >= target_x
            else:
                succ = pose[0] <= target_x
            if succ:
                break
    
    def set_trajectories(self):
        x, y, z, xr, yr, zr = self.home_pos_np
        press_depth = 4
        slide_length = 50
        twist_x = 20
        twist_z = 20
        self.trajectories_np = np.array([
            [
                [x, y, z, xr, yr, zr],
                [x, y, z, xr, yr, zr],
                [x, y, z-press_depth, xr, yr, zr],
            ],
            [
                [x, y, z, xr, yr, zr],
                [x, y, z-press_depth, xr, yr, zr],
                [x-slide_length, y, z-press_depth+2, xr, yr, zr],
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
    
    def execute_trajectory(self, trajectory_ix, v):
        for i in range(self.trajectories_np[trajectory_ix].shape[0]):
            self.run_point(
                self.d.MovL(*self.trajectories_np[trajectory_ix][i].tolist(), coordinateMode=0, speed=v)
            )
            print(f'target {i} reached!')
        sleep(0.5)
        self.run_home_pos()

    def run_home_pos(self, v):
        self.run_point(self.d.MovJ(*self.home_pos_np.tolist(), coordinateMode=0, v=v))
        print('home position reached!')
        print()
    
    def run_upright_pos(self, v):
        self.run_point(self.d.MovJ([0] * 6, coordinateMode=1, v=v))
        print('home position reached!')
        print()

if __name__ == '__main__':
    s = SystemId()
    IPython.embed()
    # try:
    #     while True:
    #         sleep(1)  # Sleep to prevent high CPU usage
    # except KeyboardInterrupt:
    #     print("\nShutting down gracefully...")

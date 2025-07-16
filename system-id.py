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
        self.home_pos_np = np.array([-260, -25, 82, 180, 0, 0])

        print(self.d.RequestControl())
        print(self.d.EnableRobot())
        print(self.d.SetCollisionLevel(0))
        print(self.d.SetBackDistance(0))
        print(self.d.Tool(2))

        threading.Thread(target=self.get_feed_small, daemon=True).start()

    def parse_pose(self, pose_str):
        pose_part = pose_str.split('{')[1].split('}')[0]
        return [float(x) for x in pose_part.split(',')]
    
    def parse_command_id(self, pose_str):
        pose_part = pose_str.split('{')[1].split('}')[0]
        if len(pose_part) > 0:
            return int(pose_part)
        else:
            return -1

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
        command_id = self.parse_command_id(response)
        while True:
            if command_id == -1 or (self.feedback['RobotMode'] == 5 and self.feedback['CurrentCommandId'] == command_id):
                break
            sleep(0.1)
    
    def set_trajectories(self):
        self.home_pos_np = np.array(self.parse_pose(self.d.GetPose()))
        x, y, z, xr, yr, zr = self.home_pos_np
        press_depth_0 = 10
        press_depth_1 = 4
        press_depth_2 = 6
        press_depth_3 = 8
        slide_length = 50
        twist_x = 30
        twist_z = 45
        self.trajectories_np = np.array([
            [
                [x, y, z, xr, yr, zr],
                [x, y, z, xr, yr, zr],
                [x, y, z-press_depth_0, xr, yr, zr],
                [x, y, z, xr, yr, zr],
            ],
            [
                [x, y, z, xr, yr, zr],
                [x, y, z, xr, yr, zr],
                [x, y, z-press_depth_1, xr, yr, zr],
                [x-slide_length, y, z-press_depth_1, xr, yr, zr],
            ],
            [
                [x, y, z, xr, yr, zr],
                [x, y, z, xr, yr, zr],
                [x, y, z-press_depth_2, xr, yr, zr],
                [x, y, z-press_depth_2, xr+twist_x, yr, zr],
            ],
            [
                [x, y, z, xr, yr, zr],
                [x, y, z-press_depth_3, xr, yr, zr],
                [x, y, z-press_depth_3, xr, yr, zr+twist_z],
                [x, y, z, xr, yr, zr+twist_z],
            ],
        ])
    
    def run_point_servo(self, target_pose, v_pos, v_ori, Kp, threshold_pos, threshold_ori):
        target_pose = np.array(target_pose)
        t = 0
        while True:
            pose = np.array(self.parse_pose(self.d.GetPose()))
            error_pos = target_pose[:3] - pose[:3]
            error_ori = ((target_pose[3:] - pose[3:] + 180) % 360) - 180
            error = np.concatenate([error_pos, error_ori])
            if np.all(np.abs(error_pos) < threshold_pos) and np.all(np.abs(error_ori) < threshold_ori):
                break
            vel = Kp * error
            vel[:3] = np.clip(vel[:3], -v_pos, v_pos)
            vel[3:] = np.clip(vel[3:], -v_ori, v_ori)
            next_pose = pose + vel * 0.030
            self.d.ServoP(*next_pose.tolist())
            sleep(0.030)
            t += 1
    
    def execute_trajectory(self, trajectory_ix):
        for i in range(self.trajectories_np[trajectory_ix].shape[0]):
            self.run_point_servo(
                target_pose=self.trajectories_np[trajectory_ix][i],
                v_pos=10,
                v_ori=20,
                Kp=10.0,
                threshold_pos=0.1,
                threshold_ori=1.0,
            )
            print(f'target {i} reached!')
        sleep(0.5)

    def run_home_pos(self):
        self.run_point(self.d.MovJ(*self.home_pos_np.tolist(), coordinateMode=0, v=20))
        print('home position reached!')
        print()
    
    def run_upright_pos(self):
        self.run_point(self.d.MovJ(*np.zeros(shape=(6,)).tolist(), coordinateMode=1, v=20))
        print('home position reached!')
        print()

if __name__ == '__main__':
    s = SystemId()
    IPython.embed()


from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
import re
from time import sleep
import numpy as np
import IPython

import cv2
import time
import threading


class SystemId:
    def __init__(self):
        ip = '192.168.5.1'
        dashboard_port = 29_999
        feedback_port = 30_004
        self.d = DobotApiDashboard(ip, dashboard_port)
        self.f = DobotApiFeedBack(ip, feedback_port)
        self.feedback = dict()

        # self.home_pos_np = np.array([-260, -25, 38, 180, 0, 0])
        self.home_pos_np = np.array([-310.0000,20.0000,27.0000,180.0000,0.0000,0.0000], dtype=float)
        # self.home_pos_np = np.array([35.7724,274.4154,28.0001,-180.0000,0.0000,0.0000], dtype=float)
        self.set_trajectories()
        self.trajectories_initialised = False
        self.output = []

        print(self.d.RequestControl())
        print(self.d.EnableRobot())
        print(self.d.SetCollisionLevel(0))
        print(self.d.SetBackDistance(0))
        print(self.d.User(0))
        print(self.d.Tool(2))
        print(self.d.SetPayload(0.200, 0, 0, 0))

        threading.Thread(target=self.get_feed_small, daemon=True).start()

        self.set_up_video_capture()

    def set_up_video_capture(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, -64)
        self.cap.set(cv2.CAP_PROP_CONTRAST, 48)
        self.cap.set(cv2.CAP_PROP_SATURATION, 0)
        self.cap.set(cv2.CAP_PROP_HUE, 0)
        self.cap.set(cv2.CAP_PROP_SHARPNESS, 3)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 5000)
        self.cap.set(cv2.CAP_PROP_GAMMA, 100)
        self.cap.set(cv2.CAP_PROP_GAIN, 0)
        self.cap.set(cv2.CAP_PROP_BACKLIGHT, 0)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 100)
        print(f"Actual camera settings:")
        print(
            f"Resolution: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}"
        )
        print(f"FPS: {self.cap.get(cv2.CAP_PROP_FPS)}")
        print(f"Exposure: {self.cap.get(cv2.CAP_PROP_EXPOSURE)}")
        print(f"White Balance: {self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE)}")
        self.frame_lock = threading.Lock()
        self.current_frame_number = 0
        self.video_thread = None
        self.recording = False

    def start_video_recording(self):
        if self.video_thread and self.video_thread.is_alive():
            self.end_video_recording()
            
        if not self.cap.isOpened():
            self.set_up_video_capture()
            
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        output_path = f"robot_video_{timestamp}.avi"
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        
        self.current_frame_number = 0
        self.recording = True
        self.video_thread = threading.Thread(target=self.capture_video, daemon=True)
        self.video_thread.start()
    
    def end_video_recording(self):
        self.recording = False
        if self.video_thread:
            self.video_thread.join()
        if hasattr(self, 'out') and self.out:
            self.out.release()
        cv2.destroyAllWindows()

    def capture_video(self):
        while self.recording and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
            with self.frame_lock:
                self.current_frame_number += 1
            self.out.write(frame)
    
    def capture_photo(self):
        if not self.cap.isOpened():
            self.set_up_video_capture()
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to capture photo")
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        output_path = f"robot_photo_{timestamp}.jpg"
        cv2.imwrite(output_path, frame)

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
            # self.feedback['User'] = feedback['User']
            # self.feedback['Tool'] = feedback['Tool']
    
    def run_point(self, response):
        command_id = self.parse_command_id(response)
        while True:
            if command_id == -1 or (self.feedback['RobotMode'] == 5 and self.feedback['CurrentCommandId'] == command_id):
                break
            sleep(0.1)
    
    def set_trajectories(self):
        self.home_pos_np = np.array(self.parse_pose(self.d.GetPose()))
        x, y, z, xr, yr, zr = self.home_pos_np
        press_depth = 4
        r = 20
        d_short = 105 - 2*r
        d_long = 180 - 2*r
        d_single = r
        min_y = y - d_long

        # og phantom setup - see video on iPhone
        traj_1 = [
            [x, y, z, xr, yr, zr],
            [x, y, z-press_depth, xr, yr, zr],
        ]
        xy_dirs = [
            [1, 0],
            [0, -1],
            [-1, 0],
            [0, -1],
        ]
        xy_i = 0
        while True:
            a, b, c, d, e, f = traj_1[-1]
            if b < min_y - d_single:
                break
            x_dir, y_dir = xy_dirs[xy_i]
            xy_i += 1
            xy_i %= 4
            a2 = a + x_dir * d_short
            b2 = b + y_dir * d_single
            traj_1.append(
                [a2, b2, c, d, e, f]
            )
        traj_1 = np.array(traj_1, dtype=float)

        # phantom setup as in traj_1 but the sensor slides along the veins rather than across them
        max_x = x + d_short
        traj_2 = [
            [x, y, z, xr, yr, zr],
            [x, y, z-press_depth, xr, yr, zr],
        ]
        xy_dirs = [
            [0, -1],
            [1, 0],
            [0, 1],
            [1, 0],
        ]
        xy_i = 0
        while True:
            a, b, c, d, e, f = traj_2[-1]
            if a > max_x + d_single:
                break
            x_dir, y_dir = xy_dirs[xy_i]
            xy_i += 1
            xy_i %= 4
            a2 = a + x_dir * d_single
            b2 = b + y_dir * d_long
            traj_2.append(
                [a2, b2, c, d, e, f]
            )
        traj_2 = np.array(traj_2, dtype=float)

        min_x = x - d_long
        # phantom as in traj_1 but transposed
        traj_3 = [
            [x, y, z, xr, yr, zr],
            [x, y, z-press_depth, xr, yr, zr],
        ]
        xy_dirs = [
            [0, 1],
            [-1, 0],
            [0, -1],
            [-1, 0]
        ]
        xy_i = 0
        while True:
            a, b, c, d, e, f = traj_3[-1]
            if a < min_x - d_single:
                break
            x_dir, y_dir = xy_dirs[xy_i]
            xy_i += 1
            xy_i %= 4
            a2 = a + x_dir * d_single
            b2 = b + y_dir * d_short
            traj_3.append(
                [a2, b2, c, d, e, f]
            )
        traj_3 = np.array(traj_3, dtype=float)

        self.trajectories = [
            traj_1,
            traj_2,
            traj_3
        ]
        self.trajectories_initialised = True
    
    def set_trajectories_for_photo(self, press_depth=1, angle=10, slide_length=50):
        self.home_pos_np = np.array(self.parse_pose(self.d.GetPose()))
        x, y, z, xr, yr, zr = self.home_pos_np
        press = [
            [x, y, z, xr, yr, zr],
            [x, y, z-press_depth, xr, yr, zr],
        ]
        twist_z = [
            [x, y, z, xr, yr, zr],
            [x, y, z-press_depth, xr, yr, zr],
            [x, y, z-press_depth, xr, yr, zr+angle],
        ]
        twist_x = [
            [x, y, z, xr, yr, zr],
            [x, y, z-press_depth, xr, yr, zr],
            [x, y, z-press_depth, xr+angle, yr, zr],
        ]
        slide = [
            [x, y, z, xr, yr, zr],
            [x, y, z-press_depth, xr, yr, zr],
            [x+slide_length, y, z-press_depth, xr, yr, zr],
        ]
        self.trajectories = [
            press,
            twist_z,
            twist_x,
            slide,
        ]
        self.trajectories_initialised = True
    
    def run_point_servo(self, target_pose, v_pos, v_ori, Kp, threshold_pos, threshold_ori, downward_motion=False):
        target_pose = np.array(target_pose)
        t = 0
        while True:
            pose = np.array(self.parse_pose(self.d.GetPose()))
            # with self.frame_lock:
            #     data = [self.current_frame_number, *pose]
            #     self.output.append(
            #         data
            #     )

            error_pos = target_pose[:3] - pose[:3]
            error_ori = ((target_pose[3:] - pose[3:] + 180) % 360) - 180
            error = np.concatenate([error_pos, error_ori])
            if np.all(np.abs(error_pos) < threshold_pos) and np.all(np.abs(error_ori) < threshold_ori):
                break
            vel = Kp * error
            if downward_motion:
                vel[2] -= 100
            vel[:3] = np.clip(vel[:3], -v_pos, v_pos)
            vel[3:] = np.clip(vel[3:], -v_ori, v_ori)
            next_pose = pose + vel * 0.030
            self.d.ServoP(*next_pose.tolist())
            sleep(0.030)
            t += 1
    
    def execute_trajectory(self, trajectory_ix, downward_motion=False, Kp=20.0, v_pos=12.5, v_ori=90):
        if self.trajectories_initialised:
            self.output = []
            # self.start_video_recording()
            # sleep(2)
            for i in range(len(self.trajectories[trajectory_ix])):
                self.run_point_servo(
                    target_pose=self.trajectories[trajectory_ix][i],
                    v_pos=v_pos,
                    v_ori=v_ori,
                    Kp=Kp,
                    threshold_pos=0.1,
                    threshold_ori=1.0,
                    downward_motion=downward_motion
                )
            self.capture_photo()
            sleep(1)
            # sleep(2)
            # self.end_video_recording()
            output = np.array(self.output)
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            path = f'output_{timestamp}.npz'
            np.savez(
                path,
                output=output,
            )
        else:
            print("you need to set_trajectories")

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


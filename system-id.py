from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
import re
from time import sleep
import numpy as np
import IPython
from pathlib import Path

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

        # home_pos_og = np.array([-260, -25, 38, 180, 0, 0])
        # home_pos_near_window = np.array([35.7724,274.4154,28.0001,-180.0000,0.0000,0.0000], dtype=float)
        home_pose_endgame = np.array([-310.0000,20.0000,60.0000,180.0000,0.0000,0.0000])
        phantom_vertices = np.array([
            [-245.0000,46.0000,30.0000,180.0000,0.0000,0.0000],
            [-245.0000,-54.0000,30.0000,180.0000,0.0000,0.0000],
            [-425.0000,-54.0000,30.0000,180.0000,0.0000,0.0000],
            [-425.0000,46.0000,30.0000,180.0000,0.0000,0.0000],
        ], dtype=float)
        phantom_upper_surface_z = 20.5000
        self.dx = 180
        self.dy = 100
        self.r = 20
        self.ori = np.array([180, 0, 0], dtype=float)
        self.home_pose = home_pose_endgame
        self.set_trajectories()
        self.trajectories_initialised = False
        self.output = []
        self.calibrated = False
        self.video_capture_set_up = False

        print(self.d.RequestControl())
        print(self.d.EnableRobot())
        print(self.d.SetCollisionLevel(0))
        print(self.d.SetBackDistance(0))
        print(self.d.User(0))
        print(self.d.Tool(2))
        print(self.d.SetPayload(0.200, 0, 0, 0))

        # threading.Thread(target=self.get_feed_small, daemon=True).start()

        # self.set_up_video_capture()

    def set_up_video_capture(self, exposure):
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, -64)
        self.cap.set(cv2.CAP_PROP_CONTRAST, 48)
        self.cap.set(cv2.CAP_PROP_SATURATION, 0)
        self.cap.set(cv2.CAP_PROP_HUE, 0)
        self.cap.set(cv2.CAP_PROP_SHARPNESS, 6)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 5000)
        self.cap.set(cv2.CAP_PROP_GAMMA, 100)
        self.cap.set(cv2.CAP_PROP_GAIN, 0)
        self.cap.set(cv2.CAP_PROP_BACKLIGHT, 0)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, int(exposure))
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
        self.video_capture_set_up = True

    def start_video_recording(self):
        if self.video_thread and self.video_thread.is_alive():
            self.end_video_recording()
            
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        output_path = f'{self.path}.avi'
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
    
    def get_capture_file_path_extensionless(self):
        timestamp = time.strftime("%Y-%m-%d-%H-%M-%S")
        traj_name = self.trajectory_names[self.trajectory_ix]
        dir = "domain_adaptation"
        return f"{dir}/{traj_name}_press_depth={self.press_depth}_angle={self.angle}_slide_length={self.slide_length}_timestamp={timestamp}"
    
    def capture_photo(self):
        for _ in range(30):
            self.cap.read()
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to capture photo")
        output_path = self.get_capture_file_path_extensionless()
        output_path = f'{output_path}.jpg'
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
        self.home_pose = np.array(self.parse_pose(self.d.GetPose()))
        x, y, z, xr, yr, zr = self.home_pose
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
        self.press_depth = press_depth
        self.angle = angle
        self.slide_length = slide_length
        self.home_pose = np.array(self.parse_pose(self.d.GetPose()))
        x, y, z, xr, yr, zr = self.home_pose
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
            [x, y-slide_length, z-press_depth, xr, yr, zr],
        ]
        self.trajectories = [
            press,
            twist_z,
            twist_x,
            slide,
        ]
        self.trajectory_names = [
            'press',
            'twist_z',
            'twist_x',
            'slide',
        ]
        self.trajectories_initialised = True

    def calibrate(self, x, y, z):
        self.vx = x
        self.vy = y
        self.vz = z
        self.calibrated = True

    def set_trajectories_for_endgame(self):
        if not self.calibrated:
            print("you need to calibrate first")
            return
        x0 = self.vx
        y0 = self.vy
        z0 = self.vz
        x1 = x0-self.dx
        y1 = y0-self.dy
        z1 = z0-3
        k = 5
        xs = np.linspace(x0-self.r, x1+self.r, k, endpoint=True)
        ys = np.array([y0+self.r, y1-self.r], dtype=float)
        trajectories = []
        metadata_all = []
        for i in range(xs.shape[0]):
            x = xs[i]
            trajectory = [
                [x, ys[0], z0],
                [x, ys[0], z1],
                [x, ys[1], z1],
                [x, ys[1], z0],
            ]
            metadata = [
                i, 0, self.vx, self.vy, self.vz,
            ]
            trajectories.append(trajectory)
            metadata_all.append(metadata)
            trajectory = [
                [x, ys[1], z0],
                [x, ys[1], z1],
                [x, ys[0], z1],
                [x, ys[0], z0],
            ]
            metadata = [
                i, 1, self.vx, self.vy, self.vz,
            ]
            trajectories.append(trajectory)
            metadata_all.append(metadata)
        trajectories = np.array(trajectories, dtype=float)
        self.metadata = np.array(metadata_all, dtype=float)

        orientation_broadcast = np.broadcast_to(self.ori, trajectories.shape)
        self.trajectories = np.concatenate([trajectories, orientation_broadcast], axis=-1)

        self.trajectories_initialised = True
    
    def run_point_servo(self, target_pose, v_pos, v_ori, Kp, threshold_pos, threshold_ori, downward_motion=False):
        target_pose = np.array(target_pose)
        t = 0
        while True:
            pose = np.array(self.parse_pose(self.d.GetPose()))
            with self.frame_lock:
                data = [self.current_frame_number, *pose]
                self.output.append(
                    data
                )

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
    
    def execute_trajectory(self, trajectory_ix, downward_motion=False, Kp=20.0, v_pos=20, v_ori=90, prep_time=5):
        if self.trajectories_initialised:
            self.output = []
            self.trajectory_ix = trajectory_ix
            if self.trajectory_names[self.trajectory_ix] == 'slide':
                sleep(prep_time)
                self.start_video_recording()
                sleep(1)
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
            if self.trajectory_names[self.trajectory_ix] == 'slide':
                self.end_video_recording()
            else:
                sleep(0.5)
                self.capture_photo()
                sleep(0.5)
            s.run_home_pos()
            # output = np.array(self.output)
            # timestamp = time.strftime("%Y%m%d-%H%M%S")
            # path = f'output_{timestamp}.npz'
            # np.savez(
            #     path,
            #     output=output,
            # )
        else:
            print("you need to set_trajectories")
    
    def execute_endgame_trajectories(self, downward_motion=False, Kp=20.0, v_pos=25, v_ori=90, prep_time=5):
        if (
            not self.trajectories_initialised or 
            not self.calibrate or
            not self.video_capture_set_up
        ):
            print("you need to set up video capture, calibrate and set trajectories")
            return

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        dir_path = f'endgame/{timestamp}'
        Path(dir_path).mkdir(parents=True, exist_ok=True)
        sleep(prep_time)
        for i in range(self.trajectories.shape[0]):
            metadata = self.metadata[i]
            a, b = metadata[:2]
            self.path = f'{dir_path}/metadata_{a}_{b}'

            self.output = []
            self.trajectory_ix = i
            self.start_video_recording()
            sleep(1)
            for j in range(self.trajectories.shape[1]):
                self.run_point_servo(
                    target_pose=self.trajectories[i][j],
                    v_pos=v_pos,
                    v_ori=v_ori,
                    Kp=Kp,
                    threshold_pos=0.1,
                    threshold_ori=1.0,
                    downward_motion=downward_motion
                )
            self.end_video_recording()
            output = np.array(self.output)
            path = f'{self.path}.npz'
            np.savez(
                path,
                output=output,
                metadata=metadata,
            )

    def run_home_pos(self):
        self.run_point(self.d.MovJ(*self.home_pose.tolist(), coordinateMode=0, v=20))
        print('home position reached!')
        print()
    
    def run_upright_pos(self):
        self.run_point(self.d.MovJ(*np.zeros(shape=(6,)).tolist(), coordinateMode=1, v=20))
        print('home position reached!')
        print()

if __name__ == '__main__':
    s = SystemId()
    IPython.embed()


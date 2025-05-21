from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
import re
import pickle
import IPython
from time import sleep
from datetime import datetime
import time

class SystemId:
    def __init__(self):
        ip = '192.168.5.1'
        dashboard_port = 29_999
        feedback_port = 30_004
        self.d = DobotApiDashboard(ip, dashboard_port)
        self.f = DobotApiFeedBack(ip, feedback_port)
        self.command_path_mapping = dict()
        self.feedback = dict()
        self.speed = 1
        self.v = 2

        print(self.d.RequestControl())
        print(self.d.EnableRobot())

        threading.Thread(target=self.get_feed, daemon=True).start()
    
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
    
    def get_feed(self):
        buffer_size = 100
        file_path = './feedback_all.pkl'
        data_buffer = []
        with open(file_path, 'wb') as f:
            pickle.dump(data_buffer, f)
        try:
            while True:
                feedback = self.f.feedBackData()
                now = datetime.now()
                if feedback is None:
                    continue
                if hex((feedback['TestValue'][0])) != '0x123456789abcdef':
                    raise Exception("TestValue not as expected")
                self.feedback['RobotMode'] = feedback['RobotMode'][0]
                self.feedback['CurrentCommandId'] = feedback['CurrentCommandId'][0]
                relevant_fields = [
                    'TargetQuaternion',
                    'ActualQuaternion',
                    'QTarget',
                    'QActual',
                    'QDTarget',
                    'QDActual',
                    'CurrentCommandId',
                    'RobotMode',
                ]
                feedback_filetred = dict()
                for field in relevant_fields:
                    feedback_filetred[field] = feedback[field][0]
                feedback_filetred['TimeStamp'] = now.isoformat(timespec='milliseconds')
                data_buffer.append(feedback_filetred)
                if len(data_buffer) >= buffer_size:
                    with open(file_path, 'ab') as f:
                        pickle.dump(data_buffer, f)
                    data_buffer = []
        except KeyboardInterrupt:
            if data_buffer:
                with open(file_path, 'ab') as f:
                    pickle.dump(data_buffer, f)
    
    def save_command_id(self, response, filter_in, path_id):
        command_id = self.parse_result_id(response)[1]
        if filter_in:
            self.command_path_mapping[command_id] = path_id
    
    def run_point(self, response, current_speed=1, name="n/a"):
        start_time = time.perf_counter()

        command_id = self.parse_result_id(response)[1]
        while True:
            if self.feedback['RobotMode'] == 5 and self.feedback['CurrentCommandId'] == command_id:
                break
            sleep(0.1)
        
        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        target_time = 10
        if name != "n/a":
            print(f"Run to: {name}")
            print(f"Elapsed time: {elapsed_time:.1f} s; target time: {target_time:.1f} s")
            # speed = distance / time
            # target_speed = foo / target_time = foo / elapsed_time * ratio = current_speed * ratio
            # target_time * ratio = elapsed_time
            # ratio = elapsed_time / target_time

            speed_ratio = elapsed_time / target_time
            target_speed = current_speed * speed_ratio
            print(f"Current speed: {current_speed:.1f}; target speed: {target_speed:.1f}")
            print()

        return response

    def home_pos(self):
        self.save_command_id(self.run_point(self.d.MovJ(-310, -25, 63, 180, 0, 0, coordinateMode=0, v=self.speed)), False, -1)
    
    def press(self, path_id):
        self.save_command_id(self.run_point(self.d.MovJ(-310, -25, 63, 180, 0, 0, coordinateMode=0, v=self.speed)), False, path_id)
        self.save_command_id(self.run_point(self.d.MovL(-310, -25, 53, 180, 0, 0, coordinateMode=0, speed=self.speed), current_speed=self.speed, name="press"), True, path_id)

    def press_slide(self):
        path_id = 0
        self.press(path_id)
        self.save_command_id(self.run_point(self.d.MovL(-320, -25, 53, 180, 0, 0, coordinateMode=0, speed=self.speed), current_speed=self.speed, name="slide"), True, path_id)

    def press_twist_x(self):
        path_id = 1
        self.press(path_id)
        self.save_command_id(self.run_point(self.d.MovL(-310, -25, 53, 200, 0, 0, coordinateMode=0, v=self.v), current_speed=self.v, name="twist-x"), True, path_id)

    def press_twist_z(self):
        path_id = 1
        self.press(path_id)
        self.save_command_id(self.run_point(self.d.MovL(-310, -25, 53, 180, 0, 20, coordinateMode=0, v=self.v), current_speed=self.v, name="twist-z"), True, path_id)
    
    def go(self):
        n = 3
        paths = [
            self.press_slide,
            self.press_twist_x,
            self.press_twist_z,
        ]
        for i in range(n):
            for path in paths:
                path()
        self.home_pos()

    def save_to_file(self):
        with open('./command_path_mapping.pkl', 'wb') as file:
            pickle.dump(self.command_path_mapping, file)

if __name__ == '__main__':
    system_id = SystemId()
    system_id.go()
    system_id.save_to_file()

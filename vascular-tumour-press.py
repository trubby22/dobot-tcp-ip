from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
import re
from time import sleep

class Controller:
    def __init__(self):
        ip = '192.168.5.1'
        dashboard_port = 29_999
        feedback_port = 30_004
        self.d = DobotApiDashboard(ip, dashboard_port)
        self.f = DobotApiFeedBack(ip, feedback_port)
        self.robot_mode = None
        self.current_command_id = None

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
            self.robot_mode = feedback['RobotMode'][0]
            self.current_command_id = feedback['CurrentCommandId'][0]
    
    def run_point(self, response):
        command_id = self.parse_result_id(response)[1]
        while True:
            if self.robot_mode == 5 and self.current_command_id == command_id:
                break
            sleep(0.1)
        return response
    
    def go(self):
        trajectory_xy = [
            [-400, 150],
            [-400+150, 150],
            [-400+150, 150-14],
            [-400, 150-14],
            [-400, 150-28],
            [-400+150, 150-28],
            [-400+150, 150-42],
            [-400, 150-42],
            [-400, 150-56],
            [-400+150, 150-56],
        ]
        for x, y in trajectory_xy:
            self.run_point(self.d.MovL(x, y, 77, 180, 0, 0, coordinateMode=0, v=2))

if __name__ == '__main__':
    controller = Controller()
    controller.go()

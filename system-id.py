from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
import re
import pickle

class SystemId:
    def __init__(self):
        ip = '192.168.5.1'
        dashboard_port = 29_999
        feedback_port = 30_004
        self.d = DobotApiDashboard(ip, dashboard_port)
        self.f = DobotApiFeedBack(ip, feedback_port)
        self.command_path_mapping = dict()

        response = self.d.RequestControl()
        print(response)
        response = self.d.EnableRobot()
        print(response)

        threading.Thread(target=self.get_feed, daemon=True).start()
    
    def parse_result_id(self, response):
        if "Not Tcp" in response:
            print("Control Mode Is Not Tcp")
            return [1]
        return [int(num) for num in re.findall(r'-?\d+', response)] or [2]

    def __del__(self):
        del self.d
        del self.f
    
    def get_feed(self):
        buffer_size = 100
        file_path = './feedback_all.pkl'
        data_buffer = []
        try:
            while True:
                feedback = self.f.feedBackData()
                if feedback is None:
                    continue
                if hex((feedback['test_value'][0])) != '0x123456789abcdef':
                    continue
                relevant_fields = [
                    'SixForceValue',
                    'ActualTCPForce',
                    'TimeStamp',
                    'TargetQuaternion',
                    'ActualQuaternion',
                    'QTarget',
                    'QActual',
                    'QDTarget',
                    'QDActual',
                    'currentcommandid',
                    'robot_mode',
                ]
                feedback_filetred = dict()
                for field in relevant_fields:
                    if field in feedback:
                        feedback_filetred[field] = feedback[field]
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
    
    def press(self, path_id):
        self.save_command_id(self.d.MovL(100, 0, 100, 0, 0, 0, coordinateMode=0), False, path_id)
        self.save_command_id(self.d.MovL(100, 0, 5, 0, 0, 0, coordinateMode=0), False, path_id)
        self.save_command_id(self.d.MovL(100, 0, -5, 0, 0, 0, coordinateMode=0, speed=1), True, path_id)

    def press_slide(self):
        path_id = 0
        self.press(path_id)
        self.save_command_id(self.d.MovL(110, 0, -5, 0, 0, 0, coordinateMode=0, speed=1), True, path_id)

    def press_twist_x(self):
        path_id = 1
        self.press(path_id)
        self.save_command_id(self.d.MovL(110, 0, -5, 20, 0, 0, coordinateMode=0, speed=1), True, path_id)

    def press_twist_z(self):
        path_id = 1
        self.press(path_id)
        self.save_command_id(self.d.MovL(110, 0, -5, 0, 0, 20, coordinateMode=0, speed=1), True, path_id)
    
    def go(self):
        n = 10
        paths = [
            self.press_slide,
            self.press_twist_x,
            self.press_twist_z,
        ]
        for path in paths:
            for i in range(n):
                path()

    def save_to_file(self):
        with open('./command_path_mapping.pkl', 'wb') as file:
            pickle.dump(self.command_path_mapping, file)

if __name__ == '__main__':
    system_id = SystemId()
    system_id.go()
    system_id.save_to_file()

from dobot_api import DobotApiFeedBack,DobotApiDashboard
import IPython

ip = '192.168.5.1'
dashboard_port = 29_999
feedback_port = 30_004

d = DobotApiDashboard(ip, dashboard_port)
f = DobotApiFeedBack(ip, feedback_port)

response = d.RequestControl()
print(response)

IPython.embed()

del d
del f
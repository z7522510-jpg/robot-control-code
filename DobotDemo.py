from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
from time import sleep
import re
import math

class DobotDemo:
    def __init__(self, ip):
        self.ip = ip
        self.dashboardPort = 29999
        self.feedPortFour = 30004
        self.dashboard = None
        self.feedInfo = []
        self.__globalLockValue = threading.Lock()
        
        class item:
            def __init__(self):
                self.robotMode = -1     #
                self.robotCurrentCommandID = 0
                self.MessageSize = -1
                self.DigitalInputs =-1
                self.DigitalOutputs = -1
                self.robotCurrentCommandID = -1
                # 自定义添加所需反馈数据

        self.feedData = item()  # 定义结构对象

    def start(self):
        # 启动机器人并使能
        self.dashboard = DobotApiDashboard(self.ip, self.dashboardPort)
        self.feedFour = DobotApiFeedBack(self.ip, self.feedPortFour)
        enable_result = self.dashboard.EnableRobot()
        print("EnableRobot:", enable_result)
        if self.parseResultId(enable_result)[0] != 0:
            print("使能失败: 请检查机器人是否在 TCP/IP 模式、是否有报警/急停、以及 29999 端口连接")
            return
        print("使能成功")

        # 启动状态反馈线程
        speed_ratio = 20
        speed_commands = [
            ("SpeedFactor", self.dashboard.SpeedFactor(speed_ratio)),
            ("VelJ", self.dashboard.VelJ(speed_ratio)),
            ("AccJ", self.dashboard.AccJ(speed_ratio)),
        ]
        for name, result in speed_commands:
            print(f"{name}:", result)
            if self.parseResultId(result)[0] != 0:
                print(f"{name} set failed, stop demo")
                return
        print("Speed set to", speed_ratio, "%")

        confirm = input("Input 1 to start motion, other input to exit: ").strip()
        if confirm != "1":
            print("Motion canceled")
            return

        feed_thread = threading.Thread(
            target=self.GetFeed)  # 机器状态反馈线程
        feed_thread.daemon = True
        feed_thread.start()

        sleep(1)

        center = self.GetCurrentPose()
        circle_points = self.GenerateXZCirclePoints(center, radius=50, point_count=36)

        print("圆心:", center)
        print("半径: 50 mm")
        print("XZ 圆最低 Z:", center[2] - 50, "最高 Z:", center[2] + 50)

        # 先从圆心移动到圆周起点，再沿 XZ 平面走一圈并回到起点闭合圆
        self.RunPoint(circle_points[0])
        for point in circle_points[1:]:
            self.RunPoint(point)
        self.RunPoint(circle_points[0])

    def GetFeed(self):
        # 获取机器人状态
        while True:
            feedInfo = self.feedFour.feedBackData()
            with self.__globalLockValue:
                if feedInfo is not None:   
                    if hex((feedInfo['TestValue'][0])) == '0x123456789abcdef':
                        # 基础字段
                        self.feedData.MessageSize = feedInfo['len'][0]
                        self.feedData.robotMode = feedInfo['RobotMode'][0]
                        self.feedData.DigitalInputs = feedInfo['DigitalInputs'][0]
                        self.feedData.DigitalOutputs = feedInfo['DigitalOutputs'][0]
                        self.feedData.robotCurrentCommandID = feedInfo['CurrentCommandId'][0]
                        # 自定义添加所需反馈数据
                        '''
                        self.feedData.DigitalOutputs = int(feedInfo['DigitalOutputs'][0])
                        self.feedData.RobotMode = int(feedInfo['RobotMode'][0])
                        self.feedData.TimeStamp = int(feedInfo['TimeStamp'][0])
                        '''

    def RunPoint(self, point_list):
        # 走点指令
        recvmovemess = self.dashboard.MovJ(*point_list, 0)
        print("MovJ:", recvmovemess)
        print(self.parseResultId(recvmovemess))
        currentCommandID = self.parseResultId(recvmovemess)[1]
        print("指令 ID:", currentCommandID)
        #sleep(0.02)
        while True:  #完成判断循环

            print(self.feedData.robotMode)
            if self.feedData.robotMode == 5 and self.feedData.robotCurrentCommandID == currentCommandID:
                print("运动结束")
                break
            sleep(0.1)

    def GenerateXZCirclePoints(self, center, radius=50, point_count=36):
        points = []
        for index in range(point_count):
            theta = 2 * math.pi * index / point_count
            point = [
                center[0] + radius * math.cos(theta),
                center[1],
                center[2] + radius * math.sin(theta),
                center[3],
                center[4],
                center[5],
            ]
            points.append(point)
        return points

    def GetCurrentPose(self):
        # GetPose 返回格式中包含错误码和 6 个位姿值，这里取 X/Y/Z/Rx/Ry/Rz
        recv = self.dashboard.GetPose()
        print("GetPose:", recv)
        values = [float(num) for num in re.findall(r'-?\d+(?:\.\d+)?', recv)]
        if len(values) >= 7 and int(values[0]) == 0:
            return values[1:7]
        if len(values) >= 6:
            return values[:6]
        raise ValueError("GetPose failed: " + recv)

    def parseResultId(self, valueRecv):
        # 解析返回值，确保机器人在 TCP 控制模式
        if "Not Tcp" in valueRecv:
            print("Control Mode Is Not Tcp")
            return [1]
        return [int(num) for num in re.findall(r'-?\d+', valueRecv)] or [2]

    def __del__(self):
        del self.dashboard
        del self.feedFour

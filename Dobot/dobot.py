try:
    from .dobot_api import DobotApiFeedBack, DobotApiDashboard
except ImportError:
    from dobot_api import DobotApiFeedBack, DobotApiDashboard
import threading
from time import sleep
import time
import math
import re

class Dobot:
    def __init__(self, ip):
        self.ip = ip
        self.dashboardPort = 29999
        self.feedPortFour = 30004
        self.dashboard = None
        self.feedFour = None
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

    def connect(self):
        self.dashboard = DobotApiDashboard(self.ip, self.dashboardPort)
        self.feedFour = DobotApiFeedBack(self.ip, self.feedPortFour)

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

    def RunPoint(self, point_list, cp=-1, wait=True):
        # 走点指令
        recvmovemess = self.dashboard.MovJ(*point_list, 0, cp=cp)
        print("MovJ:", recvmovemess)
        if wait:
            self.WaitCommandDone(recvmovemess)

    def MoveLinearPoint(self, point, speed_ratio):
        move_result = self.dashboard.MovL(*point, 0, v=speed_ratio)
        print("MovL:", move_result)
        if not self.WaitCommandDone(move_result):
            raise RuntimeError("MovL failed or timed out")
        return True

    def SetDigitalOutput(self, do_index, value):
        result = self.dashboard.DO(do_index, value)
        print(f"DO({do_index},{value}):", result)
        return result

    def SendDOPulse(self, do_index, pulse_seconds):
        do_on_result = self.SetDigitalOutput(do_index, 1)
        sleep(pulse_seconds)
        do_off_result = self.SetDigitalOutput(do_index, 0)
        return do_on_result, do_off_result

    def RunArc(self, mid_point, end_point, cp=-1, wait=True):
        # 圆弧指令：从当前位置出发，经过 mid_point，到达 end_point
        recvmovemess = self.dashboard.Arc(*mid_point, *end_point, 0, cp=cp)
        print("Arc:", recvmovemess)
        if wait:
            self.WaitCommandDone(recvmovemess)

    def WaitCommandDone(self, recvmovemess, timeout=30):
        result_ids = self.parseResultId(recvmovemess)
        print(result_ids)
        if len(result_ids) < 2 or result_ids[0] != 0:
            print("指令下发失败，跳过等待:", recvmovemess)
            return False

        currentCommandID = result_ids[1]
        print("指令 ID:", currentCommandID)
        start_time = time.perf_counter()
        last_print_time = start_time

        while True:
            now = time.perf_counter()
            if self.feedData.robotMode == 5 and self.feedData.robotCurrentCommandID >= currentCommandID:
                print("运动结束")
                return True

            if now - last_print_time >= 1:
                print(
                    "等待运动完成: "
                    f"mode={self.feedData.robotMode}, "
                    f"currentCommandID={self.feedData.robotCurrentCommandID}, "
                    f"targetCommandID={currentCommandID}"
                )
                last_print_time = now

            if now - start_time >= timeout:
                print(
                    "等待运动完成超时: "
                    f"mode={self.feedData.robotMode}, "
                    f"currentCommandID={self.feedData.robotCurrentCommandID}, "
                    f"targetCommandID={currentCommandID}"
                )
                return False

            sleep(0.1)

    def GenerateXZArcPoints(self, center, radius=100):
        return [
            [center[0] + radius, center[1], center[2], center[3], center[4], center[5]],
            [center[0], center[1], center[2] + radius, center[3], center[4], center[5]],
            [center[0] - radius, center[1], center[2], center[3], center[4], center[5]],
            [center[0], center[1], center[2] - radius, center[3], center[4], center[5]],
        ]

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

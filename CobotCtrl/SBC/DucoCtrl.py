from array import array
import sys
import time
import threading
import math

sys.path.append('DucoCobotAPI_py')

from DucoCobotAPI_py.DucoCobot import DucoCobot

class DucoCtrl():

    # 初始化用于控制心跳线程停止的变量
    stopheartthread = False

    def __init__(self, ipAddr, Port):
        self.ip = ipAddr
        self.port = Port
        self.DucoRobot = DucoCobot(ipAddr, Port)
        self.DucoRobot.open()
        pass

    def HeartThread(self):
        '''
        * Function:     hearthread_fun
        * Description:  用于执行心跳线程的任务
        * Inputs:       
        * Outputs:      
        * Returns:      
        * Notes:
        '''
        duco_heartbeat = DucoCobot(self.ip, self.port)
        duco_heartbeat.open()

        # 不断向DucoCobot发送心跳信号以保持连接
        # 当stopheartthread为True时结束循环并关闭连接
        while not self.stopheartthread:
            duco_heartbeat.rpc_heartbeat()
            time.sleep(1)
        duco_heartbeat.close()

    def MainThread(self):
        '''
            * Function:     thread_fun
            * Description:  用于执行主线程的任务
            * Inputs:       
            * Outputs:      
            * Returns:      
            * Notes:
        '''
        duco_cobot = DucoCobot(self.ip, self.port)
        duco_cobot.open()

        # 获取 DucoCobot 的当前状态，并输出到控制台
        while not self.stopheartthread:
            tcp_pose = []
            tcp_pose = duco_cobot.get_robot_state()
            print("state: ", tcp_pose)
            time.sleep(1)
        duco_cobot.close()

    def DucoMoveJ(self, q_near, vel, acc):
        self.DucoRobot.movej2(q_near,vel,acc,r=0,block=False)
        time.sleep(1)
        # 当法兰停止运动时，认为运动到位，跳出循环进入下一阶段
        while(True):
            isMoving = self.DucoRobot.robotmoving()
            if isMoving == False:
                return 1

    def DucoTcpMove(self, OffsetPos, vel, acc, ToolName):
        self.DucoRobot.tcp_move(OffsetPos, vel, acc,r=0,tool=ToolName,block=False)
        time.sleep(0.1)

        # 当法兰停止运动时，认为运动到位，跳出循环进入下一阶段
        while(True):
            # PosNow = self.GetDucoPos(0)
            FlangeSpeedList = self.DucoRobot.get_flange_speed()
            # print("当前法兰速度为: ", FlangeSpeed)
            FlangeSpeed = math.sqrt(FlangeSpeedList[0]**2 + FlangeSpeedList[1]**2+ FlangeSpeedList[2]**2)
            isMoving = self.DucoRobot.robotmoving()
            if isMoving == False:
                if FlangeSpeed <= 0.0001:
                    return 1
    def DucoMovel(self, TargetPos, vel, acc, q_near, ToolName):
        '''
            * Function:     DucoMovelTcp
            * Description:  控制机械臂末端从当前状态按照直线路径移动到目标状态
            * Inputs:       
                            TargetPos: 目标位姿 [x, y, z, rx, ry, rz]，位置单位: m，姿态范围[-2*pi, 2*pi]，单位rad
                            vel: 末端运动速度，范围[0, 5]，单位(m/s)
                            acc: 末端加速度，范围[0, 5]，单位(m/s²)
                            ToolName: 工具坐标系的名字
            * Outputs:        
            * Returns:      1: 到位完成
            * Notes:
        '''
        # heard_thd = threading.Thread(target=self.HeartThread)
        # heard_thd.start
        # 控制机械臂末端从当前状态按照直线路径移动到目标状态
        # p: 目标位姿 [x, y, z, rx, ry, rz]，位置单位: m，姿态范围[-2*pi, 2*pi]，单位rad
        # v: 末端速度，范围[0, 5]，单位(m/s)
        # a: 末端加速度，范围[0, 5]，单位(m/s²)
        # block: 是否阻塞
        self.DucoRobot.movel(p=TargetPos,v=vel,a=acc,block=False,r=0,q_near=q_near,tool=ToolName,wobj='default')
        time.sleep(0.1)

        # 当法兰停止运动时，认为运动到位，跳出循环进入下一阶段
        while(True):
            PosNow = self.GetDucoPos(1)
            dist = math.sqrt((PosNow[0] - TargetPos[0])**2 + (PosNow[1] - TargetPos[1])**2 + (PosNow[2] - TargetPos[2])**2)
            # print("当前距离终点距离: ",dist)
            DucoState = self.IsDucoMoving()
            # isMoving = self.DucoRobot.robotmoving()
            # if isMoving == False:
            if DucoState[1] == 0:
                if dist <= 0.0001:
                    return 1

    def DucoMoveL(self, TargetPos, vel, acc, q_near):
        '''
            * Function:     DucoMove
            * Description:  控制机械臂末端从当前状态按照直线路径移动到目标状态
            * Inputs:       
                            TargetPos: 目标位姿 [x, y, z, rx, ry, rz]，位置单位: m，姿态范围[-2*pi, 2*pi]，单位rad
                            vel: 末端运动速度，范围[0, 5]，单位(m/s)
                            acc: 末端加速度，范围[0, 5]，单位(m/s²)
            * Outputs:        
            * Returns:      1: 到位完成
            * Notes:
        '''
        # heard_thd = threading.Thread(target=self.HeartThread)
        # heard_thd.start
        # 控制机械臂末端从当前状态按照直线路径移动到目标状态
        # p: 目标位姿 [x, y, z, rx, ry, rz]，位置单位: m，姿态范围[-2*pi, 2*pi]，单位rad
        # v: 末端速度，范围[0, 5]，单位(m/s)
        # a: 末端加速度，范围[0, 5]，单位(m/s²)
        # block: 是否阻塞
        self.DucoRobot.movel(p=TargetPos,v=vel,a=acc,block=False,r=0,q_near=q_near,tool='default',wobj='default')
        time.sleep(1)

        # 当法兰停止运动时，认为运动到位，跳出循环进入下一阶段
        while(True):
            PosNow = self.GetDucoPos(0)
            dist = math.sqrt((PosNow[0] - TargetPos[0])**2 + (PosNow[1] - TargetPos[1])**2 + (PosNow[2] - TargetPos[2])**2)


            isMoving = self.DucoRobot.robotmoving()
            if isMoving == False:
                if dist <= 0.0001:
                    return 1

    def GetDucoPos(self,mod):
        '''
            * Function:     GetDucoPos
            * Description:  获取末端姿态
            * Inputs:       
                            mod: 
                                1: 返回工具坐标系末端姿态
                                其他: 返回末端法兰姿态
            * Outputs:        
            * Returns:      
            * Notes:
        '''

        if mod == 1:
            return self.DucoRobot.get_tcp_pose()
        else:        
            return self.DucoRobot.get_flange_pose()


    def GetQNear(self):
        return self.DucoRobot.get_actual_joints_position()
    
    def IsDucoMoving(self):
        DucoState = self.DucoRobot.get_robot_state()
        return DucoState
    
    def DucoStop(self):
        return self.DucoRobot.stop(block=False)
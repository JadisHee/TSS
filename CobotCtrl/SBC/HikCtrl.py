import math
import numpy as np
import cv2
import socket
from scipy.spatial.transform import Rotation

import time

class HikCtrl:

    msgStart = '123'
    def __init__(self,ip,port):
        self.ip = ip
        self.port = port
        pass

    def GetHikDPos(self, diameter):
        '''
            * Function:     GetHikDPos
            * Description:  获取识别后圆心在相机中心坐标系的坐标,单位:mm
            * Inputs:       diameter:
                                待识别物的实际测量直径
            * Outputs:      
            * Returns:      
                                DPos: 坐标 list[dx,dy]
                                    dx:float
                                    dy:float
                                0: 相机出错
            * Notes:
        '''
        result = self.GetDataFromHik(self.msgStart)
        if result != 0:
            # print ("相机反馈结果: ",result)
            DPos = self.GetDPosMillimeter(result[1] * 2, [result[2], result[3]], diameter)
            return DPos
        else:
            print("请检查相机设置 ! ! !")
            return 0

    def SetHikSwitchPlan(self, SwitchCode, PlanName):
        '''
        * Function:     SetHikSwitchPlan
        * Description:  控制海康相机切换方案
        * Inputs:
                            SwitchCode: 切换语句
                            PlanName:   待切换方案的名称
        * Outputs:      切换方案成功
        * Returns:      
        * Notes:
        '''
        # 创建客户端
        HikClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 链接服务端

    
        HikClient.connect((self.ip, self.port))
        # 整理发送数据
        msgSend = SwitchCode + ' ' + PlanName
        # print("发送给相机: ", msgSend)
        # 发送数据
        while 1:
            HikClient.send(msgSend.encode('utf-8'))
            data = HikClient.recv(1024)
            # data1 = data.decode('utf-8')
            # print(data1[0])
            data = str(data, 'utf-8')
            # print(data[0])
            if data == str('ok'):
                # print('成功切换至方案： ', PlanName)
                return 1
            else:
                # print('切换方案失败')
                return 0

    def GetDataFromHik(self, StartSignal):
        '''
            * Function:     GetDataFromHik
            * Description:  控制海康相机进行识别并获取检测结果
            * Inputs:
                                StartSignal:   开始信号
            * Outputs:      
            * Returns:      0: 超过3s未检测目标
                            data: 检测结果
            * Notes:        
        '''
        # 创建客户端
        HikClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 链接客户端
        HikClient.connect((self.ip, self.port))

        # 程序开始时间
        TimeStart = time.time()

        while 1:
            HikClient.send(StartSignal.encode('utf-8'))
            FeedBack = HikClient.recv(1024)
            FeedBackString = FeedBack.decode()
            if FeedBackString[0] == '1': 
                # print("检测成功! ! !")
                DataList = FeedBackString.split(";")[:-1]

                UsefulData = [int(DataList[0])]
                UsefulData.extend(float(i) for i in DataList[1:])
                return UsefulData
            else:
                TimeEnd = time.time()
                RunTime = TimeEnd - TimeStart
                if RunTime >= 3:
                    print("检测失败")
                    break   
            # time.sleep(0.1)
    
        return 0

    # def GetDataFromHik(self, StartMsg, Num):
    #     '''
    #     * Function:     GetDataFromHik
    #     * Description:  控制海康相机进行识别并获取检测结果
    #     * Inputs:
    #                         StartMsg:   切换语句
    #                         Num:        需要读取的数据个数，所有数据均需采用4.2长度设计，各数据含义需在SC MVS软件中自行注明
    #     * Outputs:      
    #     * Returns:      0: 超过3s未检测目标
    #                     data: 检测结果
    #     * Notes:        检测结果的数据
    #     '''
    #     # 创建客户端
    #     HikClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     # 链接客户端
    #     HikClient.connect((self.ip, self.port))
    #     # 读取客户端
    #     # Flag = True
        
    #     T1 = time.time()
    #     while 1:
    #         msgStart = StartMsg
    #         HikClient.send(msgStart.encode('utf-8'))
    #         data = HikClient.recv(1024)
    #         # data1 = data.decode('utf-8')
    #         # print(data1[0])
    #         data = str(data, 'utf-8')
    #         # print(data[0])
    #         T2 = time.time()
    #         if data[0] == str(1):
    #             print('收到智能相机数据')
    #             print(data)
    #             break
    #         if T2 - T1 >= 3:
    #             return 0
    #         # else:
    #         # sleep(0.1)
    #     Data = np.zeros((Num, 1))
    #     # print(Data)
    #     for i in range(Num):
    #         Data[i] = data[8 * i + 2+i:8 * i + 10+i]
    #     # np.array(Data)
    #     HikClient.close()
    #     return Data

    def Euler2RotMat(self,rx, ry, rz):
        """
        Convert Euler angles [Rx, Ry, Rz] to rotation matrix.

        Parameters:
            rx: float
                Rotation angle around X-axis in radians.
            ry: float
                Rotation angle around Y-axis in radians.
            rz: float
                Rotation angle around Z-axis in radians.

        Returns:
            rotation_matrix: numpy array
                3x3 rotation matrix.
        """
        # Calculate rotation matrices for each axis
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(rx), -np.sin(rx)],
                    [0, np.sin(rx), np.cos(rx)]])
        
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                    [0, 1, 0],
                    [-np.sin(ry), 0, np.cos(ry)]])
        
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                    [np.sin(rz), np.cos(rz), 0],
                    [0, 0, 1]])
        
        # Combine the rotation matrices
        rotation_matrix = np.dot(Rz, np.dot(Ry, Rx))
        
        return rotation_matrix

    def QuartToRpy(self,x,y,z,w):
        '''
        * Function:     quart_to_rpy
        * Description:  
        * Inputs:       
                            
        * Outputs:      无输出
        * Returns:      
                            
                            
        * Notes:
        '''
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return roll, pitch, yaw
    
    def GetRotVec2RotMat(self,RotVec):
        '''
        * Function:     GetRotVec2RotMat
        * Description:  将旋转向量转换为旋转矩阵
        * Inputs:       RotVec: 旋转向量
        * Outputs:      无输出
        * Returns:      旋转矩阵
        * Notes:
        '''
        RotMat = cv2.Rodrigues(RotVec)[0]
        return RotMat

    def PosTrans(self,PosVec,TransMat,DPos):
        RotMat = self.Euler2RotMat(PosVec[3],PosVec[4],PosVec[5])
        PosMat = np.array([[RotMat[0][0],RotMat[0][1],RotMat[0][2],PosVec[0]],
                           [RotMat[1][0],RotMat[1][1],RotMat[1][2],PosVec[1]],
                           [RotMat[2][0],RotMat[2][1],RotMat[2][2],PosVec[2]],
                           [0,0,0,1]])

        # 当前位置在工具坐标系下的姿态矩阵
        TransPos = np.dot(PosMat,TransMat)
        
        # 当前工具坐标系下的旋转矩阵
        TransPosRotMat = np.array([[TransPos[0][0],TransPos[0][1],TransPos[0][2]],
                                   [TransPos[1][0],TransPos[1][1],TransPos[1][2]],
                                   [TransPos[2][0],TransPos[2][1],TransPos[2][2]]])
        # 变换为欧拉角
        TransPosRot_ = Rotation.from_matrix(TransPosRotMat)
        TransPosRot = TransPosRot_.as_euler('xyz',degrees=False)
        TransPosVec = [TransPos[0][3],TransPos[1][3],TransPos[2][3],TransPosRot[0],TransPosRot[1],TransPosRot[2]]
        print("当前工具坐标系下的位姿: ",TransPosVec)
        
        TransPosMoved = np.array([[TransPos[0][0],TransPos[0][1],TransPos[0][2],TransPos[0][3]+DPos[0]],
                                  [TransPos[1][0],TransPos[1][1],TransPos[1][2],TransPos[0][3]+DPos[1]],
                                  [TransPos[2][0],TransPos[2][1],TransPos[2][2],TransPos[0][3]],
                                  [TransPos[3][0],TransPos[3][1],TransPos[3][2],TransPos[0][3]]])
        print("位移之后的姿态矩阵:\n" , TransPosMoved)
        
        TransMat_inv = np.linalg.inv(TransMat)
        TargetPos = np.dot(TransPosMoved, TransMat_inv)
        # 变换为欧拉角
        TargetPosRotMat = np.array([[TargetPos[0][0],TargetPos[0][1],TargetPos[0][2]],
                                    [TargetPos[1][0],TargetPos[1][1],TargetPos[1][2]],
                                    [TargetPos[2][0],TargetPos[2][1],TargetPos[2][2]]])
        TargetPosRot_ = Rotation.from_matrix(TargetPosRotMat)
        TargetPosRot = TargetPosRot_.as_euler('xyz',degrees=False)
        TargetPosVec = [TargetPos[0][3],TargetPos[1][3],TargetPos[2][3],TargetPosRot[0],TargetPosRot[1],TargetPosRot[2]]
        print("位移之后在法兰坐标系下的姿态:\n" ,TargetPosVec)
        return TransPosVec

    def GetTargetPos(self, DPos, TcpPosNow, ToolTransMat):
        DPosMat = np.array([[1,0,0,DPos[0]],
                            [0,1,0,DPos[1]],
                            [0,0,1,0],
                            [0,0,0,1]])

        RotMat = self.Euler2RotMat(TcpPosNow[3],TcpPosNow[4],TcpPosNow[5])
        PosMat = np.array([[RotMat[0][0],RotMat[0][1],RotMat[0][2],TcpPosNow[0]],
                           [RotMat[1][0],RotMat[1][1],RotMat[1][2],TcpPosNow[1]],
                           [RotMat[2][0],RotMat[2][1],RotMat[2][2],TcpPosNow[2]],
                           [0,0,0,1]])

        TargetPos = np.dot(np.dot(PosMat,ToolTransMat),DPosMat)
        # 变换为欧拉角
        TargetPosRotMat = np.array([[TargetPos[0][0],TargetPos[0][1],TargetPos[0][2]],
                                    [TargetPos[1][0],TargetPos[1][1],TargetPos[1][2]],
                                    [TargetPos[2][0],TargetPos[2][1],TargetPos[2][2]]])
        TargetPosRot_ = Rotation.from_matrix(TargetPosRotMat)
        TargetPosRot = TargetPosRot_.as_euler('xyz',degrees=False)
        TargetPosVec = [TargetPos[0][3],TargetPos[1][3],TargetPos[2][3],TargetPosRot[0],TargetPosRot[1],TargetPosRot[2]]
        


        return TargetPosVec

    def GetDPosMillimeter(self,DiameterPixel,PosPixel,DiameterMillimeter):
        '''
        * Function:     GetDPosMillimeter
        * Description:  计算检测圆在图像中的图像中心的XY偏移
        * Inputs:       
                        DiameterPixel: 像素直径
                        PosPixel: 像素坐标
                        DiameterMillimeter: 真实直径(mm)
        * Outputs:      无输出
        * Returns:      相机屏幕内的步进位移量(m)
        * Notes:
        '''
        # PictureSize = [2368,1760]
        # 图像中心像素坐标
        CenterPixel = [1184,880]

        dx_pixel = CenterPixel[0] - PosPixel[0]
        dy_pixel = CenterPixel[1] - PosPixel[1]

        Druler = DiameterMillimeter / DiameterPixel

        DPos = [-dx_pixel*Druler/1000, -dy_pixel*Druler/1000]

        return DPos

    # def GetTargetPos(self,width,height,disCamX,disCamY,RotationEndToCam,dRuler,PosNow,CirclePos):
    #     '''
    #     * Function:     GetTargetPos
    #     * Description:  计算目标点在机械臂下的位置
    #     * Inputs:       width: 图像的宽度
    #                     height: 图像的高度
    #                     disCamX: 电批头相对于相机的X轴偏移值
    #                     disCamY: 电批头相对于相机的X轴偏移值
    #                     RotationEndToCam: 末端到相机的旋转矩阵
    #                     dRuler: 比例尺
    #                     PosNow: 机械臂末端当前姿态
    #                     CirclePos: 孔在图像中的坐标
    #     * Outputs:      无输出
    #     * Returns:      目标孔位的位置坐标 list[list]
    #     * Notes:        
    #     '''
        
        
    #     # 计算旋转矩阵
    #     RotVec = np.array([PosNow[3], PosNow[4], PosNow[5]])
    #     RotBaseToEnd = self.GetRotVec2RotMat(RotVec)

    #     MoveX = (CirclePos[0] - width / 2) * dRuler + disCamX
    #     MoveY = (CirclePos[1] - height / 2) * dRuler + disCamY

    #     Move1 = np.array([[MoveX], [MoveY], [0]])
    #     Move1 = Move1 / 1000

    #     MoveCam = RotBaseToEnd @ RotationEndToCam @ Move1

    #     px = PosNow[0] + MoveCam[0]
    #     py = PosNow[1] + MoveCam[1]
    #     pz = PosNow[2]

    #     return [px[0],py[0],pz,PosNow[3], PosNow[4], PosNow[5]]
    
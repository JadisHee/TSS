import socket
import threading
import math
import numpy as np
import time

from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl
from HikCtrl import HikCtrl
from SocketCtrl import CommunicateData
from TransferCtrl import TransferCtrl
from CalcTools import CalcTools


import xml.etree.ElementTree as ET

tools = CalcTools()
xml_data = CommunicateData()

# -----------------------------------------------------------------------------------------
# 装配-协作臂
duco_GetAntenna_ip = "192.168.1.47"
duco_GetAntenna_port = 7003
duco_GetAntenna = DucoCtrl(duco_GetAntenna_ip,duco_GetAntenna_port)

# 装配-迁移服务器
transfer_ip = "192.168.1.10"
transfer_port = 5700
transfer = TransferCtrl(transfer_ip,transfer_port)

# -----------------------------------------------------------------------------------------
# 拧钉-协作臂
duco_TwistScrew_ip = "192.168.1.16"
duco_TwistScrew_port = 7003
duco_TwistScrew = DucoCtrl(duco_TwistScrew_ip,duco_TwistScrew_port)

# 拧钉-海康相机服务器
hik_ip = "192.168.1.17"
hik_port = 8192
hik = HikCtrl(hik_ip, hik_port)

# 拧钉-电批服务器
bit_ip = "192.168.1.15"
bit_connection_port = 8888
danikor = DanikorCtrl(duco_TwistScrew_ip, duco_TwistScrew_port, bit_ip, bit_connection_port)


host = '192.168.1.225'
py_Antenna_port = 9998
py_Screw_port = 9999


cam_hik_port = 9995
bit_port = 9996
cam_TransferCtrler_port = 9997


# M6螺钉的识别直径和螺纹孔的识别识别直径
# M6ScrewDiameter = 8.80
# M6HoleDiameter = 14.12
M4ScrewDiameter = 5.8
M4ScrewDiameter_1 = 6.8
M4HoleDiameter = 4.38
# 4.38

transfer_command = '110,122'

# 3D装配位姿
photo_pos_3d = []
# 装配位姿到披头的拍照位姿变换
V_turn = [0,0,0.12,0,0,3.14159265]
T_turn = tools.PosVecToPosMat(V_turn)

# DucoScrew_Base --> DucoAntenna_Base
trans_S2A = np.array([
    [ 7.57802317e-03, -9.99861960e-01,  1.47858391e-02, -1.10862363e-02],
    [ 9.99969389e-01,  7.60359218e-03,  1.66546895e-03,  1.95917636e+00], 
    [-1.77776450e-03,  1.47728271e-02,  9.99889208e-01,  1.40208922e-03], 
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00],
])




def bit_ctrler():

    # 创建socket对象
    bit_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    bit_server.bind((host,bit_port))

    # 设置最大连接数，超过后排队
    bit_server.listen(1)
    print('bit_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = bit_server.accept()
    print(f"bit_ctrler: 协作臂已连接，地址: {addr_B}")
    
    while True:
        print("bit_ctrler: 等待触发信号 ! ! !")
        data = cobot.recv(1024).decode('utf-8')
        print('bit_ctrler: 收到协作臂指令 ', data)
        if data == "rot_bit_inv":
            danikor.ScrewMotorCtrl(2)
        elif data == "M4_rot_bit":
            result = danikor.ScrewMotorCtrl(1)
            if result == 0:
                print('bit_ctrler: 力矩返回失败，请从电批控制器中查看最终力矩！！')
                time.sleep(30)
                cobot.sendall(str(0).encode('utf-8'))
            else:
                print('bit_ctrler: 拧紧力矩为 ', result[0], "Nm")
                cobot.sendall(str(result[0]).encode('utf-8'))

def transfer_ctrler():
    
    # 创建socket对象
    transfer_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    transfer_server.bind((host,cam_TransferCtrler_port))

    # 设置最大连接数，超过后排队
    transfer_server.listen(1)
    print('transfer_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = transfer_server.accept()
    print(f"transfer_ctrler: 协作臂已连接，地址: {addr_B}")
    
    while True:
        print("transfer_ctrler: 等待触发信号 ! ! !")
        data = cobot.recv(1024).decode('utf-8')
        print('transfer_ctrler: 收到出发指令 ', data)
        if str(data) == 'GetTcp_1' or str(data) == 'GetTcp_2' or str(data) == 'GetTcp_3' or str(data) == 'GetTcp_4' or str(data) == 'GetTcp_5':
            # print('使用第一种获取tcp的方案')
            if str(data) == 'GetTcp_1':
                TcpVec = transfer.GetDataFromTransfer([0, 0], transfer_command)
            elif str(data) == 'GetTcp_2':
                TcpVec = transfer.GetDataFromTransfer([0, 1], transfer_command)
            elif str(data) == 'GetTcp_3':
                TcpVec = transfer.GetDataFromTransfer([0, 2], transfer_command)
            elif str(data) == 'GetTcp_4':
                TcpVec = transfer.GetDataFromTransfer([0, 3], transfer_command)
            elif str(data) == 'GetTcp_5':
                TcpVec = transfer.GetDataFromTransfer([0, 4], transfer_command)
            # TcpVec = transfer.GetDataFromTransfer([0,4],transfer_command)
            send_data = '(' + str(TcpVec[0]) + ',' + str(TcpVec[1]) + ',' + str(TcpVec[2]) + ',' + str(TcpVec[3]) + ',' + str(TcpVec[4]) + ',' + str(TcpVec[5]) + ')'
            print('transfer_ctrler: 当前tcp为：',send_data)

            cobot.sendall(send_data.encode('utf-8'))
        elif str(data) == 'GetTarget_1' or str(data) == 'GetTarget_2' or str(data) == 'GetTarget_3' or str(data) == 'GetTarget_4' or str(data) == 'GetTarget_5':
            PosNow = duco_GetAntenna.GetDucoPos(0)
            print('transfer_ctrler: 协作臂当前法兰姿态：', PosNow)
            
            # 从迁移获取相机的结果
            if str(data) == 'GetTarget_1':
                TargetVec = transfer.GetDataFromTransfer([1, 0], transfer_command, PosNow=PosNow)
            elif str(data) == 'GetTarget_2':
                TargetVec = transfer.GetDataFromTransfer([1, 1], transfer_command, PosNow=PosNow)
            elif str(data) == 'GetTarget_3':
                TargetVec = transfer.GetDataFromTransfer([1, 2], transfer_command, PosNow=PosNow)
            elif str(data) == 'GetTarget_4':
                TargetVec = transfer.GetDataFromTransfer([1, 3], transfer_command, PosNow=PosNow)
            elif str(data) == 'GetTarget_5':
                TargetVec = transfer.GetDataFromTransfer([1, 4], transfer_command, PosNow=PosNow)
            # TargetVec = transfer.GetDataFromTransfer([1,4],transfer_command,PosNow=PosNow)

            # 计算目标点位上方的位置
            TargetMat = tools.PosVecToPosMat(TargetVec)
            UpVec = [0,0,-0.22,0,0,0]
            UpMat = tools.PosVecToPosMat(UpVec)
            UpTargetMat = np.dot(TargetMat,UpMat)
            UpTargetVec = tools.PosMatToPosVec(UpTargetMat)
            photo_pos_3d.clear()
            for i in range(len(UpTargetVec)):
                photo_pos_3d.append(UpTargetVec[i])
            
            
            send_data = '(' + str(UpTargetVec[0]) + ',' + str(UpTargetVec[1]) + ',' + str(UpTargetVec[2]) + ',' + str(UpTargetVec[3]) + ',' + str(UpTargetVec[4]) + ',' + str(UpTargetVec[5]) + ')'

            
            print("transfer_ctrler: 目标位置为：",TargetVec)
        # elif str(data) == 'GetAngle_1':

        cobot.sendall(send_data.encode('utf-8'))


def cam_hik_ctrler():
    
    # 创建socket对象
    cam_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    cam_server.bind((host,cam_hik_port))

    # 设置最大连接数，超过后排队
    cam_server.listen(1)
    print('cam_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = cam_server.accept()
    print(f"cam_ctrler: 协作臂已连接，地址: {addr_B}")
    
    while True:
        print("cam_ctrler: 等待触发信号 ! ! !")
        data = cobot.recv(1024).decode('utf-8')
        if data == "GetTwistTarget":
            if len(photo_pos_3d) < 6 or len(photo_pos_3d) > 6:
                TwistTarget = '(0,0,0,0,0,0)'
                print('cam_ctrler: 拧钉拍照参考位姿 ', TwistTarget)
                cobot.sendall(TwistTarget.encode('utf-8'))
            else:
                # 基于装配协作臂的3D目标位置
                photo_pos_3d_mat = tools.PosVecToPosMat(photo_pos_3d)
                # 基于装配协作臂的拧钉相机目标位置
                mid_mat_1 = np.dot(photo_pos_3d_mat,T_turn)
                # 基于拧钉协作臂的拧钉相机目标位置
                TwistTarget_mat = np.dot(trans_S2A,mid_mat_1)
                TwistTarget_vec = tools.PosMatToPosVec(TwistTarget_mat)
                TwistTarget = '(' + str(TwistTarget_vec[0]) + ',' + str(TwistTarget_vec[1]) + ',' + str(TwistTarget_vec[2]) + ',' + str(TwistTarget_vec[3]) + ',' + str(TwistTarget_vec[4]) + ',' + str(TwistTarget_vec[5]) + ')'
                print('cam_ctrler: 拧钉拍照参考位姿 ', TwistTarget)
                cobot.sendall(TwistTarget.encode('utf-8'))
                # photo_pos_3d.clear()
        else:
            # 切换方案
            IsSwitchPlanSuccessful = hik.SetHikSwitchPlan('switch', data)
            if IsSwitchPlanSuccessful == 0:
                print("cam_ctrler: 切换方案失败")
                return 0
            # 吸钉识别螺帽方案
            if data == "M4FindScrew" or data == "M4FindScrew_1":
                DPos = hik.GetHikDPos(M4ScrewDiameter_1)
            # elif data == "M4FindScrew_1":
            #     DPos = hik.GetHikDPos(M4ScrewDiameter_1)

            # 拧钉识别螺孔方案
            elif data == "M4FindHole_3" or data == "M4FindHole_4":
                DPos = hik.GetHikDPos(M4HoleDiameter)
            
            if DPos != 0:
                str_DPos = '(' + str(DPos[0]) + ',' + str(DPos[1]) + ')'
                cobot.sendall(str_DPos.encode('utf-8'))
            else:
                str_DPos = '(0,0)'
                cobot.sendall(str_DPos.encode('utf-8'))


def py_Antenna_ctrler():

    # 创建socket对象
    py_Antenna_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    py_Antenna_server.bind((host,py_Antenna_port))

    # 设置最大连接数，超过后排队
    py_Antenna_server.listen(2)
    print('py_Antenna_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = py_Antenna_server.accept()
    print(f"py_Antenna_ctrler: 协作臂已连接，地址: {addr_B}")

    # ctrl_system, addr_A = py_server.accept()
    # print(f"py_ctrler: 主控系统已连接，地址: {addr_A}")

    while True:
        print("py_Antenna_ctrler: 等待主控系统连接 ! ! !")

        ctrl_system, addr_A = py_Antenna_server.accept()
        print(f"py_Antenna_ctrler: 主控系统已连接，地址: {addr_A}")

        try:
            while True:
                print("py_Antenna_ctrler: 等待主控系统指令 ! ! !")
                data = ctrl_system.recv(1024).decode('utf-8')
                if not data:  # 检查是否收到空数据，表示客户端已断开连接
                    print("py_Antenna_ctrler: 主控系统断开连接")
                    break
                Message = ET.fromstring(data)

                # 一共有五种天线
                # 第一种为：高度表发射天线×1    
                #   ProductType = 1
                #   productNum = 1
                # 第二种为：高度表接收天线×1
                #   ProductType = 2
                #   productNum = 1
                # 第三种为：遥测天线×2
                #   ProductType = 3
                #   productNum = 1/2
                # 第四种为：安控天线×2
                #   ProductType = 4
                #   productNum = 1/2
                # 第五种为：GPS天线×1
                #   ProductType = 5
                #   productNum = 1
                for Command in Message.findall('Command'):                    
                    for ProductType in Message.findall('ProductType'):
                        for ProductNum in Message.findall('ProductNum'):
                            data_ = '(' + str(int(Command.text)) + ','  + str(int(ProductType.text)) + ',' + str(int(ProductNum.text)) + ')'
                            print('py_Antenna_ctrler: 即将发送的消息为：',data_)
                            cobot.sendall(data_.encode('utf-8'))
                    print("py_Antenna_ctrler: 指令发送完成，等待协作臂回传完成消息")
                    recv = cobot.recv(1024).decode('utf-8')
                    if int(recv) == 100:
                        xml_data.Error_Data = ""
                    elif int(recv) == 101:
                        xml_data.Error_Data = "101" #可循环五次
                    elif int(recv) == 102:
                        xml_data.Error_Data = "102" 
                    elif int(recv) == 103:
                        xml_data.Error_Data = "103"
                    
                    print("py_Antenna_ctrler: 协作臂完成任务：" + str(recv) + " ! ! !")
                    
                    # xml_data.TypeData = Command.text
                    xml_data.Command_Data = Command.text
                    # xml_data.Error_Data = "no error"

                    ctrl_system.sendall(str(xml_data.XmlData_Antenna()).encode('utf-8'))


                    

        except Exception as e:
            print(f"py_ctrler: 主控系统通信错误: {e}")

        # 关闭当前主控系统连接，并重新等待
        ctrl_system.close()

        # print("py_ctrler: 等待主控系统指令 ! ! !")
        # data = ctrl_system.recv(1024).decode('utf-8')
        # cobot.sendall(data.encode('utf-8'))
        # print("py_ctrler: 指定发送 ", data)

def py_Screw_ctrler():
     # 创建socket对象
    py_Screw_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    py_Screw_server.bind((host,py_Screw_port))

    # 设置最大连接数，超过后排队
    py_Screw_server.listen(2)
    print('py_Screw_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = py_Screw_server.accept()
    print(f"py_Screw_ctrler: 协作臂已连接，地址: {addr_B}")

    # ctrl_system, addr_A = py_server.accept()
    # print(f"py_ctrler: 主控系统已连接，地址: {addr_A}")

    while True:
        print("py_Screw_ctrler: 等待主控系统连接 ! ! !")

        ctrl_system, addr_A = py_Screw_server.accept()
        print(f"py_Screw_ctrler: 主控系统已连接，地址: {addr_A}")

        try:
            while True:
                print("py_Screw_ctrler: 等待主控系统指令 ! ! !")
                data = ctrl_system.recv(1024).decode('utf-8')
                if not data:  # 检查是否收到空数据，表示客户端已断开连接
                    print("py_Screw_ctrler: 主控系统断开连接")
                    break
                # 从主控发送的消息中构建XML格式
                Message = ET.fromstring(data)
                for ProductType in Message.findall('ProductType'):
                    print('ProductType:' , int(ProductType.text))
                    time.sleep(1)
                    
                    if int(ProductType.text) == 1:
                        for ScrewIndex in Message.findall('ScrewIndex'):
                            cobot.sendall(ScrewIndex.text.encode('utf-8'))
                    print("py_Screw_ctrler: 指令发送完成，等待协作臂回传完成消息")
                    recv = cobot.recv(1024).decode('utf-8')
                    if int(recv) == 100:
                        xml_data.Error_Data_Screw = ""
                    elif int(recv) == 101:
                        xml_data.Error_Data_Screw = "101"
                    elif int(recv) == 102:
                        xml_data.Error_Data_Screw = "102"
                    print("py_Screw_ctrler: 协作臂完成任务：" + str(recv) + " ! ! !")
                    
                    xml_data.ProductType_Data = ProductType.text
                    xml_data.ScrewIndex_Data = ScrewIndex.text
                    # xml_data.Error_Data = "no error"

                    ctrl_system.sendall(str(xml_data.XmlData_Screw).encode('utf-8'))

                # cobot.sendall(data.encode('utf-8'))
                # print("py_ctrler: 指定发送 ", data)
        except Exception as e:
            print(f"py_Screw_ctrler: 主控系统通信错误: {e}")

        # 关闭当前主控系统连接，并重新等待
        ctrl_system.close()

        # print("py_ctrler: 等待主控系统指令 ! ! !")
        # data = ctrl_system.recv(1024).decode('utf-8')
        # cobot.sendall(data.encode('utf-8'))
        # print("py_ctrler: 指定发送 ", data)


def thread_ctrler():
    # 启动装配协作臂的服务器
    py_Antenna_thread = threading.Thread(target=py_Antenna_ctrler)
    transfer_thread = threading.Thread(target=transfer_ctrler)
    
    # 定义拧顶协作臂的服务器
    py_Screw_thread = threading.Thread(target=py_Screw_ctrler)
    cam_hik_thread = threading.Thread(target=cam_hik_ctrler)
    bit_thread = threading.Thread(target=bit_ctrler)
    
    # 启动各个服务器
    py_Antenna_thread.start()
    transfer_thread.start()

    py_Screw_thread.start()
    cam_hik_thread.start()
    bit_thread.start()


def show_pos():
    PosVec = duco_GetAntenna.GetDucoPos(0)
    print(PosVec)

if __name__ == "__main__":
    # server()
    # danikor_test()
    # CamCtrler()
    thread_ctrler()
    # show_pos()
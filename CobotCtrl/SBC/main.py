import socket
import threading
import time
import math
import numpy as np

from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl
from HikCtrl import HikCtrl
from SocketCtrl import CommunicateData
from TransferCtrl import TransferCtrl
from CalcTools import CalcTools

tools = CalcTools()


import xml.etree.ElementTree as ET


# 实例化xml通讯内容
xml_data = CommunicateData()

# 实例化协作臂控制器
duco_ip = "192.168.1.16"
duco_port = 7003
duco = DucoCtrl(duco_ip, duco_port)

# 实例化电批控制器
danikor_ip = "192.168.1.15"
danikor_port = 8888
danikor = DanikorCtrl(duco_ip, duco_port, danikor_ip, danikor_port)

# 实例化海康相机控制器
hik_ip = "192.168.1.17"
hik_port = 8192
hik = HikCtrl(hik_ip, hik_port)

# 实例化迁移相机控制器
transfer_ip = "192.168.1.10"
transfer_port = 5700
transfer = TransferCtrl(transfer_ip,transfer_port)

# 定义本机控制器ip与各个端口
host = '192.168.1.225'
# py控制器端口
py_port = 9999
# 海康相机控制器端口
cam_port = 9995
# 电批控制器端口
bit_port = 9996
# 迁移相机控制器端口
cam_3d_port = 9997

# M8螺钉的识别直径和螺纹孔的识别识别直径
M8ScrewDiameter = 13.80
M8HoleDiameter = 14.12

# Duco --> Kuka
transmat_d2k = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

def cam_ctrler():
    
    # 创建socket对象
    cam_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    cam_server.bind((host,cam_port))

    # 设置最大连接数，超过后排队
    cam_server.listen(1)
    print('cam_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = cam_server.accept()
    print(f"cam_ctrler: 协作臂已连接，地址: {addr_B}")
    
    while True:
        print("cam_ctrler: 等待触发信号 ! ! !")
        data = cobot.recv(1024).decode('utf-8')
        
        # 切换方案
        IsSwitchPlanSuccessful = hik.SetHikSwitchPlan('switch', data)
        if IsSwitchPlanSuccessful == 0:
            print("cam_ctrler: 切换方案失败")
            return 0
        # 吸钉识别螺帽方案
        if data == "M8FindScrew":
            DPos = hik.GetHikDPos(M8ScrewDiameter)
        # 拧钉识别螺孔方案
        elif data == "M8FindHole":
            DPos = hik.GetHikDPos(M8HoleDiameter)
        
        if DPos != 0:
            str_DPos = '(' + str(DPos[0]) + ',' + str(DPos[1]) + ')'
            cobot.sendall(str_DPos.encode('utf-8'))
        else:
            str_DPos = '(0,0)'
            cobot.sendall(str_DPos.encode('utf-8'))


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
        elif data == "M8_rot_bit":
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
    transfer_server.bind((host,cam_3d_port))

    # 设置最大连接数，超过后排队
    transfer_server.listen(1)
    print('cam_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = transfer_server.accept()
    print(f"transfer_ctrler: 协作臂已连接，地址: {addr_B}")

    while True:
        print("transfer_ctrler: 等待触发信号 ! ! !")
        data = cobot.recv(1024).decode('utf-8')
        print('transfer_ctrler: 收到出发指令 ', data)

        # 给transfer相机发送获取目标姿态的命令
        recv_data = transfer.GetRotData('123')
        if recv_data == 0:
            send_data = '(0,0,0)'
        
        else:
            mat_recv = tools.PosVecToPosMat(recv_data)
            # 变换指定方向
            if data == "125":
                transmat = np.array([
                    [1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1]
                ])
                

                
            elif data == "126":
                transmat = np.array([
                    [1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1]
                ])
            
            mat_target = np.dot(transmat,mat_recv)
            vec_target = tools.PosMatToPosVec(mat_target)
            # 变换至指定方向


            send_data = '(' + str(vec_target[3]) + ',' + str(vec_target[4]) + ',' + str(vec_target[5]) + ')'
            cobot.sendall(send_data.encode('utf-8'))



def py_ctrler():

    # 创建socket对象
    py_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定端口
    py_server.bind((host,py_port))

    # 设置最大连接数，超过后排队
    py_server.listen(2)
    print('py_ctrler: 等待协作臂连接 ! ! !')

    cobot, addr_B = py_server.accept()
    print(f"py_ctrler: 协作臂已连接，地址: {addr_B}")

    # ctrl_system, addr_A = py_server.accept()
    # print(f"py_ctrler: 主控系统已连接，地址: {addr_A}")

    while True:
        print("py_ctrler: 等待主控系统连接 ! ! !")

        ctrl_system, addr_A = py_server.accept()
        print(f"py_ctrler: 主控系统已连接，地址: {addr_A}")

        try:
            while True:
                print("py_ctrler: 等待主控系统指令 ! ! !")
                data = ctrl_system.recv(1024).decode('utf-8')
                if not data:  # 检查是否收到空数据，表示客户端已断开连接
                    print("py_ctrler: 主控系统断开连接")
                    break
                Message = ET.fromstring(data)
                for Type in Message.findall('Type'):
                    print('type:' , int(Type.text))
                    time.sleep(1)
                    # 当Tpye==0时为取电批
                    if int(Type.text) == 0:
                        for Command in Message.findall('Command'):
                            if int(Command.text) == 1:
                                cobot.sendall("GM4".encode('utf-8'))
                            elif int(Command.text) == 2:
                                cobot.sendall("GM6".encode('utf-8'))
                    # 当Type==2时为放电批
                    elif int(Type.text) == 2:
                        for Command in Message.findall('Command'):
                            if int(Command.text) == 1:
                                cobot.sendall("PM4".encode('utf-8'))
                            elif int(Command.text) == 2:
                                cobot.sendall("PM6".encode('utf-8'))
                    # 当Type==1时为拧钉子
                    elif int(Type.text) == 1:
                        for Command in Message.findall('Command'):
                            cobot.sendall(Command.text.encode('utf-8'))
                    print("指令发送完成，等待协作臂回传完成消息")
                    recv = cobot.recv(1024).decode('utf-8')
                    if int(recv) == 100:
                        xml_data.Error_Data = ""
                    elif int(recv) == 101:
                        xml_data.Error_Data = "101"
                    elif int(recv) == 102:
                        xml_data.Error_Data = "102"
                    print("协作臂完成任务：" + str(recv) + " ! ! !")
                    
                    xml_data.TypeData = Type.text
                    xml_data.Command_Data = Command.text
                    # xml_data.Error_Data = "no error"

                    ctrl_system.sendall(str(xml_data.XmlData()).encode('utf-8'))

                # cobot.sendall(data.encode('utf-8'))
                # print("py_ctrler: 指定发送 ", data)
        except Exception as e:
            print(f"py_ctrler: 主控系统通信错误: {e}")

        # 关闭当前主控系统连接，并重新等待
        ctrl_system.close()

        # print("py_ctrler: 等待主控系统指令 ! ! !")
        # data = ctrl_system.recv(1024).decode('utf-8') 
        # cobot.sendall(data.encode('utf-8'))
        # print("py_ctrler: 指定发送 ", data)
       

def thread_ctrler():
    py_thread = threading.Thread(target=py_ctrler)
    bit_thread = threading.Thread(target=bit_ctrler)
    cam_thread = threading.Thread(target=cam_ctrler)

    py_thread.start()
    bit_thread.start()
    cam_thread.start()

def calc_trans():
    '''
        * Function:     calc_trans
        * Description:  计算kuka基座在duco基座下的位姿
        * Inputs:
        * Outputs:      
        * Returns:  
        * Notes:        
        '''
    # 由迁移一次性提供数据
    # kuka_flange --> kuka_cam
    # vec_fc = [1,1,1,1,1,1]
    # kuka_base --> kuka_flange
    vec_bf = [0.62485,-0.88621,0.90063,179.93*math.pi/180,-0.02*math.pi/180,-105.19*math.pi/180]
    
    # vec_Bc = [1,1,1,1,1,1]

    # 将位姿向量变换为姿态矩阵
    # mat_fc = tools.PosVecToPosMat(vec_fc)
    # kuka_flange --> kuka_cam
    mat_fc = np.array([
        [0.98404837, -0.00521527, 0.17782426, 0.062254906],
        [0.17787091, 0.010545616, -0.9839973, -0.22911476],
        [0.00325654,   0.9999308, 0.01130504, 0.077875854],
        [0,0,0,1]
    ])
    mat_bf = tools.PosVecToPosMat(vec_bf)


    # Duco_Base --> kuka_cam
    mat_Bc = np.array([
        [-0.39366165, -0.00508397,   0.9192413,  -0.23946294],
        [  -0.919205,  0.01264599, -0.39357617,   0.93604486],
        [-0.00962385, -0.99990714, -0.00965151,   0.53141846],
        [0,0,0,1]
    ])
    # Duco_Base --> kuka_base
    mat_Bb = tools.B_b_calc(mat_fc,mat_bf,mat_Bc)

    return mat_Bb


if __name__ == "__main__":
    # server()
    # danikor_test()
    # CamCtrler()
    # thread_ctrler()
    # show_pos()
    # danikor_test(1)
    trans = calc_trans()
    print(trans)
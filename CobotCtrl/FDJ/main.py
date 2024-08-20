import socket
import threading
import math
import time

from DucoCtrl import DucoCtrl
from DanikorCtrl import DanikorCtrl
from HikCtrl import HikCtrl
from SocketCtrl import CommunicateData
import xml.etree.ElementTree as ET


xml_data = CommunicateData()

duco_ip = "192.168.1.16"
duco_port = 7003
duco = DucoCtrl(duco_ip, duco_port)

bit_ip = "192.168.1.15"
bit_port = 8888
danikor = DanikorCtrl(duco_ip, duco_port, bit_ip, bit_port)

hik_ip = "192.168.1.17"
hik_port = 8192
hik = HikCtrl(hik_ip, hik_port)

# host = '192.168.18.65'
host = '192.168.1.225'
py_port = 9999

cam_port = 9995
bit_port = 9996

# M6螺钉的识别直径和螺纹孔的识别识别直径
M6ScrewDiameter = 8.80
M6HoleDiameter = 14.12
M4ScrewDiameter = 6.10
M4HoleDiameter = 14.12


# def cam_ctrler_():
#     # 创建socket对象
#     cam_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#     # 绑定端口
#     cam_server.bind((host,cam_port))

#     # 设置最大连接数，超过后排队
#     cam_server.listen(1)
#     print('cam_ctrler: 等待协作臂连接 ! ! !')

#     cobot, addr_B = cam_server.accept()
#     print(f"cam_ctrler: 协作臂已连接，地址: {addr_B}")
    
#     while True:
#         print("cam_ctrler: 等待切换方案的信号 ! ! !")
#         data = cobot.recv(1024).decode('utf-8')
#         IsSwitchPlanSuccessful = hik.SetHikSwitchPlan('switch', data)
#         if IsSwitchPlanSuccessful == 0:
#             print("cam_ctrler: 切换方案失败 ! ! !")
#             cobot.sendall("switch failed".encode('utf-8'))
#         else:
#             print("cam_ctrler: 成功切换到方案 " + str(data) + " ! ! !")
#             while True:
#                 print("cam_ctrler:  " + str(data) +" 等待检测触发检测指令")
#                 data_ = cobot.recv(1024).decode('utf-8')
#                 if data_ == "check":
#                     DPos = hik.GetHikDPos(M8ScrewDiameter)
#                     if DPos != 0:
#                         str_DPos = '(' + str(DPos[0]) + ',' + str(DPos[1]) + ')'
#                         cobot.sendall(str_DPos.encode('utf-8'))
#                     else:
#                         str_DPos = '(0,0)'
#                         cobot.sendall(str_DPos.encode('utf-8'))
#                         break
#                 elif data_ == "check_over":
#                     break


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
        if data == "M6FindScrew":
            DPos = hik.GetHikDPos(M6ScrewDiameter)
        elif data == "M6FindHole":
            DPos = hik.GetHikDPos(M6HoleDiameter)
        elif data == "M4FindScrew":
            DPos = hik.GetHikDPos(M4ScrewDiameter)
        elif data == "M4FindHole":
            DPos = hik.GetHikDPos(M4HoleDiameter)
        
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
        # print("收到的信息: ", data)
        # 在模组下降至中间位置后,接收旋转信号控制电批反转与批头锁紧
        # signal = cobot_socket.recv(1024).decode('utf-8')
        # print("recv signal: ", data)
        if data == "rot_bit_inv":
            danikor.ScrewMotorCtrl(2)
        elif data == "M6_rot_bit":
            result = danikor.ScrewMotorCtrl(3)
            if result == 0:
                print('bit_ctrler: 力矩返回失败，请从电批控制器中查看最终力矩！！')
                time.sleep(30)
                cobot.sendall(str(0).encode('utf-8'))
            else:
                print('bit_ctrler: 拧紧力矩为 ', result[0], "Nm")
                cobot.sendall(str(result[0]).encode('utf-8'))
            
        elif data == "M4_rot_bit":
            result = danikor.ScrewMotorCtrl(1)
            if result == 0:
                print('bit_ctrler: 力矩返回失败，请从电批控制器中查看最终力矩！！')
                time.sleep(30)
                cobot.sendall(str(0).encode('utf-8'))
            else:
                print('bit_ctrler: 拧紧力矩为 ', result[0], "Nm")
                cobot.sendall(str(result[0]).encode('utf-8'))


def danikor_test(mod):
    while(1):
        result = danikor.ScrewMotorCtrl(mod)
        print('result: ', result)
        if result == 0:
            print('bit_ctrler: 力矩返回失败，请从电批控制器中查看最终力矩！！')
            time.sleep(10)
        else:
            print('bit_ctrler: 拧紧力矩为 ', result[0], "Nm")
        time.sleep(1)


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

def show_pos():
    PosVec = duco.GetDucoPos(1)
    print(PosVec)

if __name__ == "__main__":
    # server()
    # danikor_test()
    # CamCtrler()
    thread_ctrler()
    # show_pos()
    # danikor_test(1)
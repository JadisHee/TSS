from array import array
import sys
import time
import threading

sys.path.append('gen_py')
sys.path.append('lib')
from DucoCobot import DucoCobot
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from gen_py.robot.ttypes import StateRobot, StateProgram, OperationMode,TaskState,Op,RealTimeControlData


ip='127.0.0.1'
# 初始化用于控制心跳线程停止的变量
stopheartthread = False


def hearthread_fun():
    '''
        * Function:     hearthread_fun
        * Description:  用于执行心跳线程的任务
        * Inputs:       
        * Outputs:      
        * Returns:      
        * Notes:
        '''
    duco_heartbeat = DucoCobot(ip, 7003)
    duco_heartbeat.open()

    # 不断向DucoCobot发送心跳信号以保持连接
    # 当stopheartthread为True时结束循环并关闭连接
    while not stopheartthread:
        duco_heartbeat.rpc_heartbeat()
        time.sleep(1)
    duco_heartbeat.close()
    

def thread_fun():
    '''
        * Function:     thread_fun
        * Description:  用于执行主线程的任务
        * Inputs:       
        * Outputs:      
        * Returns:      
        * Notes:
    '''
    duco_cobot = DucoCobot(ip, 7003)
    # Connect!
    duco_cobot.open()

    # 获取 DucoCobot 的当前状态，并输出到控制台
    while not stopheartthread:
        tcp_pose = []
        tcp_pose = duco_cobot.get_robot_state()
        print("state: ", tcp_pose)
        time.sleep(1)
    duco_cobot.close()

def main():
    thd_A = threading.Thread(target=thread_fun)
    thd_A.start()
    thd_B = threading.Thread(target=hearthread_fun)
    thd_B.start()
    
    duco_cobot = DucoCobot(ip,7003)
    op = Op()

    # 轨迹起始点触发类型
    # 0: 启动
    # 1: 时间触发
    # 2: 距离触发
    op.time_or_dist_1 = 0

    # 轨迹触发控制柜IO的输出序号，范围1~16
    op.trig_io_1 = 1

    # 轨迹触发控制柜IO的高低电平
    # False: 低电平
    # True:  高电平
    op.trig_value_1 = False

    # 当time_or_dist_1为1时，代表轨迹运行多少时间长度触发IO,单位ms
    op.trig_time_1 = 0.0

    # 当time_or_dist_1为2时，代表轨迹运行多少距离长度触发IO,单位m
    op.trig_dist_1 = 0.0

    # 轨迹触发的用户自定义事件名称
    op.trig_event_1 = ''

    # 轨迹结束点触发类型
    # 0: 不启用
    # 1: 时间触发
    # 2: 距离触发
    op.time_or_dist_2 = 0
    
    # 轨迹触发控制柜IO的输出序号，范围1~16
    op.trig_io_2 = 1

    # 轨迹触发控制柜IO的电平高低
    # False: 低电平
    # True:  高电平
    op.trig_value_2 = False

    # 当time_or_dist_2为1时，当trig_time_2 >= 0时，代表轨迹运行剩余多少时间长度触发IO,单位ms
    # 当trig_time_2 < 0时，代表代表轨迹运行结束后多少时间长度触发IO
    op.trig_time_2 = 0.0

    # 当time_or_dist_2为2时，当trig_ dist _2 >= 0时，代表轨迹运行剩余多少距离长度触发IO,单位m
    # 当trig_ dist _2 < 0时，代表代表轨迹运行结束后多少距离长度触发IO
    op.trig_dist_2 = 0.0

    # 轨迹触发的用户自定义事件名称
    op.trig_event_2 = ''

    # Connect!
    rlt = duco_cobot.open()
    print("open:", rlt)
    rlt = duco_cobot.power_on(True)
    print("power_on:", rlt)
    rlt = duco_cobot.enable(True)
    print("enable:", rlt)
    rlt = duco_cobot.set_tool_data("default",[0,0,0,0,0,0],[1,0,0,0],[0,0,0,0,0,0])
    print("set tool:", rlt)
    rlt = duco_cobot.movej2([0,0,1.57,0,-1.57,0],1.0,1.0,0,True)
    print("movej2", rlt)
    rlt = duco_cobot.tcp_move([0.35,0.09,0.2,1.7,0.45,-0.13],0.5,0.5,0,"default",True)
    print("tcp move:",rlt)

    input()
    stopheartthread = True
    rlt = duco_cobot.close()
    print("close:", rlt)

if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as tx:
        print('%s' % tx.message)

from DucoCobotAPI_py.DucoCobot import DucoCobot
import socket
import time
import binascii


class DanikorCtrl:

    def __init__(self, DucoIp, DucoPort, DanikorIp, DanikorPort):

        self.DucoIp = DucoIp
        self.DucoPort = DucoPort
        self.DanikorIp = DanikorIp
        self.DanikorPort = DanikorPort

        # 与机械臂建立连接
        self.robot = DucoCobot(self.DucoIp, self.DucoPort)
        self.robot.open()
        pass

    def ClawCtrl(self, target_status):
        '''
        * Function:     ClawCtrl
        * Description:  对气动夹爪进行控制
        * Inputs:       想要控制的状态
                            0:闭合夹爪
                            1:张开夹爪
        * Outputs:      无输出
        * Returns:      最终的状态
                            0:夹爪闭合
                            1:夹爪张开
        * Notes:
        '''
        if target_status == 0:

            self.robot.set_standard_digital_out(2, 0, True)
            # self.robot.set_board_io_status(5,"U_DO_03",0)
            self.robot.set_standard_digital_out(1, 1, True)
            # self.robot.set_board_io_status(5,"U_DO_04",1)
            time.sleep(1)
            # self.robot.set_standard_digital_out(1,0,True)
            return 0
        elif target_status == 1:
            self.robot.set_standard_digital_out(1, 0, True)
            self.robot.set_standard_digital_out(2, 1, True)

            time.sleep(1)
            # self.robot.set_standard_digital_out(2,0,True)
            return 1

    def VacuumCtrl(self, target_status):
        '''
        * Function:     VacuumCtrl
        * Description:  对真空阀进行控制
        * Inputs:       想要控制的状态
                            0:关闭真空阀
                            1:打开真空阀
        * Outputs:      无输出
        * Returns:      最终的状态
                            0:真空阀关闭
                            1:真空阀打开
                            2:气压异常
        * Notes:
        '''
        if target_status == 0:
            self.robot.set_standard_digital_out(5, 0, True)
            # self.robot.set_board_io_status(5,"U_DO_05",0)
            return 0
        elif target_status == 1:
            # 设置启动io
            self.robot.set_standard_digital_out(5, 1, True)

            # 等待两秒
            time.sleep(2)
            # 通过气压表io检测气压是否达标
            IsPressOk = self.robot.get_standard_digital_in(1)
            if IsPressOk == 0:
                return 1
            else:
                return 2

    def DriverCtrl(self, target_status):
        '''
        * Function:     DriverCtrl
        * Description:  对拧钉模组气缸进行控制
        * Inputs:       想要控制的状态
                            0:模组收回
                            1:模组伸出
        * Outputs:      无输出
        * Returns:      最终的状态
                            0:模组已收回
                            1:模组已伸出
                            2:模组未运动到位
        * Notes:
        '''
        if target_status == 0:
            # 通过io控制模组收回
            self.robot.set_standard_digital_out(4, 0, True)
            self.robot.set_standard_digital_out(3, 1, True)

            # 循环读取到位模块，若到位则正常返回，若超过5s未到位，则异常返回
            for i in range(0, 5, 1):
                IsBackOk = self.robot.get_standard_digital_in(2)
                if IsBackOk == 1:
                    return 0
                else:
                    time.sleep(1)
                    if i > 5:
                        return 2
        elif target_status == 1:
            # 通过io控制模组伸出
            self.robot.set_standard_digital_out(3, 0, True)
            self.robot.set_standard_digital_out(4, 1, True)

            # 循环读取到位模块，若到位则正常返回，若超过3s未到位，则异常返回
            for i in range(0, 5, 1):
                IsUpOk = self.robot.get_standard_digital_in(3)
                if IsUpOk == 1:
                    return 1
                else:
                    time.sleep(1)
                    if i > 3:
                        return 2

    def LiveDataDecode(self, result):
        HexDATA = binascii.hexlify(result).decode()

        # 所需数据的起始与结束字节索引
        StartIndex = 10
        EndIndex = None

        TorqueStartIndex = None
        TorqueEndIndex = None

        # 实时数据读取截至信号
        EndSignalIndex = None

        # 分号计数
        semicolon_count = 0

        # ----------------------------------------------------------------------
        # --------------------根据数据类型在结果中获取数据-------------------------
        # ----------------------------------------------------------------------
        for i, byte in enumerate(result[StartIndex:], start=StartIndex):
            if byte == ord(';'):
                semicolon_count += 1
                if semicolon_count == 2:
                    EndSignalIndex = i + 6
                if semicolon_count == 4:
                    TorqueStartIndex = i
                if semicolon_count == 5:
                    EndIndex = i
                    break
        # if EndIndex is not None:
        UsefulData = result[TorqueStartIndex + 6:EndIndex]
        # UsefulDataString = ''.join(chr(byte) for byte in UsefulData[:])

        comma_count = 0
        for i, byte in enumerate(UsefulData[:], start=0):
            if byte == ord(','):
                comma_count += 1
                if comma_count == 1:
                    TorqueEndIndex = i - 1
                    break

        TorqueData = UsefulData[0:TorqueEndIndex]
        TorqueDataString = ''.join(chr(byte) for byte in TorqueData[:])

        EndSignal = chr(result[EndSignalIndex])
        # DecodeData = ''.join(chr(byte) for byte in result[10:])

        return [EndSignal, TorqueDataString]

    def FinalResultDecode(self, result):
        '''
        * Function:     FinalResultDecode
        * Description:  对电批返回的最终数据进行解码
        * Inputs:       
                            result: 电批拧紧完成后返回数据
                            
        * Outputs:      
        * Returns:      list[TorqueData,AngleData,TimeData,ResultData]
                            TorqueData: 最终力矩
                            AngleData: 最终角度
                            TimeData: 拧紧时间
                            ResultData: 拧紧结果
                                1: 合格
                                2: 不合格
        * Notes:
        '''

        # 所需数据的起始与结束字节索引
        StartIndex = 10
        EndIndex = None

        UsefulData = ''

        # 最终力矩字节索引
        EndIndex_Torque = None

        # 最终角度字节索引
        EndIndex_Angle = None

        # 拧紧时间字节索引
        StartIndex_Time = None
        EndIndex_Time = None

        # 最终拧紧结果字节索引
        EndIndex_Result = None

        # 分号计数
        semicolon_count = 0

        # ----------------------------------------------------------------------
        # --------------------根据数据类型在结果中获取数据-------------------------
        # ----------------------------------------------------------------------
        for i, byte in enumerate(result[StartIndex:], start=StartIndex):
            if byte == ord(';'):
                semicolon_count += 1
                if semicolon_count == 1:
                    EndIndex_Angle = i
                if semicolon_count == 2:
                    EndIndex_Result = i
                if semicolon_count == 3:
                    EndIndex = i
                    break
        if EndIndex is not None:
            UsefulData = result[StartIndex:EndIndex + 1]
            UsefulDataString = ''.join(chr(byte) for byte in result[StartIndex:EndIndex + 1])

        semicolon_count = 0
        for i, byte in enumerate(UsefulData[:], start=0):
            if byte == ord(';'):
                semicolon_count += 1
                if semicolon_count == 1:
                    EndIndex_Angle = i
                if semicolon_count == 2:
                    EndIndex_Result = i
                if semicolon_count == 3:
                    EndIndex = i
                    break

        comma_count = 0
        for i, byte in enumerate(UsefulData[:], start=0):
            if byte == ord(','):
                comma_count += 1
                if comma_count == 1:
                    EndIndex_Torque = i
                if comma_count == 2:
                    StartIndex_Time = i + 1
                if comma_count == 3:
                    EndIndex_Time = i
                    break
        # ----------------------------------------------------------------------
        # ----------------------------------------------------------------------
        # ----------------------------------------------------------------------

        TorqueData = ''.join(chr(byte) for byte in UsefulData[6:EndIndex_Torque])
        AngleData = ''.join(chr(byte) for byte in UsefulData[EndIndex_Time + 1:EndIndex_Angle])
        TimeData = ''.join(chr(byte) for byte in UsefulData[StartIndex_Time:EndIndex_Time])
        ResultData = ''.join(chr(byte) for byte in UsefulData[EndIndex_Angle + 1 + 6:EndIndex_Result])

        # print('解析数据为: ', UsefulDataString)
        # print('拧紧力矩为: ', TorqueData)
        # print('拧紧角度为: ', AngleData)
        # print('拧紧时间为: ', TimeData)
        # print('拧紧结果为: ', ResultData)

        return [float(TorqueData), float(AngleData), float(TimeData), int(ResultData)]

    def ScrewMotorCtrl(self, CtrlMod, XmlData=None):
        '''
        * Function:     ScrewMotorCtrl
        * Description:  对拧钉电批进行控制
        * Inputs:       CtrlMod:电批的运行模式
                            1: 正常拧钉指令
                            2: 快速反拧寻帽指令
        * Outputs:      无输出
        * Returns:      0: 通讯失败
                        list[TorqueData,AngleData,TimeData,ResultData]: 拧紧结果
                            TorqueData: 最终力矩
                            AngleData: 最终角度
                            TimeData: 拧紧时间
                            ResultData: 拧紧结果
                                1: 合格
                                2: 不合格
        * Notes:
        '''
        # 与电批建立连接的指令
        SetConnection = "0200000005523030303103"

        # 订阅拧紧结果数据
        SubResult = "0200000005523032303203"

        # 订阅实时拧紧曲线
        SubLiveData = "0200000005523032303303"

        # 切换正常拧钉指令
        Pset_1 = "020000000A573031303330313D313B03"

        # 切换反拧寻帽指令
        Pset_2 = "020000000A573031303330313D323B03"

        # 正向运行
        MotorStart = "020000000A573033303130313D313B03"

        SelectMod = ""

        if CtrlMod == 1:
            SelectMod = Pset_1
        elif CtrlMod == 2:
            SelectMod = Pset_2
        else:
            print("input wrong!!!")
            return 0

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            if CtrlMod == 1:
                # 连接到目标设备
                s.connect((self.DanikorIp, self.DanikorPort))

                # 发送订阅实时拧紧数据
                byte_data_0 = bytes.fromhex(SubLiveData)
                s.sendall(byte_data_0)
                s.recv(1024)

                # 发送订阅拧紧结果数据
                byte_data_1 = bytes.fromhex(SubResult)
                s.sendall(byte_data_1)
                s.recv(1024)

                # 发送切换模式指令
                byte_data_2 = bytes.fromhex(SelectMod)
                s.sendall(byte_data_2)
                s.recv(1024)

                # 发送电批启动指令
                byte_data_3 = bytes.fromhex(MotorStart)
                s.sendall(byte_data_3)
                data = s.recv(1024)

                while True:
                    LiveData = s.recv(1024)
                    LiveData_ = self.LiveDataDecode(LiveData)

                    if int(LiveData_[0]) == 1:
                        break
                    if float(LiveData_[1]) != 0.0 and XmlData != None:
                        XmlData.ClampingForceData = float(LiveData_[1])
                        print(float(LiveData_[1]))
                    # print(LiveData_[1])

                # 监听电批返回的数据
                ResultData = s.recv(1024)

                Result = self.FinalResultDecode(ResultData)

                print(Result[0])

                return Result
            elif CtrlMod == 2:
                # 连接到目标设备
                s.connect((self.DanikorIp, self.DanikorPort))

                # 发送订阅实时拧紧数据
                byte_data_0 = bytes.fromhex(SubLiveData)
                s.sendall(byte_data_0)
                s.recv(1024)

                # 发送订阅拧紧结果数据
                byte_data_1 = bytes.fromhex(SubResult)
                s.sendall(byte_data_1)
                s.recv(1024)

                # 发送切换模式指令
                byte_data_2 = bytes.fromhex(SelectMod)
                s.sendall(byte_data_2)
                s.recv(1024)

                # 发送电批启动指令
                byte_data_3 = bytes.fromhex(MotorStart)
                s.sendall(byte_data_3)
                return 1

        except Exception as e:
            print("出现了错误: ", e)
            return 0
            # print(f"\n电批启动时发生错误：{e}")


        finally:
            # 关闭连接
            s.close()

    def ScrewConferm(self):
        '''
        * Function:     ScrewConferm
        * Description:  判断是否螺丝是否吸上
        * Inputs:       
        * Outputs:      无输出
        * Returns:      最终的状态
                            False:末端有螺钉
                            True:末端无螺钉
        * Notes:
        '''
        IsScrewOk = self.robot.get_standard_digital_in(4)

        time.sleep(1)

        if IsScrewOk == True:
            return True
        else:
            return False

    def InitialAllMould(self):
        '''
        * Function:     InitialAllMould
        * Description:  初始化所有模块
                            拧钉模组气缸收回
                            真空阀关闭
                            夹爪张开
        * Inputs:
        * Outputs:      
        * Returns:      初始化结果
                            0: 初始化失败
                            1: 初始化成功
        * Notes:
        '''
        # 控制夹爪张开
        ClawStatus = self.ClawCtrl(1)

        # 控制真空阀关闭
        VacuumStatus = self.VacuumCtrl(0)

        # 控制拧钉模组气管收回
        DriverStatus = self.DriverCtrl(0)

        time.sleep(2)


        if ClawStatus == 1 & VacuumStatus == 0 & DriverStatus == 0:
            return 1
        else:
            return 0

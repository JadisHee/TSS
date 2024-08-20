import numpy as np
import math
from scipy.spatial.transform import Rotation

class CalcTools:
    def __init__(self):
        pass
    
    # 定义旋转矩阵
    def rotation_matrix_x(rx):
        return np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])

    def rotation_matrix_y(ry):
        return np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])

    def rotation_matrix_z(rz):
        return np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])

    def ZYX2XYZ(self,zyxVec):
        zyx = np.radians(zyxVec)

        # 计算旋转矩阵
        Rz = self.rotation_matrix_z(zyx[0])
        Ry = self.rotation_matrix_y(zyx[1])
        Rx = self.rotation_matrix_x(zyx[2])

        # 组合旋转矩阵 (顺序 [rx, ry, rz])
        R = Rz @ Ry @ Rx


    def Euler2RotMat(self,rx, ry, rz):
        '''
        * Function:     Euler2RotMat
        * Description:  将[Rx, Ry, Rz]欧拉角转换为旋转矩阵
        * Inputs:       弧度角
                        rx: float
                        ry: float
                        rz: float
        * Outputs:      
        * Returns:      
                        rotation_matrix: numpy array
                        3x3 rotation matrix.
        * Notes:
        '''
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
    
    def PosVecToPosMat(self,PosVec):
        RotMat = self.Euler2RotMat(PosVec[3],PosVec[4],PosVec[5])
        PosMat = np.array([[RotMat[0][0],RotMat[0][1],RotMat[0][2],PosVec[0]],
                           [RotMat[1][0],RotMat[1][1],RotMat[1][2],PosVec[1]],
                           [RotMat[2][0],RotMat[2][1],RotMat[2][2],PosVec[2]],
                           [0,0,0,1]])

        return PosMat

    def PosMatToPosVec(self,PosMat):
        RotMat = np.array([[PosMat[0][0],PosMat[0][1],PosMat[0][2]],
                           [PosMat[1][0],PosMat[1][1],PosMat[1][2]],
                           [PosMat[2][0],PosMat[2][1],PosMat[2][2]]])
        RotVec_ = Rotation.from_matrix(RotMat)
        RotVec = RotVec_.as_euler('xyz',degrees=False)

        PosVec = [PosMat[0][3],PosMat[1][3],PosMat[2][3],RotVec[0],RotVec[1],RotVec[2]]

        return PosVec

    def GetPhotoRot(self,StepNum,TargetPos,TransMat):
        '''
        * Function:     GetPhotoRot
        * Description:  从安装平面获取电批应该对准的姿态
        * Inputs:       StepNum:
                            第几颗钉，int
                        TargetPos:
                            产品安装位姿，list[x,y,z,rx,ry,rz]
                        TransMat:
                            拧钉机械臂到安装机械臂的变换，np.array([4x4])
        * Outputs:      
        * Returns:      
                        RotVec:
                            拧钉电批应该对准的姿态,list[rx,ry,rz]
        * Notes:
        '''
        # DucoBase --> KukaBase
        T1 = TransMat.copy()

        # KukaBase --> KukaTarget
        V2 = TargetPos.copy()
        T2 = self.PosVecToPosMat(V2)

        # KukaTarget --> BitRot
        if StepNum <= 2:
            V3 = [0, 0, 0, 90 * math.pi / 180, -90 * math.pi / 180, 0]
        else:
            V3 = [0, 0, 0, 90 * math.pi / 180, 90 * math.pi / 180, 0]
        T3 = self.PosVecToPosMat(V3)

        # KukaBase --> BitRot
        T4 = np.dot(T2, T3)

        # DucoBase --> BitRot
        T5 = np.dot(T1, T4)
        V5 = self.PosMatToPosVec(T5)

        # 提取出旋转分量
        RotVec = V5[3:]

        return RotVec
              


    def PosTrans(self, PosNow, TransMat):
        # RotMat = self.Euler2RotMat(PosNow[3],PosNow[4],PosNow[5])
        # PosMat = np.array([[RotMat[0][0],RotMat[0][1],RotMat[0][2],PosNow[0]],
        #                    [RotMat[1][0],RotMat[1][1],RotMat[1][2],PosNow[1]],
        #                    [RotMat[2][0],RotMat[2][1],RotMat[2][2],PosNow[2]],
        #                    [0,0,0,1]])

        PosMat = self.PosVecToPosMat(PosNow)

        TargetPos = np.dot(PosMat, TransMat)

        # 变换为欧拉角
        TargetPosVec = self.PosMatToPosVec(TargetPos)

        return TargetPosVec

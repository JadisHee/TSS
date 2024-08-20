import numpy as np
import math

def calc(T_1, T_2, T_3):
    '''
        * Function:     calc
        * Description:  根据手眼标定内容计算手眼机械臂基座到标定板机械臂基座的变换
        * Inputs:
                            T_1:    flange --> cam,
                                    眼在手眼法兰下的位姿（固定值）
                            T_2:    base --> flange,
                                    手眼法兰在手眼基座下此时的位姿（固定值）
                            T_3:    BASE --> cam,
                                    眼在标定板机械臂基座的位姿（标定值）
        * Outputs:      
        * Returns:  T_Bb:   BASE --> base, np.array([4x4])
                    手眼机械臂基座在标定板机械臂基座下的变换
        * Notes:        
    '''

    # base --> cam
    T_bc = np.dot(T_2,T_1)

    # cam --> base
    T_cb = np.linalg.inv(T_bc)

    # BASE --> base
    T_Bb = np.dot(T_3,T_cb)

    return T_Bb
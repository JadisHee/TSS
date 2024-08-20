import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from CalcTools import CalcTools

tools = CalcTools()

def draw_axes(ax, origin, rotation_matrix, length=1.0, colors=('r', 'g', 'b'),system_label=''):
    """
    绘制3D坐标轴
    :param ax: 3D坐标轴对象
    :param origin: 坐标轴原点
    :param rotation_matrix: 旋转矩阵
    :param length: 坐标轴长度
    :param colors: 坐标轴颜色
    """
    axes = np.eye(3)  # 基础坐标轴
    transformed_axes = rotation_matrix @ axes  # 应用旋转矩阵
    for i in range(3):
        ax.quiver(*origin, *transformed_axes[:, i], length=length, color=colors[i], arrow_length_ratio=0.1)
    if system_label:
        ax.text(*origin, system_label, color='k', fontsize=12, ha='center', va='center')


def draw_fig(T_base_1, T_tool_1, T_base_2, T_tool_2):
    '''
        * Function:     draw_fig
        * Description:  绘制坐标系关系示意图
        * Inputs:
                            T_base_1:   基座参考系，请输入四维单位阵
                            T_tool_1:   BASE --> TOOL,
                                        工具1在参考系下的变换
                            T_base_2:   BASE --> base,
                                        基座2在参考系下的变换
                            T_tool_2:   base --> tool,
                                        工具2在基座2下的变换
        * Outputs:      坐标关系示意图
        * Returns:      
        * Notes:        
    '''
    # 创建一个图形对象
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 设置坐标轴范围
    ax.set_xlim([-1.5,1.5])
    ax.set_ylim([-1.5,1.5])
    ax.set_zlim([-1.5,1.5])

    # 绘制以基座1作为参考系的坐标系
    draw_axes(ax, T_base_1[:3, 3], T_base_1[:3, :3], length=1.0, colors=('r', 'g', 'b'),system_label='BASE')

    # 绘制工具1在参考系下的坐标系
    draw_axes(ax, T_tool_1[:3, 3], T_tool_1[:3, :3], length=1.0, colors=('r', 'g', 'b'),system_label='TOOL')

    # 绘制基座2在参考系下的坐标系
    draw_axes(ax, T_base_2[:3, 3], T_base_2[:3, :3], length=1.0, colors=('r', 'g', 'b'),system_label='base')

    # 绘制工具2在参考系下的坐标系
    T_target = np.dot(T_base_2,T_tool_2)
    draw_axes(ax, T_target[:3, 3], T_target[:3, :3], length=1.0, colors=('r', 'g', 'b'),system_label='tool')

    # 设置标签和标题
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('KukaBase-INSTarget')

    # 显示图形
    plt.pause(5)

    # 运行结束但保持窗口打开
    input("Press [enter] to close the plot")

# print("lalala")
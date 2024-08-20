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


# 创建一个新的图形对象
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 设置坐标轴范围
# ax.set_xlim([0, 2])
# ax.set_ylim([-2, 0])
# ax.set_zlim([0, 2])

ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])


# DucoScrew_Base
T0 = np.eye(4)
draw_axes(ax, T0[:3, 3], T0[:3, :3], length=1.0, colors=('r', 'g', 'b'),system_label='Base1')

# DucoScrew_Base --> DucoAntenna_Base
V1 = [1.0666099999999998, -0.30418, 0.276, -3.140370923113397, -0.02181661564992912, 3.1353094682826135]
T1 = np.array([
    [ 7.57802317e-03, -9.99861960e-01,  1.47858391e-02 , -1.10862363e-02] ,
    [ 9.99969389e-01,  7.60359218e-03 , 1.66546895e-03 , 1.95917636e+00], 
    [-1.77776450e-03 , 1.47728271e-02 , 9.99889208e-01 , 1.40208922e-03], 
    [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00],
])
draw_axes(ax, T1[:3, 3], T1[:3, :3], length=0.5, colors=('r', 'g', 'b'),system_label='Base2')

# {-0.9578502066730278,0.16234268956958423,0.4947726440709969,-3.0586895141200623,0.004014257279587152,1.5791739072044697}


# DucoAntenna_Base --> Target
V2 = [-0.9578502066730278,0.16234268956958423,0.4947726440709969,-3.0586895141200623,0.004014257279587152,1.5791739072044697]
v7 = [-1.0319800000000001, 0.66915, 0.25306, -0.004537856055185257, 2.9759609075755313, 0.012217304763960306]
T2 = tools.PosVecToPosMat(V2)

# 绕Z轴旋转180度
V4 = [0,0,0,0,0,0]
T4 = tools.PosVecToPosMat(V4)
print(T4)
T5 = np.dot(T2,T4)

# DucoScrew_Base --> Target
T3 = np.dot(T1,T5)
V3 = tools.PosMatToPosVec(T3)
print(V3)
draw_axes(ax, T3[:3, 3], T3[:3, :3], length=0.5, colors=('r', 'g', 'b'),system_label='Target')



# # Trans Bit --> Cam
# TT4 = np.array([
#                 [ 9.99158203e-01 ,-2.49560079e-02 ,-3.25589188e-02, 0.004037055],
#                 [ 2.49422997e-02 , 9.99688550e-01, -8.27179277e-04, 0.16635849733351],
#                 [ 3.25694214e-02 , 1.43886513e-05 , 9.99469476e-01, 0],
#                 [ 0, 0, 0, 1]
#                 ])
# TTT4 = np.array([
#                 [ 9.99158203e-01 ,-2.49560079e-02 ,-3.25589188e-02],
#                 [ 2.49422997e-02 , 9.99688550e-01, -8.27179277e-04],
#                 [ 3.25694214e-02 , 1.43886513e-05 , 9.99469476e-01]
#                 ])
# TTT4_inv = np.linalg.inv(TTT4)

# # Bit --> Cam
# T6 = np.array([
#                 [ 9.99158203e-01 , 2.49422997e-02 ,3.25694214e-02 , 0.003137055],
#                 [ -2.49560079e-02 , 9.99688550e-01, 1.43886513e-05, 0.16504149733351],
#                 [ -3.25589188e-02 , -8.27179277e-04 , 9.99469476e-01, 0],
#                 [ 0, 0, 0, 1]
#                 ])
# # DucoBase --> Bit
# T3 = np.dot(T1,T2)

# # Flange --> Cam
# T4 = np.dot(T3,T6)

# draw_axes(ax, T3[:3, 3], T3[:3, :3], length=0.5, colors=('r', 'g', 'b'),system_label='Bit')
# draw_axes(ax, T4[:3, 3], T4[:3, :3], length=0.5, colors=('r', 'g', 'b'),system_label='Bit')


# # DucoFlange --> Cam
# T5 = np.array([
#             [ 9.99158203e-01 ,-2.49560079e-02 ,-3.25589188e-02 ,T4[0][3]],
#             [ 2.49422997e-02 , 9.99688550e-01, -8.27179277e-04 ,T4[1][3]],
#             [ 3.25694214e-02 , 1.43886513e-05 , 9.99469476e-01 ,T4[2][3]],
#             [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]
#             ])
# draw_axes(ax, T5[:3, 3], T5[:3, :3], length=0.5, colors=('r', 'g', 'b'),system_label='Cam')
# T3 = np.dot(T1,T2)
# draw_axes(ax, T3[:3, 3], T3[:3, :3], length=0.5, colors=('r', 'g', 'b'),system_label='Cam')


# draw_axes(ax, T5[:3, 3], T5[:3, :3], length=0.5, colors=('r', 'g', 'b'),system_label='Bit')
# T1 = np.array([[ 0.69350914,0.72032586,0.01319006,-1.0707572 ],
#                                 [-0.72008126  ,0.69361943 ,-0.01938605 , 2.18730646],
#                                 [-0.02311231 , 0.00394696 , 0.99972512, -0.78090808],
#                                 [ 0.        ,  0.   ,       0.   ,       1.        ]])
# # Duco -- > Kuka
# T1 = np.array([[ 0.69350914,0.72032586,0.01319006,-1.0707572 ],
#                                 [-0.72008126  ,0.69361943 ,-0.01938605 , 2.18730646],
#                                 [-0.02311231 , 0.00394696 , 0.99972512, -0.78090808],
#                                 [ 0.        ,  0.   ,       0.   ,       1.        ]])
# draw_axes(ax, T1[:3, 3], T1[:3, :3], length=1.0, colors=('r', 'g', 'b'),system_label='Kuka_Base')

# # Kuka --> TargetKuka
# V2 = [1032.7104/1000,-1131.4907/1000,905.4189/1000,179.59668*math.pi/180,3.2624285*math.pi/180,134.49574*math.pi/180]
# T2 = tools.PosVecToPosMat(V2)

# V3 = [0,0,0,90*math.pi/180,90*math.pi/180,0]
# T3 = tools.PosVecToPosMat(V3)

# T4 = np.dot(T2,T3)

# # Duco --> TargetKuka
# T5 = np.dot(T1,T4)

# # Duco --> TargetUp
# V6 = [-0.7798377275466919, 0.6467392444610596, 0.17206327617168427, 0.27459419179179356, 1.5635934209400553, -2.8965454018957812]
# T6 = tools.PosVecToPosMat(V6)

# # TargetUp --> Target
# V7 = [0,0,0.275,0,0,0]
# T7 = tools.PosVecToPosMat(V7)


# T8 = np.dot(T6,T7)

# draw_axes(ax, T5[:3, 3], T5[:3, :3], length=0.2, colors=('r', 'g', 'b'),system_label='Bit_Target')
# # draw_axes(ax, T8[:3, 3], T8[:3, :3], length=0.2, colors=('r', 'g', 'b'),system_label='Bit_Target_')
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
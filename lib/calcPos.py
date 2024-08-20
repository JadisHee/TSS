import numpy as np
import math
from CalcTools import CalcTools

tools = CalcTools()





# Duco1Base --> Duco1Flange
V1 = [-0.68891298,-0.0347185,0.54820448,1.985069,-0.519059,-2.083448]
T1 = tools.PosVecToPosMat(V1)

# Duco1Flange --> cam
T2 = np.array([
    [-0.78093714,-0.6129527,0.12010925,0.076374405],
        [0.61048436,-0.7896955,-0.06074494,0.16449086],
        [0.1320835,0.025886832,0.9909008,0.020342691],
        [0,0,0,1]
])

# Duco2Base --> cam
T3 = np.array([
    [-0.987412,-0.01384510,-0.15756233,-0.017010557],
    [0.14372057,0.3374213,-0.93031776,1.1997401],
    [0.066045254,-0.9412519,-0.33118406,0.71255786],
    [0,0,0,1]
])


# Duco1Base --> cam
T4 = np.dot(T1,T2)

# cam --> Duco1Base
T5 = np.linalg.inv(T4)

# Duco2Base --> Duco1Base
T6 = np.dot(T3,T5)

print(T6)

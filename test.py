import numpy as np
from scipy.spatial.transform import Rotation


damping = [1,2,3,4,5,6]
n = 1

if n > len(damping): damping.extend([0]*(n-len(damping)))
else: damping = damping[0:n]

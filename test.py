import numpy as np
from scipy.spatial.transform import Rotation

a = Rotation.from_euler('xyz',[0,0,0])
b = Rotation.from_euler('xyz',[1.57,0,0])

c = a * b
print(c.as_euler("xyz"))

# c=np.dot(a,b) / (np.linalg.norm(a) * np.linalg.norm(b)) #旋转弧度
# print(c)
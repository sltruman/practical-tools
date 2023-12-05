import pylinalg as la
from scipy.spatial.transform import Rotation
import numpy as np

R = Rotation.from_euler('xyz',[1.57,0,3.14])
out1 = R.as_quat()
print(out1  )

out2 = la.quat_from_euler([1.57,-0,3.14])
print(np.round(out2,3))
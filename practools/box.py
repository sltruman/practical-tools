import pybullet as p
import os
from .active_obj import ActiveObject

class Box(ActiveObject):
    def __init__(self,scene,**kwargs):
        if 'size' not in kwargs: kwargs['size'] = [1,1,1]
        if 'thickness' not in kwargs: kwargs['thickness'] = 0.01
        self.size = kwargs['size']
        self.thickness = kwargs['thickness']
        super().__init__(scene,**kwargs)

    def properties(self):
        properties = super().properties()
        properties.update(kind='Box',size=self.size,thickness=self.thickness)
        return properties

    def set_base(self,base):
        path = os.path.join(self.scene.data_dir,base)
        v_out = 0.5
        v_in = v_out - self.thickness
        with open(path,'w') as f:
            f.write(f'<?xml version="1.0" ?>\n\
<robot name="box">\n\
  <link name="base">\n\
    <inertial>\n\
      <origin rpy="0 0 0" xyz="0 0 0"/>\n\
      <mass value=".1"/>\n\
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>\n\
    </inertial>\n\
    <visual>\n\
      <origin rpy="0 0 0" xyz="0 0 {v_out*self.size[2]}"/>\n\
      <geometry>\n\
        <mesh filename="box.urdf.obj" scale="1 1 1"/>\n\
      </geometry>\n\
    </visual>\n\
    <collision>\n\
      <origin rpy="0 0 0" xyz="0 0 {v_out*self.size[2]}"/>\n\
      <geometry>\n\
        <mesh filename="box.urdf.obj" scale="1 1 1"/>\n\
      </geometry>\n\
    </collision>\n\
  </link>\n\
</robot>')
                    
        path = os.path.join(self.scene.data_dir,base+'.obj')
        with open(path,'w') as f:
            f.write(f'mtllib box.mtl\n\
o 立方体.007_立方体.005\n\
v -{v_out*self.size[0]} -{v_out*self.size[1]} -{v_out*self.size[2]}\n\
v -{v_in*self.size[0]} -{v_in*self.size[1]} {v_out*self.size[2]}\n\
v -{v_out*self.size[0]} {v_out*self.size[1]} -{v_out*self.size[2]}\n\
v -{v_in*self.size[0]} {v_in*self.size[1]} {v_out*self.size[2]}\n\
v {v_out*self.size[0]} -{v_out*self.size[1]} -{v_out*self.size[2]}\n\
v {v_in*self.size[0]} -{v_in*self.size[1]} {v_out*self.size[2]}\n\
v {v_out*self.size[0]} {v_out*self.size[1]} -{v_out*self.size[2]}\n\
v {v_in*self.size[0]} {v_in*self.size[1]} {v_out*self.size[2]}\n\
v -{v_out*self.size[0]} -{v_out*self.size[1]} {v_out*self.size[2]}\n\
v -{v_out*self.size[0]} {v_out*self.size[1]} {v_out*self.size[2]}\n\
v {v_out*self.size[0]} {v_out*self.size[1]} {v_out*self.size[2]}\n\
v {v_out*self.size[0]} -{v_out*self.size[1]} {v_out*self.size[2]}\n\
v -{v_in*self.size[0]} -{v_in*self.size[1]} -{v_in*self.size[2]}\n\
v -{v_in*self.size[0]} {v_in*self.size[1]} -{v_in*self.size[2]}\n\
v {v_in*self.size[0]} {v_in*self.size[1]} -{v_in*self.size[2]}\n\
v {v_in*self.size[0]} -{v_in*self.size[1]} -{v_in*self.size[2]}\n\
vt 0.375000 0.000000\n\
vt 0.625000 0.000000\n\
vt 0.625000 0.250000\n\
vt 0.375000 0.250000\n\
vt 0.625000 0.500000\n\
vt 0.375000 0.500000\n\
vt 0.625000 0.750000\n\
vt 0.375000 0.750000\n\
vt 0.625000 1.000000\n\
vt 0.375000 1.000000\n\
vt 0.125000 0.500000\n\
vt 0.125000 0.750000\n\
vt 0.640398 0.734602\n\
vt 0.640398 0.515398\n\
vt 0.640398 0.515398\n\
vt 0.640398 0.734602\n\
vt 0.859602 0.734602\n\
vt 0.859602 0.515398\n\
vt 0.875000 0.500000\n\
vt 0.875000 0.750000\n\
vt 0.859602 0.515398\n\
vt 0.859602 0.734602\n\
vn -1.0000 0.0000 0.0000\n\
vn 0.0000 1.0000 0.0000\n\
vn 1.0000 0.0000 0.0000\n\
vn 0.0000 -1.0000 0.0000\n\
vn 0.0000 0.0000 -1.0000\n\
vn -0.9999 -0.0130 0.0000\n\
vn 0.0000 0.0000 1.0000\n\
vn 1.0000 -0.0066 0.0061\n\
vn 0.0065 1.0000 0.0062\n\
usemtl None\n\
s off\n\
f 1/1/1 9/2/1 10/3/1 3/4/1\n\
f 3/4/2 10/3/2 11/5/2 7/6/2\n\
f 7/6/3 11/5/3 12/7/3 5/8/3\n\
f 5/8/4 12/7/4 9/9/4 1/10/4\n\
f 3/11/5 7/6/5 5/8/5 1/12/5\n\
f 6/13/6 8/14/6 15/15/6 16/16/6\n\
f 2/17/7 4/18/7 10/19/7 9/20/7\n\
f 4/18/7 8/14/7 11/5/7 10/19/7\n\
f 8/14/7 6/13/7 12/7/7 11/5/7\n\
f 6/13/7 2/17/7 9/20/7 12/7/7\n\
f 15/15/7 14/21/7 13/22/7 16/16/7\n\
f 4/18/8 2/17/8 13/22/8 14/21/8\n\
f 2/17/9 6/13/9 16/16/9 13/22/9\n\
f 8/14/4 4/18/4 14/21/4 15/15/4')
                    
        
        super().set_base(base)
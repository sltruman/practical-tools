import pybullet as p
import os
from digitaltwin.active_obj import ActiveObject

class Cylinder(ActiveObject):
    def __init__(self,scene,**kwargs):
        if 'radius' not in kwargs: kwargs['radius'] = 0.5
        if 'length' not in kwargs: kwargs['length'] = 1
        self.radius = kwargs['radius']
        self.length = kwargs['length']
        super().__init__(scene,**kwargs)

    def properties(self):
        properties = super().properties()
        properties.update(kind='Cylinder',radius=self.radius,length=self.length)
        return properties
    
    def set_base(self,base):
        path = os.path.join(self.scene.data_dir,base)
        with open(path,'w') as f:
            f.write(f'<?xml version="1.0" ?>\n\
<robot name="cylinder">\n\
  <link name="base">\n\
    <inertial>\n\
      <origin rpy="0 0 0" xyz="0 0 0"/>\n\
      <mass value="1"/>\n\
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>\n\
    </inertial>\n\
    <visual>\n\
      <origin rpy="0 0 0" xyz="0 0 {self.length/2}"/>\n\
      <geometry>\n\
        <cylinder length="{self.length}" radius="{self.radius}"/>\n\
      </geometry>\n\
    </visual>\n\
    <collision>\n\
      <origin rpy="0 0 0" xyz="0 0 {self.length/2}"/>\n\
      <geometry>\n\
        <cylinder length="{self.length}" radius="{self.radius}"/>\n\
      </geometry>\n\
    </collision>\n\
  </link>\n\
</robot>')
        super().set_base(base)
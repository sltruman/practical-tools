import pybullet as p
import os
from .active_obj import ActiveObject

class Cube(ActiveObject):
    def __init__(self,scene,**kwargs):
        if 'size' not in kwargs: kwargs['size'] = [1,1,1]
        self.size = kwargs['size']
        super().__init__(scene,**kwargs)
        
    def properties(self):
        properties = super().properties()
        properties.update(kind='Cube',size=self.size)
        return properties
    
    def set_base(self,base):
        path = os.path.join(self.scene.data_dir,base)
        with open(path,'w') as f:
            f.write(f'<?xml version="1.0" ?>\n\
<robot name="box">\n\
<link name="base">\n\
    <inertial>\n\
    <origin rpy="0 0 0" xyz="0 0 0"/>\n\
    <mass value="10"/>\n\
    <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>\n\
    </inertial>\n\
    <visual>\n\
    <origin rpy="0 0 0" xyz="0 0 {self.size[2]/2}"/>\n\
    <geometry>\n\
        <box size="{self.size[0]} {self.size[1]} {self.size[2]}" />\n\
    </geometry>\n\
    </visual>\n\
    <collision>\n\
    <origin rpy="0 0 0" xyz="0 0 {self.size[2]/2}"/>\n\
    <geometry>\n\
        <box size="{self.size[0]} {self.size[1]} {self.size[2]}" />\n\
    </geometry>\n\
    </collision>\n\
</link>\
</robot>')

        super().set_base(base)
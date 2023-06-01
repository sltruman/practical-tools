import pybullet as p
import math
import numpy as np
import os

class Editor:
    def __init__(self,scene):
        self.scene = scene
        pass
        
    def ray(self,x,y):
        try:
            width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()
            mouseX,mouseY = x*width,y*height

            camPos = [
                camTarget[0] - dist * camForward[0],
                camTarget[1] - dist * camForward[1],
                camTarget[2] - dist * camForward[2]
            ]
            
            farPlane = 10000
            rayForward = [
                (camTarget[0] - camPos[0]), 
                (camTarget[1] - camPos[1]), 
                (camTarget[2] - camPos[2])]

            invLen = farPlane * 1. / (math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] + rayForward[2] * rayForward[2]))
            rayForward = [
                invLen * rayForward[0], 
                invLen * rayForward[1], 
                invLen * rayForward[2]]
            
            rayFrom = camPos
            oneOverWidth = float(1) / float(width)
            oneOverHeight = float(1) / float(height)
            dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
            dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
            
            rayToCenter = [
                rayFrom[0] + rayForward[0], 
                rayFrom[1] + rayForward[1], 
                rayFrom[2] + rayForward[2]]
            
            rayTo = [
                rayFrom[0] + rayForward[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0], 
                rayFrom[1] + rayForward[1] - 0.5 * horizon[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1], 
                rayFrom[2] + rayForward[2] - 0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]]
        except:
            return dict(name='',id=-1,pos=[0.,0.,0.])
        rayInfo = p.rayTest(rayFrom, rayTo)
        
        if not rayInfo: return dict(name='',id=-1,pos=[0.,0.,0.])
        id,linkindex,fraction,pos,norm = rayInfo[0]
        if id not in self.scene.active_objs: return dict(name='',id=-1,pos=pos)
        return dict(name=self.scene.active_objs[id].name,id=id,pos=pos)
        
    def move(self,name,pos):
        self.scene.active_objs_by_name[name].properties()
        pass

    def select(self,name):
        return self.scene.active_objs_by_name[name].properties()

    def add(self,kind,base,pos,rot,scale,extra_params={}):
        object_info = {
            "kind": kind,
            "base":base,
            "pos":pos,
            "rot":rot,
            "scale":[1,1,1],
            "name":kind.lower()
        }

        object_info.update(extra_params)

        try:
            import digitaltwin
            active_obj = eval(f'digitaltwin.{kind}(self.scene,**object_info)')
        except :
            return {}
        
        self.scene.active_objs[active_obj.id] = active_obj
        
        name = active_obj.name
        i = 1
        while name in self.scene.active_objs_by_name:
            name = active_obj.name + str(i)
            i+=1
        
        active_obj.name = name
        self.scene.active_objs_by_name[active_obj.name] = active_obj
        return active_obj.properties()

    def add_cube(self,pos,rot,size):
        cube_path = os.path.join(self.scene.data_dir,'objects/cube/cube.urdf')
        
        with open(cube_path,'w') as f:
            f.write(f'<?xml version="1.0" ?>\n\
<robot name="box">\n\
  <link name="base">\n\
    <inertial>\n\
      <origin rpy="0 0 0" xyz="0 0 0"/>\n\
      <mass value="10"/>\n\
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>\n\
    </inertial>\n\
    <visual>\n\
      <origin rpy="0 0 0" xyz="0 0 {size[2]/2}"/>\n\
      <geometry>\n\
        <box size="{size[0]} {size[1]} {size[2]}" />\n\
      </geometry>\n\
    </visual>\n\
    <collision>\n\
      <origin rpy="0 0 0" xyz="0 0 {size[2]/2}"/>\n\
      <geometry>\n\
        <box size="{size[0]} {size[1]} {size[2]}" />\n\
      </geometry>\n\
    </collision>\n\
  </link>\
</robot>')

        return self.add('ActiveObject','objects/cube/cube.urdf',pos,rot,[1,1,1])
        pass

    def add_cylinder(self,pos,rot,radius,length):
        path = os.path.join(self.scene.data_dir,'objects/cylinder/cylinder.urdf')
        
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
      <origin rpy="0 0 0" xyz="0 0 {length/2}"/>\n\
      <geometry>\n\
        <cylinder length="{length}" radius="{radius}"/>\n\
      </geometry>\n\
    </visual>\n\
    <collision>\n\
      <origin rpy="0 0 0" xyz="0 0 {length/2}"/>\n\
      <geometry>\n\
        <cylinder length="{length}" radius="{radius}"/>\n\
      </geometry>\n\
    </collision>\n\
  </link>\n\
</robot>')

        return self.add('ActiveObject','objects/cylinder/cylinder.urdf',pos,rot,[1,1,1])
        pass

    def add_box(self,pos,rot,size,thickness):
        path = os.path.join(self.scene.data_dir,'objects/box/box.urdf')
        
        v_out = 0.5
        v_in = v_out - thickness
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
      <origin rpy="0 0 0" xyz="0 0 {v_out*size[2]}"/>\n\
      <geometry>\n\
        <mesh filename="box.obj" scale="1 1 1"/>\n\
      </geometry>\n\
    </visual>\n\
    <collision>\n\
      <origin rpy="0 0 0" xyz="0 0 {v_out*size[2]}"/>\n\
      <geometry>\n\
        <mesh filename="box.obj" scale="1 1 1"/>\n\
      </geometry>\n\
    </collision>\n\
  </link>\n\
</robot>')
                    
        path = os.path.join(self.scene.data_dir,'objects/box/box.obj')

        with open(path,'w') as f:
            f.write(f'mtllib box.mtl\n\
o 立方体.007_立方体.005\n\
v -{v_out*size[0]} -{v_out*size[1]} -{v_out*size[2]}\n\
v -{v_in*size[0]} -{v_in*size[1]} {v_out*size[2]}\n\
v -{v_out*size[0]} {v_out*size[1]} -{v_out*size[2]}\n\
v -{v_in*size[0]} {v_in*size[1]} {v_out*size[2]}\n\
v {v_out*size[0]} -{v_out*size[1]} -{v_out*size[2]}\n\
v {v_in*size[0]} -{v_in*size[1]} {v_out*size[2]}\n\
v {v_out*size[0]} {v_out*size[1]} -{v_out*size[2]}\n\
v {v_in*size[0]} {v_in*size[1]} {v_out*size[2]}\n\
v -{v_out*size[0]} -{v_out*size[1]} {v_out*size[2]}\n\
v -{v_out*size[0]} {v_out*size[1]} {v_out*size[2]}\n\
v {v_out*size[0]} {v_out*size[1]} {v_out*size[2]}\n\
v {v_out*size[0]} -{v_out*size[1]} {v_out*size[2]}\n\
v -{v_in*size[0]} -{v_in*size[1]} -{v_in*size[2]}\n\
v -{v_in*size[0]} {v_in*size[1]} -{v_in*size[2]}\n\
v {v_in*size[0]} {v_in*size[1]} -{v_in*size[2]}\n\
v {v_in*size[0]} -{v_in*size[1]} -{v_in*size[2]}\n\
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
                    
        return self.add('ActiveObject','objects/box/box.urdf',pos,rot,[1,1,1])
        pass

    def remove(self,name):
        active_obj = self.scene.active_objs_by_name[name]

        del self.scene.active_objs[active_obj.id]
        del self.scene.active_objs_by_name[name]
        del active_obj
        
    def rename(self,name,new_name):
        active_obj = self.scene.active_objs_by_name[name]
        del self.scene.active_objs_by_name[name]
        active_obj.name = new_name
        self.scene.active_objs_by_name[new_name] = active_obj


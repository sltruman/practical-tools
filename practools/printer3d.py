import os
import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
import xml.dom.minidom as xml
import gcodeparser as gp

from .active_obj import ActiveObject

class Printer3D(ActiveObject):
    def __init__(self,scene,**kwargs):
        self.width = kwargs['width']
        self.height = kwargs['height']
        self.depth = kwargs['depth']
        self.nozzle_radius = kwargs['nozzle_radius']
        super().__init__(scene, **kwargs)
        pass

    def properties(self):
        info = super().properties()
        info.update(dict(kind='Printer3D'))
        return info
    
    def update(self,dt):
        super().update(dt)
        pass

    def print(self,gcode):
        command = gp.GcodeParser(gcode,True)

        def excute_gcode_line(lines:list[gp.GcodeLine]):
            line = lines.pop(0)
            print(len(lines),line.gcode_str)

            self.actions.append((excute_gcode_line,[lines]))

        self.actions.append((excute_gcode_line,[command.lines]))
        pass

class G50Printer3D(Printer3D):
    def __init__(self,scene,**kwargs):
        kwargs['width'] = 3.725
        kwargs['height'] = 1.330
        kwargs['depth'] = 2.500
        kwargs['nozzle_radius'] = 0.006
        super().__init__(scene, **kwargs)

    def set_extrusion_mode(self,mode): # M82，绝对挤出长度：absolute；M83，相对挤出长度：relative
        pass

    def set_steppers_power_off(self): # M84，停止轴电机
        pass

    def set_positioning_mode(self,mode): # G90，绝对移动位置：absolute；G91，相对移动位置：relative
        pass

    def set_fan_speed(self,value): # M106 S{value}，设置风扇速度，速度：0~255。
        pass

    def set_fan_power_off(self): # M107，风扇关闭
        pass

    def set_hotend_temperature(self,value): # M104 S{value}，设置热端温度，摄氏度
        pass

    def wait_hotend_temprature(self,value): # M109 S{value}，等待热端到指定温度，摄氏度
        pass

    def set_bed_temperature(self,value): # M140 S{value}，设置平台温度，摄氏度
        pass

    def wait_bed_temperatura(self,value): # M190 S{value}，等待平台到指定温度，摄氏度
        pass

    def reset_extrusion(value_in_mm): # G92 P{value}，挤出长度：毫米
        pass

    def set_auto_home(self,x,y,z): # G28 X{x} Y{y} Z{z}，回原点，位置：x,y,z
        pass

    def move_quick(self,x,y,z,speed): # G0 F{speed} X{x} Y{y} Z{z}，快速移动，位置：x,y,z；速度：mm/s
        pass

    def move_linear(self,x,y,z,extrude,speed): # G1 F{speed} X{x} Y{y} Z{z} V{extrude}，直线移动，位置：x,y,z；挤出长度：毫米；速度：mm/s
        pass

    def move_cycle(self,x,y,z,extrude,speed): 
        pass

    def set_pinch_roller_pose(self,value_in_degrees): # G90 W{value}，压轮控制，位姿：环绕角度0~359度
        pass


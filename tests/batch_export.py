# exports each selected object into its own file
import bpy
import os
import shutil
import numpy as np
import xml.dom.minidom as xml
from xml.etree import ElementTree as et

objs = bpy.context.selected_objects

if not objs: 
    raise Exception("nothing!")

obj_base = objs[0]
basedir = os.path.join(os.path.dirname(bpy.data.filepath),obj_base.name)
if os.path.exists(basedir): shutil.rmtree(basedir)
os.mkdir(basedir)
urdf = os.path.join(basedir,obj_base.name+'.urdf')

# 创建根节点 
robot = et.Element('robot',dict(name=obj_base.name))
link_base = et.SubElement(robot,'link',dict(name='link_'+obj_base.name))
inertial = et.SubElement(link_base,'inertial')
et.SubElement(inertial,'origin',dict(xyz='0 0 0',rpy='0 0 0'))
et.SubElement(inertial,'mass',dict(value='0'))
et.SubElement(inertial,'inertia',dict(ixx='0',ixy='0',ixz='0',iyy='0',iyz='0',izz='0'))

locations = list()
def export_obj(root_node,parent_obj):
    for obj in parent_obj.children:
        name = obj.name
        x,y,z = np.round(obj.location,3)
        rx,ry,rz = np.deg2rad(np.round(obj.rotation_euler,3))
        filename = name + '.obj'
        filepath = os.path.join(basedir,filename)
        joint = et.SubElement(root_node,'joint',dict(name='joint_'+obj.name,type='fixed'))
        et.SubElement(joint,'parent',dict(link='link_'+parent_obj.name))
        et.SubElement(joint,'child',dict(link='link_'+obj.name))
        et.SubElement(joint,'origin',dict(xyz=f'{x} {y} {z}',rpy=f'{rx} {ry} {rz}'))
        link = et.SubElement(root_node,'link',dict(name='link_'+obj.name))
        visual = et.SubElement(link,'visual')
        geometry = et.SubElement(visual,'geometry')
        et.SubElement(geometry,'mesh',dict(filename=filename))
        
        collision = et.SubElement(link,'collision')
        geometry = et.SubElement(collision,'geometry')
        et.SubElement(geometry,'mesh',dict(filename=filename))
        
        location = obj.location.copy()
        locations.append(location)
        obj.location = [0,0,0]
        obj.select_set(True)
        bpy.ops.export_scene.obj(filepath = filepath,use_selection = True,
                                 axis_forward='Y',axis_up='Z',
                                 use_triangles = True,use_materials = True)
        obj.select_set(False)
        export_obj(root_node,obj)
        location = locations.pop()
        obj.location = location


export_obj(robot,obj_base)

tree = et.ElementTree(robot)
rough_str = et.tostring(robot, 'utf-8')
reparsed = xml.parseString(rough_str)
f = open(urdf, 'w', encoding='utf-8')
f.write(reparsed.toprettyxml(indent='\t'))
f.close()

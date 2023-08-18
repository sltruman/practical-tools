import pybullet as p
import numpy as np
import os
from scipy.spatial.transform import Rotation
import multiprocessing as mp
import socket as s
from time import time,sleep
import traceback
import json
import numpy as np
import trimesh
import pyrender
import xml

class Renderer:
    def __init__(self,context):
        self.context = context
        self.viewer_headless = pyrender.OffscreenRenderer(context.width,context.height)
        self.run()
        pass

    def __del__(self):
        pass
    
    def run(self):
        scene = self.scene = pyrender.Scene(ambient_light=[0.02, 0.02, 0.02],
                                            bg_color=[0.0, 1.0, 1.0])

        light = pyrender.DirectionalLight(color=[1.0, 1.0, 1.0],intensity=10)
        light.shadow_texture = pyrender.Texture()
        light_pose = np.eye(4)
        light_pose[:3,:3] = Rotation.from_euler('xyz',[0,0.875,0]).as_matrix()
        light_pose[:3,3] = [0,0,1.0]
        scene.add(light, pose=light_pose)
    
        for active_obj in self.context.active_objs.values():
            properties = active_obj.get_properties()
            links = properties['nodes']
            for link in links:
                if not os.path.exists(link['base']): continue
                tris = trimesh.load(link['base'])
                node = pyrender.Mesh.from_trimesh(tris)
                pose = np.eye(4)
                pose[:3,:3] = Rotation.from_euler('xyz',link['rot']).as_matrix()
                pose[:3,3] = link['pos']
                scene.add(node,pose=pose)
                
        viewer = pyrender.Viewer(scene,viewport_size=self.context.viewport_size)
        
    def sync_bodies(self):
        pass

    def sync_status(self):
        for active_obj in self.context.active_objs.values():
            links = active_obj.get_nodes()
            for link in links:
                pose = np.eye(4)
                pose[:3,:3] = Rotation.from_euler('xyz',link['rot']).as_matrix()
                pose[:3,3] = link['pos']

    def render_to_texture(self):
        color, depth = self.viewer_headless.render(self.scene)
        return color,depth

    def render_to_texture_of_color(self):
        color, _ = self.viewer_headless.render(self.scene)
        return color

    def render_to_texture_of_depth(self):
        _, depth = self.viewer_headless.render(self.scene)
        return depth
    

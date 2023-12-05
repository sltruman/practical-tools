import pybullet as p
import pygfx as gfx
from wgpu.gui.offscreen import WgpuCanvas
import numpy as np
import math as m
from scipy.spatial.transform import Rotation
import imageio.v3 as iio
import traceback as tb
import os

from .active_obj import ActiveObject
from .store import get_object_path

class Editor():
    canvas = WgpuCanvas(size=(1024, 768))
    renderer = gfx.renderers.WgpuRenderer(canvas)
    camera = gfx.PerspectiveCamera(70, 16 / 9)
    target = np.array([0,0,0])

    def __init__(self,scene : gfx.Scene):
        self.scene = scene
        
        # Read cube image and turn it into a 3D image (a 4d array)
        env_img = iio.imread("imageio:meadow_cube.jpg")
        cube_size = env_img.shape[1]
        env_img.shape = 6, cube_size, cube_size, env_img.shape[-1]
        env_tex = gfx.Texture(env_img, dim=2, size=(cube_size, cube_size, 6), generate_mipmaps=True)
        background = gfx.Background(None, gfx.BackgroundSkyboxMaterial(map=env_tex))
        background.world.up=(0,0,1)
        scene.add(background)
        
        obj = ActiveObject(base=get_object_path('env/Room'),position=[0,0,0],euler=[0,0,0])
        obj.set_env_map(env_tex)
        scene.add(obj)

        self.camera.local.up = 0,0,1
        self.camera.local.position = 0,0,20
        self.camera.local.rotation = Rotation.from_euler('xyz',[0,0,0]).as_quat()
        pass

    def viewport_snapshot(self):
        self.renderer.render(self.scene, self.camera)
        rgb = self.canvas.draw()
        
        if not rgb:
            size = self.canvas.get_physical_size()
            rgb = np.full((size[1],size[0],4),0,dtype = np.uint8)
        rgb = np.asarray(rgb)
        return rgb
    
    def viewport_rotate(self,x,y):
        camera_pos = self.camera.world.position - self.target

        rx = self.camera.world.euler_x + y * 0.001
        ry = self.camera.world.euler_y
        rz = self.camera.world.euler_z + x * 0.001
        
        rx = max(0, rx)
        rx = min(rx, 1.553343)

        R = Rotation.from_euler('xyz',[rx,ry,rz])
        self.camera.world.position = R.apply([0,0,np.linalg.norm(camera_pos)])
        self.camera.world.rotation = R.as_quat()
        pass

    def viewport_pan(self,x,y):
        # rz,rx,ry = self.base.cam.getHpr()
        # cam = np.array(self.base.cam.getPos())
        # cam_target = self.cam_target
        
        # len = self.base.camNode.getLens(0)
        # width,height = self.context.viewport_size

        # direction_in_image = LPoint2(x/width,y/height)
        # direction_in_camera = LPoint3(0,0,0)
        # direction2_in_camera = LPoint3(0,0,0)
        # len.extrude(direction_in_image,direction_in_camera,direction2_in_camera)
        
        # distance = np.linalg.norm(cam - cam_target) * 2
        # direction_in_camera = Rotation.from_euler('xyz',[0,0,rz],True).apply([direction_in_camera[0],-direction_in_camera[2],0]) * distance
        # cam += direction_in_camera
        # self.cam_target += direction_in_camera
        # self.base.cam.setPos(cam[0],cam[1],cam[2])
        pass

    def viewport_zoom(self,factor):
        origin = self.camera.local.position
        target = self.target
        direction = target - origin
        x,y,z = origin - factor * direction * 0.1
        self.camera.local.position = x,y,z

    def add(self,name,active_obj):
        self.scene.active_objs[name] = active_obj
        return active_obj.properties()

    def remove(self,name):
        active_obj = self.scene.active_objs[name]
        del self.scene.active_objs[active_obj.id]
        return active_obj
    
    def pick(self,x,y):
        pass

    
import os
import math as m
import numpy as np
from scipy.spatial.transform import Rotation
import panda3d as p3d
from direct.showbase.ShowBase import *

class Viewer:
    cam_target = np.array([0.0,0.0,0.0])

    def __init__(self,context):
        self.context = context
        self.nodes = dict()
        self.run(context)
        pass

    def __del__(self):
        pass
    
    def run(self,context):
        self.render = base = ShowBase(fStartDirect=True, windowType='offscreen')
        base.cam.set_pos(0,0,10)
        base.cam.look_at(self.cam_target[0],self.cam_target[1],self.cam_target[2])
        
        fb_prop = p3d.core.FrameBufferProperties()
        fb_prop.setRgbColor(True)
        fb_prop.setRgbaBits(8, 8, 8, 0)
        fb_prop.setDepthBits(24)
        
        win_prop = p3d.core.WindowProperties.size(context.viewport_size[0], context.viewport_size[1])
        self.window = base.graphicsEngine.makeOutput(base.pipe, "viewport", 0, fb_prop, win_prop, p3d.core.GraphicsPipe.BFRefuseWindow)
        
        self.render.cam.setPos(0,0,10)
        self.render.cam.setHpr(0,-89,0)

        self.disp_region = self.window.makeDisplayRegion()
        self.disp_region.setCamera(self.render.cam)
        
        self.viewport_texture = p3d.core.Texture()
        self.window.addRenderTexture(self.viewport_texture, p3d.core.GraphicsOutput.RTMCopyRam, p3d.core.GraphicsOutput.RTPColor)
        self.update()

    def update(self):
        nodes = self.nodes.copy()

        for active_obj in self.context.active_objs.values():
            properties = active_obj.get_properties()
            links = properties['links']
            id = properties['name']

            for i,link in enumerate(links):
                if not os.path.exists(link['base']): continue
                name = f'{id}/{i}'

                if name not in nodes:
                    node = self.render.loader.loadModel(link['base'])
                    node.setName(name)
                    node.reparentTo(self.render.render)
                    self.nodes[name] = node
                else:
                    node = nodes.pop(name)
                rx,ry,rz = link['rot']
                x,y,z = link['pos']
                node.setPos(x,y,z)
                node.setHpr(np.rad2deg(rz),np.rad2deg(rx),np.rad2deg(ry))
            
        for node in nodes:
            self.nodes.pop(node)
            node.detachNode()

        self.render.graphicsEngine.renderFrame()
        img = np.frombuffer(self.viewport_texture.getRamImageAs('RGBA'),dtype=np.uint8)
        img.shape = (self.context.viewport_size[1],self.context.viewport_size[0],4)
        self.viewport_color_texture = np.flipud(img).tobytes()

    def rotate(self,x,y):
        cam = np.array(self.render.cam.getPos())
        rz,rx,ry = self.render.cam.getHpr()
        rz += x / 2
        rx += y / 4
        rz %= 360
        if rx < -89: rx = -89
        if rx > -2: rx = -2
        direction = [0,-np.linalg.norm(cam - self.cam_target),0]
        R = Rotation.from_euler('xyz',[rx,0,rz],True)
        pos = self.cam_target + R.apply(direction)
        self.render.cam.setPos(pos[0],pos[1],pos[2])
        self.render.cam.setHpr(rz,rx,ry)
        pass
    
    def pan(self,x,y):
        rz,rx,ry = self.render.cam.getHpr()
        cam = np.array(self.render.cam.getPos())
        cam_target = self.cam_target
        
        len = self.render.camNode.getLens(0)
        width,height = self.disp_region.getPixelSize(0)

        direction_in_image = p3d.core.LPoint2f(x/width,y/height)
        direction_in_camera = p3d.core.LPoint3f(0,0,0)
        direction2_in_camera = p3d.core.LPoint3f(0,0,0)
        len.extrude(direction_in_image,direction_in_camera,direction2_in_camera)
        
        distance = np.linalg.norm(cam - cam_target) * 2
        direction_in_camera = Rotation.from_euler('xyz',[0,0,rz],True).apply([direction_in_camera[0],-direction_in_camera[2],0]) * distance
        cam += direction_in_camera
        self.cam_target += direction_in_camera
        self.render.cam.setPos(cam[0],cam[1],cam[2])

    def zoom(self,factor):
        origin = np.array(self.render.cam.get_pos())
        target = self.cam_target
        direction = target - origin
        x,y,z = origin - factor * direction * 0.1
        self.render.cam.set_pos(x,y,z)
    
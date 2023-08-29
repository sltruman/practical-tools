import numpy as np
import os
from scipy.spatial.transform import Rotation
import numpy as np
import panda3d as p3d
from direct.showbase.ShowBase import *

class Renderer:
    def __init__(self,context):
        self.context = context
        self.nodes = dict()
        self.run(context)
        pass

    def __del__(self):
        pass
    
    def run(self,context):
        self.app = base = ShowBase(fStartDirect=True, windowType='offscreen')
        base.cam.setPos(0,0,10)
        
        fb_prop = p3d.core.FrameBufferProperties()
        fb_prop.setRgbColor(True)
        fb_prop.setRgbaBits(8, 8, 8, 0)
        fb_prop.setDepthBits(24)
        
        win_prop = p3d.core.WindowProperties.size(context.viewport_size[0], context.viewport_size[1])
        self.window = base.graphicsEngine.makeOutput(base.pipe, "viewport", 0, fb_prop, win_prop, p3d.core.GraphicsPipe.BFRefuseWindow)
        
        self.app.cam.setHpr(0,-90,0)
        self.app.cam.setPos(0,-0,10)

        self.disp_region = self.window.makeDisplayRegion()
        self.disp_region.setCamera(self.app.cam)
        
        self.color_texture = p3d.core.Texture()
        self.window.addRenderTexture(self.color_texture, p3d.core.GraphicsOutput.RTMCopyRam, p3d.core.GraphicsOutput.RTPColor) 
        self.sync_bodies()

    def sync_bodies(self):
        nodes = self.nodes.copy()
        for active_obj in self.context.active_objs.values():
            properties = active_obj.get_properties()
            links = properties['links']
            id = properties['name']

            for i,link in enumerate(links):
                if not os.path.exists(link['base']): continue
                name = f'{id}/{i}'

                if name not in nodes:
                    node = self.app.loader.loadModel(link['base'])
                    node.setName(name)
                    node.reparentTo(self.app.render)
                    self.nodes[name] = node
                else:
                    node = nodes.pop(name)
                r,p,h = link['rot']
                x,y,z = link['pos']
                node.setPos(x,y,z)
                node.setHpr(p3d.core.deg2Rad(h),p3d.core.deg2Rad(p),p3d.core.deg2Rad(r))
            
        for node in nodes:
            self.nodes.pop(node)
            node.detachNode()

    def render_to_texture_of_color(self):
        self.app.graphicsEngine.renderFrame()
        rgb = np.frombuffer(self.color_texture.getRamImage(), dtype=np.uint8)
        rgb.shape = ((self.color_texture.getYSize(), self.color_texture.getXSize(), self.color_texture.getNumComponents()))
        return rgb

    def render_to_texture_of_depth(self):
        self.app.graphicsEngine.renderFrame()
        rgb = np.frombuffer(self.color_texture.getRamImage(), dtype=np.uint8)
        rgb.shape = (self.color_texture.getYSize(), self.color_texture.getXSize(), self.color_texture.getNumComponents())
        return rgb
    
    def render_to_texture(self):
        return self.render_to_texture_of_color(),self.render_to_texture_of_depth()
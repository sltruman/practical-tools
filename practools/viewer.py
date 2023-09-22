import os
import math as m
import numpy as np
from scipy.spatial.transform import Rotation
from panda3d.core import FrameBufferProperties,WindowProperties,GraphicsPipe,GraphicsOutput,\
                        Texture,LPoint2,LPoint3,\
                        CollisionRay,CollisionTraverser,CollisionHandlerQueue,\
                        CollisionNode,BitMask32

from direct.showbase.ShowBase import ShowBase
from direct.directtools.DirectSelection import SelectionRay

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
        self.base = base = ShowBase(fStartDirect=True,windowType='offscreen')
        base.cam.setPos(0,0,10)
        base.cam.setHpr(0,-89,0)
        
        # dlight = DirectionalLight('dlight')
        # dlight.setColor((1, 1, 1, 1))
        # dlnp = self.base.render.attachNewNode(dlight)
        # dlnp.setHpr(0, -90, 0)
        # base.render.setLight(dlnp)

        # shader = Shader.load(Shader.SL_GLSL,
        #              vertex="myshader.vert",
        #              fragment="myshader.frag")
        # self.base.render.setShader(shader)

        self.picker = CollisionTraverser()
        self.pq = CollisionHandlerQueue()

        pickerNode = CollisionNode('mouseRay')
        pickerNode.setFromCollideMask(BitMask32.bit(1))
        self.pickerRay = CollisionRay()
        pickerNode.addSolid(self.pickerRay)

        self.pickerNP = self.base.cam.attachNewNode(pickerNode)
        self.picker.addCollider(self.pickerNP,self.pq)

        fb_prop = FrameBufferProperties()
        fb_prop.setRgbColor(True)
        fb_prop.setRgbaBits(8, 8, 8, 8)
        fb_prop.setDepthBits(24)
        
        win_prop = WindowProperties.size(context.viewport_size[0], context.viewport_size[1])
        framebuffer = base.graphicsEngine.makeOutput(base.pipe, "viewport", 0, fb_prop, win_prop, GraphicsPipe.BFRefuseWindow)
        
        display_region = framebuffer.makeDisplayRegion()
        display_region.setCamera(self.base.cam)

        self.viewport_texture = Texture()
        framebuffer.addRenderTexture(self.viewport_texture, GraphicsOutput.RTMCopyRam, GraphicsOutput.RTPColor)
        self.update()

    def update(self):
        nodes = self.nodes.copy()

        for active_obj in self.context.active_objs.values():
            properties = active_obj.get_properties()
            links = properties['links']
            id = properties['id']

            for i,link in enumerate(links):
                if not os.path.exists(link['base']): continue
                name = f'{id}/{i}'

                if name not in nodes:
                    node = self.base.loader.loadModel(link['base'])
                    node.setPythonTag('owner_id',id)
                    node.setPythonTag('owner_link_index',i-1)
                    node.reparentTo(self.base.render)
                    node.setCollideMask(BitMask32.bit(1)) 
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

        self.base.graphicsEngine.renderFrame()
        img = np.frombuffer(self.viewport_texture.getRamImageAs('RGBA'),dtype=np.uint8)
        img.shape = (self.context.viewport_size[1],self.context.viewport_size[0],4)
        self.viewport_color_texture = np.flipud(img).tobytes()

    def rotate(self,x,y):
        cam = np.array(self.base.cam.getPos())
        rz,rx,ry = self.base.cam.getHpr()
        rz += x / 2
        rx += y / 4
        rz %= 360
        if rx < -89: rx = -89
        if rx > -2: rx = -2
        direction = [0,-np.linalg.norm(cam - self.cam_target),0]
        R = Rotation.from_euler('xyz',[rx,0,rz],True)
        pos = self.cam_target + R.apply(direction)
        self.base.cam.setPos(pos[0],pos[1],pos[2])
        self.base.cam.setHpr(rz,rx,ry)
        pass
    
    def pan(self,x,y):
        rz,rx,ry = self.base.cam.getHpr()
        cam = np.array(self.base.cam.getPos())
        cam_target = self.cam_target
        
        len = self.base.camNode.getLens(0)
        width,height = self.context.viewport_size

        direction_in_image = LPoint2(x/width,y/height)
        direction_in_camera = LPoint3(0,0,0)
        direction2_in_camera = LPoint3(0,0,0)
        len.extrude(direction_in_image,direction_in_camera,direction2_in_camera)
        
        distance = np.linalg.norm(cam - cam_target) * 2
        direction_in_camera = Rotation.from_euler('xyz',[0,0,rz],True).apply([direction_in_camera[0],-direction_in_camera[2],0]) * distance
        cam += direction_in_camera
        self.cam_target += direction_in_camera
        self.base.cam.setPos(cam[0],cam[1],cam[2])

    def zoom(self,factor):
        origin = np.array(self.base.cam.get_pos())
        target = self.cam_target
        direction = target - origin
        x,y,z = origin - factor * direction * 0.1
        self.base.cam.set_pos(x,y,z)
    
    def pick(self,x,y):
        width,height = self.context.viewport_size
        x = (x - width / 2) / (width / 2)
        y = (y - height / 2) / (height / 2)

        self.pickerRay.setFromLens(self.base.camNode,x,-y)
        self.picker.traverse(self.base.render)
    
        if not self.pq.getNumEntries():
            return None
        
        self.pq.sortEntries()
        collision_entry = self.pq.getEntry(0)
        geomNode = collision_entry.getIntoNodePath()
        node = geomNode.getParent().getParent()
        pos = collision_entry.getInteriorPoint(geomNode)
        id = node.getPythonTag('owner_id')
        link_index = node.getPythonTag('owner_link_index')
        return pos,id,link_index
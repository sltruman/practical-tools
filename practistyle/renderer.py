import numpy as np
import os
from scipy.spatial.transform import Rotation
import numpy as np
import trimesh
import pyrender

class Renderer:
    def __init__(self,context):
        self.context = context
        self.nodes = dict()
        self.run()
        pass

    def __del__(self):
        pass
    
    def run(self):
        scene = self.scene = pyrender.Scene(ambient_light=[0.02, 0.02, 0.02],bg_color=[0.52941,0.80784,0.92157])

        light = pyrender.DirectionalLight(color=[1.0, 1.0, 1.0],intensity=10)
        light_pose = np.eye(4)
        light_pose[:3,:3] = Rotation.from_euler('xyz',[0.785,0,0]).as_matrix()
        light_pose[:3,3] = [0,0,10.0]
        scene.add(light, pose=light_pose)

        camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0)
        scene.add(camera, pose=light_pose)

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
                    tris = trimesh.load(link['base'])
                    mesh = pyrender.Mesh.from_trimesh(tris)

                    self.nodes[name] = node = self.scene.add(mesh,name)
                else:
                    node = nodes.pop(name)

                pose = np.eye(4)
                pose[:3,:3] = Rotation.from_euler('xyz',link['rot']).as_matrix()
                pose[:3,3] = link['pos']
                self.scene.set_pose(node,pose)
            
        for node in nodes:
            self.nodes.pop(node.name)
            self.scene.remove_node(node)


    def render_to_texture(self):
        viewer_headless = pyrender.OffscreenRenderer(self.context.width,self.context.height)
        color, depth = viewer_headless.render(self.scene)
        return color,depth

    def render_to_texture_of_color(self):
        viewer_headless = pyrender.OffscreenRenderer(self.context.width,self.context.height)
        color,_ = viewer_headless.render(self.scene,pyrender.RenderFlags.RGBA | pyrender.RenderFlags.SHADOWS_DIRECTIONAL)
        return color

    def render_to_texture_of_depth(self):
        viewer_headless = pyrender.OffscreenRenderer(self.context.width,self.context.height)
        depth = viewer_headless.render(self.scene,pyrender.RenderFlags.DEPTH_ONLY)
        return depth
    

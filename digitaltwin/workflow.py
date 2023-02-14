from .scene import Scene
import pybullet as p
from threading import Thread
import json

class Workflow():
    def __init__(self,scene: Scene):
        self.scene = scene
        pass
    
    def get_active_obj_nodes(self):
        nodes = [
            dict(kind='Robot',funs=['move','pick','place'],names=[]),
            dict(kind='Camera3D',funs=['capture'],names=[]),
            dict(kind='Packer',funs=['generate'],names=[]),
            dict(kind='Stacker',funs=['generate'],names=[]),
            dict(kind='ActiveObj',funs=[],names=[])
        ]

        for name,obj in enumerate(self.scene.active_objs_by_name):
            kind = type(obj)
            nodes = [n for n in nodes if n['kind'] == kind]
            for n in nodes: n.names.append(name)
        return nodes

    def start(self):
        self.task = Thread(target=self.run)
        pass

    def stop(self):
        pass
    
    def run(self):
        wf = self.scene.profile['workflow']
        declare = wf['declare']
        next = wf['run']
        result = ()
        
        while next:
            act = declare[next]
            kind = act['kind']
            name = act['name']
            fun = act['fun']
            result = eval(f'self.scene.active_objs[{name}].{fun}(*result)')

            next=None
            if 'next' in act:
                next = act['next']
            elif 'alt' in act:
                for opt in act['alt']:
                    condition = opt['cond']
                    if not eval(f'{condition}'): continue
                    next = opt['next']
                    break
        pass

    def set(self,workflow):
        self.scene.profile['workflow'] = json.loads(workflow)

    def get(self):
        return self.scene.profile['workflow']
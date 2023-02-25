from .scene import Scene
import pybullet as p
from threading import Thread
import json
import time

class Workflow():
    def __init__(self,scene: Scene):
        self.scene = scene
        pass
    
    def get_active_obj_nodes(self):
        nodes = [
            dict(kind='Robot',funs=[
                dict(f='move',errs=[]),
                dict(f='pick',errs=[]),
                dict(f='place',errs=[])],names=[]),
            dict(kind='Camera3D',funs=[
                dict(f='capture',errs=[])],names=[]),
            dict(kind='Placer',funs=[
                dict(f='generate',errs=["out of amount"])],names=[]),
            dict(kind='Stacker',funs=[
                dict(f='generate',errs=[])],names=[]),
            dict(kind='ActiveObj',funs=[],names=[])
        ]

        for name,obj in enumerate(self.scene.active_objs_by_name):
            kind = type(obj)
            nodes = [n for n in nodes if n['kind'] == kind]
            for n in nodes: n.names.append(name)
        return nodes

    def start(self):
        self.task = Thread(target=self.run)
        self.task.start()
        pass

    def stop(self):
        self.task.join()
        pass
    
    def run(self):
        wf = self.scene.profile['workflow']
        declare = wf['declare']
        next = wf['run']
        val = ()

        while next:
            act = declare[next]
            kind = act['kind']
            name = act['name']
            fun = act['fun']
            args = act['args']

            print('signal',fun)
            obj = self.scene.active_objs_by_name[name]
            eval(f'obj.signal_{fun}(*val,**args)')
            while not obj.idle(): time.sleep(0.5)

            res = obj.result
            err,val = res[0],res[1:]
            
            next=None
            if 'next' in act:
                next = act['next']
            elif 'alt' in act:
                for opt in act['alt']:
                    if err != opt['err']: continue
                    next = opt['next']
                    break
        pass

    def set(self,workflow):
        self.scene.profile['workflow'] = json.loads(workflow)

    def get(self):
        return self.scene.profile['workflow']
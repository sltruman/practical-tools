from .scene import Scene
import pybullet as p
from threading import Thread
import json
import time

class Workflow():
    def __init__(self,scene: Scene):
        self.scene = scene
        self.running = False
        self.actions = list()
        pass
    
    def update(self):

        pass

    def get_active_obj_nodes(self):
        nodes = [
            dict(kind='Robot',funs=[
                    dict(f='pick_plan',errs=[],args=[
                        dict(name="pick_poses",kind="PoseList")
                    ]),
                    dict(f='place_plan',errs=[],args=[
                        dict(name="pick_poses",kind="PoseList")
                    ]),
                    dict(f='plan_move',errs=[],args=[
                        dict(name="x",kind="Float"),
                        dict(name="y",kind="Float"),
                        dict(name="z",kind="Float"),
                        dict(name="rx",kind="Float"),
                        dict(name="rz",kind="Float"),
                        dict(name="ry",kind="float"),
                    ]),
                    dict(f='move',errs=[],args=[
                        dict(name="x",kind="Float"),
                        dict(name="y",kind="Float"),
                        dict(name="z",kind="Float"),
                        dict(name="rx",kind="Float"),
                        dict(name="ry",kind="Float"),
                        dict(name="rz",kind="Float"),
                    ]),
                    dict(f='move_relatively',errs=[],args=[
                        dict(name="x",kind="Float"),
                        dict(name="y",kind="Float"),
                        dict(name="z",kind="Float"),
                        dict(name="rx",kind="Float"),
                        dict(name="ry",kind="Float"),
                        dict(name="rz",kind="Float"),
                    ]),
                    dict(f='do',errs=[],args=[
                        dict(name='pickup',kind='Bool')]),
                ],names=[]),
            dict(kind='Camera3D',funs=[
                    dict(f='capture',errs=["failed"],args=[]),
                    dict(f='pose_recognize',errs=["failed"],args=[])
                ],names=[]),
            dict(kind='Placer',funs=[
                    dict(f='generate',errs=["failed"],args=[])
                ],names=[]),
            dict(kind='Stacker',funs=[
                    dict(f='generate',errs=["failed"],args=[])
                ],names=[]),
            dict(kind='ActiveObj',funs=[],names=[]),
            dict(king='Plugin',funs=[],names=['Planalgo'],args=[])
        ]

        for name,obj in enumerate(self.scene.active_objs_by_name):
            kind = type(obj)
            nodes = [n for n in nodes if n['kind'] == kind]
            for n in nodes: n.names.append(name)
        return nodes

    def start(self):
        self.running = True
        wf = self.scene.profile['workflow']
        self.declare = wf['declare']
        self.next = wf['run']
        self.val = ()

        def task():
            if self.next and self.running:
                act = self.declare[self.next]
                kind = act['kind']
                name = act['name']
                fun = act['fun']
                args = act['args'] if 'args' in act else {}
                
                print('signal',fun,args)
                obj = self.scene.active_objs_by_name[name]
                eval(f'obj.signal_{fun}(*val,**args)')
                while not obj.idle(): time.sleep(0.5)

                res = obj.result
                err,self.val = res[0],res[1:]
                
                self.next=None
                if 'next' in act:
                    self.next = act['next']
                elif 'alt' in act:
                    for opt in act['alt']:
                        if err != opt['err']: continue
                        next = opt['next']
                        break
        
        self.actions.append(task)

    def stop(self):
        self.running = False
        self.scene.reset()
        pass
    

    def set(self,workflow):
        self.scene.profile['workflow'] = json.loads(workflow)

    def get(self):
        return self.scene.profile['workflow']
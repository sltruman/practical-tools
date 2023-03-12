from .scene import Scene
import pybullet as p
from threading import Thread
import json
import time

class Workflow():
    def __init__(self,scene: Scene):
        self.scene = scene
        self.running = False
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
        declare = wf['declare']

        def task(last):
            if not self.running: return

            res = None,
            next = None
            if last:
                act = declare[last]
                name = act['name']
                last_obj = self.scene.active_objs_by_name[name]
                
                if not last_obj.idle():
                    self.scene.actions.append((task,(last)))
                    return
                
                res = last_obj.result
                if 'next' in act:
                    next = act['next']
                elif 'alt' in act:
                    for opt in act['alt']:
                        if err != opt['err']: continue
                        next = opt['next']
                        break
                else: return
            else:
                next = wf["run"]

            err,val = res[0],res[1:]
            act = declare[next]
            kind = act['kind']
            name = act['name']
            fun = act['fun']
            args = act['args'] if 'args' in act else {}
            obj = self.scene.active_objs_by_name[name]

            print('signal',fun,args)
            eval(f'obj.signal_{fun}(*val,**args)')

            self.scene.actions.append((task,(next)))
        task(None)

    def stop(self):
        self.running = False
        self.scene.reset()
        pass
    
    def set(self,workflow):
        self.scene.profile['workflow'] = json.loads(workflow)

    def get(self):
        return self.scene.profile['workflow']
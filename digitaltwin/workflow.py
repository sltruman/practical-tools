from .scene import Scene
import pybullet as p
import json
import traceback

class Workflow():
    def __init__(self,scene: Scene):
        self.scene = scene
        self.running = False
        self.nodes = [
            {'kind':'Robot','names':[],'funs':[
                {'label':'Motion','f':'pick_move','errs':[],'args':[ #拾取移动
                    {'name':'mode','kind':'String'}, #模式，关节：joint 点：point
                    {'name':'speed','kind':'Flaot'}, #速度，0.0 ~ 1.0
                    {'name':'vision_flow','kind':'String'}, #视觉流程
                    {'name':'pickup','kind':'Bool'} #拾取设置
                    ]},
                {'label':'Motion','f':'move','errs':[],'args':[ #移动
                    {'name':'mode','kind':'String'}, #模式，关节：joint 点：point
                    {'name':'speed','kind':'Flaot'}, #速度，值：0.0 ~ 1.0，默认：0.2
                    {'name':'pickup','kind':'Bool'}, #拾取设置，？？
                    {'name':'joints','kind':'List'}, #关节位置，[弧度值1,...弧度值n]
                    {'name':'point','kind':'List'}, #点位置，[x,y,z,rx,ry,rz]
                    {'name':'home','kind':'Bool'}, #回到home
                    ]},
                {'label':'Motion','f':'move_relatively','errs':[],'args':[ #相对移动
                    {'name':'mode','kind':'String'}, #模式，关节：joint 点：point
                    {'name':'speed','kind':'Flaot'}, #速度，值：0.0 ~ 1.0，默认：0.2
                    {'name':'pickup','kind':'Bool'}, #拾取设置，？？
                    {'name':'joints','kind':'List'}, #关节位置，[弧度值1,...弧度值n]
                    {'name':'point','kind':'List'}, #点位置，[x,y,z,rx,ry,rz]
                    {'name':'target','kind':'String'}, #相对目标，当前任务：task_current，下一个任务：next，选择的任务：selected，工具坐标系：frame_end_effector，机械臂坐标系：frame_robot，全局坐标系：frame_global
                    ]},
                {'label':'EndEffector','f':'pick','errs':[],'args':[]},  #开
                {'label':'EndEffector','f':'place','errs':[],'args':[]}  #合
                ]},
            {'kind':'Camera3DReal','names':[],'funs':[  #相机
                {'label':'Vision','f':'capture','errs':["failed"],'args':[ #拍照
                    {'name':'wait_for_seconds','kind':'Float'}] #等待时间
                }
                ]},
            {'kind':'Camera3D','names':[],'funs':[  #相机
                {'label':'Vision','f':'capture','errs':["failed"],'args':[ #拍照
                    {'name':'wait_for_seconds','kind':'Float'}] #等待时间
                }
                ]},
            {'kind':'Placer','names':[],'funs':[{'label':'PlacingContainer','f':'generate','errs':["failed"],'args':[]}]}, #放置器
            {'kind':'Stacker','names':[],'funs':[{'label':'StackingContainer','f':'generate','errs':["failed"],'args':[]}]}, #堆垛器
            {'kind':'Vision','names':['PickLight'],'funs':[
                {'label':'Vision','f':'detect','errs':[],'args':[ #视觉检测
                    {'name':'vision_flow','kind':'String'}]} #视觉流程，？？
                ]}
        ]

        self.active_plugins_by_name = dict()

        import plugins.vision
        plugin = plugins.vision.Vision(tmp_dir = scene.tmp_dir)
        self.active_plugins_by_name[plugin.name] = plugin
        pass

    def get_active_obj_nodes(self):
        from copy import deepcopy
        
        nodes = deepcopy(self.nodes)
        for name in self.scene.active_objs_by_name:
            obj = self.scene.active_objs_by_name[name]
            kind = obj.__class__.__name__
            for i,n in enumerate(nodes):
                if nodes[i]['kind'] == kind:
                    nodes[i]['names'].append(name)
        
        return [n for n in nodes if n['names']]

    def start(self):
        if self.running: 
            print('Workflow has started',flush=True)
            return

        self.running = True
        wf = self.scene.profile['workflow']
        declare = wf['declare']

        def task(last):
            if not self.running: return

            res = None,
            err,val = res[0],res[1:]
            next = None
            if last:
                act = declare[last]
                name = act['name']

                last_obj = self.scene.active_objs_by_name[name] if name in self.scene.active_objs_by_name else self.active_plugins_by_name[name]
                if not last_obj.idle():
                    self.scene.actions.append((task,(last,)))
                    return
                
                res = last_obj.result
                err,val = res[0],res[1:]
                if 'next' in act and not err:
                    next = act['next']
                elif 'alt' in act:
                    for opt in act['alt']:
                        if err != opt['err']: continue
                        next = opt['next']
                        break
            else:
                next = wf["run"]

            if not next: 
                print('Workflow finished.',flush=True)
                return

            try:
                act = declare[next]
                name = act['name']
                fun = act['fun']
                args = act['args'] if 'args' in act else {}
                obj = self.scene.active_objs_by_name[name] if name in self.scene.active_objs_by_name else self.active_plugins_by_name[name]
                
                print('error', err,flush=True)
                print('signal',fun,flush=True)
                print('val',val,flush=True)
                print('args',args,flush=True)
                eval(f'obj.signal_{fun}(*val,**args)')
            except:
                traceback.print_exc()
                print('Workflow stopped!',flush=True)
                return 

            self.scene.actions.append((task,(next,)))

        task(None)

    def stop(self):
        self.running = False
        self.scene.restore()
        pass
    
    def set(self,workflow):
        self.scene.profile['workflow'] = json.loads(workflow)

    def get(self):
        return self.scene.profile['workflow']
import sys
import socket as s
import os
import json
import numpy as np
from scipy.spatial.transform import Rotation

class Vision:
    def __init__(self,*args,**kwargs):
        self.name = 'PickLight'
        self.actions = list()
        self.result = None,
        self.tmp_dir = kwargs['tmp_dir']
        pass
    
    def idle(self):
        if not self.actions: return True
        fun,args = act = self.actions[0]
        fun(*args)
        self.actions.pop(0)
        return False
    
    def signal_detect(self,eye_to_hand_transform,**kwargs):
        sk = s.socket(s.AF_UNIX,s.SOCK_STREAM)
        sock_path = os.path.join(self.tmp_dir,self.name + '.sock')
        print('detect sock path',sock_path,flush=True)
        sk.connect(sock_path)
        buf = b''
        self.result = None,[]
        
        def task(buf:bytes):
            try:
                c = sk.recv(1024,s.MSG_DONTWAIT)
                if not c: 
                    pick_points = eval(buf.decode())
                    raise ConnectionResetError
                buf += c
                self.actions.append((task,(buf,)))
            except BlockingIOError: self.actions.append((task,(buf,)))
            except (ConnectionResetError,BrokenPipeError):
                print('pick points', pick_points,flush=True)
                pick_point = np.array(pick_points[0]['mat'])
                pick_point = eye_to_hand_transform @ pick_point['mat']
                R = pick_point[:3, :3]
                T = pick_point[:3, 3]
                pos = T[0],T[1],T[2]
                rot = Rotation.from_matrix(R).as_euler('xyz')
                rot = rot[0],rot[1],rot[2]
                self.result = None,[T[0],T[1],T[2],rot[0],rot[1],rot[2]]
                sk.close()
            except SyntaxError:pass

        self.actions.append((task,(buf,)))

    def signal_check(self,eye_to_hand_transform,**kwargs):
        self.result = None,()

        pick_points = kwargs['pick_points']
        if not pick_points: 
            self.result = 'failed',
            return

        pick_point = np.array(pick_points.pop(0))
        pick_point = eye_to_hand_transform @ pick_point
        R = pick_point[:3, :3]
        T = pick_point[:3, 3]
        rot = Rotation.from_matrix(R).as_euler('xyz')
        self.result = None,[T[0],T[1],T[2],rot[0],rot[1],rot[2]]
import sys
import socket as s
import os
import json
import numpy as np
from scipy.spatial.transform import Rotation

class Vision:
    def __init__(self):
        self.name = 'PickLight'
        self.actions = list()
        self.result = None,
        pass
    
    def idle(self):
        if not self.actions: 
            return True
        fun,args = act = self.actions[0]
        fun(*args)
        self.actions.pop(0)

        return False
    
    def signal_detect(self,eye_to_hand_transform,**kwargs):
        sk = s.socket(s.AF_UNIX,s.SOCK_STREAM)
        tmp_dir = sys.argv[4]
        sock_path = os.path.join(tmp_dir,self.name + '.sock')
        sk.connect(sock_path)
        buf = b''
        self.result = None,[]
        
        def task(buf:bytes):
            try:
                c = sk.recv(1024,s.MSG_DONTWAIT)
                if not c: 
                    pick_points = np.array(eval(buf.decode()))
                    raise ConnectionResetError
                buf += c
                self.actions.append((task,(buf,)))
            except BlockingIOError: self.actions.append((task,(buf,)))
            except (ConnectionResetError,BrokenPipeError):
                for pick_point in pick_points:
                    pick_point = eye_to_hand_transform @ pick_point
                    R = pick_point[:3, :3]
                    T = pick_point[:3, 3]
                    pos = T[0],T[1],T[2]
                    rot = Rotation.from_matrix(R).as_euler('xyz')
                    rot = rot[0],rot[1],rot[2]
                    self.result[1].append((pos,rot))
                
                sk.close()
                print(self.result)
            except SyntaxError:pass

        self.actions.append((task,(buf,)))
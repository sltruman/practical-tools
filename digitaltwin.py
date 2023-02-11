import os
import socket as s
import json
import traceback
import sys
from time import time
# sys.path.insert(0,'/home/truman/Desktop/bullet3/build_cmake/examples/pybullet')

if len(sys.argv) < 2:
    print("Useage: program <scene_path> <width> <height>")
else:
    scene_path = sys.argv[1]
    sock_path = scene_path + '.sock'
    width = sys.argv[2]
    height = sys.argv[3]
    
    # sk = s.socket(s.AF_INET,s.SOCK_STREAM)
    sk = s.socket(s.AF_UNIX,s.SOCK_STREAM)
    sk.setsockopt(s.SOL_SOCKET,s.SO_REUSEADDR,1)
    if os.path.exists(sock_path): os.remove(sock_path)
    sk.bind(sock_path)
    # sk.bind(('localhost',6000))
    print(f'Serving on {sys.argv[1]}')
    sk.listen(1)

    from digitaltwin import Scene,Editor,Workflow
    scene = Scene(width,height)
    editor = Editor(scene)
    workflow = Workflow(scene)
    scene.load(scene_path)

    conn,addr = sk.accept()
    print(f'Client addr {addr}')
    
    buf = b''
    try:
        while True:
            elapsed = time()
            
            try:
                c = conn.recv(1024,s.MSG_DONTWAIT)
                if not c: raise ConnectionResetError
                buf += c
                for req in buf.splitlines(True):
                    if req[-1] != 0xa:
                        buf = req
                        raise BlockingIOError

                    res = eval(req.decode())
                    if res == None: pass
                    elif type(res) == bytes: conn.sendall(res)
                    else: conn.sendall(json.dumps(res).encode() + b'\n')
                buf = b''
            except SyntaxError:
                traceback.print_exc()
            except BlockingIOError:
                pass
                        
            scene.update_for_tick(1/240.)

            # tick = round(time() - elapsed, 3)
            # if tick > 0.01: print('tick: ',tick)
    except (ConnectionResetError,BrokenPipeError):
        pass
    except:
        traceback.print_exc()
        pass
    
    del workflow
    del editor
    del scene
    sk.close()
    if os.path.exists(sock_path): os.remove(sock_path)
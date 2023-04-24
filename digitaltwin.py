import sys
import os
import socket as s
import json
import traceback
from time import time
import shutil
from digitaltwin import Scene,Editor,Workflow

if len(sys.argv) < 2:
    print("Useage: program <scene_path> <width> <height> <tmp_dir>")
else:
    scene_path = sys.argv[1]
    scene_name = os.path.basename(scene_path)

    width = sys.argv[2]
    height = sys.argv[3]
    tmp_dir = sys.argv[4]

    if os.path.exists(tmp_dir): shutil.rmtree(tmp_dir)
    os.makedirs(tmp_dir,exist_ok=True)
    sock_path = os.path.join(tmp_dir,scene_name + '.sock')

    # sk = s.socket(s.AF_INET,s.SOCK_STREAM)
    sk = s.socket(s.AF_UNIX,s.SOCK_STREAM)
    sk.setsockopt(s.SOL_SOCKET,s.SO_REUSEADDR,1)
    sk.bind(sock_path)
    # sk.bind(('localhost',6000))
    print(f'Serving on {sys.argv[1]}')
    sk.listen(1)

    scene = Scene(width,height,tmp_dir)
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
                    # res = exec(req.decode())
                    if type(res) == tuple: 
                        for v in res: conn.sendall(v)
                    elif type(res) == dict or type(res) == list: 
                        conn.sendall(json.dumps(res).encode() + b'\n')
                    else: pass
                buf = b''
            except SyntaxError: traceback.print_exc()
            except BlockingIOError: pass

            tick = time() - elapsed
            scene.update_for_tick(tick)
            tick = round(tick, 3)
            print(end='',flush=True)
    except (ConnectionResetError,BrokenPipeError):
        pass
    except:
        traceback.print_exc()
        pass
    
    del workflow
    del editor
    del scene
    sk.close()
    if os.path.exists(tmp_dir): shutil.rmtree(tmp_dir)
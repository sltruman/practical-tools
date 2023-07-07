import sys
import os
import socket as s
import json
import traceback
from time import time,sleep
import shutil

if len(sys.argv) < 2:
    print("Useage: program <width> <height> <scene_path> <data_dir> <tmp_dir>")
else:
    width = int(sys.argv[1])
    height = int(sys.argv[2])
    scene_path = sys.argv[3]
    scene_name = os.path.basename(scene_path)
    data_dir = sys.argv[4]
    tmp_dir = sys.argv[5]

    if os.path.exists(tmp_dir): shutil.rmtree(tmp_dir)
    os.makedirs(tmp_dir,exist_ok=True)
    sock_path = os.path.join(tmp_dir,scene_name + '.sock')
    sk = s.socket(s.AF_UNIX,s.SOCK_STREAM)
    sk.setsockopt(s.SOL_SOCKET,s.SO_REUSEADDR,1)
    sk.bind(sock_path)
    print(f'Serving on {sock_path}',flush=True)
    sk.listen(1)

    from practistyle import Scene,Editor,Workflow
    scene = Scene(width,height,data_dir,tmp_dir)
    editor = Editor(scene)
    workflow = Workflow(scene)
    scene.load(scene_path)
    
    conn,addr = sk.accept()

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

                    if type(res) == bytes:
                        conn.sendall(res)
                    elif type(res) == dict or type(res) == list: 
                        conn.sendall(json.dumps(res).encode() + b'\n')
                    elif type(res) == tuple: 
                        for v in res: conn.sendall(v)
                buf = b''
            except SyntaxError: print(traceback.print_exc(),flush=True)
            except BlockingIOError: pass

            tick = time() - elapsed
            scene.update_for_tick(tick)
            # tick = round(tick, 3)
            # if tick < 0.010:
            #     print(tick,flush=True)
    except (ConnectionResetError,BrokenPipeError):
        pass
    except:
        print(traceback.print_exc(),flush=True)
        pass

    del workflow
    del editor
    del scene
    sk.close()
    if os.path.exists(tmp_dir): shutil.rmtree(tmp_dir)
    print('digitaltwin exit!',flush=True)
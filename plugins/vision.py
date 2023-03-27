import socket as s

class Vision:
    def __init__(self):
        self.busying = False
        pass

    def idle(self):
        return not self.busying

    def signal_detect(self,*args,**kwargs):
        self.busying = True

        import sys
        import socket as s
        import os
        try:
            sk = s.socket(s.AF_UNIX,s.SOCK_STREAM)
            scene_path = sys.argv[1]
            scene_name = os.path.basename(scene_path)
            width = sys.argv[2]
            height = sys.argv[3]
            tmp_dir = sys.argv[4]
            sock_path = os.path.join(tmp_dir,'VisionPickLightdetect.sock')
            sk.connect(tmp_dir)
            c = sk.recv(1024)
            if not c: raise ConnectionResetError
            buf += c
            for req in buf.splitlines(True):
                pass
            sk.close()
        except:
            pass

        self.result = None,[]
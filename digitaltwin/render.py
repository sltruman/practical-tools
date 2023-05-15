# import pybullet as p
# import numpy as np
# import os
# from scipy.spatial.transform import Rotation
# import multiprocessing as mp
# import socket as s
# from time import time,sleep
# import traceback
# import json

# from ctypes import *

# try:
#     rt = CDLL('librt.so')
# except:
#     rt = CDLL('librt.so.1')

# ftok = rt.ftok
# ftok.argtypes = [c_char_p,c_int]
# ftok.restype = c_int

# shmget = rt.shmget
# shmget.argtypes = [c_int, c_size_t, c_int]  
# shmget.restype = c_int  

# shmat = rt.shmat  
# shmat.argtypes = [c_int, POINTER(c_void_p), c_int]
# shmat.restype = c_void_p

# shmdt = rt.shmdt
# shmdt.argtypes = [POINTER(c_void_p)]
# shmdt.restype = c_int

# shmctl = rt.shmctl
# shmctl.argtypes = [c_int, c_int, POINTER(c_void_p)]
# shmctl.restype = c_int


# class Render:
#     def __init__(self,scene):
#         self.scene = scene
#         self.p = mp.Process(target=run,args=(scene,))
#         self.p.start()
#         pass

#     def __del__(self):
#         self.p.kill()
#         pass

#     def sync_bodies(self):
#         pass
    
#     def draw_points(self):
#         pass

#     def draw_lines(self):
#         pass

# def run(scene):
#     shmkey = ftok(scene.scene_path.encode(), 1)
#     texture_size = scene.width*scene.height*4
#     shmid = shmget(shmkey, texture_size, 512 | 438)
#     if shmid == -1:
#         print ("Failed to create shared memory!",flush=True)
#         exit(-1)

#     shmaddr = shmat(shmid, None, 0)
#     id = p.connect(p.SHARED_MEMORY,options=f'--width={scene.width} --height={scene.height} --shared_memory_key=12345')

#     def rtt():
#         _,_,pixels,depth_pixels,_ = p.getCameraImage(scene.width,scene.height,renderer=p.ER_BULLET_HARDWARE_OPENGL,physicsClientId=id)
#         return pixels.tobytes(),depth_pixels.tobytes()

#     while True:
#         try:
#             rgba,_ = rtt()
#             texture = create_string_buffer(rgba,texture_size)
#             memmove(shmaddr,texture,texture_size)
#             sleep(0.050)
#             pass
#         except SyntaxError: traceback.print_exc()
#         except BlockingIOError: pass

# # def run():
# #     from math import pi, sin, cos
# #     from direct.task import Task
# #     from direct.showbase.ShowBase import ShowBase
# #     from direct.filter.CommonFilters import CommonFilters
# #     from direct.filter.FilterManager import FilterManager
# #     from panda3d.core import NodePath

# #     class App(ShowBase):
# #         def __init__(self):
# #             ShowBase.__init__(self)
# #             self.scene = self.loader.loadModel("models/environment")
# #             self.scene.reparentTo(self.render)
# #             self.scene.setScale(0.25, 0.25, 0.25)
# #             self.scene.setPos(-8, 42, 0)
# #             self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")

# #             self.buffer = self.win.makeTextureBuffer("hello", 256, 256)
# #             self.altCam = self.makeCamera(self.buffer)
# #             self.altCam.reparentTo(self.render)
# #             self.altCam.setPos(0, -10, 0)
            
# #         def spinCameraTask(self, task):
# #             angleDegrees = task.time * 6.0
# #             angleRadians = angleDegrees * (pi / 180.0)
# #             self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 3)
# #             self.camera.setHpr(angleDegrees, 0, 0)
# #             texture = self.buffer.getTexture()
# #             print(texture.getRamImageAs('RGBA'))

# #             return Task.cont

# #     app = App()
# #     app.run()
# #     pass

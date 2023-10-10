import sys
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
import simplepbr

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        simplepbr.init()

        # quit when esc is pressed
        self.accept('escape',sys.exit)

        #base.disableMouse()
        # load the box model
        box = self.loader.loadModel("./casing.glb")
        box.reparentTo(render)



app = MyApp()
app.run()
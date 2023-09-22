import sys
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *


class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)


        # quit when esc is pressed
        self.accept('escape',sys.exit)

        #base.disableMouse()


        # load the box model
        box = self.loader.loadModel("models/box")
        box.reparentTo(render)
        box.setScale(2.0, 2.0, 2.0)
        box.setPos(8, 50, 0)

        panda = base.loader.loadModel("models/panda")
        panda.reparentTo(render)
        panda.setPos(0, 10, 0)
        panda.setScale(0.1, 0.1, 0.1)
        cNodePanda = panda.attachNewNode(CollisionNode('cnode_panda'))
        cNodePanda.node().addSolid(CollisionSphere(0,0,5,5))
        cNodePanda.show()

        # CollisionTraverser  and a Collision Handler is set up
        self.picker = CollisionTraverser()
        self.picker.showCollisions(render)
        self.pq = CollisionHandlerQueue() 

        self.pickerNode = CollisionNode('mouseRay')
        self.pickerNP = camera.attachNewNode(self.pickerNode)
        self.pickerNode.setFromCollideMask(BitMask32.bit(1))
        box.setCollideMask(BitMask32.bit(1)) 
        panda.setCollideMask(BitMask32.bit(1))

        self.pickerRay = CollisionRay()
        self.pickerNode.addSolid(self.pickerRay)
        self.picker.addCollider(self.pickerNP,self.pq)

        self.accept("mouse1",self.mouseClick)


    def mouseClick(self):
        print('mouse click')
        # check if we have access to the mouse
        if base.mouseWatcherNode.hasMouse():

            # get the mouse position
            mpos = base.mouseWatcherNode.getMouse()

            # set the position of the ray based on the mouse position
            print(mpos.getX(),mpos.getY())
            self.pickerRay.setFromLens(base.camNode,mpos.getX(),mpos.getY())
            self.picker.traverse(render)
            # if we have hit something sort the hits so that the closest is first and highlight the node
            if self.pq.getNumEntries() > 0:
                self.pq.sortEntries()
                pickedObj = self.pq.getEntry(0).getIntoNodePath()
                print('click on ' + pickedObj.getName())

app = MyApp()
app.run()
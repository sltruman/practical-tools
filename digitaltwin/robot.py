import pybullet as p

class Robot:
    def __init__(self,scene,**kwargs):
        self.pos = kwargs['pos']
        self.rpy = kwargs['rpy']
        self.set_base(kwargs['base'])
        self.set_end_effector(kwargs['end_effector'])
        pass
    
    def update(self,dt):
        pass

    def set_base(self,base):
        self.base = base
        if 'id' in vars(self): p.removeBody(self.id)
        self.id = p.loadURDF(self.base, self.pos, p.getQuaternionFromEuler(self.rpy),useFixedBase=True)
        return self.id

    def set_end_effector(self,end_effector):
        self.end_effector = end_effector
        if 'end_effector_id' in vars(self): p.removeBody(self.end_effector_id)
        num_joints = p.getNumJoints(self.id)
        pos,orn,_,_,_,_ = p.getLinkState(self.id,num_joints-1)

        self.end_effector_id = p.loadURDF(self.end_effector, pos, orn)
        p.setCollisionFilterGroupMask(self.end_effector_id,-1,0x7fffffff,0x7ffffff0)
        self.joint_end_effector = p.createConstraint(self.id,num_joints-1,self.end_effector_id,-1,p.JOINT_FIXED,[0,0,1],[0,0,0],[0,0,0])
        
        return self.end_effector_id

    def properties(self):
        pos,orn = p.getBasePositionAndOrientation(self.id)
        rpy = p.getEulerFromQuaternion(orn)
        return dict(id=self.id,kind='Robot',base=self.base,pos=pos,rpy=rpy,end_effector=self.end_effector)
    
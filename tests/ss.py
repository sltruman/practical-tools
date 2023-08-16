import numpy as np
from numpy import array
from numpy.linalg import norm
import matplotlib.pyplot as plt
plt.title("title")#括号当中输入标题的名称

def robot():
    p1 = array([-0.05,0.05,0])
    p2 = array([0.05,0.05,0])
    p3 = array([0.05,-0.05,0])
    p4 = array([-0.05,-0.05,0])

    plt.scatter(p1[0],p1[1])
    plt.scatter(p2[0],p2[1])
    plt.scatter(p3[0],p3[1])
    plt.scatter(p4[0],p4[1])
    plt.plot([p1[0],p2[0],p3[0],p1[0]],[p2[1],p2[1],p3[1],p1[1]])
    plt.plot([p3[0],p4[0],p1[0],p3[0]],[p3[1],p4[1],p1[1],p3[1]])

    o = (p1 + p2 + p3 + p4) / 4
    plt.scatter(o[0],o[1])
    return o

def camera():
    p1 = array([-0.04935893,-0.04927776,-0.49868192])
    p2 = array([ 0.04940569,-0.04927776,-0.49868192])
    p3 = array([ 0.04940569, 0.04944738,-0.49868192])
    p4 = array([-0.04935893, 0.04944738,-0.49868192])

    plt.scatter(p1[0],p1[1])
    plt.scatter(p2[0],p2[1])
    plt.scatter(p3[0],p3[1])
    plt.scatter(p4[0],p4[1])
    plt.plot([p1[0],p2[0],p3[0],p1[0]],[p2[1],p2[1],p3[1],p1[1]])
    plt.plot([p3[0],p4[0],p1[0],p3[0]],[p3[1],p4[1],p1[1],p3[1]])

    o = (p1 + p2 + p3 + p4) / 4
    plt.scatter(o[0],o[1])
    return o


np.set_printoptions(suppress=True)
tohand = robot() - camera()
print(tohand)

# plt.scatter(tohand[0],tohand[1])



plt.show()

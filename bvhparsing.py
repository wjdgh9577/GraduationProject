import pybullet as p
import numpy as np
from math import radians, cos, sin
from bvh import Bvh
import math
import time

def transpose(pos):
    return np.array([[1, 0, 0, pos[0]],
                     [0, 1, 0, pos[1]],
                     [0, 0, 1, pos[2]],
                     [0, 0, 0, 1]])

def xRotation(seta):
    return np.array([[1, 0, 0, 0],
                     [0, cos(radians(seta)), -sin(radians(seta)), 0],
                     [0, np.sin(seta*np.pi/180), np.cos(radians(seta)), 0],
                     [0, 0, 0, 1]])

def yRotation(seta):
    return np.array([[cos(radians(seta)), 0, sin(radians(seta)), 0],
                     [0, 1, 0, 0],
                     [-sin(radians(seta)), 0, cos(seta*np.pi/180), 0],
                     [0, 0, 0, 1]])

def zRotation(seta):
    return np.array([[cos(radians(seta)), -sin(radians(seta)), 0, 0],
                     [sin(radians(seta)), cos(radians(seta)), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)


# rotation matrix -> euler angle
def rotationMatrixToEulerAngles(R) :
    
    sy = math.sqrt(R[2,2] * R[2,2] +  R[2,0] * R[2,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = -math.atan2(-R[2,1], sy)
        y = math.atan2(-R[2,0] , R[2,2])
        z = -math.atan2(R[0,1], R[1,1])
    else :
        x = -math.atan2(-R[2,1], sy)
        y = math.atan2(R[2,0], R[2,2])
        z = -0
    #phi = 0.0
    #if isclose(R[2,0],-1.0):
    #    theta = math.pi/2.0
    #    psi = math.atan2(R[0,1],R[0,2])
    #elif isclose(R[2,0],1.0):
    #    theta = -math.pi/2.0
    #    psi = math.atan2(-R[0,1],-R[0,2])
    #else:
    #    theta = -math.asin(R[2,0])
    #    cos_theta = math.cos(theta)
    #    psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
    #    phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    #return np.array([psi, theta, phi])

    return np.array([z, x, y])



class Joint:
    def __init__(self, name):
        self.name = name
        self.offset = None
        self.channels = None

        self.transform = None
        self.euler = None
    def transformation(self):
        return self.transform @ np.array([0, 0, 0, 1])

def scan(mocap):
    joints = list()   #[Hips, ToSpine, Spine, ...]
    tree = dict()   #{parent_joint_name : [child_joint_class1, child_joint_class2, ...]}
    for name in mocap.get_joints_names():
        joints.append(name)
        joint = Joint(name)
        joint.offset = np.array(mocap.joint_offset(name))
        joint.channels = mocap.joint_channels(name)
        joint.euler = np.array(mocap.joint_offset(name))
        parent = mocap.joint_parent(name)
        if parent is None:
            tree['ROOT'] = [joint]
        else:
            if parent.name in tree:
                tree[parent.name].append(joint)
            else:
                tree[parent.name] = [joint]

    return joints, tree

def rotation(mocap, tree, frame_number, joint='ROOT', matrix=np.identity(4)):
    if joint not in tree:
        return

    for child in tree[joint]:
        name = child.name
        offset = child.offset
        channels = child.channels
        rotate = mocap.frame_joint_channels(frame_number, name, channels)
        if joint == 'ROOT':
            pos = 3
        else:
            pos = 0
        M = transpose(offset)
        for i in range(pos, pos+3):
            M = M @ rotation1(channels[i], rotate[i])
        transform = matrix @ M
        child.transform = transform
        child.euler = rotationMatrixToEulerAngles(transform) * 180 / math.pi
        
        print(child.euler)
        rotation(mocap, tree, frame_number, name, transform)

def rotation1(channel, rotate):
    _channel = str.lower(channel)
    _rotate = rotate
    return {'xrotation' : xRotation(_rotate), 'yrotation' : yRotation(_rotate), 'zrotation' : zRotation(_rotate)}[_channel]

def draw(tree, frame_number, joint='ROOT', parentPos=None, color=[0, 1, 0]):
    if joint not in tree:
        return
    C = color.copy()
    c = C.pop(0)
    C.append(c)

    for child in tree[joint]:
        childPos = child.transformation()[:-1]#/100
        if parentPos is not None:
            childPos += np.array(mocap.frame_joint_channels(frame_number, 'Hips', ['Xposition', 'Yposition', 'Zposition']))/100
            p.addUserDebugLine(parentPos, childPos, color)
        else:
            childPos += np.array(mocap.frame_joint_channels(frame_number, child.name, ['Xposition', 'Yposition', 'Zposition']))/100

        draw(tree, frame_number, child.name, childPos, C)

with open("Male2_bvh/Male2_A3_SwingArms.bvh") as f:
    mocap = Bvh(f.read())

joints, tree = scan(mocap)
print(len(joints))
physicsClientID = p.connect(p.GUI)

p.setGravity(0, 0, -10)

timeStep = mocap.frame_time
frames = mocap.nframes

while p.isConnected():
    for n in range(frames):
        rotation(mocap, tree, n)
        draw(tree, n)
        time.sleep(timeStep)
        p.removeAllUserDebugItems()
        #time.sleep(timeStep)

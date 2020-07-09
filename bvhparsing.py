import pybullet as p
import numpy as np
from math import radians, cos, sin
from bvh import Bvh
import time
'''
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

class Joint:
    def __init__(self, name):
        self.name = name
        self.offset = None
        self.channels = None

        self.transform = None

    def transformation(self):
        return self.transform @ np.array([0,0,0,1])

def scan(mocap):
    joints = list()   #[Hips, ToSpine, Spine, ...]
    tree = dict()   #{parent_joint_name : [child_joint_class1, child_joint_class2, ...]}
    for name in mocap.get_joints_names():
        joints.append(name)
        joint = Joint(name)
        joint.offset = (xRotation(90) @ np.append(np.array(mocap.joint_offset(name)), [1]))[:-1]
        joint.channels = mocap.joint_channels(name)
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
        rotate = mocap.frame_joint_channels(frame_number, child.name, child.channels)
        if joint == 'ROOT':
            M = transpose(child.offset) @ zRotation(rotate[3]) @ xRotation(rotate[4]) @ yRotation(rotate[5])
            transform = M
        else:
            M = transpose(child.offset) @ zRotation(rotate[0]) @ xRotation(rotate[1]) @ yRotation(rotate[2])
            transform = matrix @ M
        child.transform = transform
        rotation(mocap, tree, frame_number, child.name, child.transform)

def draw(tree, frame_number, joint='ROOT', parentPos=None, color=[0, 1, 0]):
    if joint not in tree:
        return
    C = color.copy()
    c = C.pop(0)
    C.append(c)

    for child in tree[joint]:
        childPos = child.transformation()[:-1]/100
        if parentPos is not None:
            childPos += np.array(mocap.frame_joint_channels(frame_number, 'Hips', ['Xposition', 'Yposition', 'Zposition']))/100
            p.addUserDebugLine(parentPos, childPos, color)
        else:
            childPos += np.array(mocap.frame_joint_channels(frame_number, child.name, ['Xposition', 'Yposition', 'Zposition']))/100

        draw(tree, frame_number, child.name, childPos, C)

with open("Male2_bvh/Male2_A1_Stand.bvh") as f:
    mocap = Bvh(f.read())

joints, tree = scan(mocap)

physicsClientID = p.connect(p.GUI)

p.setGravity(0, 0, -10)
#planeID = p.loadURDF("bullet3/examples/pybullet/gym/pybullet_data/plane.urdf")

timeStep = mocap.frame_time
frames = mocap.nframes

while p.isConnected():
    for n in range(frames):
        rotation(mocap, tree, n)
        draw(tree, n)
        time.sleep(timeStep)
        p.removeAllUserDebugItems()
    time.sleep(timeStep)
'''
class Joint:
    def __init__(self, name):
        self.name = name
        self.offset = None
        self.channels = None

        self.transform = None

    def transformation(self):
        return self.transform @ np.array([0,0,0,1])

def xRotation(seta):
    return np.array([[1, 0, 0],
                     [0, cos(radians(seta)), -sin(radians(seta))],
                     [0, np.sin(seta*np.pi/180), np.cos(radians(seta))]])

def yRotation(seta):
    return np.array([[cos(radians(seta)), 0, sin(radians(seta))],
                     [0, 1, 0],
                     [-sin(radians(seta)), 0, cos(seta*np.pi/180)]])

def zRotation(seta):
    return np.array([[cos(radians(seta)), -sin(radians(seta)), 0],
                     [sin(radians(seta)), cos(radians(seta)), 0],
                     [0, 0, 1]])

def scan(mocap):
    joints = list()   #[Hips, ToSpine, Spine, ...]
    tree = dict()   #{parent_joint_name : [child_joint_class1, child_joint_class2, ...]}
    for name in mocap.get_joints_names():
        joints.append(name)
        joint = Joint(name)
        joint.offset = xRotation(90) @ np.array(mocap.joint_offset(name))
        joint.channels = mocap.joint_channels(name)
        parent = mocap.joint_parent(name)
        if parent is None:
            tree['ROOT'] = [joint]
        else:
            if parent.name in tree:
                tree[parent.name].append(joint)
            else:
                tree[parent.name] = [joint]

    return joints, tree
import pybullet as p
import numpy as np
from bvh import Bvh
import time

def xRotation(seta):
    return np.array([[1, 0, 0],
                     [0, np.cos(seta*np.pi/180), -np.sin(seta*np.pi/180)],
                     [0, np.sin(seta*np.pi/180), np.cos(seta*np.pi/180)]])

def yRotation(seta):
    return np.array([[np.cos(seta*np.pi/180), 0, -np.sin(seta*np.pi/180)],
                     [0, 1, 0],
                     [np.sin(seta*np.pi/180), 0, np.cos(seta*np.pi/180)]])

def zRotation(seta):
    return np.array([[np.cos(seta*np.pi/180), -np.sin(seta*np.pi/180), 0],
                     [np.sin(seta*np.pi/180), np.cos(seta*np.pi/180), 0],
                     [0, 0, 1]])

class Joint:
    def __init__(self, name):
        self.name = name
        self.offset = None
        self.channels = None
        self.rotate = None

    def rotation(self):
        return self.rotate @ self.offset

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

def rotation(mocap, tree, frame_number, joint='ROOT', matrix=np.identity(3)):
    if joint not in tree:
        return

    for child in tree[joint]:
        rotate = mocap.frame_joint_channels(frame_number, child.name, child.channels)
        if joint == 'ROOT':
            rotate = yRotation(rotate[5]) @ xRotation(rotate[4]) @ zRotation(rotate[3]) @ matrix
        else:
            rotate = yRotation(rotate[2]) @ xRotation(rotate[1]) @ zRotation(rotate[0]) @ matrix
        child.rotate = rotate
        rotation(mocap, tree, frame_number, child.name, child.rotate)

def draw(tree, frame_number, joint='ROOT', parentPos=None, color=[0, 1, 0]):
    if joint not in tree:
        return
    C = color.copy()
    c = C.pop(0)
    C.append(c)
    for child in tree[joint]:
        childPos = child.rotation()/50
        if parentPos is not None:
            childPos += parentPos
            p.addUserDebugLine(parentPos, childPos, color)
        else:
            childPos += np.array(mocap.frame_joint_channels(frame_number, child.name, ['Xposition', 'Yposition', 'Zposition']))/50

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
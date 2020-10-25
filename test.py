#os.sys.path.insert is only needed when pybullet is not installed
#but running from github repo instead
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
parentdir = os.path.join(currentdir, "../gym")
os.sys.path.insert(0, parentdir)

import pybullet as p
import pybullet_data

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

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

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
        self.euler = None
        self.transform = None

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
            #p.resetBasePositionAndOrientation(obj[0], np.array([channels[1], channels[2] , channels[0]]), [0, 0, 0, 1])
        else:
            pos = 0
        M = transpose(offset)
        for i in range(pos, pos+3):
            M = M @ rotation1(channels[i], rotate[i])
        M = xRotation(90) @ M
        transform = matrix @ M
        
        child.transform = transform
        
        child.euler = rotationMatrixToEulerAngles(M)# * 180 / math.pi
        findjoint(child)
        
        rotation(mocap, tree, frame_number, name, transform)
        
def findjoint(child):
    name = child.name
    maxForce = 1
    numJoints = p.getNumJoints(obj[0])
    for i in range(numJoints):
        if name==str(p.getJointInfo(obj[0], i)[1])[2:-1].split('_')[0]:
            print(name)
            print(str(p.getJointInfo(obj[0], i)[1])[2:-1].split('_')[0])
            if str(p.getJointInfo(obj[0], i)[1])[2:-1].split('_')[1]=='z':
                p.setJointMotorControl2(bodyIndex=obj[0], jointIndex=p.getJointInfo(obj[0], i)[0], controlMode=p.POSITION_CONTROL, targetPosition=child.euler[0],force=maxForce)        
            elif str(p.getJointInfo(obj[0], i)[1])[2:-1].split('_')[1]=='x':
                p.setJointMotorControl2(bodyIndex=obj[0], jointIndex=p.getJointInfo(obj[0], i)[0], controlMode=p.POSITION_CONTROL, targetPosition=child.euler[1],force=maxForce)
            elif str(p.getJointInfo(obj[0], i)[1])[2:-1].split('_')[1]=='y':
                p.setJointMotorControl2(bodyIndex=obj[0], jointIndex=p.getJointInfo(obj[0], i)[0], controlMode=p.POSITION_CONTROL, targetPosition=child.euler[2],force=maxForce)
            

def rotation1(channel, rotate):
    _channel = str.lower(channel)
    _rotate = rotate
    return {'xrotation' : xRotation(_rotate), 'yrotation' : yRotation(_rotate), 'zrotation' : zRotation(_rotate)}[_channel]





#choose connection method: GUI, DIRECT, SHARED_MEMORY
p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
#load URDF, given a relative or absolute file+path
obj = p.loadMJCF("model.xml")


with open("Male2_bvh/Male2_A3_SwingArms.bvh") as f:
    mocap = Bvh(f.read())

joints, tree = scan(mocap)


posX = 0
posY = 3
posZ = 2

#query the number of joints of the object
print(obj[0])
numJoints = p.getNumJoints(obj[0])
#for i in range(numJoints):
#    print(str(p.getJointInfo(obj[0], i)[1])[2:-1].split('_'))
    #print(p.getJointStateMultiDof(obj[0], i))
#    print()
#print("Number of Joints:", numJoints)



joints, tree = scan(mocap)
print(len(joints))

#set the gravity acceleration
p.setGravity(0, 0, 0)
p.setRealTimeSimulation(True)



f = 0
start = time.time()
#while True:#time.time() < t_end:
#    p.stepSimulation()
#    current = time.time()
    #if f % 30 == 0:
        #print(current - start)
#    time.sleep(0.02)
#    f += 1
    #posAndOrn = pybullet.getBasePositionAndOrientation(obj[0])
    #print(posAndOrn)

timeStep = mocap.frame_time
frames = mocap.nframes


while p.isConnected():
    for n in range(frames):
        rotation(mocap, tree, n)
        
        time.sleep(timeStep)
        p.removeAllUserDebugItems()
        #time.sleep(timeStep)



print("finished")
#remove all objects
p.resetSimulation()

#disconnect from the physics server
p.disconnect()

import pybullet as p
import numpy as np
import time

def xRotation(seta):
    return np.array([[1, 0, 0],
                     [0, np.cos(seta), -np.sin(seta)],
                     [0, np.sin(seta), np.cos(seta)]])

def yRotation(seta):
    return np.array([[np.cos(seta), 0, -np.sin(seta)],
                     [0, 1, 0],
                     [np.sin(seta), 0, np.cos(seta)]])

def zRotation(seta):
    return np.array([[np.cos(seta*np.pi/180), -np.sin(seta*np.pi/180), 0],
                     [np.sin(seta*np.pi/180), np.cos(seta*np.pi/180), 0],
                     [0, 0, 1]])

physicsClientID = p.connect(p.GUI)

p.setGravity(0, 0, -10)

dot1 = np.array([0, 0, 1])
vec1 = np.array([1, 1, 1])
dot2 = dot1 + vec1
vec2 = np.array([-1, -1, 1])
dot3 = dot2 + vec2
t = 0

while p.isConnected():
    p.addUserDebugLine(dot1, dot2, [1, 0, 0])
    p.addUserDebugLine(dot2, dot3, [0, 0, 0])

    vec1 = zRotation(t) @ np.array([1,1,1])
    vec2 = xRotation(t) @ np.array([-1,-1,1])
    dot2 = dot1 + vec1
    dot3 = dot2 + vec2
    time.sleep(0.1)
    print(t)
    t += 0.1
    p.removeAllUserDebugItems()
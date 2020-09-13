#os.sys.path.insert is only needed when pybullet is not installed
#but running from github repo instead
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
parentdir = os.path.join(currentdir, "../gym")
os.sys.path.insert(0, parentdir)

import pybullet as p
import pybullet_data

import time

#choose connection method: GUI, DIRECT, SHARED_MEMORY
p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
#load URDF, given a relative or absolute file+path
obj = p.loadMJCF(os.path.join(pybullet_data.getDataPath(), "mjcf/model.xml"))

posX = 0
posY = 3
posZ = 2

#query the number of joints of the object
print(obj[0])
numJoints = p.getNumJoints(obj[0])
for i in range(numJoints):
    print(p.getJointInfo(obj[0], i))
    print(p.getJointStateMultiDof(obj[0], i))
    print()
print("Number of Joints:", numJoints)

#set the gravity acceleration
p.setGravity(0, 0, -9.8)
#p.setRealTimeSimulation(True)

f = 0
start = time.time()
while True:#time.time() < t_end:
    p.stepSimulation()
    current = time.time()
    if f % 30 == 0:
        print(current - start)
    time.sleep(0.02)
    f += 1
    #posAndOrn = pybullet.getBasePositionAndOrientation(obj[0])
    #print(posAndOrn)

print("finished")
#remove all objects
p.resetSimulation()

#disconnect from the physics server
p.disconnect()

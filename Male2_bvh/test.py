from bvh import Bvh

with open('Male2_A1_Stand.bvh') as f:
    mocap = Bvh(f.read())

for name in mocap.get_joints_names():
    print(name + " : ", mocap.frame_joint_channels(0, name, mocap.joint_channels(name)))

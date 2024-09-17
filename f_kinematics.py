

import numpy as np

# Function to compute DH transformation matrix
def dh_transform(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Forward Kinematics for a 6 DOF robot
def forward_kinematics(joint_angles, dh_params):
    T = np.eye(4)
    for i in range(6):
        T = np.dot(T, dh_transform(joint_angles[i], dh_params[i][0], dh_params[i][1], dh_params[i][2]))
    return T

# Define DH parameters: (d, a, alpha)
# You need to adjust these for your specific robot
dh_params = [
    (0.1, 0.5, 0),  # Joint 1
    (0.2, 0.3, np.pi/2),  # Joint 2
    (0.15, 0.2, 0),  # Joint 3
    (0, 0.1, -np.pi/2),  # Joint 4
    (0, 0, np.pi/2),  # Joint 5
    (0.1, 0, 0)  # Joint 6
]

# Example joint angles in radians
joint_angles = [0, np.pi/4, np.pi/3, np.pi/6, np.pi/4, 0]

# Compute the end-effector position and orientation
end_effector_pose = forward_kinematics(joint_angles, dh_params)
print("End-effector pose (Forward Kinematics):")
print(end_effector_pose)

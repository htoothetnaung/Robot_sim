# Jacobian-based Inverse Kinematics using the Newton-Raphson method
def inverse_kinematics(desired_pose, initial_angles, dh_params, max_iters=100, tol=1e-6):
    joint_angles = np.array(initial_angles)

    for _ in range(max_iters):
        # Calculate current pose using forward kinematics
        current_pose = forward_kinematics(joint_angles, dh_params)
        
        # Calculate position and orientation error
        pose_error = desired_pose[:3, 3] - current_pose[:3, 3]  # position error
        
        if np.linalg.norm(pose_error) < tol:
            break

        # Compute the Jacobian numerically (you can use an analytical method too)
        J = numerical_jacobian(joint_angles, dh_params)

        # Update joint angles using the inverse Jacobian
        joint_angles += np.dot(np.linalg.pinv(J), pose_error)
    
    return joint_angles

# Numerical Jacobian calculation
def numerical_jacobian(joint_angles, dh_params, delta=1e-6):
    J = np.zeros((3, 6))
    for i in range(6):
        # Perturb the joint angles
        perturbed_angles = np.copy(joint_angles)
        perturbed_angles[i] += delta
        
        # Compute the perturbed pose
        perturbed_pose = forward_kinematics(perturbed_angles, dh_params)
        original_pose = forward_kinematics(joint_angles, dh_params)
        
        # Calculate the derivative
        J[:, i] = (perturbed_pose[:3, 3] - original_pose[:3, 3]) / delta
    
    return J

# Example usage for IK
desired_pose = np.eye(4)  # Define a desired pose
desired_pose[:3, 3] = [0.5, 0.2, 0.3]  # Desired position

# Initial joint angles guess
initial_angles = [0, 0, 0, 0, 0, 0]

# Perform inverse kinematics to get joint angles
ik_solution = inverse_kinematics(desired_pose, initial_angles, dh_params)
print("Joint angles (Inverse Kinematics):")
print(ik_solution)

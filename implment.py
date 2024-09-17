if fingerState == [0,1,0,0,0]:
    desired_pose[:3, 3] = [0.4, 0.2, 0.3]  # Example target position for one finger
    ik_solution = inverse_kinematics(desired_pose, initial_angles, dh_params)
    robot.MoveJ(ik_solution)
elif fingerState == [0,1,1,0,0]:
    desired_pose[:3, 3] = [0.5, 0.1, 0.2]  # Example target for two fingers
    ik_solution = inverse_kinematics(desired_pose, initial_angles, dh_params)
    robot.MoveJ(ik_solution)

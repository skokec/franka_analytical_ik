import numpy as np
import franka_ik_pybind



translation = np.array([0.2, 0.5, 0.2])
quaternion = np.array([1, 0, 0, 0])
q7 = 0
initialJointPosition = np.array([0.0000, 0.0000, 0.0000, -0.9425, 0.0000, 1.1205, 0.0000])

jointPositionAnalytical = franka_ik_pybind.franka_IKCC(
            translation, quaternion, 
            0, initialJointPosition)

print(jointPositionAnalytical)
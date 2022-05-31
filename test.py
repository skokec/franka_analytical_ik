import numpy as np
import franka_ik_pybind


# target_pose = np.array( [0.9999880957989564, -0.00026010595844866424, 0.002118373544817788, 0.0, -0.0002612978731243998, -0.999990181121265, 0.0005623928249146481, 0.0, 0.0021182472458635944, -0.0005629504951206364, -0.9999975980547883, 0.0, 0.3075659649768959, -0.00047487722212063093, 0.4523224403632257, 1.0])
# target_pose = target_pose.reshape(4,4).T

translation = np.array([0.59648, -0.3214, 0.71111])
quaternion = np.array([-0.7584, 0.33941, -0.4620, 0.31002]) # xyzw
q7 = 0.694695
initialJointPosition = np.array([-0.00034485234427340453, -0.7847331501140928,
                                 -0.00048777872079729497, -2.3551600892113274,
                                 -0.0009046530319716893, 1.5725385250250496,
                                  0.694695])
jointPositionAnalytical = franka_ik_pybind.franka_IK(
            translation, quaternion, 
            q7, initialJointPosition)


print(jointPositionAnalytical)
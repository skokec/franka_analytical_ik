import numpy as np
import franka_ik_pybind
from scipy.spatial.transform import Rotation as R


# target_pose = np.array( [0.9999880957989564, -0.00026010595844866424, 0.002118373544817788, 0.0, -0.0002612978731243998, -0.999990181121265, 0.0005623928249146481, 0.0, 0.0021182472458635944, -0.0005629504951206364, -0.9999975980547883, 0.0, 0.3075659649768959, -0.00047487722212063093, 0.4523224403632257, 1.0])
# target_pose = target_pose.reshape(4,4).T

# translation = np.array([0.59648, -0.3214, 0.71111])
# quaternion = np.array([-0.7584, 0.33941, -0.4620, 0.31002]) # xyzw

def move_backward_grasp(transform, standoff = 0.1):
    '''
    transform: [4,4]
    '''
    standoff_mat = np.eye(4)
    standoff_mat[2] = -standoff
    new = np.matmul(transform,standoff_mat)
    return new[:3,3]

def check_limits(joint_angles, q_min_epsilon=0.2, q_max_epsilon=0.2):
    '''
    q: [4,7]
    '''
    q_min = np.array([-2.89, -1.76, -2.89, -3.07, -2.89, 0, -2.89]) + q_min_epsilon
    q_max = np.array([2.89, 1.76, 2.89, -0.0698, 2.89, 3.75, 2.89]) - q_max_epsilon
    for i in range(joint_angles.shape[0]):
        q = joint_angles[i,:]
        if np.isnan(np.sum(q)):
            continue
        lower_mask = (q >= q_min).all()
        upper_mask = (q <= q_max).all()
        if lower_mask and upper_mask:
            return True
    return False

def check_reachability(quaternion, translation, max_iterations=10):
    '''
    quaternion: [4]
    translation: [3]
    '''
    #translation = np.array([0.5344186663627625,  0.06687421351671219, 0.3184833526611328])
    #quaternion = np.array([ 0.4859273373217348, 0.8214059952365719, -0.22271585683462522, -0.19890817214796522]) # xyzw
    angles = np.linspace(-3.14, 3.14, max_iterations)

    fake_target = np.eye(4)
    fake_target[:3,:3] = R.from_quat(quaternion).as_matrix()
    fake_target[:3,3] = translation
    fake_translation = move_backward_grasp(fake_target)

    for angle in angles:
        q7 = angle

        initialJointPosition = np.array([-0.00034485234427340453, -0.7847331501140928,
                                        -0.00048777872079729497, -2.3551600892113274,
                                        -0.0009046530319716893, 1.5725385250250496,
                                        q7])

        jointPositionAnalytical = franka_ik_pybind.franka_IK(
                    translation, quaternion, 
                    q7, initialJointPosition)

        jointPositionAnalytical_fake = franka_ik_pybind.franka_IK(
                    fake_translation.astype(np.float64), quaternion, 
                    q7, initialJointPosition)

        checker = np.sum(np.isnan(np.sum(jointPositionAnalytical, axis=1)))
        checker_fake = np.sum(np.isnan(np.sum(jointPositionAnalytical_fake, axis=1)))
        if (checker < 4) and (checker_fake < 4):
            #print(jointPositionAnalytical)
            if check_limits(jointPositionAnalytical) and check_limits(jointPositionAnalytical_fake):
                # check fake target
                # print(jointPositionAnalytical)
                # print(jointPositionAnalytical_fake)
                return True

            #print(jointPositionAnalytical)
        
    return False


translation = np.array([0.5344186663627625,  0.06687421351671219, 0.3184833526611328])
quaternion = np.array([ 0.4859273373217348, 0.8214059952365719, -0.22271585683462522, -0.19890817214796522]) # xyzw
# translation = np.array([0.5588969230651855,  0.0063073597848415375, 0.2631653845310211])
# quaternion = np.array([-0.5149032426188047, 0.7359848969492421, 0.31914526066961735, -0.302236967949614]) # xyzw

# translation = np.array([0.4459571838378906,  -0.006933244876563549, 0.28426724672317505])
# quaternion = np.array([ 0.755987501127552, 0.06571591799503634, 0.6469619546460547, 0.07486351248469168]) # xyzw


print(check_reachability(translation=translation, quaternion=quaternion, max_iterations=10))

# angles = np.linspace(-3.14, 3.14, 1000)#[-1.7444444444444445]#np.linspace(-3.14, 3.14, 10)
# for angle in angles:

#     q7 = angle
#     #print(q7)

#     initialJointPosition = np.array([-0.00034485234427340453, -0.7847331501140928,
#                                     -0.00048777872079729497, -2.3551600892113274,
#                                     -0.0009046530319716893, 1.5725385250250496,
#                                     q7   ])


#     jointPositionAnalytical = franka_ik_pybind.franka_IK(
#                 translation, quaternion, 
#                 q7, initialJointPosition)


#     checker = np.sum(np.isnan(np.sum(jointPositionAnalytical, axis=1)))
#     if checker < 4:
#         print("Found!")
#         print(jointPositionAnalytical)
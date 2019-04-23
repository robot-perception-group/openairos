import numpy as np
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from openai_ros import robot_gazebo_env
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from openai_ros.openai_ros_common import ROSLauncher


class FetchSimpleEnv(robot_gazebo_env.RobotGazeboEnv):

    def __init__(self, ros_ws_abspath):
        rospy.logdebug("Entered Fetch Env")

        # We launch the ROSlaunch that spawns the robot into the world
        ROSLauncher(rospackage_name="fetch_simple_description",
                    launch_file_name="put_fetchsimple_in_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        self.controllers_list = []

        self.robot_name_space = ""
        self.reset_controls = False

        super(FetchSimpleEnv, self).__init__(controllers_list=self.controllers_list,
                                             robot_name_space=self.robot_name_space,
                                             reset_controls=False,
                                             start_init_physics_parameters=False,
                                             reset_world_or_sim="WORLD")

        # We Start all the ROS related Subscribers and publishers

        self.JOINT_STATES_SUBSCRIBER = '/fetch/joint_states'
        self.join_names = ["joint0",
                           "joint1",
                           "joint2",
                           "joint3",
                           "joint4",
                           "joint5",
                           "joint6"]

        self.gazebo.unpauseSim()
        # Start Move Fetch Object, that checks all systems are ready
        self.move_fetch_object = FetchSimpleMove()
        # Wait until Fetch goes to the init pose
        self.move_fetch_object.set_travel_arm_pose()

        # We pause until the next step
        self.gazebo.pauseSim()

    # RobotGazeboEnv virtual methods
    # ----------------------------

    def get_joints(self):
        return self.joints

    def get_joint_names(self):
        return self.joints.name

    def set_trajectory_joints(self, initial_qpos):

        positions_array = [None] * 7
        positions_array[0] = initial_qpos["joint0"]
        positions_array[1] = initial_qpos["joint1"]
        positions_array[2] = initial_qpos["joint2"]
        positions_array[3] = initial_qpos["joint3"]
        positions_array[4] = initial_qpos["joint4"]
        positions_array[5] = initial_qpos["joint5"]
        positions_array[6] = initial_qpos["joint6"]

        self.move_fetch_object.joint_traj(positions_array)

        return True

    def create_joints_dict(self, joints_positions):
        """
        Based on the Order of the positions, they will be assigned to its joint name
        names_in_order:
          joint0: 0.0
          joint1: 0.0
          joint2: 0.0
          joint3: -1.5
          joint4: 0.0
          joint5: 1.5
          joint6: 0.0
        """

        assert len(joints_positions) == len(
            self.join_names), "Wrong number of joints, there should be "+str(len(self.join_names))
        joints_dict = dict(zip(self.join_names, joints_positions))

        return joints_dict

    def get_ee_pose(self):
        """
        Returns geometry_msgs/PoseStamped
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            geometry_msgs/Pose pose
              geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
              geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
        """
        self.gazebo.unpauseSim()
        gripper_pose = self.move_fetch_object.ee_pose()
        self.gazebo.pauseSim()

        return gripper_pose

    def get_ee_rpy(self):

        gripper_rpy = self.move_fetch_object.ee_rpy()

        return gripper_rpy

    # ParticularEnv methods
    # ----------------------------

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()


class FetchSimpleMove(object):
    def __init__(self):
        rospy.loginfo("Initialising...")

        self.name_joints = ["bellows_joint",
                            "elbow_flex_joint",
                            "forearm_roll_joint",
                            "head_pan_joint",
                            "head_tilt_joint",
                            "l_gripper_finger_joint",
                            "r_gripper_finger_joint",
                            "shoulder_lift_joint",
                            "shoulder_pan_joint",
                            "torso_lift_joint",
                            "upperarm_roll_joint",
                            "wrist_flex_joint",
                            "wrist_roll_joint"]

        self.joint_upper_limits = [0.4,
                                   2.251,
                                   6.27,  # Was none but placed a limit
                                   1.57,
                                   1.45,
                                   0.05,
                                   0.05,
                                   1.518,
                                   1.6056,
                                   0.38615,
                                   6.27,  # Was none but placed a limit
                                   2.16,
                                   6.27,  # Was none but placed a limit
                                   ]
        self.joint_lower_limits = [0.0,
                                   -2.251,
                                   0.0,  # Was none but placed a limit
                                   -1.57,
                                   -0.76,
                                   0.0,
                                   0.0,
                                   -1.221,
                                   -1.6056,
                                   0.0,
                                   0.0,  # Was none but placed a limit
                                   -2.16,
                                   0.0,  # Was none but placed a limit
                                   ]

        self.travel_arm_pose = [0.0,
                                -1.8,  # elbow_flex_joint
                                0.0,  # forearm_roll_joint
                                0.0,
                                0.0,
                                0.05,
                                0.04,
                                0.7,  # shoulder_lift_joint
                                1.32,  # shoulder_pan_joint
                                0.0,  # upperarm_roll_joint
                                0.0,
                                -0.4,  # wrist_flex_joint
                                0.1]

        self.joint_array = len(self.name_joints)*[0.0]

        self.pub_position_array = []
        for joint in self.name_joints:
            topic_name = "/fetch/"+joint+"_position_controller/command"
            self.pub_position_array.append(
                rospy.Publisher(topic_name, Float64, queue_size=1))

        # Wait for publishers to be ready
        self.wait_publishers_to_be_ready()

        self._check_joint_states_ready()

        rospy.Subscriber("/fetch/joint_states", JointState,
                         self.join_state_callback)

    def _check_joint_states_ready(self):
        self.joints_state = None
        while self.joints_state is None and not rospy.is_shutdown():
            try:
                self.joints_state = rospy.wait_for_message(
                    "/fetch/joint_states", JointState, timeout=1.0)
                rospy.logdebug(
                    "Current /fetch/joint_states READY=>" + str(self.joints_state))

            except:
                rospy.logerr(
                    "Current /fetch/joint_states not ready yet, retrying for getting joint_states")
        return self.joints_state

    def join_state_callback(self, msg):
        self.joints_state = msg

    def init_position(self):
        self.move_all_joints(joints_pos_array=self.joint_array)

    def set_travel_arm_pose(self):
        self.move_all_joints(joints_pos_array=self.travel_arm_pose)

    def wait_for_joints_to_get_there(self, desired_pos_array, error=0.2):

        are_equal = False
        rate = rospy.Rate(10)
        rospy.logwarn("Waiting for joint to get to the position")
        while not are_equal and not rospy.is_shutdown():

            current_pos = [self.joints_state.position]

            are_equal = np.allclose(a=current_pos,
                                    b=desired_pos_array,
                                    atol=error)

            rospy.logdebug("are_equal="+str(are_equal))
            rospy.logdebug(str(desired_pos_array))
            rospy.logdebug(str(current_pos))

            rate.sleep()

        rospy.logwarn(
            "Joints are in the desired position with an erro of "+str(error))

    def wait_publishers_to_be_ready(self):

        rate_wait = rospy.Rate(10)
        rospy.logdebug("Waiting for Publishers to be ready...")
        i = 0
        for publisher_obj in self.pub_position_array:
            publisher_ready = False
            while not publisher_ready:
                connection_num = publisher_obj.get_num_connections()
                publisher_ready = connection_num > 0
                rospy.logdebug("Pub joint NOT Ready=" +
                               str(self.name_joints[i]))
                rate_wait.sleep()
            rospy.logdebug("Publisher for joint Ready=" +
                           str(self.name_joints[i]))
            i += 1

    def move_all_joints(self, joints_pos_array):

        assert len(joints_pos_array) == len(
            self.joint_array), "Lengths dont match"
        i = 0
        for angle in joints_pos_array:
            angle_msg = Float64()
            angle_msg.data = angle
            # Publish Joint Position
            self.pub_position_array[i].publish(angle_msg)
            i += 1

        self.wait_for_joints_to_get_there(self.joint_array)
        self.update_joints(new_joints_pos=joints_pos_array)

    def update_joints(self, new_joints_pos):

        i = 0

        assert len(new_joints_pos) == len(
            self.joint_array), "Lengths don't match in Update"

        for new_joint_value in new_joints_pos:
            upper = self.joint_upper_limits[i]
            lower = self.joint_lower_limits[i]
            if upper is None or lower is None:
                self.joint_array[i] = new_joint_value
            else:
                if upper >= new_joint_value >= lower:
                    self.joint_array[i] = new_joint_value
                elif new_joint_value < lower:
                    self.joint_array[i] = lower
                else:
                    self.joint_array[i] = upper
            rospy.logdebug("index =" + str(i))
            rospy.logdebug("length of name_joints =" +
                           str(len(self.name_joints[i])))
            rospy.logdebug("name_joints=" + str(self.name_joints[i]))

            i += 1

    def delta_joints(self, delta_array):
        """
        delta_array = [bellows_joint, elbow_flex_joint, forearm_roll_joint, head_pan_joint, head_tilt_joint,
  l_gripper_finger_joint, r_gripper_finger_joint, shoulder_lift_joint, shoulder_pan_joint,
  torso_lift_joint, upperarm_roll_joint, wrist_flex_joint, wrist_roll_joint]
        :param delta_array:
        :return:
        """
        new_pos_array = len(delta_array)*[0.0]
        i = 0
        for delta in delta_array:
            new_pos_array[i] = self.joint_array[i] + delta
            i += 1

        self.move_all_joints(new_pos_array)

    def get_current_angles(self):
        return self.joint_array
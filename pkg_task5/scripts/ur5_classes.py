'''
This module consist of ur5 classes which will be required by the warehouse control system
'''

#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import rospkg
import yaml


# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import
from hrwros_gazebo.msg import *
from geometry_msgs.msg import *
from pkg_vb_sim.srv import *

from std_srvs.srv import Empty



# pylint: disable=too-many-instance-attributes
# pylint: disable=assignment-from-no-return
class Ur51(object):
    '''The class dedicated for controlling ur5_1'''
    def __init__(self):



        self._robot_ns = '/ur5_1'
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, \
        robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns +\
         '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns + \
        '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        self.value = 0


        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        r_p = rospkg.RosPack()
        self._pkg_path = r_p.get_path('pkg_task5')
        self.file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self.file_path))


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        '''for clearing the octomap'''
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):
        '''This method is used for setting joint angles of ur5_1

           :param arg_list_joint_angles: list of angles so that ur5 can reach to that position
           :type: list
        '''
        #list_joint_values = self._group.get_current_joint_values()
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        #list_joint_values = self._group.get_current_joint_values()
        #pose_values = self._group.get_current_pose().pose
        if flag_plan:
            pass
        else:
            pass
        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        '''for hard set joint angles

           :param arg_list_joint_angles: list of angles so that ur5 can reach to that position
           :type: list
           :param arg_max_attempts: Maximum attempts that moveit can take to plan the trajectory
           :type: int
        '''
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))



    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''for moveit play planned path from file

           :param arg_file_path: The path of the file where saved trajectory is residing
           :type: str
           :param arg_file_name: The name of saved trajectory yaml file
           :type: str

           :return: The status whether specified saved trajectory is computed or not
        '''
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)

        return ret


    def moveit_hard_play_planned_path_from_file(self, arg_file_path, \
     arg_file_name, arg_max_attempts):
        '''moveit hard play planned path from file

           :param arg_file_path: The path of the file where saved trajectory is residing
           :type: str
           :param arg_file_name: The name of saved trajectory yaml file
           :type: str
           :param arg_max_attempts: Maximum attempts that moveit can take to plan the trajectory
           :type: int
        '''
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))


        return True

    def activate_conveyor(self, value):
        '''function for activating conveyor

           :param value: The speed of conveyor ranging from 0 to 100
           :type: int
        '''
        self.value = value
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        conveyor_state = rospy.ServiceProxy(
            '/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        conveyor_state(value)


    def activate_gripper(self, value):
        '''function for activating gripper

           :param value: The value can be True for activating the grippe or False for
                         deactivating the gripper
           :type: bool
        '''
        self.value = value
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        activate_gripper_client = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)

        activate_gripper_client(value)

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur51Moveit Deleted." + '\033[0m')






class Ur52(object):
    '''The class dedicated for controlling ur5_1'''
    # Constructor
    def __init__(self):

        # rospy.init_node('node_moveit_eg71', anonymous=True)

        self._robot_ns = '/ur5_2'
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns +\
         "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= \
        self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns + \
        '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns +\
         '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        self.box_detected = []
        self._y = 100
        self._x = 100
        self._z = 100
        self.t_x = 0
        self.t_y = 0
        self.t_z = 0
        self.value = 0
        self._pkgno = 1
        self._index = 0
        self._packages = set()

        self.pkg_under_camera = ''


        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        r_p = rospkg.RosPack()
        self._pkg_path = r_p.get_path('pkg_task5')
        self.file_path = self._pkg_path + '/config/saved_trajectories/ur5_2/'
        rospy.loginfo("Package Path: {}".format(self.file_path))


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        '''for clear octomap'''
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):
        '''This method is used for setting joint angles of ur5_2

           :param arg_list_joint_angles: list of angles so that ur5 can reach to that position
           :type: list
        '''
        #list_joint_values = self._group.get_current_joint_values()
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        #list_joint_values = self._group.get_current_joint_values()
        #pose_values = self._group.get_current_pose().pose
        if flag_plan:
            pass
        else:
            pass
        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        '''for hard set joint angles

           :param arg_list_joint_angles: list of angles so that ur5 can reach to that position
           :type: list
           :param arg_max_attempts: Maximum attempts that moveit can take to plan the trajectory
           :type: int
        '''
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))



    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''for moveit play planned path from file

           :param arg_file_path: The path of the file where saved trajectory is residing
           :type: str
           :param arg_file_name: The name of saved trajectory yaml file
           :type: str

           :return: The status whether specified saved trajectory is computed or not
        '''
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)

        return ret


    def moveit_hard_play_planned_path_from_file(self, arg_file_path,\
     arg_file_name, arg_max_attempts):
        '''moveit hard play planned path from file

           Parameters :
           a)arg_file_path = The path of the file where saved trajectory is residing
           b)arg_file_name = The name of saved trajectory yaml file
           c)arg_max_attempts = Maximum attempts that moveit can take to plan the trajectory
        '''
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))

        return True

    def activate_conveyor(self, value):
        '''function for activating conveyor

           :param value: The speed of conveyor ranging from 0 to 100
           :type: int
        '''
        self.value = value
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        conveyor_state = rospy.ServiceProxy(
            '/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        conveyor_state(value)


    def activate_gripper(self, value):
        '''function for activating gripper

           :param value: The value can be True for activating the grippe or False for
                         deactivating the gripper
           :type: bool
        '''
        self.value = value
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        activate_gripper_client = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
        activate_gripper_client(value)



    def function_callback(self, _p):
        '''function call back of logical camera 2
           This function will stop the conveyor once the pkg is under the logical camera2

           :param _p: This is the message that is published on topic of logical camera2
        '''
        _length = len(_p.models)
        if _length:
            for model in _p.models:

                if model.type[0] == "p":
                    # rospy.loginfo(model.type)
                    self._y = model.pose.position.y
                    self._x = model.pose.position.x
                    self._z = model.pose.position.z
                    self._packages.add(model.type)
                    self.pkg_under_camera = model.type
                    if (-0.0145 <= self._y <= 0.01399 and self._pkgno == len(self._packages)):
                        self.box_detected.append(True)
                        self._pkgno = self._pkgno + 1
                        self.activate_conveyor(0)


    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur52Moveit Deleted." + '\033[0m')

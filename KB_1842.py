#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('maniStack', anonymous=True)

        # self._planning_group1 = "gripper"
        # self._planning_group = "ebot_base"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)     #initialized the motion planner node
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group2 = moveit_commander.MoveGroupCommander('arm')       ##move group for arm group
        self._group1 = moveit_commander.MoveGroupCommander('gripper')   ##move group for gripper group
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)        #displays the planned path in rviz

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)      #defined the execution property of a given goal
        self._exectute_trajectory_client.wait_for_server()                      #waits for the feedback data of a execution command in a feedback loop to reach the desired goal

        self._planning_frame = self._group1.get_planning_frame()        #getting info for printing via log msgs
        self._eef_link = self._group1.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        #prints few model parameters of the motion planner via log msgs
        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    ##takes command from given pre defined poses to execute the command
    def go_to_predefined_pose(self, arg_pose_name):         
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        if arg_pose_name in ['open',"close"]:                           #if the arg_pose_name argument has elements of gripper group than that specific group is used to plan and give the desired
            self._group1.set_named_target(arg_pose_name)                #goal to the execution part
            plan = self._group1.plan()                                  
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()              
            try:
                goal.trajectory = plan[1]
            except:
                goal.trajectory = plan
        else:                                                           #if the arg_pose_name not from gripper pose group it must be from the other ,Hence used Else
            self._group2.set_named_target(arg_pose_name)
            plan = self._group2.plan()
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            try:
                goal.trajectory = plan[1]
            except:
                goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)                #Executes the commands according to the goal input we created from previous logic
        self._exectute_trajectory_client.wait_for_result()              #Contineous Feedback loop to reach the goal according to the mption planner
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

    # Destructor
    def __del__(self):                          
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    while not rospy.is_shutdown():
        ur5.go_to_predefined_pose("Orangest-1")
        #rospy.sleep(2)
        ur5.go_to_predefined_pose("Orange_hold")
        #rospy.sleep(2)
        ur5.go_to_predefined_pose("close")
        #rospy.sleep(2)
        ur5.go_to_predefined_pose("Orangest-1")
        #rospy.sleep(2)
        ur5.go_to_predefined_pose("rest")
        #rospy.sleep(2)
        ur5.go_to_predefined_pose("open")           ###picked up the orange fruit
        # rospy.sleep(3)
        ur5.go_to_predefined_pose("Red_step1")
        #rospy.sleep(2)
        ur5.go_to_predefined_pose("Red_hold")
        #rospy.sleep(2)
        ur5.go_to_predefined_pose("close")
        #rospy.sleep(2)
        # ur5.go_to_predefined_pose("Orangest-1")
        # #rospy.sleep(2)
        ur5.go_to_predefined_pose("rest_Red_mod")
        #rospy.sleep(2)
        ur5.go_to_predefined_pose("open")
        #rospy.sleep(2)
        break                              ##picked up red fruit

    del ur5


if __name__ == '__main__':
    main()
#! /usr/bin/env python
# LastEditors: sujit-168 su2054552689@gmail.com
# LastEditTime: 2024-03-22 10:23:42

"""
使用 move_base/NavfnROS/plan 订阅全局 NavfnROS 的路径点个数的长度比例，来控制提前发布
问题 1：move_base/NavfnROS/plan 话题的发布频率不够
"""

import os
import rospy, rospkg
import actionlib # 引用 actionlib 库
import waypoint_race.utils as utils
import nav_msgs.msg as nav_msgs
import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as viz_msgs

world = os.getenv("TIANRACER_WORLD", "tianracer_racetrack")

class RaceStateMachine(object):
    def __init__(self, filename, repeat=True):
        """
        init RaceStateMachine
        filename: path to your yaml file
        reapeat: determine whether to visit waypoints repeatly(usually True in 110)
        """
        self._waypoints = utils.get_waypoints(filename) # 获取一系列目标点的值

        action_name = 'move_base'
        self._ac_move_base = actionlib.SimpleActionClient(action_name, move_base_msgs.MoveBaseAction) # 创建一个 SimpleActionClient
        rospy.loginfo('Wait for %s server' % action_name)
        self._ac_move_base.wait_for_server
        self._counter = 0
        self._repeat = repeat
        self._early_pub = False
        self._len_path_list = []
        self._tolerance_length = 0

        # pub first goal
        rospy.sleep(0.2)     # wait for move_base to be ready, it is necessary and very important
        self._current_goal = utils.create_move_base_goal(self._waypoints[self._counter])
        self._ac_move_base.send_goal(self._current_goal)
        self._counter += 1
        
        # 以下为了显示目标点
        self._pub_viz_marker = rospy.Publisher('viz_waypoints', viz_msgs.MarkerArray, queue_size=1, latch=True)
        self._viz_markers = utils.create_viz_markers(self._waypoints)
        
        # 开始订阅全局路径
        # self._rest_of_path = rospy.Subscriber("move_base/NavfnROS/plan", nav_msgs.Path, self._rest_of_path_callback, queue_size=1)                   # bug: 发布频率不够，仅有 2hz 左右，可以调整 move_base 参数改善
        self._rest_of_path = rospy.Subscriber("move_base/TebLocalPlannerROS/global_plan", nav_msgs.Path, self._rest_of_path_callback, queue_size=1)    # 发布频率在 10hz 左右

    def _rest_of_path_callback(self, data):
        """
        count how many waypoints left
        """
        length = len(data.poses)
       
        if not self._len_path_list:
            rospy.sleep(0.2)   # wait for the new global planner path to be ready
            self._len_path_list.append(length)
            print("len_path_list: ", self._len_path_list)
            self._tolerance_length = max(self._len_path_list) / 2.5    # 控制当前规划的路径中还剩多少个途经点时，发送下一个目标点
            rospy.loginfo("the tolerance: %d", self._tolerance_length)
        else:
            pass
        if length < self._tolerance_length:
            self._early_pub = True
        else:
            self._early_pub = False
            rospy.loginfo("the length of current global waypoints: %d", length)
    
    def move_to_next(self):
        pos = self._get_next_destination()

        if not pos:
            rospy.loginfo("Finishing Race")
            return True
        # 把文件读取的目标点信息转换成 move_base 的 goal 的格式：
        goal = utils.create_move_base_goal(pos)
        rospy.loginfo("Move to %s" % pos['name'])
        # 这里也是一句很简单的 send_goal:
        self._ac_move_base.send_goal(goal)

    def _get_next_destination(self):
        """
        determine what's the next goal point according to repeat 
        """
        if self._counter == len(self._waypoints):
            if self._repeat:
                self._counter = 0
            else:
                next_destination = None
        next_destination = self._waypoints[self._counter]
        self._counter = self._counter + 1
        return next_destination

    def spin(self):
        self._pub_viz_marker.publish(self._viz_markers)
        
        while not rospy.is_shutdown() and self._repeat:
            if self._early_pub:
                self._len_path_list = []
                self.move_to_next()
                self._early_pub = False
            else:
                rospy.sleep(1.0)   # 控制发送目标点后车能继续行驶时间的长度

if __name__ == '__main__':
    rospy.init_node('race')
    package_name = "tianracer_gazebo"

    # Get the package path
    try:
        pkg_path = rospkg.RosPack().get_path(package_name)

        # Construct the path to scripts directory
        filename= os.path.join(pkg_path, f"scripts/waypoint_race/{world}_points.yaml")
        print(f"yaml: {filename}")
    except rospkg.ResourceNotFound:
        rospy.logerr("Package '%s' not found" % package_name)
        exit(1)

    filename = rospy.get_param("~filename",filename)
    repeat = rospy.get_param('~repeat', True)

    m = RaceStateMachine(filename, repeat)
    rospy.loginfo('Initialized')
    m.spin()
    rospy.loginfo('Finished')
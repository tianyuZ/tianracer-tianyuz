#!/usr/bin/env python

#发布名称为init_pose 消息类型为nav_msgs::Odometry的话题，并且通过设置param来改变init_pose,发布频率为1hz。
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

def init_pose_node():
    # Initialize the ROS node
    rospy.init_node('init_pose_pub', anonymous=False)

    # Retrieve the parameters for the position and orientation of the initial pose
    init_position = rospy.get_param('~init_position', [0.0, 0.0, 0.0])  # Default position [x, y, z]
    init_orientation = rospy.get_param('~init_orientation', [0.0, 0.0, 0.0, 1.0])  # Default orientation [x, y, z, w]

    # Set up the publisher to publish Odometry messages to the 'init_pose' topic
    pub = rospy.Publisher('init_pose', Odometry, queue_size=10)

    # Set the publishing rate to 1 Hz
    rate = rospy.Rate(1)

    # Create an Odometry message and set its initial values
    odom_msg = Odometry()
    
    # Set the position using the parameters
    odom_msg.pose.pose.position = Point(*init_position)
    
    # Set the orientation using the parameters
    odom_msg.pose.pose.orientation = Quaternion(*init_orientation)

    # Keep publishing the message at 1 Hz
    while not rospy.is_shutdown():
        # Publish the odometry message
        pub.publish(odom_msg)
        
        # Sleep to maintain the 1 Hz publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        init_pose_node()
    except rospy.ROSInterruptException:
        pass

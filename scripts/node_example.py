#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform
from nav_msgs.msg import Odometry
import random
import os

class DroneController:
    # Variables
    def __init__(self):
        self.__drone_msg = MultiDOFJointTrajectory()
        self.__drone_msg.header.stamp = rospy.Time.now()
        self.__drone_msg.header.frame_id = "base_link"

        self.__drone_msg.joint_names.append('base_link')
        self.__drone_msg.points = []

        self.__velocities = Twist()
        self.__acceleration = Twist()
        
        self.__x, self.__y, self.__z = 0.0, 0.0, 1.0	

        self.__first = True
        self._flag = None

    # This function receives the message from the odometry topic to define the first point
    def _odometryCallback(self, msg):
        if (self.__first):
            transforms = Transform()

            transforms.translation.x = msg.pose.pose.position.x
            transforms.translation.y = msg.pose.pose.position.y
            transforms.translation.z = msg.pose.pose.position.z
            transforms.rotation = msg.pose.pose.orientation

            point = MultiDOFJointTrajectoryPoint([transforms], [self.__velocities], [self.__acceleration], rospy.Time(1/10))
            self.__drone_msg.points.append(point)
            self.__first = False

    # Generate more than a thousand points
    def __generatePoints(self):
        for i in range(1000):
            transforms = Transform()

            rand_num = random.randint(0, 2)        
            if (rand_num == 0):
                self.__x += random.uniform(0, 0.1)
            elif (rand_num == 1):
                self.__y += random.uniform(0, 0.1)
            else:
                self.__z += random.uniform(0, 0.1)

            transforms.translation.x = self.__x
            transforms.translation.y = self.__y
            transforms.translation.z = self.__z

            point = MultiDOFJointTrajectoryPoint([transforms], [self.__velocities], [self.__acceleration], rospy.Time((i+2)/10))
            self.__drone_msg.points.append(point)

        self._flag = True
        print(self.__x, self.__y, self.__z)
        
    def run(self):
        if (not self.__first and self._flag == None):
            rospy.Timer(rospy.Duration(3), self.__shutdown)
            self.__generatePoints()

    def __shutdown(self, event):
        self._flag = False
        rospy.signal_shutdown("Start trajectory")

    def getDronemsg(self):
        return self.__drone_msg

# Stop condition
def stop():
    # Stop message
    print("Stopping")

if __name__ == '__main__':
    rospy.init_node('node_example') 
    rospy.on_shutdown(stop)

    controller = DroneController()

    # Publishers and subscribers
    waypoint_publisher = rospy.Publisher('/hummingbird/command/trajectory', MultiDOFJointTrajectory, queue_size = 10)
    rospy.Subscriber('/hummingbird/ground_truth/odometry', Odometry, controller._odometryCallback)

    print("The Controller is Running")
    
    #Run the node
    while not rospy.is_shutdown():
        try:
            controller.run()
            if (controller._flag):
                waypoint_publisher.publish(controller.getDronemsg())

        except rospy.ROSInterruptException:
            pass

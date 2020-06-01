#!/usr/bin/env python
import rospy
import sys
import random
import numpy as np
from geometry_msgs.msg import Pose, Twist,Vector3
from nav_msgs.msg import Odometry


from sensor_msgs.msg import Range
from sensor_msgs.msg import Illuminance
from math import sqrt, pow

from tf.transformations import euler_from_quaternion


class Task2Controller:
    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node('Track_Controller')

        self.name = rospy.get_param('~robot_name')

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)

        # create pose subscriber
        self.pose_subscriber = rospy.Subscriber(self.name + '/odom', Odometry, self.log_odometry )

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # initialize pose to (X=0, Y=0, theta=0)
        self.pose = Pose()

        # initialize linear and angular velocities to 0
        self.velocity = Twist()

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)

        # set the sensors
        self.library = self.name + '/proximity/'

        self.sensor1 = None
        self.sensor2 = None
        self.sensor3 = None
        self.sensor4 = None
        self.sensor5 = None
        self.sensor6 = None
        self.sensor7 = None

        self.sensorRangeMin = 0.0305260086805
        self.sensorRangeMax = 0.119999997318

        rospy.Subscriber(self.library + 'left', Range, self.SensorData1)
        rospy.Subscriber(self.library + 'center_left', Range, self.SensorData2)
        rospy.Subscriber(self.library + 'center', Range, self.SensorData3)
        rospy.Subscriber(self.library + 'center_right', Range, self.SensorData4)
        rospy.Subscriber(self.library + 'right', Range, self.SensorData5)
        rospy.Subscriber(self.name + '/ground/left', Range, self.SensorData6)
        rospy.Subscriber(self.name + '/ground/right', Range, self.SensorData7)

        self.state = 'start'

    def SensorData1(self, data):
        self.sensor1 = self.ComputeSensorData(data.range)
        return self.sensor1

    def SensorData2(self, data):
        self.sensor2 = self.ComputeSensorData(data.range)
        return self.sensor2

    def SensorData3(self, data):
        self.sensor3 = self.ComputeSensorData(data.range)
        return self.sensor3

    def SensorData4(self, data):
        self.sensor4 = self.ComputeSensorData(data.range)
        return self.sensor4

    def SensorData5(self, data):
        #print(data)
        self.sensor5 = self.ComputeSensorData(data.range)
        return self.sensor5

    def SensorData6(self, data):
        #print(data) 
        self.sensor6 = (0.0299999993294 - data.range)  / (0.0299999993294 - 0.00999999977648)
        return self.sensor6

    def SensorData7(self, data):
        #print(data)
        self.sensor7 = (0.0299999993294 - data.range) / (0.0299999993294 - 0.00999999977648)
        return self.sensor7

    def ComputeSensorData(self, r):
        #print(r)
        return (r - self.sensorRangeMin) / (self.sensorRangeMax - self.sensorRangeMin)



    def human_readable_pose2d(self, pose):
        """Converts pose message to a human readable pose tuple."""

        # create a quaternion from the pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        # convert quaternion rotation to euler rotation
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        result = (
            pose.position.x,  # x position
            pose.position.y,  # y position
            yaw  # theta angle
        )

        return result,pose.position.x, pose.position.y, yaw

    def log_odometry(self, data):
        """Updates robot pose and velocities, and logs pose to console."""

        self.pose = data.pose.pose
        self.velocity = data.twist.twist

        printable_pose,_,_,_ = self.human_readable_pose2d(self.pose)

        # log robot's pose
        rospy.loginfo_throttle(
            period=2,  # log every 10 seconds
            msg=self.name + ' (%.3f, %.3f, %.3f) ' % printable_pose  # message
        )

    def CloseBarrier(self):
        sensors = [self.sensor1,self.sensor2,self.sensor3,self.sensor4,self.sensor5]
        for s in sensors:
            if s is not None and s < 0.5:
                closeBarrier = True
            else:
                closeBarrier = False
        
        return closeBarrier
                
        

    def ControlMove(self):

        while not rospy.is_shutdown():
            sensors = [self.sensor1,self.sensor2,self.sensor3,self.sensor4,self.sensor5]
            grounds = [self.sensor6,self.sensor7]
            #searchD = 0.2
            print(grounds)

            if self.state == 'start':
                    
                    self.GoStraignt(0.2)

                    if (grounds[0] > 0) or (grounds[1] > 0):
                        self.stop()
                        #self.stop()
                        self.state = 'road'

            if self.state == 'road':
                if (grounds[0] > 0) & (grounds[1] > 0):
                    #self.GoStraignt(0.2)
                    if not self.CloseBarrier() :
                        self.GoStraignt(0.2)
                    else:
                        self.state = 'avoid'
                        self.stop()


                else:
                    self.stop()
                    
                    if grounds[1]  > grounds[0]:
                        self.TurnRight(0.3) 

                    else:   
                        self.TurnLeft(0.3)

                    #self.turn(-0.1,0)

            if self.state == 'avoid':
                #
                print(sensors)
                self.GoBack(0.2)
                self.TurnRight(1.5)
                if not self.CloseBarrier() :
                    self.state = 'search'
                    self.stop()

            if self.state == 'search':
                print('searching the road')
                self.stop()

                if (grounds[0] < 0) & (grounds[1] < 0):
                    print(grounds)
                    velocity = self.get_control(-0.2,0.2)
                    self.velocity_publisher.publish(velocity)
                    self.rate.sleep()

                else:
                    self.state = 'road'
                    print(grounds)
                    print('road')
                    




    def get_control(self,z,x):
            return Twist(linear=Vector3(x, .0, .0,),angular=Vector3(.0,.0,z))
        
    def GoStraignt(self,x):
        velocity  = Twist(linear=Vector3(x, .0, .0,),angular=Vector3(.0,.0,0))
        self.velocity_publisher.publish(velocity)
        self.rate.sleep()

    def turn(self, angles, speed):
        
        velocity = Twist(linear=Vector3(speed, .0, .0,),angular=Vector3(.0,.0,angles))

            
        self.velocity_publisher.publish(velocity)
        self.rate.sleep()

    def GoBack(self, goal):
        self.stop()
        _,starPoint_x,starPoint_y,_ = self.human_readable_pose2d(self.pose)

        distance = 0
        goalDistance = goal

        while distance < goalDistance:
            velocity = Twist(linear=Vector3(-0.1, .0, .0,),angular=Vector3(.0,.0,.0))
            
            self.velocity_publisher.publish(velocity)
            self.rate.sleep()

            _,Point_x, Point_y,_ = self.human_readable_pose2d(self.pose)

            distance = sqrt(pow((Point_x - starPoint_x), 2) + pow((Point_y - starPoint_y), 2))

        print(distance)

        self.stop()
        #self.state = 'road'

    def TurnRight(self, goal):
        _,_,_,starPoint_angle = self.human_readable_pose2d(self.pose)

        angle = 0
        goalAngle = goal

        while angle < goalAngle:
            self.turn(-0.2, -0.02)

            _,_,_,Point_angle = self.human_readable_pose2d(self.pose)
            angle = abs(Point_angle - starPoint_angle)
        print('angle = ',angle)

        self.stop

    def TurnLeft(self,goal):
        _,_,_,starPoint_angle = self.human_readable_pose2d(self.pose)

        angle = 0
        goalAngle = goal


        while angle < goalAngle:
            self.turn(0.2, -0.02)

            _,_,_,Point_angle = self.human_readable_pose2d(self.pose)
            angle = abs(Point_angle - starPoint_angle)

        print(angle)
        
        #self.stop
       
                       

    def stop(self):
        """Stops the robot."""

        self.velocity_publisher.publish(
            Twist()  # set velocities to 0
        )

        self.rate.sleep()


if __name__ == '__main__':
    controller = Task2Controller()

    try:
        controller.ControlMove()
    except rospy.ROSInterruptException as e:
        pass






        


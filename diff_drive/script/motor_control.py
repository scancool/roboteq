#!/usr/bin/env python 

import rospy
import tf
import serial
from math import cos, sin
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3



rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()


x_in = 0
y_in = 0
theta_in = 0

vx = 0.1
vy = -0.1
vth = 0.1

current_time = rospy.Time.now()
last_time = rospy.Time.now()


prev_message = [0, 0]

class driver:
    def __init__(self,ser):
        self.ser=ser
        rospy.Subscriber('/cmd_vel', Twist, self.get_cmd_vel)

    def get_cmd_vel(self, data):
        x = data.linear.x
        angular = data.angular.z
        self.send_cmd_to_motorcontrol(x, angular)

    # translate x, and angular velocity to PWM signal of each wheels, and send to motor control
    def send_cmd_to_motorcontrol(self, x, angular):
        # calculate right and left wheels' signal
        go = int(x * 250)
        turn = int(angular * 125)
        # format for Roboteq controller
        sendgo = '!g 1 {}_'.format(go)
        sendturn = '!g 2 {}_'.format(turn)

        #print message
        print sendgo
        print sendturn

        # send by serial
        self.ser.write(sendgo)
        self.ser.write(sendturn)



def read_controller():
    message = ser.read_until('\r\r')
    message = message.replace('\r\r', '')
    message = message.replace('\r', '')
    message = message.replace('\n', '')
    message = message.replace(',', '')
    message = message.split("CB=") #lots of cleaning before reading
    message = message[1].split(":")        
    message = [int(message[0]), int(message[1])] 
    print(message)   
    return message



ser = serial.Serial('/dev/ttyACM0', 115200)
ser.write("# c\r")                      # Clear buffer
ser.write("?CB\r")                      # select CB for hall sensor or C for encoder
ser.write("# 10\r")                     # read data every 10ms
ser.write("!CB 1 0_!CB 2 0\r")          # set counter to 0


def calculate_diff(new,old):
    if old>25000 and new <-25000:
        return 65536 - (old-new)
    else:
        return new-old


try:
    d = driver(ser)
except rospy.ROSInterruptException: 
    pass



while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()

    message = read_controller()
    # diff drive formula
    msg_diff = [calculate_diff(message[0],prev_message[0]), calculate_diff(message[1], prev_message[1])]
    prev_message = message
    wheel_rad = 0.045   #wheel radius
    wheel_dist = 0.25 #weel distance
    encoder_coef = 7#number of pulses for a full turn
    #kinematics and integration to get coordinates:
    theta = (float(wheel_rad)*(float(msg_diff[0]-msg_diff[1])/encoder_coef))/float(wheel_dist)
    theta_in += theta
    x = float(wheel_rad)/2.0*(float(msg_diff[0]+msg_diff[1])/encoder_coef)*cos(theta_in) 
    x_in += x
    y = float(wheel_rad)/2.0*(float(msg_diff[0]+msg_diff[1])/encoder_coef)*sin(theta_in) 
    y_in += y

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta_in)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x_in, y_in, 0),
        odom_quat,
        current_time,
        "chassis", 
        "world"
    ) 
     
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "chassis"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "world"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
        



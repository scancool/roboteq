#!/usr/bin/env python 
# andre.bella@roboteq.com

import rospy
import nav_msgs.msg
import serial
import tf
from math import cos, sin
from std_msgs.msg import String
from geometry_msgs.msg import Twist

x_in = 0
y_in = 0
theta_in = 0

prev_message = [0, 0]

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

def handle_pose(serial_msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((serial_msg.x, serial_msg.y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, serial_msg.theta),
                        rospy.Time.now(),
                        "chassis", #Robot frame name
                        "world") #Map or world frame name

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



def calculate_diff(new,old):
    if old>25000 and new <-25000:
        return 65536 - (old-new)
    else:
        return new-old

if __name__ == '__main__':
    rospy.init_node('robot_tf_broadcaster')
    ser = serial.Serial('/dev/ttyACM0', 115200)
    try:
        d = driver(ser)
    except rospy.ROSInterruptException: 
        pass

    while not rospy.is_shutdown():
        message = ser.readline() #got line from controller
        message =message.replace('+\r+\r', '') 
        message = message.split(",") #got list of 2 string numbers
        message = [int(message[0]), int(message[1])] #convert to int
        #print("Values from Motor Control ::"+str(message))
        msg_diff = [calculate_diff(message[0],prev_message[0]), calculate_diff(message[1], prev_message[1])]
        prev_message = message
        wheel_rad = 0.12   #wheel radius
        wheel_dist = 0.50 #wheel distance
        encoder_coef = 5000 #number of pulses for a full turn
        #kinematics and integration to get coordinates:
        theta = (float(wheel_rad)*(float(msg_diff[0]-msg_diff[1])/encoder_coef))/float(wheel_dist)
        theta_in += theta
        x = float(wheel_rad)/2.0*(float(msg_diff[0]+msg_diff[1])/encoder_coef)*cos(theta_in) 
        x_in += x
        y = float(wheel_rad)/2.0*(float(msg_diff[0]+msg_diff[1])/encoder_coef)*sin(theta_in) 
        y_in += y
        print("Calculated velocities: ", x_in, y_in, theta_in)
        message = Pose(x_in, y_in, theta_in)
        handle_pose(message) 


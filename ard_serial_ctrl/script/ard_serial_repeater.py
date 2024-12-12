#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from time import time
import struct
import serial

port = '/dev/ttyUSB0'
baud = 9600

#ser = serial.Serial(port,baud)



class data_sub():
    def __init__(self):
        self.rospy_rate = 10
        #self.rospy_rate = rospy.get_param("/rovigos_middlewear/rospy_rate_param")
        self.twist = Twist()
        self.twist.linear.x = 0.0; self.twist.linear.y = 0.0; self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0; self.twist.angular.y = 0.0; self.twist.angular.z = 0.0

        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb, queue_size=1)
        self.time_prev = 0
        self.time_gap = 1
    
    def cmd_vel_cb(self, data):
        self.twist = data
        lin_byte = bytearray(struct.pack("f", self.twist.linear.x))
        ang_byte = bytearray(struct.pack("f", self.twist.angular.z))
        serial_byte = bytearray(b'AT+VEL=')+lin_byte+ang_byte+bytearray(b'\n')

        if(time()-self.time_prev >self.time_gap):
            rospy.loginfo("Read Linear : %3.2f, %3.2f, %3.2f, Angular : %3.2f, %3.2f, %3.2f ", self.twist.linear.x,self.twist.linear.y,self.twist.linear.z,self.twist.angular.x,self.twist.angular.y,self.twist.angular.z)
            self.time_prev = time()

            rospy.loginfo([ "0x%02x" % b for b in serial_byte ])
            #rospy.loginfo(str(serial_byte, 'utf-8'))

        #if serial_error == False: # Check Serial failure
        #    ser.write(x_y.encode('utf-8'))



def main():
    rospy.init_node("ard_serial_repeater_node")
     
    ros_subs_ = data_sub()

    r = rospy.Rate(ros_subs_.rospy_rate)
    while not rospy.is_shutdown():
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
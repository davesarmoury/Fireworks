#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32
import Jetson.GPIO as GPIO

servoPIN = 18
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servoPIN, GPIO.OUT)

MIN = 1.0
MAX = 12.0

p = GPIO.PWM(servoPIN, 50)
p.start(MIN)

def bool_callback(msg):
    rospy.loginfo(msg.data)
    if msg.data:
        setpoint = MAX
    else:
        setpoint = MIN

    p.ChangeDutyCycle(setpoint)

def float_callback(msg):
    rospy.loginfo(msg.data)
    setpoint = msg.data

    p.ChangeDutyCycle(setpoint)

def servo_driver():
    rospy.init_node('lighter', anonymous=True)

    rospy.Subscriber("fire", Bool, bool_callback)
    rospy.Subscriber("angle", Float32, float_callback)

    p.ChangeDutyCycle(MIN)
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_driver()
    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()

    p.stop()
    GPIO.cleanup()


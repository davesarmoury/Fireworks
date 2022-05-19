#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO

servoPIN = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

MAX = 6.27450980392
MIN = 1.96078431373

p = GPIO.PWM(servoPIN, 50)
p.start(MIN)

def callback(msg):
    if msg.data:
        setpoint = MAX
    else:
        setpoint = MIN

    p.ChangeDutyCycle(setpoint)

def servo_driver():
    rospy.init_node('lighter', anonymous=True)

    rospy.Subscriber("fire", Bool, callback)

    p.ChangeDutyCycle(MAX)
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_driver()
    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()

    p.stop()
    GPIO.cleanup()


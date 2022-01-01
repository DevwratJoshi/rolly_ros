#!/usr/bin/env python3
import rospy
from rolly_controllers.constants import DIM
from rolly_controllers.sensor_turret_handler import SensorTurretHandler
import RPi.GPIO as GPIO
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import copy
# Create class inheriting from Sensor turret handler
class SensorTurretHandlerROS(SensorTurretHandler):
    ''' 
    This class should wrap the sensor turret handler class and create ros laser scan messages to be visualized and used for slam
    '''
    def __init__(self, servoPin=25, ultraTrigger=18, ultraEcho=24, scanFreq=1.5, scanRes=0.0174533):
        super().__init__(servoPin, ultraTrigger, ultraEcho, scanFreq, scanRes)
        self.make_scan_message()

    def make_scan_message(self):
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = "scanner_frame"
        self.scan_msg.angle_min = 0.0
        self.scan_msg.angle_max = math.pi
        self.scan_msg.angle_increment = self.scanRes
        self.scan_msg.time_increment = self.stepTime
        self.scan_msg.range_min = 0.02
        self.scan_msg.range_max = 4.0

    def get_scan_message(self):
        scan_array = np.zeros(self.latestScan.shape)
        self.scan_mutex.acquire()
        scan_array = np.copy(self.latestScan)
        self.scan_mutex.release()

        scan = copy.deepcopy(self.scan_msg)
        scan.header.stamp = rospy.Time.now()

        for i in range(scan_array.shape[0]):
            scan.ranges.append(scan_array[i][0])

        return scan

if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    rospy.init_node('turret_scanner')

    sc = SensorTurretHandlerROS()
    rospy.sleep(1.0)
    while not rospy.is_shutdown():
        rospy.sleep(2.0)
        print(sc.get_scan_message())


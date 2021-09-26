#/usr/bin/env python

from sensor_turret_handler import SensorTurretHandler, CONSTS
from sensor_msgs.msg import LaserScan
import numpy as np
import math
# Create class inheriting from Sensor turret handler
class SensorTurretHandlerROS(SensorTurretHandler):
    ''' 
    This class should wrap the sensor turret handler class and create ros laser scan messages to be visualized and used for slam
    '''
    def make_scan_message():
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = "scanner_frame"
        self.scan_msg.angle_min = 0.0
        self.scan_msg.angle_max = math.pi
        self.scan_msg.angle_increment = self.scanRes
        self.scan_msg.time_increment = self.stepTime
        self.scan_msg.range_min = 0.02
        self.scan_msg.range_max = 4.0

    def get_scan_message():
        scan_array = np.zeros(self.latestScan.shape)
        self.scan_mutex.acquire()
        scan_array = np.copy(self.latestScan)
        self.scan_mutex.release()

        scan = copy.deepcopy(self.scan_msg)
        scan.header.stamp = rospy.time.now()

        for i in range(scan_array.shape[0]):
            scan_msg.ranges.append(scan_array[i][0])

        return scan_msg


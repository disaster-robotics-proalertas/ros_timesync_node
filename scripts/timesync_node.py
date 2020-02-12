#!/usr/bin/env python

import ctypes
import ctypes.util
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from math import sqrt
import rospy
from sensor_msgs.msg import TimeReference
import socket

CLOCK_REALTIME = 0

class Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long),
                ("tv_nsec", ctypes.c_long)]

class Node:
    def __init__(self, **kwargs):
        self.system_name = socket.gethostname()
        rospy.init_node('timesync_node', anonymous=True)

        # Parameters
        self.timeref_topic = rospy.get_param("~topic", "walltime/time_reference")
        self.mode = rospy.get_param("~mode", "server")
        self.tolerance = rospy.get_param("~tolerance", 2)   # Tolerance in seconds for considering as "synchronized"

        # Publishers
        self.timeref_pub = rospy.Publisher(self.timeref_topic, TimeReference, queue_size=10)
        self.status_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

        # Subscribers
        rospy.Subscriber(self.timeref_topic, TimeReference, callback=self.timeref_callback)

        # Messages
        self.timeref_msg = TimeReference()
        self.diag_status = DiagnosticStatus()
        self.last_update = 0

        # Rate
        self.rate = rospy.Rate(0.5) # Once every 2 secs

    def timeref_callback(self, msg):
        if self.mode == 'client':
            self.timeref_msg = msg

    # Publish own walltime in topic
    def server(self):
        # Update time reference message
        time = rospy.Time.now()
        self.timeref_msg.header.stamp = time
        self.timeref_msg.time_ref = time
        self.timeref_msg.source = self.system_name
        self.timeref_pub.publish(self.timeref_msg)

        # Update diagnostic status message
        self.diag_status.level = DiagnosticStatus.OK
        self.diag_status.message = '%s mode' % self.mode
        self.diag_status.values = [KeyValue(key='Update Status', value='OK')]

        self.last_update = rospy.Time.now().to_sec()

    # Get walltime from topic and compare to own clock
    def client(self):        
        # Compare absolute time difference from reference
        abs_time_diff = sqrt(( self.timeref_msg.time_ref.to_sec() - rospy.Time.now().to_sec() )**2)

        # Update diagnostic status message
        # If absolute difference is lower than some tolerance (tolerance param)
        if abs_time_diff <= self.tolerance:
            self.diag_status.level = DiagnosticStatus.OK
            self.diag_status.message = '%s mode' % self.mode
            self.diag_status.values = [KeyValue(key='Update Status', value='OK')]
        else:
            self.diag_status.level = DiagnosticStatus.ERROR
            self.diag_status.message = '%s mode' % self.mode
            self.diag_status.values.insert(0, KeyValue(key='Update Status', value='ERROR: Could not update walltime'))

        self.last_update = rospy.Time.now().to_sec()

    def run(self):
        while not rospy.is_shutdown():
            self.diag_status = DiagnosticStatus()
            self.diag_status.name = '%s timesync' % self.system_name
            self.diag_status.hardware_id = self.system_name
            self.diag_status.level = DiagnosticStatus.ERROR

            if self.mode == 'server':
                self.server()
            elif self.mode == 'client':
                self.client()
            else:
                rospy.logerr('[timesync_node] Invalid mode selection, must be server or client')
                rospy.signal_shutdown(reason='invalid mode selection')
            
            # Check for stale diagnostics
            elapsed = rospy.Time().now().to_sec() - self.last_update
            if elapsed > 35:
                self.diag_status.level = DiagnosticStatus.STALE
                self.diag_status.message = 'Stale'
                self.diag_status.values = [KeyValue(key = 'Update Status', value = 'Stale'),
                                           KeyValue(key = 'Time Since Update', value = str(elapsed))]

            diag_msg = DiagnosticArray()
            diag_msg.status.append(self.diag_status)
            self.status_pub.publish(diag_msg)

            self.rate.sleep()

if __name__ == "__main__":
    node = Node()
    node.run()

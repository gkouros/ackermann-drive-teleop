#!/usr/bin/env python

'''
joyop_stamper.py:
    A script that adds timestamp to ackermann velocity messages
'''

__author__ = 'Prasanna Kannappan'
__license__ = 'GPLv3'
__maintainer__ = 'Prasanna Kannappan'
__email__ = 'prasanna@imover.xyz'

import rospy
import time
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from std_msgs.msg import Header

class AckermannDriveKeyopStamper:
  def __init__(self):
    """
    Constructor
    """

    self.ackermann_drive_sub = rospy.Subscriber("vel_in", AckermannDrive, self.velCallback)
    self.ackermann_drive_timestamp_pub = rospy.Publisher("vel_out", AckermannDriveStamped, queue_size=1)
    
    # Wait till time server starts
    time_server_timeout_sec = rospy.get_param("time_server_timeout_sec", 60)
    rospy.loginfo("Waiting for time server ...")
    start_time = time.time()
    while(rospy.Time.now().is_zero()):
      time.sleep(1)
      curr_time = time.time()
      if (curr_time - start_time) > time_server_timeout_sec:
        rospy.logfatal("Time server not available after waiting for {}".format(time_server_timeout_sec))
        rospy.signal_shutdown("Time server not available");

  def velCallback(self, msg):
    """
    Callback for keyboard teleop velocity message. It adds time stamp to the incoming message
    Args
      msg (AckermannDrive): Incoming ackerman velocity messages
    """
    out_msg = AckermannDriveStamped()
    out_msg.header.stamp = rospy.Time.now()
    out_msg.drive = msg
    
    self.ackermann_drive_timestamp_pub.publish(out_msg)
  
  
if __name__=="__main__":
  rospy.init_node('keyop_stamper')
  msg_stamper = AckermannDriveKeyopStamper()
  rospy.spin()


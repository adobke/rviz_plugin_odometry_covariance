import rospy
from nav_msgs.msg import Odometry
from math import cos,sin,pi
from tf.transformations import quaternion_from_euler
import numpy as np

opub = rospy.Publisher("odom_cov", Odometry)
rospy.init_node("test_odom_covariance")

rate = rospy.Rate(10)

odomMsg = Odometry()
odomMsg.header.frame_id = "odom"

angle = pi/4
dist = 1

err = np.zeros([3,3])
R = np.matrix([[.25, 0, 0],
               [0, .05, 0],
               [0, 0, .25]
              ])
def F(theta_approx): 
    return np.matrix([[cos(theta_approx), -sin(theta_approx), 0],
                      [sin(theta_approx),  cos(theta_approx), 0],
                      [0,                                  0, 1]
                     ])

while not rospy.is_shutdown():
  odomMsg.header.stamp = rospy.Time.now()

  odomMsg.pose.pose.position.x += dist * cos(angle)
  odomMsg.pose.pose.position.y += dist * sin(angle)
  err += F(angle) * R * F(angle).T
  print err

  
  quat = quaternion_from_euler(0,0,angle)
  odomMsg.pose.pose.orientation.x = quat.item(0)
  odomMsg.pose.pose.orientation.y = quat.item(1)
  odomMsg.pose.pose.orientation.z = quat.item(2)
  odomMsg.pose.pose.orientation.w = quat.item(3)

  odomMsg.pose.covariance[0] = err.item(0,0)
  odomMsg.pose.covariance[1] = err.item(0,1)
  odomMsg.pose.covariance[6] = err.item(1,0)
  odomMsg.pose.covariance[7] = err.item(1,1)

  opub.publish(odomMsg)
  
  angle += .05
  rate.sleep()


#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np

class RMSECalculator:
    def __init__(self):
        # Initialize static variable for cumulative sum (similar to rmse_sum in C++)
        self.rmse_sum = np.zeros(3)  # 3D vector for x, y, z
        
        # Latest messages storage
        self.latest_est = None
        self.latest_gt = None
        self.latest_vml = None

        # Subscribers for estimated and ground truth poses
        self.est_sub = rospy.Subscriber('/MAV4/mavros/local_position/pose', PoseStamped, self.est_callback)
        self.gt_sub = rospy.Subscriber('/MAV2/mavros/local_position/pose_initialized', PoseStamped, self.gt_callback)
        self.vml = rospy.Subscriber('/vml_maha/pose', PoseStamped, self.vml_callback)
        # Publisher for RMSE
        self.pub = rospy.Publisher('/vision_rmse', Float64, queue_size=10)
        self.pub_vml = rospy.Publisher('/vml_rmse', Float64, queue_size=10)
    def rmse(self, gt, est):
        # Extract position vectors
        gt_vec = np.array([gt.x, gt.y])
        est_vec = np.array([est.x, est.y])
        
        # Calculate difference (gt - est, similar to gt - self.X.segment(0, 3))
        self.rmse_sum = gt_vec - est_vec
        
        # Compute RMSE as the norm of the difference
        rmse_val = np.linalg.norm(self.rmse_sum)
        
        return rmse_val
    def vml_callback(self, vml_msg):
        # Store latest vml pose
        self.latest_vml = vml_msg.pose.position
        # Compute and publish RMSE if both messages are available
        if self.latest_gt is not None and self.latest_vml is not None:
            rmse_val = self.rmse(self.latest_gt, self.latest_vml)
            self.pub_vml.publish(rmse_val)
            rospy.loginfo(f"Published RMSE: {rmse_val}")

    def est_callback(self, est_msg):
        # Store latest estimated pose
        self.latest_est = est_msg.pose.position
        # Compute and publish RMSE if both messages are available
        if self.latest_est is not None and self.latest_gt is not None:
            rmse_val = self.rmse(self.latest_gt, self.latest_est)
            self.pub.publish(rmse_val)
            rospy.loginfo(f"Published RMSE: {rmse_val}")
    
    def gt_callback(self, gt_msg):
        # Store latest ground truth pose
        self.latest_gt = gt_msg.pose.position
        # Compute and publish RMSE if both messages are available
        if self.latest_est is not None and self.latest_gt is not None:
            rmse_val = self.rmse(self.latest_gt, self.latest_est)
            self.pub.publish(rmse_val)
            rospy.loginfo(f"Published RMSE: {rmse_val}")

if __name__ == '__main__':
    try:
        rospy.init_node('rmse_calculator', anonymous=True)
        RMSECalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
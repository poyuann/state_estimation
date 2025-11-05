#!/usr/bin/env python

import rospy
import numpy as np
from collections import deque
from geometry_msgs.msg import PoseStamped

def calculate_mahalanobis(x, data):
    """
    Calculates the Mahalanobis Distance.
    Assumes `data` is a numpy array where each row is an observation.
    """
    mean_mu = np.mean(data, axis=0)
    cov_matrix = np.cov(data, rowvar=False)
    # print(data)
    # print ("Covariance Matrix:\n", cov_matrix)
    # Ensure covariance matrix is invertible, especially at the beginning
    if np.linalg.det(cov_matrix) == 0:
        rospy.logwarn("Covariance matrix is singular. Cannot compute Mahalanobis distance yet.")
        return None
        
    inv_cov_matrix = np.linalg.inv(cov_matrix)
    x_minus_mu = x - mean_mu
    
    left_term = np.dot(x_minus_mu, inv_cov_matrix)
    mahalanobis_squared = np.dot(left_term, x_minus_mu.T)
    
    return np.sqrt(mahalanobis_squared)

class MahalanobisDetector:
    def __init__(self, topic_name, pub_topic_name, queue_size=5):
        """
        Initializes the ROS node and the data queue.
        
        Args:
            topic_name (str): The name of the ROS topic to subscribe to.
            queue_size (int): The number of historical data points to store.
        """
        rospy.init_node('mahalanobis_detector', anonymous=True)
        
        # Use a deque for an efficient fixed-size queue
        self.data_queue = deque(maxlen=queue_size)
        self.queue_size = queue_size
        
        # Subscribe to the specified PoseStamped topic
        self.subscriber = rospy.Subscriber(topic_name, PoseStamped, self.pose_callback)
        self.vml_pub = rospy.Publisher(pub_topic_name, PoseStamped, queue_size=10)
        rospy.loginfo(f"Mahalanobis detector node started. Subscribed to {topic_name}")
        rospy.loginfo(f"vml publisher started. Publishing to {pub_topic_name}")
        rospy.loginfo(f"Waiting for {self.queue_size} messages to fill the data queue...")

    def pose_callback(self, msg):
        """
        This function is called every time a new message is received.
        """
        # Extract the position data (x, y, z) from the message
        # We are focusing on position; orientation (quaternion) is ignored for this calculation.
        current_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            # msg.pose.position.z
        ])
        # rospy.loginfo(f"Received new position: {current_position}")
        # --- Check if the queue is full before calculating distance ---
        if len(self.data_queue) < self.queue_size:
            # If not full, just add the data and wait
            self.data_queue.append(current_position)
            vml_pub_msg = PoseStamped()
            vml_pub_msg.header = msg.header
            vml_pub_msg.pose = msg.pose
            self.vml_pub.publish(vml_pub_msg)
            # Provide feedback on the filling status
            if len(self.data_queue) == self.queue_size:
                rospy.loginfo("Data queue is now full. Starting Mahalanobis calculations.")
            else:
                rospy.loginfo(f"Queue filling: {len(self.data_queue)}/{self.queue_size}")
            return

        # --- If the queue is full, perform the calculation ---
        
        # The historical data is what's currently in the queue
        historical_data = np.array(self.data_queue)
        
        # Calculate the distance for the new data point against the historical data
        distance = calculate_mahalanobis(x=current_position, data=historical_data)
        print(current_position)
        if distance is not None:
            rospy.loginfo(f"Mahalanobis Distance: {distance:.4f}")
            
            # You can add your outlier detection logic here
            # For example, using a threshold like 20
            threshold = 9.0
            if distance < threshold:
                vml_pub_msg = PoseStamped()
                vml_pub_msg.header = msg.header
                vml_pub_msg.pose = msg.pose
                self.vml_pub.publish(vml_pub_msg)
                rospy.loginfo(f"Published to VML topic: {vml_pub_msg}")
                self.data_queue.append(current_position)
                if len(self.data_queue) >= 10:
                    self.data_queue.popleft()  # Maintain fixed size
            else:
                rospy.logwarn(f"Anomaly Detected! Distance {distance:.4f} > threshold {threshold}")
        
        # --- Add the new data point to the queue ---
        # The deque will automatically remove the oldest item

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        # Define the topic you want to subscribe to
        # You can change '/your_topic_name' to your actual topic
        topic = '/vml/pose' 
        pub_topic = '/vml_maha/pose'
        detector = MahalanobisDetector(topic_name=topic, pub_topic_name=pub_topic, queue_size=5)
        detector.run()
    except rospy.ROSInterruptException:
        pass

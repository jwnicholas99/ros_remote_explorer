#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage

class TFMinimal:
    def __init__(self):
        rospy.init_node("tf_minimal")
        rospy.Subscriber("/tf", TFMessage, self._publishMinimalTf)
        self.tfMinimalPub = rospy.Publisher('/tf_minimal', TFMessage, queue_size=10)
        self.baseName = rospy.get_param("baseName", "/base_footprint")
        self.odomName = rospy.get_param("odomName", "odom")
        rospy.spin()
    
    def _publishMinimalTf(self, tfMessage):
        transforms = tfMessage.transforms
        tfMsg = TFMessage()
        for transform in transforms:
            if transform.child_frame_id == self.baseName or transform.child_frame_id == self.odomName:
                self.tfMinimalPub.publish(tfMessage)
                return

if __name__ == "__main__":
    node = TFMinimal()
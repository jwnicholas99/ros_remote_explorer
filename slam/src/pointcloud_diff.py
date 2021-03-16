#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

class PointCloudDiff:
    def __init__(self):
        rospy.init_node("pointcloud_diff")
        rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, self._publishPointcloudDiff)
        self.pointcloudDiffPub = rospy.Publisher('/pointcloud_diff', PointCloud2, queue_size=10)
        self.cur_size = 0
        self.points_set = set()
        self.point_step = 0
        rospy.spin()
    
    def _publishPointcloudDiff(self, msg):
        new_msg = PointCloud2()

        new_msg.header = msg.header
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = self.point_step = msg.point_step
        new_msg.row_step = msg.row_step
        new_msg.is_dense = msg.is_dense
        
        point_tuples = self._convertDataToTuples(msg.data)
        #print(point_tuples)
        if self.cur_size == 0:
            new_msg.data = msg.data
            self.points_set.update(point_tuples)
        elif self.cur_size == len(msg.data):
            return
        else:
            #cannot use a list here
            diff_data = []
            for point_tuple in point_tuples:
                if point_tuple not in self.points_set:
                    # Need to convert using bytes as tuple() turn it into strings
                    diff_data.append(bytes(point_tuple))
                    self.points_set.add(point_tuple)
            diff_data = ''.join(diff_data)
            new_msg.data = diff_data

        self.cur_size = len(msg.data)
        self.pointcloudDiffPub.publish(new_msg)

    def _convertDataToTuples(self, data):
        tuples = []
        for i in range(0, len(data), self.point_step):
            tuples.append(data[i:i+self.point_step])
        return tuples


if __name__ == "__main__":
    node = PointCloudDiff()
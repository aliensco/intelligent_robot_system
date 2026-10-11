#!/usr/bin/env python
"""Real rolling costmap regression; no physical drivers or motion commands."""
import time
import unittest
import rospy
import rostest
from geometry_msgs.msg import PolygonStamped, Point32
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


class ClearanceTest(unittest.TestCase):
    def setUp(self):
        self.grid = None
        self.sub = rospy.Subscriber('/clearance_test/move_base/global_costmap/costmap', OccupancyGrid, self.receive)
        self.pub = rospy.Publisher('/clearance_test/checked', PolygonStamped, queue_size=1)
        self.cloud = rospy.Publisher('/clearance_test/obstacle_cloud', PointCloud2, queue_size=1)

    def receive(self, msg):
        self.grid = msg

    def value(self, x, y):
        if self.grid is None:
            return None
        g = self.grid
        ix = int((x-g.info.origin.position.x)/g.info.resolution)
        iy = int((y-g.info.origin.position.y)/g.info.resolution)
        return g.data[iy*g.info.width+ix]

    def wait(self, condition, seconds=5):
        end = time.time()+seconds
        while time.time() < end:
            h = Header(stamp=rospy.Time.now(), frame_id='clearance_odom')
            self.cloud.publish(create_cloud_xyz32(h, [[.45,.10,.25]]))
            if condition():
                return
            time.sleep(.04)
        self.fail('Condition failed; free=%s obstacle=%s outside=%s' %
                  (self.value(-.45,-.10), self.value(.45,.10), self.value(-.85,-.5)))

    def polygon(self):
        p = PolygonStamped()
        p.header.frame_id, p.header.stamp = 'clearance_odom', rospy.Time.now()
        p.polygon.points = [Point32(x=x,y=y,z=0) for x,y in
                            [(-.6,-.6),(.6,-.6),(.6,.6),(-.6,.6)]]
        return p

    def test_obstacles_and_expiration(self):
        self.wait(lambda: self.grid is not None and self.pub.get_num_connections() > 0)
        self.wait(lambda: self.value(.45,.10) == 100)
        self.assertEqual(self.value(-.45,-.1), -1)
        wrong = self.polygon(); wrong.header.frame_id = 'wrong_frame'
        self.pub.publish(wrong)
        time.sleep(.3)
        self.assertEqual(self.value(-.45,-.1), -1)
        p = self.polygon(); self.pub.publish(p)
        self.wait(lambda: self.value(-.45,-.1) == 0)
        self.assertEqual(self.value(.45,.1), 100)
        self.assertEqual(self.value(-.85,-.5), -1)
        self.wait(lambda: self.value(-.45,-.1) == -1, 4)
        self.assertEqual(self.value(.45,.1), 100)
        self.pub.publish(p)  # stale replay cannot renew
        time.sleep(.3)
        self.assertEqual(self.value(-.45,-.1), -1)
        self.pub.publish(self.polygon())
        self.wait(lambda: self.value(-.45,-.1) == 0)


if __name__ == '__main__':
    rospy.init_node('clearance_regression')
    rostest.rosrun('rgbd_following', 'commissioning_clearance', ClearanceTest)

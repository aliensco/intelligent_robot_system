#!/usr/bin/env python
"""Publish ONE operator-checked near-field polygon. Never publish velocity.

This does not infer free space from missing depth. An operator must inspect the
specified rectangle immediately before running this tool. Evidence is fixed in
odom and expires in the costmap plugin even if this process or ROS clock stops.
"""
from __future__ import print_function
import argparse
import json
import math
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--confirm-clearance', action='store_true', required=True)
    p.add_argument('--namespace', default='/method3_motion')
    p.add_argument('--front', type=float, default=.60, help='metres from base origin')
    p.add_argument('--rear', type=float, default=.50)
    p.add_argument('--half-width', type=float, default=.50)
    p.add_argument('--odom-topic', default='/odom')
    p.add_argument('--odom-frame', default='odom')
    p.add_argument('--base-frame', default='base_footprint')
    args = p.parse_args()
    if not all(0 < v <= .8 for v in (args.front, args.rear, args.half_width)):
        p.error('Clearance dimensions must be in (0, .8] metres')
    import rospy
    from geometry_msgs.msg import PolygonStamped, Point32
    from nav_msgs.msg import Odometry
    from kobuki_msgs.msg import SensorState
    rospy.init_node('codex_manual_clearance', anonymous=True)
    pub = rospy.Publisher(args.namespace.rstrip('/')+'/commissioning_clearance',
                          PolygonStamped, queue_size=1, latch=True)
    for costmap in ('global_costmap', 'local_costmap'):
        plugins = rospy.get_param(args.namespace.rstrip('/')+'/move_base/'+costmap+'/plugins', [])
        if not any(v.get('type') == 'rgbd_following/CommissioningClearanceLayer' for v in plugins):
            raise RuntimeError('Both costmaps must configure commissioning clearance')
    # roscpp shares ONE TCP connection between the two callbacks in move_base.
    deadline = time.time()+5
    while pub.get_num_connections() < 1 and time.time() < deadline:
        time.sleep(.05)
    if pub.get_num_connections() < 1:
        raise RuntimeError('Commissioning costmap subscriber must be connected')
    sensor = rospy.wait_for_message('/mobile_base/sensors/core', SensorState, 2)
    odom = rospy.wait_for_message(args.odom_topic, Odometry, 2)
    now = rospy.Time.now()
    if (any(not 0 <= (now-m.header.stamp).to_sec() <= .3 for m in (sensor, odom)) or
            sensor.bumper or sensor.cliff or sensor.wheel_drop or sensor.over_current):
        raise RuntimeError('Fresh healthy base sensors required')
    if odom.header.frame_id != args.odom_frame or odom.child_frame_id != args.base_frame:
        raise RuntimeError('Unexpected odometry frames')
    q, pos, v = odom.pose.pose.orientation, odom.pose.pose.position, odom.twist.twist
    values = [pos.x, pos.y, q.x, q.y, q.z, q.w,
              v.linear.x, v.linear.y, v.angular.z]
    if not all(not math.isnan(x) and not math.isinf(x) for x in values):
        raise RuntimeError('Nonfinite odometry')
    if (math.hypot(v.linear.x, v.linear.y) > .015 or abs(v.angular.z) > .03 or
            abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .02):
        raise RuntimeError('Robot must be stopped with valid odometry')
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    msg = PolygonStamped()
    msg.header.frame_id, msg.header.stamp = args.odom_frame, now
    for x,y in [(-args.rear,-args.half_width), (args.front,-args.half_width),
                (args.front,args.half_width), (-args.rear,args.half_width)]:
        msg.polygon.points.append(Point32(x=pos.x+c*x-s*y, y=pos.y+s*x+c*y, z=0))
    pub.publish(msg)
    print(json.dumps(dict(stamp=now.to_sec(), frame=args.odom_frame,
                          polygon=[[p.x,p.y] for p in msg.polygon.points],
                          source='explicit_operator_inspection', renews=False)))
    time.sleep(.5)


if __name__ == '__main__':
    main()

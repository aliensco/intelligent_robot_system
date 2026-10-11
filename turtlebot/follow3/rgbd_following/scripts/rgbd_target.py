#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Registered RGB-D -> appearance candidates, odom target and obstacle cloud."""
import json
import threading
import time
import cv2
import message_filters
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, String
from rgbd_following_core.depth import check_pair
from rgbd_following_core.motion import fresh
from rgbd_following_core.settings import validate
from rgbd_following_core.tracking import Tracker
from rgbd_following_core.vision import intrinsics, metric_depth, observation_clouds, detect, transform_points


class Target(object):
    def __init__(self):
        self.config = validate(rospy.get_param('~settings'))
        self.frame = rospy.get_param('~odom_frame', 'odom')
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.clock = getattr(time, 'monotonic', time.time)
        self.last_pair = self.receipt = None
        self.tracker = Tracker(**self.config['tracker'])
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pub = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
        self.valid = rospy.Publisher('target_valid', Bool, queue_size=1)
        self.cloud = rospy.Publisher('obstacle_cloud', PointCloud2, queue_size=1)
        self.clearing = rospy.Publisher('clearing_cloud', PointCloud2, queue_size=1)
        self.status = rospy.Publisher('perception_status', String, queue_size=1)
        self.preview = rospy.Publisher('target_preview', Image, queue_size=1)
        self.inputs = [message_filters.Subscriber('rgb', Image, queue_size=1, buff_size=2**24),
                       message_filters.Subscriber('registered_depth', Image, queue_size=1, buff_size=2**24),
                       message_filters.Subscriber('camera_info', CameraInfo, queue_size=1)]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.inputs, 5, self.config['sync_slop'])
        self.sync.registerCallback(self.callback)
        self.worker = threading.Thread(target=self.watchdog)
        self.worker.daemon = True
        self.worker.start()

    def reject(self, reason):
        self.tracker.miss(reason)
        self.valid.publish(False)
        evidence = dict(reason=reason, target_valid=False)
        if self.last_pair is not None:
            evidence.update(zip(('rgb_stamp', 'depth_stamp', 'info_stamp'), self.last_pair))
        self.status.publish(String(data=json.dumps(evidence)))

    def callback(self, rgb, depth, info):
        with self.lock:
            processing_started = self.clock()
            try:
                cfg, p = self.config, self.config['perception']
                stamps = [m.header.stamp.to_sec() for m in (rgb, depth, info)]
                check_pair(stamps, rospy.Time.now().to_sec(), cfg['target_timeout'], cfg['sync_slop'],
                           [m.header.frame_id for m in (rgb, depth, info)],
                           [(m.width, m.height) for m in (rgb, depth, info)], depth.encoding)
                if self.last_pair is not None and any(s <= old for s, old in zip(stamps, self.last_pair)):
                    # An obsolete synchronizer delivery is not a new negative
                    # observation. Keep the last accepted state without renewing
                    # its receipt/stamp; the watchdog still expires it normally.
                    rospy.logwarn_throttle(2, 'Ignoring repeated/out-of-order RGB-D pair')
                    return
                self.last_pair, self.receipt = stamps, self.clock()
                camera = intrinsics(info.P, rgb.width, rgb.height)
                image = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
                d = metric_depth(self.bridge.imgmsg_to_cv2(depth, 'passthrough'), depth.encoding)
                if d.shape != (depth.height, depth.width) or image.shape[:2] != d.shape:
                    return self.reject('registered_size_mismatch')
                points, clearing = observation_clouds(d, camera, p)
                clearing_msg = create_cloud_xyz32(depth.header, clearing.tolist())
                obstacle_msg = create_cloud_xyz32(depth.header, points.tolist())
                if not fresh(min(stamps), rospy.Time.now().to_sec(), cfg['target_timeout']):
                    return self.reject('processing_too_slow')
                total = len(range(0, depth.width, p['cloud_stride'])) * len(range(0, depth.height, p['cloud_stride']))
                insufficient = (len(points) < p['min_cloud_points'] or
                                len(points) / float(total) < p['min_cloud_ratio'])
                if insufficient:
                    self.reject('depth_coverage_insufficient')
                # Replace the latest marking observation even when sparse/empty:
                # persistence=0 keeps the previous one until it is replaced.
                # Otherwise old points get re-marked after every clearing pass.
                # Empty marking observations do not erase occupied voxels.
                # Keep all observed depth, including TonyPi, in collision geometry.
                self.cloud.publish(obstacle_msg)
                self.clearing.publish(clearing_msg)
                if insufficient:
                    return
                detection_started = self.clock()
                candidates, mask = detect(image, d, camera, p)
                detection_seconds = self.clock()-detection_started
                tf = self.buffer.lookup_transform(self.frame, depth.header.frame_id, depth.header.stamp, rospy.Duration(.05))
                t, q = tf.transform.translation, tf.transform.rotation
                positions = transform_points([c['point'] for c in candidates], (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
                selected = self.tracker.update([tuple(v) for v in positions], stamps[1],
                                               [c['identity_verified'] for c in candidates])
                if not fresh(min(stamps), rospy.Time.now().to_sec(), cfg['target_timeout']):
                    return self.reject('processing_too_slow')
                selected_index = next((i for i, position in enumerate(positions)
                                       if selected is not None and tuple(position) == selected), None)
                if selected is not None:
                    msg = PoseStamped()
                    msg.header.frame_id, msg.header.stamp = self.frame, depth.header.stamp
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = selected
                    msg.pose.orientation.w = 1.
                    self.pub.publish(msg)
                self.valid.publish(selected is not None)
                self.status.publish(String(data=json.dumps(dict(reason=self.tracker.reason,
                    target_valid=selected is not None, candidates=len(candidates), stamp=stamps[1],
                    rgb_stamp=stamps[0], depth_stamp=stamps[1], info_stamp=stamps[2],
                    processing_seconds=self.clock()-processing_started,
                    detection_seconds=detection_seconds,
                    observation_age_seconds=rospy.Time.now().to_sec()-min(stamps),
                    selected_candidate=selected_index,
                    identity_calibrated=cfg['identity_calibrated'], reacquired=self.tracker.reacquired,
                    obstacle_points=len(points), clearing_points=len(clearing),
                    candidate_boxes=[[int(v) for v in c['bbox']] for c in candidates],
                    candidate_points=[[float(v) for v in c['point']] for c in candidates],
                    candidate_verified=[bool(c['identity_verified']) for c in candidates],
                    candidate_sources=[c.get('sources', [c.get('feature', {}).get('kind', 'color')])
                                       for c in candidates],
                    identity_scores=[c['identity_score'] for c in candidates]))))
                view = image.copy()
                for index, candidate in enumerate(candidates):
                    x, y, w, h = candidate['bbox']
                    # Yellow means a candidate, not a confirmed followed robot.
                    color = (0, 230, 0) if index == selected_index else (0, 200, 255)
                    cv2.rectangle(view, (x, y), (x+w, y+h), color, 2)
                cv2.putText(view, self.tracker.reason, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 1)
                cv2.putText(view, 'detect %.0f ms | age %.0f ms' %
                            (detection_seconds*1000., (rospy.Time.now().to_sec()-min(stamps))*1000.),
                            (10, 47), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
                preview = self.bridge.cv2_to_imgmsg(view, 'bgr8')
                preview.header = rgb.header
                self.preview.publish(preview)
            except (ValueError, TypeError, CvBridgeError, cv2.error,
                    tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
                self.reject(str(exc))

    def watchdog(self):
        while not rospy.is_shutdown():
            with self.lock:
                if (not fresh(self.receipt, self.clock(), self.config['target_timeout']) or
                        self.last_pair is None or not fresh(min(self.last_pair), rospy.Time.now().to_sec(), self.config['target_timeout'])):
                    self.reject('input_timeout')
            time.sleep(.1)


if __name__ == '__main__':
    rospy.init_node('rgbd_target')
    Target()
    rospy.spin()

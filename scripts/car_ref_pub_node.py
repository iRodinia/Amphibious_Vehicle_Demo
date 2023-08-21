#!/usr/bin/env python
import os
import numpy as np
import csv
import rospy

from std_msgs.msg import Int8
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class refPubNode:
    def __init__(self):
        self.start_traj = False
        exec_sub_topic = rospy.get_param("start_trajectory_topic", 'car_ref_start')
        self.exec_sub = rospy.Subscriber(exec_sub_topic, Int8, self.exec_callback)
        odom_ref_topic = rospy.get_param("vehicle_odom_ref_topic", 'vehicle/odom_ref')
        self.ref_pub = rospy.Publisher(odom_ref_topic, Odometry, queue_size=1)
        self.pub_dt = rospy.get_param("reference_search_dt", 0.01)
        self.last_ref_idx = -1
        self.timer1 = rospy.Timer(rospy.Duration(self.pub_dt), self.realtime_ref_pub)

    def load_traj(self):
        file_path = rospy.get_param("trajectory_csv_file_path", "../../csv_traj/lemniscate.csv")
        if not os.path.exists(file_path):
            rospy.logwarn("Trajectory does not exist!")
            return False
        ts = []
        pos_ref = []
        vel_ref = []
        with open(file_path) as f:
            csv_reader = csv.reader(f)
            _header = next(csv_reader)
            if 'ts' in _header:
                ts_idx = _header.index('ts')
            else:
                rospy.logwarn("Time stamps does not exist.")
                return False
            if 'px' in _header:
                px_idx = _header.index('px')
            else:
                rospy.logwarn("Position X does not exist.")
                return False
            if 'py' in _header:
                py_idx = _header.index('py')
            else:
                rospy.logwarn("Position Y does not exist.")
                return False
            if 'vx' in _header:
                vx_idx = _header.index('vx')
            else:
                rospy.logwarn("Velocity X does not exist.")
                return False
            if 'vy' in _header:
                vy_idx = _header.index('vy')
            else:
                rospy.logwarn("Velocity Y does not exist.")
                return False
            for row in csv_reader:
                ts.append(row[ts_idx])
                pos_ref.append([row[px_idx], row[py_idx]])
                vel_ref.append([row[vx_idx], row[vy_idx]])
        self.ts = np.array(ts)
        self.pos_ref = np.array(pos_ref)
        self.vel_ref = np.array(vel_ref)
        self.last_ref_idx = -1
        return True

    def exec_callback(self, msg: Int8):
        _d = msg.data
        if _d == 1:
            suc = self.load_traj()
            if suc:
                self.start_traj = True
                self.start_time = rospy.Time.now()
    
    def realtime_ref_pub(self, e: rospy.timer.TimerEvent):
        if not self.start_traj:
            return
        if self.last_ref_idx >= self.ts.size - 1:
            self.start_traj = False
        t_int = (rospy.Time.now() - self.start_time).to_sec()
        _idx = self.last_ref_idx + 1
        while _idx < self.ts.size and t_int > self.ts[_idx]:
            _idx += 1
        _idx -= 1
        if _idx == self.last_ref_idx:
            return
        _pos = self.pos_ref[_idx]
        _vel = self.vel_ref[_idx]
        _yaw = np.arctan2(_vel[1], _vel[0])
        _quat = quaternion_from_euler(0., 0., _yaw)

        _msg = Odometry()
        _msg.pose.pose.position.x = _pos[0]
        _msg.pose.pose.position.y = _pos[1]
        _msg.pose.pose.position.z = 0
        _msg.pose.pose.orientation = Quaternion(*_quat)
        _msg.twist.twist.linear.x = _vel[0]
        _msg.twist.twist.linear.y = _vel[1]
        _msg.twist.twist.linear.z = 0.
        _msg.twist.twist.angular.x = 0.
        _msg.twist.twist.angular.y = 0.
        _msg.twist.twist.angular.z = 0.
        self.ref_pub.publish(_msg)

        self.last_ref_idx = _idx


if __name__ == "__main__":
    rospy.init_node("ref_pub_node")
    ref_node = refPubNode()

    rospy.spin()
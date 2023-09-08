#!/usr/bin/env python
import serial
import numpy as np
import rospy
import time
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int8, Int32MultiArray, Float64MultiArray, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

sbus_byteorder = 'big'

MIN_CTL = 1000
MAX_CTL = 2000
MID_CTL = 1500

def angle_diff(a, b):
    error = a - b
    if error < -np.pi:
        return error + 2*np.pi
    elif error >= np.pi:
        return error - 2*np.pi#大于pi减2pi
    else:
        return error

def get_bcc(inputStr: bytes) -> bytes:
    bcc = 0
    for i in inputStr:
        bcc = bcc ^ i
    return bcc.to_bytes(1, sbus_byteorder)

def val_inavchval(input: int) -> int:
    if input >= 2115:
        ret = input
    elif input >= 885:
        ret = 9+int((input-885)*1.6)
    else:
        ret =  input
    return ret

def generate_packet(throttle, yawvel, servo, clutch):
    servo = min(1800, servo)
    
    print("->input: ", throttle, " ", yawvel, " ", clutch, " ", servo)
    
    throttle = val_inavchval(throttle)
    yawvel = val_inavchval(yawvel)
    servo = val_inavchval(servo)
    clutch = val_inavchval(clutch)
    _invalid = val_inavchval(MID_CTL)
    
    print("-> cv: ", throttle, " ", yawvel, " ", clutch, " ", servo)

    head =  bytes.fromhex('0f')
    ch1 = _invalid.to_bytes(2, byteorder=sbus_byteorder)  # roll
    ch2 = _invalid.to_bytes(2, byteorder=sbus_byteorder)  # pitch
    ch3 = throttle.to_bytes(2, byteorder=sbus_byteorder)  # throttle
    ch4 = yawvel.to_bytes(2, byteorder=sbus_byteorder) # yawvel
    ch5 = _invalid.to_bytes(2, byteorder=sbus_byteorder)
    ch6 = _invalid.to_bytes(2, byteorder=sbus_byteorder)
    ch7 = servo.to_bytes(2, byteorder=sbus_byteorder)  # transform left
    ch8 = _invalid.to_bytes(2, byteorder=sbus_byteorder)
    ch9 = _invalid.to_bytes(2, byteorder=sbus_byteorder)
    ch10 = servo.to_bytes(2, byteorder=sbus_byteorder)  # transform right
    ch11 = _invalid.to_bytes(2, byteorder=sbus_byteorder)
    ch12 = _invalid.to_bytes(2, byteorder=sbus_byteorder)
    ch13 = _invalid.to_bytes(2, byteorder=sbus_byteorder)
    ch14 = _invalid.to_bytes(2, byteorder=sbus_byteorder)
    ch15 = _invalid.to_bytes(2, byteorder=sbus_byteorder)
    ch16 = _invalid.to_bytes(2, byteorder=sbus_byteorder)
    Flag = bytes.fromhex('00')

    packet = ch1 + ch2 + ch3 + ch4 + ch5 + ch6 + ch7 + ch8 + ch9 + ch10 + ch11 + ch12 + ch13 + ch14 + ch15 + ch16 + Flag
    XOR = get_bcc(packet)
    packet = head + packet + XOR
    
    print("<-packet_value: ",len(packet), packet.hex())

    return packet

def num_sat_limit(num, max_vel: float):
    if num < 0:
        result = max(-max_vel, num)
    else:
        result = min(max_vel, num)
    return result

def vec_sat_limit(vec: np.ndarray, max_vel: float = 1.):
    if max_vel <= 0:
        return vec
    result = vec.copy()
    for i in range(vec.size):
        result[i] = num_sat_limit(vec[i], max_vel)
    return result


class CarControlNode:
    def __init__(self):
        self.car_mode_enabled = False
        self.pos_set = [0., 0.]
        self.pos_state = [0., 0.]
        self.vel_set = 0.
        self.vel_state = 0.
        self.yaw_set = 0.
        self.yaw_state = 0.
        self.yawrate_set = 0.
        self.yawrate_state = 0.

        self.com = int(rospy.get_param("COM_number", 0))
        self.ser = serial.Serial("/dev/ttyUSB"+str(self.com), 115200, timeout = 0.02)
        self.ctrl_dt = rospy.get_param("control_dt", 0.01)
        mode_sub_topic = rospy.get_param("vehicle_mode_topic", 'vehicle/mode')
        self.mode_sub = rospy.Subscriber(mode_sub_topic, Int8, self.mode_callback)
        odom_sub_topic = rospy.get_param("vehicle_odom_topic", 'vehicle/odom')
        self.odom_sub = rospy.Subscriber(odom_sub_topic, Odometry, self.odom_callback)
        odom_ref_topic = rospy.get_param("vehicle_odom_ref_topic", 'vehicle/odom_ref')
        self.odom_sub = rospy.Subscriber(odom_ref_topic, Odometry, self.ref_state_callback)
        self.cmd_pub = rospy.Publisher('vehicle/realtime_cmd', Int32MultiArray, queue_size=1)

        self.allow_tune = rospy.get_param("allow_pid_tune", True)
        pid_param_topic = rospy.get_param("vehicle_pid_param_topic", 'vehicle/pid_param')
        self.pid_param_sub = rospy.Subscriber(pid_param_topic, Float64MultiArray, self.pid_param_callback)

        self.pos_kp = rospy.get_param("pos_kp", 1.)
        self.pos_kd = rospy.get_param("pos_kd", 0.)
        self.pos_ki = rospy.get_param("pos_ki", 0.)
        self.lin_vel_kp = rospy.get_param("lin_vel_kp", 10.)
        self.lin_vel_kd = rospy.get_param("lin_vel_kd", 0.)
        self.lin_vel_ki = rospy.get_param("lin_vel_ki", 0.)
        self.yaw_vel_kp = rospy.get_param("yaw_vel_kp", 10.)
        self.yaw_vel_kd = rospy.get_param("yaw_vel_kd", 0.)
        self.yaw_vel_ki = rospy.get_param("yaw_vel_ki", 0.)
        self.control = [MID_CTL, MID_CTL]
        self.last_control = [MID_CTL, MID_CTL]

        self.max_pos_err = 0.5
        self.pos_err_int = np.array([0., 0.])
        self.max_yaw_err = np.pi / 3
        self.yaw_err_int = 0.

        self.timer1 = rospy.Timer(rospy.Duration(self.ctrl_dt), self.realtime_ctl)
        self.timer2 = rospy.Timer(rospy.Duration(0.05), self.pub_realtime_cmd)

        init_bytes = generate_packet(MID_CTL, MID_CTL, MIN_CTL, MIN_CTL)
        suc = self.ser.write(init_bytes)
        if suc:
            rospy.loginfo("Initialized with flight mode.")

    def mode_callback(self, msg: Int8):
        rospy.loginfo("Receive target mode %d" % msg.data)
        rospy.loginfo("Temporary mode %d" % int(self.car_mode_enabled))
        if msg.data == 0:
            tmp_mode = False
        else:
            tmp_mode = True
        if tmp_mode == self.car_mode_enabled:
            return
        else:
            if tmp_mode:
                stable_bytes = generate_packet(MID_CTL, MID_CTL, MAX_CTL, MAX_CTL)
            else:
                stable_bytes = generate_packet(MID_CTL, MID_CTL, MIN_CTL, MIN_CTL)
            suc = self.ser.write(stable_bytes)
            if not suc:
                rospy.logwarn("Unable to change vehicle mode!")
                return
            else:
                rospy.loginfo("Mode changed to: %d" % int(self.car_mode_enabled))
                self.car_mode_enabled = tmp_mode
                self.control = [MID_CTL, MID_CTL]
                self.last_control = [MID_CTL, MID_CTL]
    
    def odom_callback(self, msg: Odometry):
        _p = msg.pose
        _t = msg.twist
        self.pos_state = [_p.pose.position.x, _p.pose.position.y]
        euler_state = euler_from_quaternion(_p.pose.orientation)
        self.yaw_state = euler_state[2]
        linear_vel = [_t.twist.linear.x, _t.twist.linear.y]
        dir = np.arctan2(linear_vel[1], linear_vel[0])
        if abs(angle_diff(dir, self.yaw_state)) < np.pi / 2:
            self.vel_state = np.linalg.norm(linear_vel)
        else:
            self.vel_state = -np.linalg.norm(linear_vel)
        self.yawrate_state = _t.twist.angular.z
    
    def ref_state_callback(self, msg: Odometry):
        _p = msg.pose
        _t = msg.twist
        self.pos_set = [_p.pose.position.x, _p.pose.position.y]
        euler_state = euler_from_quaternion(_p.pose.orientation)
        self.yaw_set = euler_state[2]
        linear_vel = [_t.twist.linear.x, _t.twist.linear.y]
        dir = np.arctan2(linear_vel[1], linear_vel[0])
        if abs(angle_diff(dir, self.yaw_set)) < np.pi / 2:
            self.vel_set = np.linalg.norm(linear_vel)
        else:
            self.vel_set = -np.linalg.norm(linear_vel)
        self.yawrate_set = _t.twist.angular.z
    
    def pid_param_callback(self, msg: Float64MultiArray):
        _d = msg.data
        if not self.allow_tune:
            return
        if len(_d) != 9:
            rospy.logwarn("Parameters length mismatch!")
            return
        self.pos_kp = max(_d[0], 0)
        self.pos_ki = max(_d[1], 0)
        self.pos_kd = max(_d[2], 0)
        self.lin_vel_kp = max(_d[3], 0)
        self.lin_vel_ki = max(_d[4], 0)
        self.lin_vel_kd = max(_d[5], 0)
        self.yaw_vel_kp = max(_d[6], 0)
        self.yaw_vel_ki = max(_d[7], 0)
        self.yaw_vel_kd = max(_d[8], 0)

    def pub_realtime_cmd(self, e: rospy.timer.TimerEvent):
        if not self.car_mode_enabled:
            return
        msg = Int32MultiArray()
        if self.car_mode_enabled:
            msg.data = [self.control[0], self.control[1], MAX_CTL-200, MAX_CTL]
        else:
            msg.data = [self.control[0], self.control[1], MIN_CTL, MIN_CTL]
        self.cmd_pub.publish(msg)
    
    def realtime_ctl(self, e: rospy.timer.TimerEvent):
        if not self.car_mode_enabled:
            return
        
        pos_err = np.array([self.pos_set[0] - self.pos_state[0], self.pos_set[1] - self.pos_state[1]])
        vel_err = self.vel_set - self.vel_state
        self.pos_err_int += self.ctrl_dt * pos_err
        self.pos_err_int = vec_sat_limit(self.pos_err_int, self.max_pos_err)
        vel_err_cmd = self.pos_kp * pos_err + self.pos_kd * vel_err + self.pos_ki * self.pos_err_int
        ang_err = angle_diff(self.yaw_set, self.yaw_state)
        ang_err_cmd = angle_diff(np.arctan2(vel_err_cmd[1], vel_err_cmd[0]), self.yaw_state)
        self.yaw_err_int += self.ctrl_dt * ang_err
        self.yaw_err_int = num_sat_limit(self.yaw_err_int, self.max_yaw_err)
        if abs(ang_err_cmd) < np.pi / 2:
            lin_vel_cmd = self.vel_set + np.linalg.norm(vel_err_cmd)
        else:
            lin_vel_cmd = self.vel_set - np.linalg.norm(vel_err_cmd)
        ang_vel_cmd = self.yaw_vel_kp * (ang_err + ang_err_cmd) + self.yaw_vel_ki * self.yaw_err_int + \
                        self.yaw_vel_kd * (self.yawrate_set - self.yawrate_state)
        
        lin_vel_ctl = num_sat_limit(lin_vel_cmd, 500)
        ang_vel_ctl = num_sat_limit(ang_vel_cmd, 500)
        ctl_bytes = generate_packet(lin_vel_ctl+MID_CTL, ang_vel_ctl+MID_CTL, MAX_CTL-200, MAX_CTL)
        suc = self.ser.write(ctl_bytes)
        if suc:
            self.last_control = self.control
            self.control = [lin_vel_ctl, ang_vel_ctl]
        

if __name__ == '__main__':
    rospy.init_node("car_control_node")
    car = CarControlNode()
    rospy.spin()

    stop_bytes = generate_packet(MID_CTL, MID_CTL, MAX_CTL-200, MAX_CTL)
    suc = False
    while not suc:
        suc = car.ser.write(stop_bytes)
        time.sleep(1.0)
    print("Car Stopped Successfully!")

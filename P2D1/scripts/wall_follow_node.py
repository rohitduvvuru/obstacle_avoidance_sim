#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import math
import time
from typing import Optional, Tuple, List

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

from q_table import (
    build_manual_q_table,
    TOO_CLOSE, CLOSE, DW, FAR,
    BLOCKED, CLEAR,
    ACTIONS,
    A_FWD_FAST, A_FWD_SLOW,
    A_TURN_L_SOFT, A_TURN_R_SOFT,
    A_TURN_L_HARD, A_TURN_R_HARD,
)


def clamp_angle_rad(a: float) -> float:
    # Normalize to [-pi, pi]
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class WallFollowD1:
    def __init__(self) -> None:
        self.gap_deg = float(rospy.get_param("~gap_deg", 15.0))
        self.sector_deg = float(rospy.get_param("~sector_deg", 60.0))

        self.d_w = float(rospy.get_param("~d_w", 0.75))
        self.d_near = float(rospy.get_param("~d_near", 1.00))
        self.d_lost = float(rospy.get_param("~d_lost", 1.50))
        self.front_blocked = float(rospy.get_param("~front_blocked", 0.60))

        self.v_fast = float(rospy.get_param("~v_fast", 0.25))
        self.v_slow = float(rospy.get_param("~v_slow", 0.12))
        self.w_soft = float(rospy.get_param("~w_soft", 0.50))
        self.w_hard = float(rospy.get_param("~w_hard", 0.90))

        self.control_hz = float(rospy.get_param("~control_hz", 10.0))
        self.robust_k = int(rospy.get_param("~robust_k", 5))

        self.use_pose_reset = bool(rospy.get_param("~use_pose_reset", False))
        self.model_name = str(rospy.get_param("~model_name", "triton"))

        self.Q = build_manual_q_table()

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.latest_scan: Optional[LaserScan] = None
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self._scan_cb, queue_size=1)

        rospy.loginfo("wall_following_d1: waiting for first /scan...")
        while not rospy.is_shutdown() and self.latest_scan is None:
            rospy.sleep(0.05)

        if self.use_pose_reset:
            self._try_reset_pose()

        rospy.loginfo("wall_following_d1: ready.")

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _try_reset_pose(self) -> None:
        # Spawns right-wall following robot near wall
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=3.0)
            set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            st = ModelState()

            st.model_name = self.model_name
            st.pose.position.x = -3.4911271113192655
            st.pose.position.y = -3.5158118437849675
            st.pose.position.z = 0.0

            st.pose.orientation.x = -1.4769940887105835e-06
            st.pose.orientation.y = 1.048504801470323e-05
            st.pose.orientation.z = -0.00023837490883773285
            st.pose.orientation.w = 0.9999999715326422

            st.twist.linear.x = 0.0
            st.twist.linear.y = 0.0
            st.twist.linear.z = 0.0
            st.twist.angular.x = 0.0
            st.twist.angular.y = 0.0
            st.twist.angular.z = 0.0
            st.reference_frame = "world"

            resp = set_state(st)
            rospy.loginfo(f"SetModelState response: success={resp.success}, status={resp.status_message}")
        except Exception as e:
            rospy.logwarn(f"Pose reset failed (safe to ignore): {e}")

    def _sector_bounds_rad(self):
        """
        Front:     [-10, +10]
        FrontRight [-60, -20]
        FrontLeft  [+20, +60]
        """
        F  = (math.radians(-10.0), math.radians(+10.0))
        FR = (math.radians(-60.0), math.radians(-20.0))
        FL = (math.radians(+20.0), math.radians(+60.0))
        return FR, F, FL

    def _robust_region_distance(self, scan: LaserScan, a_lo: float, a_hi: float) -> float:
        # k-th smallest distance reading per region to ignore noisy LiDAR values
        assert a_lo < a_hi
        vals: List[float] = []
        a = scan.angle_min
        for i, r in enumerate(scan.ranges):
            ang = a + i * scan.angle_increment
            ang = clamp_angle_rad(ang)
            if ang < a_lo or ang > a_hi:
                continue
            if not math.isfinite(r):
                continue
            if r <= 0.0:
                continue
            vals.append(r)

        if not vals:
            return float("inf")

        vals.sort()
        k = min(max(self.robust_k, 1), len(vals))
        return vals[k - 1]

    def _bin_distance(self, d: float) -> str:
        too_close_th = max(0.25, self.d_w - 0.40)  # 0.35
        close_th     = self.d_w - 0.15             # 0.6
        far_th       = self.d_w + 0.20             # 0.95

        if d < too_close_th:
            return TOO_CLOSE
        if d < close_th:
            return CLOSE
        if d <= far_th:
            return DW
        return FAR
    
    def _bin_distance_fr(self, d_fr: float) -> str:
        return self._bin_distance(d_fr)

    def _front_flag(self, d_front: float) -> str:
        return BLOCKED if d_front < self.front_blocked else CLEAR

    def _state(self, scan: LaserScan):
        (fr_lo, fr_hi), (f_lo, f_hi), (fl_lo, fl_hi) = self._sector_bounds_rad()

        d_fr = self._robust_region_distance(scan, fr_lo, fr_hi)
        d_f  = self._robust_region_distance(scan, f_lo, f_hi)
        d_fl = self._robust_region_distance(scan, fl_lo, fl_hi)

        front_flag = self._front_flag(d_f)
        fr_bin = self._bin_distance_fr(d_fr)

        return (fr_bin, front_flag)

    def _twist_for_action(self, a: str) -> Twist:
        t = Twist()
        if a == A_FWD_FAST:
            t.linear.x = self.v_fast
            t.angular.z = 0.0
        elif a == A_FWD_SLOW:
            t.linear.x = self.v_slow
            t.angular.z = 0.0
        elif a == A_TURN_L_SOFT:
            t.linear.x = self.v_slow
            t.angular.z = +self.w_soft
        elif a == A_TURN_R_SOFT:
            t.linear.x = self.v_slow
            t.angular.z = -self.w_soft
        elif a == A_TURN_L_HARD:
            t.linear.x = self.v_slow
            t.angular.z = +self.w_hard
        elif a == A_TURN_R_HARD:
            t.linear.x = self.v_slow
            t.angular.z = -self.w_hard
        else:
            t.linear.x = self.v_slow
            t.angular.z = 0.0
        return t

    def _argmax_action(self, s: Tuple[str, str]) -> str:
        qvals = self.Q.get(s, None)
        if qvals is None:
            return A_FWD_SLOW
        best_i = 0
        best_v = qvals[0]
        for i in range(1, len(qvals)):
            if qvals[i] > best_v:
                best_v = qvals[i]
                best_i = i
        return ACTIONS[best_i]

    def _loop(self) -> None:
        r = rospy.Rate(self.control_hz)
        while not rospy.is_shutdown():
            scan = self.latest_scan
            if scan is None:
                r.sleep()
                continue

            s = self._state(scan)          # (fr_bin, front_flag)
            a = self._argmax_action(s)
            cmd = self._twist_for_action(a)
            self.cmd_pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node("wall_follow_d1", anonymous=False)
    node = WallFollowD1()
    node._loop()
#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import csv
import math
import pickle
import random
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty

from q_table import (
    TOO_CLOSE, CLOSE, DW, FAR,
    BLOCKED, CLEAR,
    ACTIONS_RL, A_FWD, A_LEFT, A_RIGHT,
    build_manual_q_table_d2_initial,
)

State = Tuple[str, str, str]  # (fr_bin, front_flag, fl_bin)


def clamp_angle_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class RLWallFollowNode:
    def __init__(self) -> None:
        self.algorithm = rospy.get_param("~algorithm", "q_learning")
        self.mode = rospy.get_param("~mode", "train")

        self.alpha = float(rospy.get_param("~alpha", 0.2))
        self.gamma = float(rospy.get_param("~gamma", 0.8))
        self.epsilon0 = float(rospy.get_param("~epsilon0", 0.9))
        self.epsilon_decay = float(rospy.get_param("~epsilon_decay", 0.985))
        self.min_epsilon = float(rospy.get_param("~min_epsilon", 0.05))

        self.control_hz = float(rospy.get_param("~control_hz", 5.0))
        self.max_steps = int(rospy.get_param("~max_steps", 300))
        self.num_episodes = int(rospy.get_param("~num_episodes", 300))
        self.prev_positions: List[Tuple[float, float]] = []
        self.trap_window = int(rospy.get_param("~trap_window", 5))
        self.trap_dist_thresh = float(rospy.get_param("~trap_dist_thresh", 0.03))

        self.d_w = float(rospy.get_param("~d_w", 0.75))
        self.front_blocked = float(rospy.get_param("~front_blocked", 0.60))
        self.robust_k = int(rospy.get_param("~robust_k", 5))

        # Triton frame for Project 2: forward is +linear.y
        self.v_forward = float(rospy.get_param("~v_forward", 0.3))
        self.w_turn = float(rospy.get_param("~w_turn", math.pi / 4.0))

        self.model_name = str(rospy.get_param("~model_name", "triton"))
        self.qtable_path = str(rospy.get_param("~qtable_path", "/tmp/p2_qtable.pkl"))
        self.best_qtable_path = str(rospy.get_param("~best_qtable_path", "/tmp/p2_best_qtable.pkl"))
        self.rewards_csv = str(rospy.get_param("~rewards_csv", "/tmp/p2_rewards.csv"))

        self.latest_scan: Optional[LaserScan] = None

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self._scan_cb, queue_size=1)

        self.Q: Dict[State, List[float]] = build_manual_q_table_d2_initial()

        if self.mode == "test":
            self._load_qtable()

        rospy.loginfo("Waiting for first /scan ...")
        while not rospy.is_shutdown() and self.latest_scan is None:
            rospy.sleep(0.05)

        self._unpause_physics()

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _current_xy(self) -> Tuple[float, float]:
        rospy.wait_for_service("/gazebo/get_model_state", timeout=3.0)
        from gazebo_msgs.srv import GetModelState
        get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        resp = get_state(self.model_name, "world")
        return resp.pose.position.x, resp.pose.position.y

    def _unpause_physics(self) -> None:
        try:
            rospy.wait_for_service("/gazebo/unpause_physics", timeout=3.0)
            srv = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
            srv()
        except Exception as e:
            rospy.logwarn(f"Could not unpause physics: {e}")

    def _reset_pose(self) -> None:
        """
        Reset the robot to a randomly chosen pre-defined pose-orientation pair. 
        Each pair is defined such that the robot starts parallel to a wall on its right side.
        """
        rospy.wait_for_service("/gazebo/set_model_state", timeout=3.0)
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        start_poses = [
            {
                "x": 3.4067602157592773,
                "y": -3.4777286052703857,
                "z": 3.7426732433232246e-06,
                "qx": 1.0547367992330692e-06,
                "qy": 1.0535978617554065e-05,
                "qz": 0.05672767758369446,
                "qw": 0.9983896613121033,
            },
            {
                "x": 2.7792844772338867,
                "y": 3.0149385929107666,
                "z": 3.7426732433232246e-06,
                "qx": 7.756540071568452e-06,
                "qy": 7.2080092650139704e-06,
                "qz": 0.702582061290741,
                "qw": 0.7116026878356934,
            },
            {
                "x": 1.1256754398345947,
                "y": 2.4764487743377686,
                "z": 3.7426732433232246e-06,
                "qx": -7.195042599050794e-06,
                "qy": 7.7685690484941e-06,
                "qz": -0.7104281187057495,
                "qw": 0.7037697434425354,
            },
            {
                "x": 2.4812488555908203,
                "y": 0.7196335792541504,
                "z": 3.7426732433232246e-06,
                "qx": -1.0577068678685464e-05,
                "qy": 4.948893774781027e-07,
                "qz": -0.9999930262565613,
                "qw": 0.0037372668739408255,
            },
            {
                "x": -2.8564133644104004,
                "y": 1.441973328590393,
                "z": 3.7426732433232246e-06,
                "qx": -7.792034011799842e-06,
                "qy": -7.169623131630942e-06,
                "qz": -0.706087052822113,
                "qw": -0.7081250548362732,
            },
            {
                "x": -0.4730428457260132,
                "y": -2.6577186584472656,
                "z": 3.7426732433232246e-06,
                "qx": -7.792034011799842e-06,
                "qy": -7.169623131630942e-06,
                "qz": -0.706087052822113,
                "qw": -0.7081250548362732,
            },
        ]

        p = random.choice(start_poses)

        st = ModelState()
        st.model_name = self.model_name

        st.pose.position.x = p["x"]
        st.pose.position.y = p["y"]
        st.pose.position.z = p["z"]

        st.pose.orientation.x = p["qx"]
        st.pose.orientation.y = p["qy"]
        st.pose.orientation.z = p["qz"]
        st.pose.orientation.w = p["qw"]

        st.twist.linear.x = 0.0
        st.twist.linear.y = 0.0
        st.twist.linear.z = 0.0
        st.twist.angular.x = 0.0
        st.twist.angular.y = 0.0
        st.twist.angular.z = 0.0
        st.reference_frame = "world"

        resp = set_state(st)
        rospy.loginfo(
            f"Reset pose: x={p['x']:.3f}, y={p['y']:.3f}, "
            f"qz={p['qz']:.3f}, qw={p['qw']:.3f}, success={resp.success}"
        )

        rospy.sleep(0.2)

    def _sector_bounds_rad(self):
        """
        Since forward direction is positive y-axis in Triton simulation frame
        and considering LaserScan starts from positive x-axis, increasing
        in counterclockwise direction:
        Front      ~ 90 deg
        FrontRight ~ 45 deg
        FrontLeft  ~ 135 deg
        """
        F  = (math.radians(80.0),  math.radians(100.0))
        FR = (math.radians(25.0),  math.radians(65.0))
        FL = (math.radians(115.0), math.radians(155.0))
        return FR, F, FL

    def _robust_region_distance(self, scan: LaserScan, a_lo: float, a_hi: float) -> float:
        vals: List[float] = []
        a = scan.angle_min
        for i, r in enumerate(scan.ranges):
            ang = clamp_angle_rad(a + i * scan.angle_increment)
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
        too_close_th = 0.45
        close_th = 0.65
        far_th = 0.95

        if d < too_close_th:
            return TOO_CLOSE
        if d < close_th:
            return CLOSE
        if d <= far_th:
            return DW
        return FAR

    def _front_flag(self, d_front: float) -> str:
        return BLOCKED if d_front < self.front_blocked else CLEAR

    def _state_from_scan(self, scan: LaserScan) -> State:
        (fr_lo, fr_hi), (f_lo, f_hi), (fl_lo, fl_hi) = self._sector_bounds_rad()

        d_fr = self._robust_region_distance(scan, fr_lo, fr_hi)
        d_f  = self._robust_region_distance(scan, f_lo, f_hi)
        d_fl = self._robust_region_distance(scan, fl_lo, fl_hi)

        fr_bin = self._bin_distance(d_fr)
        front_flag = self._front_flag(d_f)
        fl_bin = self._bin_distance(d_fl)

        return (fr_bin, front_flag, fl_bin)

    def _reward(self, state: State) -> float:
        fr_bin, front_flag, fl_bin = state

        if front_flag == BLOCKED:
            return -1.0

        if fr_bin == TOO_CLOSE:
            return -2.0

        if fr_bin == FAR:
            if front_flag == CLEAR:
                return -0.2
            return -1.0

        if fr_bin == DW and front_flag == CLEAR:
            return 0.5

        if fr_bin == CLOSE:
            return 0.0

        return 0.0

    def _is_terminal(self, state: State, step_idx: int) -> bool:
        fr_bin, front_flag, _ = state

        if front_flag == BLOCKED and fr_bin == TOO_CLOSE:
            return True

        if step_idx >= self.max_steps:
            return True

        xy = self._current_xy()
        self.prev_positions.append(xy)
        if len(self.prev_positions) > self.trap_window:
            self.prev_positions.pop(0)

        if len(self.prev_positions) == self.trap_window:
            x0, y0 = self.prev_positions[0]
            x1, y1 = self.prev_positions[-1]
            dist = math.hypot(x1 - x0, y1 - y0)
            if dist < self.trap_dist_thresh:
                return True

        return False

    def _epsilon(self, episode: int) -> float:
        eps = self.epsilon0 * (self.epsilon_decay ** episode)
        return max(self.min_epsilon, eps)

    def _greedy_action(self, state: State) -> str:
        qvals = self.Q[state]
        best_i = max(range(len(qvals)), key=lambda i: qvals[i])
        return ACTIONS_RL[best_i]

    def _epsilon_greedy_action(self, state: State, episode: int) -> str:
        if self.mode == "test":
            return self._greedy_action(state)
        eps = self._epsilon(episode)
        if random.random() < eps:
            return random.choice(ACTIONS_RL)
        return self._greedy_action(state)

    def _twist_for_action(self, action: str) -> Twist:
        t = Twist()
        if action == A_FWD:
            t.linear.x = 0.0
            t.linear.y = self.v_forward
            t.angular.z = 0.0
        elif action == A_LEFT:
            t.linear.x = 0.0
            t.linear.y = 0.5 * self.v_forward
            t.angular.z = +self.w_turn
        elif action == A_RIGHT:
            t.linear.x = 0.0
            t.linear.y = 0.5 * self.v_forward
            t.angular.z = -self.w_turn
        return t

    def _step_action(self, action: str) -> None:
        self.cmd_pub.publish(self._twist_for_action(action))

    def _save_qtable_to(self, path: str) -> None:
        with open(path, "wb") as f:
            pickle.dump(self.Q, f)

    def _save_qtable(self) -> None:
        self._save_qtable_to(self.qtable_path)

    def _load_qtable(self) -> None:
        if os.path.exists(self.best_qtable_path):
            with open(self.best_qtable_path, "rb") as f:
                self.Q = pickle.load(f)
            rospy.loginfo(f"Loaded BEST Q-table from {self.best_qtable_path}")
        elif os.path.exists(self.qtable_path):
            with open(self.qtable_path, "rb") as f:
                self.Q = pickle.load(f)
            rospy.loginfo(f"Loaded final Q-table from {self.qtable_path}")
        else:
            rospy.logwarn("No saved Q-table found; using D1 initialization")

    def _append_reward_log(self, episode: int, total_reward: float, steps: int, is_best: bool, best_reward_so_far: float, best_steps_so_far: int,) -> None:
        file_exists = os.path.exists(self.rewards_csv)
        with open(self.rewards_csv, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    "episode",
                    "algorithm",
                    "mode",
                    "total_reward",
                    "steps",
                    "is_best",
                    "best_reward_so_far",
                    "best_steps_so_far",
                ])
            writer.writerow([
                episode,
                self.algorithm,
                self.mode,
                total_reward,
                steps,
                int(is_best),
                best_reward_so_far,
                best_steps_so_far,
            ])

    def run_training(self) -> None:
        rate = rospy.Rate(self.control_hz)
 
        best_reward = -float("inf")
        best_steps = -1
        best_episode = -1

        for ep in range(self.num_episodes):
            if rospy.is_shutdown():
                break

            self._reset_pose()
            self.prev_positions = []
            rospy.sleep(0.2)

            state = self._state_from_scan(self.latest_scan)
            action = self._epsilon_greedy_action(state, ep)

            total_reward = 0.0
            step_idx = 0

            while not rospy.is_shutdown():
                step_idx += 1

                self._step_action(action)
                rate.sleep()

                next_state = self._state_from_scan(self.latest_scan)
                reward = self._reward(next_state)
                total_reward += reward

                a_idx = ACTIONS_RL.index(action)

                if self.algorithm == "q_learning":
                    target = reward + self.gamma * max(self.Q[next_state])
                elif self.algorithm == "sarsa":
                    next_action = self._epsilon_greedy_action(next_state, ep)
                    next_a_idx = ACTIONS_RL.index(next_action)
                    target = reward + self.gamma * self.Q[next_state][next_a_idx]
                else:
                    raise ValueError(f"Unknown algorithm: {self.algorithm}")

                self.Q[state][a_idx] += self.alpha * (target - self.Q[state][a_idx])

                terminal = self._is_terminal(next_state, step_idx)
                if terminal:
                    break

                state = next_state
                if self.algorithm == "q_learning":
                    action = self._epsilon_greedy_action(state, ep)
                else:
                    action = next_action

            # determine whether this episode is the best so far
            is_better = False
            if total_reward > best_reward:
                is_better = True
            elif total_reward == best_reward and step_idx > best_steps:
                is_better = True

            if is_better:
                best_reward = total_reward
                best_steps = step_idx
                best_episode = ep
                self._save_qtable_to(self.best_qtable_path)
                rospy.loginfo(
                    f"New best policy saved at episode={best_episode}: "
                    f"reward={best_reward:.2f}, steps={best_steps}"
                )

            self._append_reward_log(
                ep,
                total_reward,
                step_idx,
                is_better,
                best_reward,
                best_steps,
            )

            rospy.loginfo(
                f"[{self.algorithm}] episode={ep} reward={total_reward:.2f} "
                f"steps={step_idx} epsilon={self._epsilon(ep):.3f} "
                f"best_episode={best_episode}"
            )

        self._save_qtable()

    def run_test(self) -> None:
        rate = rospy.Rate(self.control_hz)
        self._reset_pose()
        rospy.sleep(0.2)

        while not rospy.is_shutdown():
            state = self._state_from_scan(self.latest_scan)
            action = self._greedy_action(state)
            self._step_action(action)
            rospy.loginfo_throttle(1.0, f"[TEST {self.algorithm}] state={state} action={action}")
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rl_wall_follow")
    node = RLWallFollowNode()

    if node.mode == "train":
        node.run_training()
    else:
        node.run_test()
#!/usr/bin/env python3
from __future__ import annotations

from typing import Dict, List, Tuple

TOO_CLOSE = "TOO_CLOSE"
CLOSE = "CLOSE"
DW = "DW"
FAR = "FAR"

BLOCKED = "BLOCKED"
CLEAR = "CLEAR"

A_FWD = "FWD"
A_LEFT = "LEFT"
A_RIGHT = "RIGHT"

ACTIONS_RL: List[str] = [A_FWD, A_LEFT, A_RIGHT]

StateD1 = Tuple[str, str]          # (fr_bin, front_flag)
StateD2 = Tuple[str, str, str]     # (fr_bin, front_flag, fl_bin)


def build_manual_q_table_d1_seed() -> Dict[StateD1, List[float]]:
    q: Dict[StateD1, List[float]] = {}

    for fr_bin in [TOO_CLOSE, CLOSE, DW, FAR]:
        for front_flag in [CLEAR, BLOCKED]:
            s: StateD1 = (fr_bin, front_flag)
            vals = [-1.0, -1.0, -1.0]  # [FWD, LEFT, RIGHT]

            if front_flag == BLOCKED:
                vals[ACTIONS_RL.index(A_LEFT)] = 10.0
                vals[ACTIONS_RL.index(A_FWD)] = 0.0
                q[s] = vals
                continue

            if fr_bin == FAR:
                vals[ACTIONS_RL.index(A_RIGHT)] = 9.0
                vals[ACTIONS_RL.index(A_FWD)] = 4.0
            elif fr_bin == DW:
                vals[ACTIONS_RL.index(A_FWD)] = 12.0
                vals[ACTIONS_RL.index(A_LEFT)] = 0.0
                vals[ACTIONS_RL.index(A_RIGHT)] = 0.0
            elif fr_bin == CLOSE:
                vals[ACTIONS_RL.index(A_LEFT)] = 9.0
                vals[ACTIONS_RL.index(A_FWD)] = 3.0
            else:  # TOO_CLOSE
                vals[ACTIONS_RL.index(A_LEFT)] = 10.0
                vals[ACTIONS_RL.index(A_FWD)] = 1.0

            q[s] = vals

    return q


def build_manual_q_table_d2_initial() -> Dict[StateD2, List[float]]:
    q_d1 = build_manual_q_table_d1_seed()
    q_d2: Dict[StateD2, List[float]] = {}

    for fr_bin in [TOO_CLOSE, CLOSE, DW, FAR]:
        for front_flag in [CLEAR, BLOCKED]:
            base_vals = q_d1[(fr_bin, front_flag)]
            for fl_bin in [TOO_CLOSE, CLOSE, DW, FAR]:
                q_d2[(fr_bin, front_flag, fl_bin)] = list(base_vals)

    return q_d2
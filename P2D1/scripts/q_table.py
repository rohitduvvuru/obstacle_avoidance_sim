#!/usr/bin/env python3
from typing import Dict, List, Tuple

TOO_CLOSE = "TOO_CLOSE"
CLOSE = "CLOSE"
DW = "DW"
FAR = "FAR"

BLOCKED = "BLOCKED"
CLEAR = "CLEAR"

A_FWD_FAST = "FWD_FAST"
A_FWD_SLOW = "FWD_SLOW"
A_TURN_L_SOFT = "TURN_L_SOFT"
A_TURN_R_SOFT = "TURN_R_SOFT"
A_TURN_L_HARD = "TURN_L_HARD"
A_TURN_R_HARD = "TURN_R_HARD"

ACTIONS: List[str] = [
    A_FWD_FAST,
    A_FWD_SLOW,
    A_TURN_L_SOFT,
    A_TURN_R_SOFT,
    A_TURN_L_HARD,
    A_TURN_R_HARD,
]

State = Tuple[str, str]  # (fr_bin, front_flag)

def build_manual_q_table() -> Dict[State, List[float]]:
    q: Dict[State, List[float]] = {}

    for fr_bin in [TOO_CLOSE, CLOSE, DW, FAR]:
        for front_flag in [CLEAR, BLOCKED]:
            s: State = (fr_bin, front_flag)
            vals = [-1.0] * len(ACTIONS)

            if front_flag == BLOCKED:
                # obstacle ahead: turn LEFT (away from right wall) to avoid collision
                vals[ACTIONS.index(A_TURN_L_HARD)] = 10.0
                vals[ACTIONS.index(A_TURN_L_SOFT)] = 8.0
                vals[ACTIONS.index(A_FWD_SLOW)] = 0.0
                q[s] = vals
                continue

            # front clear: regulate distance using FR only
            if fr_bin == FAR:
                # too far from wall -> turn right gently to approach wall
                vals[ACTIONS.index(A_TURN_R_SOFT)] = 9.0
                vals[ACTIONS.index(A_FWD_SLOW)] = 5.0
            elif fr_bin == DW:
                # perfect -> go straight
                vals[ACTIONS.index(A_FWD_FAST)] = 12.0
                vals[ACTIONS.index(A_FWD_SLOW)] = 8.0
                vals[ACTIONS.index(A_TURN_L_SOFT)] = 0.0
                vals[ACTIONS.index(A_TURN_R_SOFT)] = 0.0
            elif fr_bin == CLOSE:
                # a bit close -> turn left gently
                vals[ACTIONS.index(A_TURN_L_SOFT)] = 9.0
                vals[ACTIONS.index(A_FWD_SLOW)] = 4.0
            else:  # TOO_CLOSE
                # dangerously close -> hard left
                vals[ACTIONS.index(A_TURN_L_HARD)] = 10.0
                vals[ACTIONS.index(A_FWD_SLOW)] = 1.0

            q[s] = vals

    return q
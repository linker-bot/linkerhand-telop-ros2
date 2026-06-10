import numpy as np
from linkerhand.handcore import HandCore

class RightHand:
    def __init__(self, handcore: HandCore, length=20):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        self.g_jointpositions = joint_arc[:20]     # 完整复制

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            slow_rimit = 2
            fast_rimit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.8)
            min_vel = int(self.last_jointvelocity[i] * 0.6)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 10 + 5
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_rimit:
                    target_vel = position_error * 10 + 30
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 10 + 10
            else:  # fast
                if position_error >= fast_rimit:
                    target_vel = position_error * 10 + 50
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_rimit < position_error < fast_rimit:
                    target_vel = position_error * 10 + 30
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_rimit:
                    target_vel = position_error * 10 + 10
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity [i] = int(target_vel)

            if self.g_jointvelocity [i] > 255:
                self.g_jointvelocity [i] = 255
            self.g_jointvelocity [i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity [i]
            self.last_jointpositions[i] = self.g_jointpositions[i]


class LeftHand:
    def __init__(self, handcore: HandCore, length=20):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        self.g_jointpositions = joint_arc[:20]     # 完整复制

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            slow_limit = 2
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.8)
            min_vel = int(self.last_jointvelocity[i] * 0.6)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 10 + 5
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 10 + 30
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 10 + 10
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 10 + 50
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 10 + 30
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 10 + 10
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity [i] = int(target_vel)

            if self.g_jointvelocity [i] > 255:
                self.g_jointvelocity [i] = 255
            self.g_jointvelocity [i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity [i]
            self.last_jointpositions[i] = self.g_jointpositions[i]

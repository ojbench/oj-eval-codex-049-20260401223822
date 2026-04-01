import sys

import config


############################
# file structure:
# test_id
# max_round robot_num
# (robot_1 ~ robot_n:)  x_init  y_init  x_tar  y_tar  r  v_max
# (repeated for 1 ~ max_round times:)
#   (robot_1 ~ robot_n:)  x_now  y_now
############################ 

class file_reader:
    def __init__(self):
        self.robot_num = None
        self.beg_round = 0
        self.end_round = None
        self.x_init = []
        self.y_init = []
        self.x_tar = []
        self.y_tar = []
        self.r = []
        self.v_max = []
        self.x_trace = []
        self.y_trace = []
        self.x_min = sys.float_info.max
        self.y_min = sys.float_info.max
        self.x_max = sys.float_info.min
        self.y_max = sys.float_info.min
        self.r_max = 0
        self.read_x = 0  # 0: none; 1: x; -1: y

    def read_info(self, file_name):
        float_get = self.read_float(file_name)
        self.end_round = int(next(float_get))
        self.robot_num = int(next(float_get))
        if config.end_round != -1:
            self.end_round = min(self.end_round, config.end_round)
        if config.beg_round != -1:
            self.beg_round = min(config.beg_round, self.end_round)
        for i in range(self.robot_num):
            self.read_x = -1  # shift status
            self.x_init.append(next(float_get))
            self.y_init.append(next(float_get))
            self.x_tar.append(next(float_get))
            self.y_tar.append(next(float_get))
            self.read_x = 0  # shift status
            self.r.append(next(float_get))
            self.r_max = max(self.r_max, self.r[-1])
            self.v_max.append(next(float_get))
            self.x_trace.append([])
            self.y_trace.append([])
        self.read_x = -1  # shift status
        for i in range(self.end_round):
            try:
                if i % config.interval == 0:
                    for j in range(self.robot_num):
                        self.x_trace[j].append(next(float_get))
                        self.y_trace[j].append(next(float_get))
                else:
                    for j in range(self.robot_num):
                        next(float_get)
                        next(float_get)
            except StopIteration:
                self.end_round = i
                break
        x_range = self.x_max - self.x_min
        y_range = self.y_max - self.y_min
        self.x_min -= x_range * 0.25
        self.x_max += x_range * 0.25
        self.y_min -= y_range * 0.25
        self.y_max += y_range * 0.25
        self.x_min -= self.r_max
        self.x_max += self.r_max
        self.y_min -= self.r_max
        self.y_max += self.r_max

    def read_float(self, file_name):
        with open(file_name, 'r') as file:
            for line in file:
                tmp = float(line.strip())
                if self.read_x == 1:
                    self.x_max = max(self.x_max, tmp)
                    self.x_min = min(self.x_min, tmp)
                elif self.read_x == -1:
                    self.y_max = max(self.y_max, tmp)
                    self.y_min = min(self.y_min, tmp)
                yield tmp
                self.read_x = - self.read_x

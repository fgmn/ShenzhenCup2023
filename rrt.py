"""
RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import time
import random

import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

import env, plotting, utils

STEP_LEN = 10

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_max):
            # 生成一个随机点，并且一定概率返回目标点
            node_rand = self.generate_random_node(self.goal_sample_rate)
            # 获取最近的邻居
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            # 调用utils中的is_collision方法判断是否冲突
            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)
                # 尝试直接和目标点相连
                if dist <= self.step_len and not self.utils.is_collision(node_new, self.s_goal):
                    self.new_state(node_new, self.s_goal)
                    # 若成功则直接返回路径
                    return self.extract_path(node_new)

        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        # 获取两个点之间的距离，方向角
        dist, theta = self.get_distance_and_angle(node_start, node_end)
        # 限幅
        # dist = min(self.step_len, dist)
        dist = self.step_len    # 直接指定步长

        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

def align_solution(paths, target_len, x_start, x_goal):
    sym_paths = []
    for path in paths:
        sym_path = [[x[0], -x[1]] for x in path]
        sym_path.reverse()
        sym_paths.append(sym_path)

    for x in paths:
        while len(x) < target_len:
            x.append(x_goal)
    for x in sym_paths:
        while len(x) < target_len:
            x.append(x_start)
    return paths, sym_paths

def check_path(x, y):
    checker = utils.Utils()     # 创建实例之后才能调用is_intersect_circle方法
    for i in range(len(x)):
        o, d = utils.Utils.get_ray(Node(x[i]), Node(y[i]))
        if not checker.is_intersect_circle(o, d, [0, 0], 500):
            return False
    return True

def pairing(pa, pb):
    sol = []
    for x in pa:
        for y in pb:
            if check_path(x, y):
                sol.append([x, y])
    return sol

def sample_the_circle(o, r):
    samples = []
    for k in range(360):
        x = o[0] + r * math.cos(np.deg2rad(k))
        y = o[1] + r * math.sin(np.deg2rad(k))
        samples.append([x, y])
    return samples

def dist(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def check_radius_of_turing(a, b, c):
    upper_bound = math.pi - 2 * math.acos(10 / 60)
    ab = dist(a, b)
    ac = dist(a, c)
    bc = dist(b, c)

    cos_value = (ab ** 2 + bc ** 2 - ac ** 2) / (2 * ab * bc)
    # 精度问题
    if cos_value > 1:
        return True
    elif cos_value < -1:
        return False

    try:
        ang = math.acos(cos_value)
    except Exception as e:
        print(f"exception: {e} cos({cos_value})×")

    ang = math.pi - ang
    return True if ang <= upper_bound else False

def generate_path_with_constraint(path, x_start, x_goal, rrt):
    rpath = []
    rpath.append(x_start)

    checker = utils.Utils()
    for i in range(1, len(path)):
        # 在圆周上均匀采样360个点
        samples = sample_the_circle(rpath[i - 1], STEP_LEN)
        # 不得和障碍圆相交
        samples0 = []
        for x in samples:
            if x[0] ** 2 + x[1] ** 2 >= 500 ** 2:
                samples0.append(x)
        # 根据“不得碰面””转弯半径不超过30m“约束进行筛选
        samples1 = []
        for x in samples0:
            o, d = utils.Utils.get_ray(Node(path[i]), Node(x))
            if checker.is_intersect_circle(o, d, [0, 0], 500):
                samples1.append(x)

        samples2 = []
        if i >= 2:
            for x in samples1:
                if check_radius_of_turing(rpath[i - 2], rpath[i - 1], x):
                    samples2.append(x)
        else:
            samples2 = samples1

        if not samples2:
            print("exit at {}".format(i))
            return []
            # exit(1)

        # selected_sample = samples2[int(np.argmin([dist(x, x_goal)
        #                                           for x in samples2]))]

        # selected_sample = random.choice(samples2)

        dist_set = [dist(x, x_goal) for x in samples2]
        # 使用enumerate()函数获取每个元素的索引和值
        enumerate_list = list(enumerate(dist_set))
        # 使用sorted()函数对元素按值进行排序
        sorted_list = sorted(enumerate_list, key=lambda x: x[1])
        # 提取前180个最小值的索引，进行抽签
        top_10_indices = [index for index, value in sorted_list[:180]]
        selected_sample = samples2[random.choice(top_10_indices)]

        rpath.append(selected_sample)

        # 终止条件1
        if dist(selected_sample, x_goal) <= STEP_LEN:
            rpath.append(x_goal)
            return rpath

    # 终止条件2：无人机B抵达，无人机A只需向站点前进即可
    while dist(rpath[-1], x_goal) > STEP_LEN:
        nxt = rrt.new_state(Node(rpath[-1]), Node(x_goal))
        rpath.append([nxt.x, nxt.y])
    rpath.append(x_goal)
    return rpath

'''
@Todo & summary
1.生成多组路径根据“不得碰面”约束进行匹配，得到可行解
    最终生成路径采样点数目不同 → 将目标点拷贝多份进行对齐即可
    采样点之间距离并不是严格一个step → 直接指定步长
result：没有匹配的解！
无人机A需要“迂回”“原地绕飞”策略，rrt对目标点有趋向性，两者冲突

2.在rrt生成路径的基础上，根据“不得碰面”“转弯半径不超过30m”约束生成一条最优的路径A→B，组合成可行解
①在下一步目标圆周上均匀采样360个点，根据2个约束进行筛选得到一个可行集合
②贪心地选取下一步：距离目标点最近 or 轮盘赌
'''

def main():
    x_start = (-1000, 0)  # Starting node
    x_goal = (3500, 0)  # Goal node

    # paths = []
    # max_len = 0
    # for k in range(5):
    #     rrt = Rrt(x_start, x_goal, STEP_LEN, 0.05, 5000)
    #     path = rrt.planning()
    #
    #     if path:
    #         print("(k, len)=({}, {})".format(k, len(path)))
    #         max_len = max(max_len, len(path))
    #         paths.append(path)
    #         rrt.plotting.animation(rrt.vertex, path, "RRT", True)
    #         # time.sleep(10)
    #     else:
    #         print("No Path Found!")


    rrt = Rrt(x_start, x_goal, STEP_LEN, 0.05, 5000)
    path = rrt.planning()
    if path:
        print("len={}".format(len(path)))
        # rpath = path[::-1]      # 使用切片创建一个反转列表
        rrt.plotting.animation(rrt.vertex, path, "RRT", True)
        rpath = []
        while not rpath:
            rpath = generate_path_with_constraint(path, x_start, x_goal, rrt)
        rrt.plotting.animation(rrt.vertex, rpath, "RRT", True)
    else:
        print("No Path Found!")

    # pa, pb = align_solution(paths, max_len, x_start, x_goal)
    # sol = pairing(pa, pb)
    #
    # with open("sol.txt", "w") as file:
    #     for item in sol:
    #         file.write(str(item) + "\n")


if __name__ == '__main__':
    main()

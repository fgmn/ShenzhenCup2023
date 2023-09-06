"""
RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import time
import random
import matplotlib.pyplot as plt
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

import env, plotting, utils

STEP_LEN = 50
TAKE_DRONE_AS_OBS = True
TURNING_CONSTRAINT = True
SIGN_POINT = (10000, 10000)
TOLERANCE_TIMES = 3

attr_repu = []

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
                    print("rrt finish in round", i)
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

    @staticmethod
    def xy(node):
        return (node.x, node.y)

    def new_state(self, node_start, node_end):
        # 获取两个点之间的距离，方向角
        dist, theta = self.get_distance_and_angle(node_start, node_end)
        # 限幅
        # dist = min(self.step_len, dist)

        dist = self.step_len    # 直接指定步长

        # 检查非完整性约束：转弯半径不小于30m
        if node_start.parent:
            if not check_radius_of_turing(self.xy(node_start.parent), self.xy(node_start), self.xy(node_end)):
                return None

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

def sample_the_circle(o, r):
    samples = []
    for k in range(360):
        x = o[0] + r * math.cos(np.deg2rad(k))
        y = o[1] + r * math.sin(np.deg2rad(k))
        if x ** 2 + y ** 2 < 500 ** 2:
            continue
        samples.append([x, y])
    return samples

def dist(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def check_radius_of_turing(a, b, c):
    # bugs but unknown!
    upper_bound = math.pi - 2 * math.acos(10 / 60)
    ab = dist(a, b)
    ac = dist(a, c)
    bc = dist(b, c)

    cos_value = (ab ** 2 + bc ** 2 - ac ** 2) / (2 * ab * bc)
    # 精度问题!!
    if cos_value > 1:
        return True
    elif cos_value < -1:
        return False

    try:
        ang = math.acos(cos_value)
    except Exception as e:
        print(f"exception: {e} cos({cos_value})!")

    ang = math.pi - ang
    return (True if ang <= upper_bound else False)

def APF_based_select(samples, x_goal, cur_pos, k_att = 100, k_rep = 1e7):
    global TAKE_DRONE_AS_OBS
    # k_att/k_rep = 1/1e5
    # APF参数：引力势能系数k_att，斥力系数作为形参传入k_rep
    # 固定k_rep，调整k_att即可
    # 计划通过打表的方式整定参数
    Q_star = 1000000

    attrs = []
    for x in samples:
        d_x_x_goal_2 = dist(x, x_goal) ** 2
        attr = 0.5 * k_att * d_x_x_goal_2
        attrs.append(attr)

    x_obs = [(0, 0)]
    if TAKE_DRONE_AS_OBS:
        x_obs.append(cur_pos)   # 将另外一架无人机也作为障碍物

    repus = []
    for x in samples:
        repu = 0
        for i in range(len(x_obs)):
            # 公式1
            recip_d_x_x_obs = 1 / dist(x, x_obs[i])
            # 公式2
            # recip_d_x_x_obs = -dist(x, x_obs[i]) ** 2
            d_x_x_goal_2 = dist(x, x_goal) ** 2
            repu += 0.5 * k_rep * ((recip_d_x_x_obs - 1 / Q_star) ** 2) * d_x_x_goal_2
        repus.append(repu)
    output = [repus[i] + attrs[i] for i in range(len(attrs))]

    enumerate_list = list(enumerate(output))
    sorted_list = sorted(enumerate_list, key=lambda x: x[1])
    top_10_indices = [index for index, value in sorted_list[:30]]
    random_choice = random.choice(top_10_indices)

    return samples[random_choice]

def draw_attr_repu_curve():
    plt.figure()
    x = list(range(len(attr_repu)))
    attr = [row[0] for row in attr_repu]
    repu = [row[1] for row in attr_repu]
    sum = [row[0] + row[1] for row in attr_repu]
    # plt.plot(x, attr, label='attr', marker='o', linestyle='-')
    plt.plot(x, attr, label='attr')
    plt.plot(x, repu, label='repu')
    plt.plot(x, sum, label='sum')
    plt.title('Attr-Repu Energy Curve')
    plt.xlabel('step')
    plt.ylabel('energy')
    plt.legend()
    plt.show()

def generate_path_with_constraint(path, x_start, x_goal, rrt, k_att = 1):
    global STEP_LEN
    global TURNING_CONSTRAINT
    rpath = []
    rpath.append(x_start)

    checker = utils.Utils()
    for i in range(1, len(path)):
        # 在圆周上均匀采样360个点，并且排除和障碍圆相交的点
        samples = sample_the_circle(rpath[i - 1], STEP_LEN)

        # 根据“不得碰面”（”转弯半径不小于30m“）约束进行
        # 筛选Begin
        samples1 = []
        filtered_samples = []
        for x in samples:
            o, d = utils.Utils.get_ray(Node(path[i]), Node(x))
            if checker.is_intersect_circle(o, d, [0, 0], 500):
                samples1.append(x)
        if TURNING_CONSTRAINT and i >= 2:
            for x in samples1:
                if check_radius_of_turing(rpath[i - 2], rpath[i - 1], x):
                    filtered_samples.append(x)
        else:
            filtered_samples = samples1
        # 筛选End

        if not filtered_samples:
            # print("exit at {}".format(i))
            rpath.append(SIGN_POINT)
            return rpath
            # return None

        # 调用人工势能场APF进行选择
        selected_sample = APF_based_select(filtered_samples, x_goal, path[i], k_att)
        rpath.append(selected_sample)

        # 终止条件1：到达离目标点不到一个步长的位置
        if dist(selected_sample, x_goal) <= STEP_LEN:
            rpath.append(x_goal)
            return rpath

    # 终止条件2：无人机B抵达，无人机A只需向站点前进即可 同时保证转弯约束
    while dist(rpath[-1], x_goal) > STEP_LEN:
        samples = sample_the_circle(rpath[-1], STEP_LEN)
        filtered_samples = []
        for x in samples:
            if check_radius_of_turing(rpath[-2], rpath[-1], x):
                filtered_samples.append(x)
        selected_sample = filtered_samples[int(np.argmin([dist(x, x_goal) for x in filtered_samples]))]
        rpath.append(selected_sample)
    rpath.append(x_goal)
    return rpath

def calc_path_cost(path):
    cost = 0
    for i in range(1, len(path)):
        cost += dist(path[i - 1], path[i])
    return cost

'''
@Todo & summary
1.生成多组路径根据“不得碰面”约束进行匹配，得到可行解
    最终生成路径采样点数目不同 → 将目标点拷贝多份进行对齐即可
    采样点之间距离并不是严格一个step → 直接指定步长
result：没有匹配的解！
无人机A需要“迂回”“原地绕飞”策略，rrt对目标点有趋向性，两者冲突

2.在rrt生成路径的基础上，根据“不得碰面”“转弯半径不小于30m”约束生成一条最优的路径A→B，组合成可行解
①在下一步目标圆周上均匀采样360个点，根据2个约束进行筛选得到一个可行集合
②贪心地选取下一步：距离目标点最近 or 轮盘赌 → 引入人工势场法进行科学决策

problem list:
①APF+两个约束很难找到解 → 先忽视转弯半径约束，最后平滑处理 → 加入随机扰动跳出局部最小
②两架uav同时走障碍圆的一侧 → 限制采样区域
③修改APF公式

找到比较好的势能系数比区间，取 多条路径 和 多次随机结果 取最优
'''

def draw_att_cost_curve(k_atts, costs):
    plt.figure()
    plt.plot(k_atts, costs, label='cost')
    plt.title('Att-cost Curve')
    plt.xlabel('k')
    plt.ylabel('cost')
    plt.legend()
    plt.show()

def main():
    x_start = (-1000, 0)  # Starting node
    x_goal = (3500, 0)  # Goal node

    rrt = Rrt(x_start, x_goal, STEP_LEN, 0.05, 100000)
    path = rrt.planning()
    if path:
        print("rrt path's len={}".format(len(path)))
        # rrt.plotting.animation(rrt.vertex, path, "RRT", True)
        ############# rrt约束改进路径生成算法 #############
        costs = []
        k_atts = list(range(1, 40))
        for k_att in k_atts:
            rpath = []
            rpath.append(SIGN_POINT)
            cnt = 0
            while rpath[-1] == SIGN_POINT and cnt <= TOLERANCE_TIMES:
                cnt += 1
                rpath = generate_path_with_constraint(path, x_start, x_goal, rrt, k_att)
            costs.append(calc_path_cost(rpath))
            print("test", k_att, "cost={}".format(costs[-1]))
            if rpath[-1] != SIGN_POINT:
                rrt.plotting.animation([], rpath, path, "RRT", True)
        draw_att_cost_curve(k_atts, costs)
        # rpath = generate_path_with_constraint(path, x_start, x_goal, rrt)
        ############# rrt约束改进路径生成算法 #############
        # draw_attr_repu_curve()
    else:
        print("No Path Found!")


if __name__ == '__main__':
    main()

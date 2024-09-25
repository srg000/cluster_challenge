#!/usr/bin/env python

import argparse
import threading
import time
import math
from collections import deque

from swarmae.SwarmAEClient import SwarmAEClient

# 线程锁
lock = threading.Lock()
target_mission = [-1, -1, -1, -1, -1]
team_flag = 0


class Steer_PID_controller(object):
    def __init__(self, node_car, k_p=0.010, k_i=0.0008, k_d=0.0001, d_t=0.01):  # 手动调节参数k_p、k_i、k_d
        """
        采用PID进行纵向速度控制，包括比例项P、积分项I、微分项D
        ego_vehicle: 是carla中的主车
        k_p: 比例项系数
        k_i: 积分项系数
        k_d: 微分项系数
        d_t: 控制间隔
        """

        self.vehicle = node_car  # ego_vehicle是carla中的主车
        self.k_p = k_p  # 比例系数
        self.k_i = k_i  # 积分系数
        self.k_d = k_d  # 微分系数
        self.d_t = d_t  # 控制间隔
        self.target_angle = None  # 目标速度
        self.error_buffer = deque(maxlen=60)  # 设置一个误差缓存区，用于积分项和差分项的计算
        self.error_threshold = 1  # 设定一个阈值

    def PID_control(self, target_angle):

        self.target_angle = target_angle  # 目标
        angle_n = self.vehicle.get_attitude()[0]
        error = self.target_angle - angle_n  # 误差
        self.error_buffer.append(error)  # 将新的角度误差放入缓存区，如果缓存区满了，最左边的溢出，整体数据左移一位，新的数据加在最右边
        # 日志
        if len(self.error_buffer) >= 2:
            integral_error = sum(self.error_buffer) * self.d_t  # 积分误差，为了解决稳态误差
            derivative_error = (self.error_buffer[-1] - self.error_buffer[-2]) / self.d_t  # 微分误差，为了缓解超调
        else:
            integral_error = 0.0
            derivative_error = 0.0
        if abs(error) > self.error_threshold:  # 一旦出现误差大于阈值的情况，只有比例项发挥作用
            integral_error = 0.0
            self.error_buffer.clear()

        steer = self.k_p * error + self.k_i * integral_error + self.k_d * derivative_error
        # print(" steer :" + str(steer) + "error: " + str(error) + "k_i * integral_error " + str(self.k_i * integral_error) + "k_d * derivative_error: " + str(self.k_d * derivative_error))
        return steer


class Parking_PID_controller(object):  # 纵向控制
    def __init__(self, node_car, k_p=0.165, k_i=1.10, k_d=0.009, d_t=0.01):  # 手动调节参数k_p、k_i、k_d
        """
        采用PID进行纵向速度控制，包括比例项P、积分项I、微分项D
        ego_vehicle: 是carla中的主车
        k_p: 比例项系数
        k_i: 积分项系数
        k_d: 微分项系数
        d_t: 控制间隔
        """

        self.vehicle = node_car  # ego_vehicle是carla中的主车
        self.k_p = k_p  # 比例系数
        self.k_i = k_i  # 积分系数
        self.k_d = k_d  # 微分系数
        self.d_t = d_t  # 控制间隔
        self.target_position = None  # 目标速度
        self.error_buffer = deque(maxlen=60)  # 设置一个误差缓存区，用于积分项和差分项的计算
        self.error_threshold = 8  # 设定一个阈值

    def PID_control(self, target_position):
        """
        函数：计算PID控制的输出
        return: u
        """
        location = self.vehicle.get_location()  # self.vehicle.get_velocity()

        self.target_position = target_position  # 目标速度D

        error = math.sqrt(
            (self.target_position[0] - location[0]) ** 2 + (self.target_position[1] - location[1]) ** 2)  # 距离差

        # if (self.target_position[0] - location[0]) < 0:
        #     error = 0 - error

        self.error_buffer.append(error)  # 将新的速度误差放入缓存区，如果缓存区满了，最左边的溢出，整体数据左移一位，新的数据加在最右边

        if len(self.error_buffer) >= 3:
            integral_error = sum(self.error_buffer) * self.d_t  # 积分误差，为了解决稳态误差
            derivative_error = (self.error_buffer[-1] - self.error_buffer[-2]) / self.d_t  # 微分误差，为了缓解超调
        else:
            integral_error = 0.0
            derivative_error = 0.0
        if abs(error) > self.error_threshold:  # 一旦出现误差大于阈值的情况，只有比例项发挥作用
            integral_error = 0.0
            self.error_buffer.clear()

        pid_speed = self.k_p * error + self.k_i * integral_error + self.k_d * derivative_error
        # print(" pid_speed :" + str(pid_speed) + "error: " + str(error) + "k_i * integral_error " + str(self.k_i * integral_error) + "k_d * derivative_error: " + str(self.k_d * derivative_error))
        return pid_speed, error


class Longitudinal_PID_controller(object):  # 纵向控制
    def __init__(self, node_car, k_p=0.190, k_i=7.0, k_d=0.001, d_t=0.01):  # 手动调节参数k_p、k_i、k_d
        """
        采用PID进行纵向速度控制，包括比例项P、积分项I、微分项D
        ego_vehicle: 是carla中的主车
        k_p: 比例项系数
        k_i: 积分项系数
        k_d: 微分项系数
        d_t: 控制间隔
        """

        self.vehicle = node_car  # ego_vehicle是carla中的主车
        self.k_p = k_p  # 比例系数
        self.k_i = k_i  # 积分系数
        self.k_d = k_d  # 微分系数
        self.d_t = d_t  # 控制间隔
        self.target_speed = None  # 目标速度
        self.error_buffer = deque(maxlen=60)  # 设置一个误差缓存区，用于积分项和差分项的计算
        self.error_threshold = 3  # 设定一个阈值

    def PID_control(self, target_speed):
        """
        函数：计算PID控制的输出
        return: u
        """
        v = self.vehicle.get_velocity()  # self.vehicle.get_velocity()
        v_now = v[3]
        global team_flag
        if (v[1] > 0) and team_flag==0:  # 反向速度解析
            v_now = 0 - v[3]
        if (v[1] < 0) and team_flag==1:  # 反向速度解析
            v_now = 0 - v[3]
        self.target_speed = target_speed  # 目标速度D

        error = self.target_speed - v_now  # 速度误差

        self.error_buffer.append(error)  # 将新的速度误差放入缓存区，如果缓存区满了，最左边的溢出，整体数据左移一位，新的数据加在最右边

        if len(self.error_buffer) >= 2:
            integral_error = sum(self.error_buffer) * self.d_t  # 积分误差，为了解决稳态误差
            derivative_error = (self.error_buffer[-1] - self.error_buffer[-2]) / self.d_t  # 微分误差，为了缓解超调
        else:
            integral_error = 0.0
            derivative_error = 0.0
        if abs(error) > self.error_threshold:  # 一旦出现误差大于阈值的情况，只有比例项发挥作用
            integral_error = 0.0
            self.error_buffer.clear()

        u = self.k_p * error + self.k_i * integral_error + self.k_d * derivative_error
        # print("error: "+str(error)+"k_i * integral_error "+str( self.k_i * integral_error)+"k_d * derivative_error: "+str( self.k_d*derivative_error))
        # print("UUUUUU:"+str(u)+"   now_speed:  "+str(v_now)+"  target_speed:  "+str(target_speed)+"  error: "+str(error))
        return u, v_now


class Vehicle_control(object):
    def __init__(self, ego_vehicle, pathway, attack_target=[], weapon=None):
        """
        初始化车辆的油门范围、转角范围、刹车范围
        """

        self.vehicle = ego_vehicle  # ego_vehicle是carla中的主车
        self.max_throttle = 1  # 最大油门 = 1，最小油门 = 0
        self.min_throttle = -1  # 最大刹车 = 1，最小刹车 = 0
        self.max_steer = 1  # 最大转角 = 1
        self.min_steer = -1  # 最小转角 = 1
        self.Path = pathway  # 目标路径
        self.Path_index = 0  # 路径起始点
        self.Attack_target = attack_target
        self.Attack_index = 0
        self.Weapon = weapon
        self.Lateral_control = Steer_PID_controller(ego_vehicle)
        self.Longitudinal_control = Longitudinal_PID_controller(ego_vehicle)
        self.Parking_control = Parking_PID_controller(ego_vehicle)

    def angle_mode_count(self, start, end):
        # 计算目标方向的角度     1 为前进  -1 为倒车
        global team_flag
        target_angle_rad = math.atan2(end[1] - start[1], end[0] - start[0])
        target_angle_deg = math.degrees(target_angle_rad)


        if team_flag < 1:
            if 0 < target_angle_deg and target_angle_deg < 180:
                return abs(target_angle_deg)-180, -1
            else:
                return target_angle_deg, 1
        else:
            if 0 < target_angle_deg and target_angle_deg < 180:
                return target_angle_deg, 1
            else:
                return 180 - abs(target_angle_deg), -1
    def avoid_barrier(self, start, end, move_distance=8, avoid_distance=10):
        flag = True
        barrier_x_y = []
        hostile_list = self.vehicle.detect_hostile_list()
        len_list = len(hostile_list)
        if (len_list == 0):
            return False, end
        elif (len_list == 2):
            barrier_x_y_1 = (hostile_list[0][1].location.x, hostile_list[0][1].location.y)
            barrier_x_y_2 = (hostile_list[1][1].location.x, hostile_list[1][1].location.y)
            if barrier_x_y_1[0] - start[0] < barrier_x_y_2[0] - start[0]:
                barrier_x_y = barrier_x_y_1
            else:
                barrier_x_y = barrier_x_y_2
        elif (len_list == 1):
            barrier_x_y = (hostile_list[0][1].location.x, hostile_list[0][1].location.y)
        x0, y0 = barrier_x_y
        x1, y1 = start
        x2, y2, t1, t2 = end

        # 计算线段的方向向量
        dx = x2 - x1
        dy = y2 - y1
        length = (dx ** 2 + dy ** 2) ** 0.5

        # 处理为零长度的情况
        if length == 0:
            return "Line segment length is zero"

        # 计算单位向量
        unit_dx = dx / length
        unit_dy = dy / length

        # 计算法向量
        perp_dx = -unit_dy
        perp_dy = unit_dx

        # 确定矩形的四个角
        rect_corners = [
            (x1 + perp_dx * (avoid_distance / 2), y1 + perp_dy * (avoid_distance / 2)),
            (x1 - perp_dx * (avoid_distance / 2), y1 - perp_dy * (avoid_distance / 2)),
            (x2 - perp_dx * (avoid_distance / 2), y2 - perp_dy * (avoid_distance / 2)),
            (x2 + perp_dx * (avoid_distance / 2), y2 + perp_dy * (avoid_distance / 2))
        ]

        def is_point_in_polygon(point, polygon):
            x, y = point
            inside = False
            n = len(polygon)
            p1x, p1y = polygon[0]

            for i in range(n + 1):
                p2x, p2y = polygon[i % n]
                if y > min(p1y, p2y):
                    if y <= max(p1y, p2y):
                        if x <= max(p1x, p2x):
                            if p1y != p2y:
                                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside
                p1x, p1y = p2x, p2y

            return inside

        if is_point_in_polygon(barrier_x_y, rect_corners):
            # 计算向量 AB 和向量 AP 的叉积
            cross_product = (x2 - x1) * (y0 - y1) - (y2 - y1) * (x0 - x1)
            # 计算 A, B, C 的值
            A_coeff = y2 - y1
            B_coeff = -(x2 - x1)
            norm = math.sqrt(A_coeff ** 2 + B_coeff ** 2)
            unit_perpendicular_vector = (-B_coeff / norm, A_coeff / norm)
            # 判断位置并计算新点
            if cross_product > 0:
                position = 'left'
                dx = -unit_perpendicular_vector[1] * move_distance
                dy = unit_perpendicular_vector[0] * move_distance
            elif cross_product <= 0:
                position = 'right'
                dx = unit_perpendicular_vector[1] * move_distance
                dy = -unit_perpendicular_vector[0] * move_distance

            new_point = (x0 + dx, y0 + dy)
            print(
                "  start: " + str(start) + "  end: " + str(end) + "  position: " + str(position) + "  new_point" + str(
                    new_point) + "  barrier_x_y" + str(barrier_x_y))
            return flag, new_point
        else:
            new_point = barrier_x_y
            flag = False
        return flag, new_point

    def run_step(self):
        """
        函数：计算横向控制与纵向控制的流程
        """
        global target_mission

        location_node = self.vehicle.get_location()

        start = (location_node[0], location_node[1])

        print(self.Path)
        print(self.Path_index)


        target_point = self.Path[self.Path_index]
        attitude_node = self.vehicle.get_attitude()

        target_angle, driver_mode = self.angle_mode_count(start, target_point)

        # 巡航攻击
        len_Attack = len(self.Attack_target)
        if len_Attack > self.Attack_index:
            attack_1 = self.Attack_target[self.Attack_index]
            if attack_1[0] - start[0] < 100:  # 进入瞄准距离
                attack_angle, need_attack = self.angle_mode_count(start, attack_1)
                if need_attack > 0:
                    self.Weapon.set_weapon_angle(attack_angle, 0 - attitude_node[1], time.time())  # 瞄准
                    if attack_1[0] - start[0] < 50 and abs(attitude_node[1]) < 1 and abs(attack_angle) > 2:  # 进入攻击距离
                        self.vehicle.gun_fire(1)
                        self.Attack_index += 1

        if target_point[3] == 'D':
            if abs(target_point[0] - (start[0])) < 7:
                self.Path_index += 1
                if len(self.Path) < self.Path_index:
                    self.Path_index -= 1
            target_speed = target_point[2] * driver_mode  # 计算需要控制的最大目标速度

        flag_avoid, new_point = self.avoid_barrier(start, target_point)

        if flag_avoid:
            if abs(new_point[0] - target_point[0]) < 20:  # 避让点和目标点很近则不需要再前往该目标点
                self.Path[self.Path_index] = [new_point[0], new_point[1], target_point[2], target_point[3]]
                target_point = self.Path[self.Path_index]
                print(f"新的点：+{target_point}")
                print(self.Path)
            else:
                self.Path.insert(self.Path_index, [new_point[0], new_point[1], target_point[2], 'D'])  # 避让点和目标点很远 则为过点
                target_point = self.Path[self.Path_index]
                print(f"新的途径点：+{target_point}")
                print(self.Path)

        # elif target_point[3] == 'S':
        #     pid_speed, error = self.Parking_control.PID_control(target_point)
        #     target_speed = min(target_point[2], abs(pid_speed)) * driver_mode
        #     if error < 2 and abs(attitude_node[1]) < 0.7:
        #         self.vehicle.gun_fire(1)
        #         time.sleep(0.1)
        #         self.Path_index += 1

        # 停车车速控制
        elif target_point[3] == 'P':
            pid_speed, error = self.Parking_control.PID_control(target_point)
            target_speed = min(target_point[2], abs(pid_speed)) * driver_mode

            node_id = self.vehicle.get_node_info()[1]
            global team_flag
            if team_flag == 1:
                node_id -= 5
            if target_mission[int(node_id) - 1] == 0:  # 收到放行指令，跳过停车点，等待下次放行任务
                self.Path_index += 1
                if len(self.Path) < self.Path_index:
                    self.Path_index -= 1
                target_mission[int(node_id) - 1] = -1

            elif target_mission[int(node_id) - 1] < 0 and error < 1.5:  # 初始为-1，当自己任务完成后改为1
                target_mission[int(node_id) - 1] = 1

        steer = self.Lateral_control.PID_control(target_angle)  # 返回横向控制的转角
        throttle, v_now = self.Longitudinal_control.PID_control(target_speed)  # 返回纵向控制的油门

        # 控制限定范围
        if v_now < 0:  # 速度向后时，反向控制方向
            steer = 0 - steer

        if steer >= 0 and throttle >= 0:
            self.vehicle.apply_control(min(self.max_throttle, throttle), min(self.max_steer, steer), 0, 0, 1)
        if steer >= 0 and throttle <= 0:
            self.vehicle.apply_control(max(self.min_throttle, throttle), min(self.max_steer, steer), 0, 0, 1)
        if steer <= 0 and throttle <= 0:
            self.vehicle.apply_control(max(self.min_throttle, throttle), max(self.min_steer, steer), 0, 0, 1)
        if steer <= 0 and throttle >= 0:
            self.vehicle.apply_control(min(self.max_throttle, throttle), max(self.min_steer, steer), 0, 0, 1)
        time.sleep(0.1)


def sdk_get_map(client, entity_name):
    # sdk读取环境信息，需要 1.0.1 sdk 才支持
    timestamp, world, code = client.get_world()
    objects = world.get_environment_objects()

    return list(filter(lambda x: (x.name.startswith(entity_name)), objects))


def create_path(client, vehicles):
    lupais_SM = sdk_get_map(client, 'lupai')  # 所有路牌信息
    BU_B4_SM = sdk_get_map(client, 'BU_B4_SM')  # 道路的集装箱
    BU_B5_SM = sdk_get_map(client, 'BU_B5_SM')  # 道路的集装箱

    node_1 = vehicles[0].get_location()
    node_2 = vehicles[1].get_location()
    node_3 = vehicles[2].get_location()
    node_4 = vehicles[3].get_location()
    node_5 = vehicles[4].get_location()

    # # 获取小车位置信息
    # x, y, z, frame_timestamp = vehicles[0].get_location()
    #
    # # 计算小车与每个路牌的距离
    # distances = [math.sqrt((x - lupai.transform.location.x) ** 2 + (y - lupai.transform.location.y) ** 2) for
    #              lupai in lupais_SM]
    #
    # # 找到最近的路牌
    # min_distance = min(distances)
    # min_index = distances.index(min_distance)
    # nearest_lupai = lupais_SM[min_index]  # 获取最近的障碍物信息
    # lupai_location = nearest_lupai.transform.location
    # print('lupai_location', lupai_location)
    #
    # lupai_x = lupai_location.x + 8
    # lupai_y = lupai_location.y
    #
    # step = 5
    # gap = 1.4

    # target_path_01 = [[lupai_x, lupai_y + 40, 9, 'D'], [lupai_x, lupai_y + 10, 7, 'D'], [lupai_x, lupai_y - 10, 7, 'P'],
    #                   [lupai_x, lupai_y - 100, 7, 'D'], [lupai_x, lupai_y - 200, 7, 'D']]
    #
    # target_path_02 = [[lupai_x, lupai_y + 40, 9 - gap, 'D'], [lupai_x, lupai_y + 10, 7, 'D'], [lupai_x, lupai_y - 10 + step, 7, 'P'],
    #                   [lupai_x, lupai_y - 100, 7, 'D'], [lupai_x, lupai_y - 200, 7, 'D']]
    #
    # target_path_03 = [[lupai_x, lupai_y + 40, 9 - gap * 2, 'D'], [lupai_x, lupai_y + 10, 7, 'D'], [lupai_x, lupai_y - 10 + step * 2, 7, 'P'],
    #                   [lupai_x, lupai_y - 100 + step * 2, 7, 'D'], [lupai_x, lupai_y - 200, 7, 'D']]
    #
    # target_path_04 = [[lupai_x, lupai_y + 40, 9 - gap * 3, 'D'], [lupai_x, lupai_y + 10, 7, 'D'], [lupai_x, lupai_y - 10 + step * 3, 7, 'P'],
    #                   [lupai_x, lupai_y - 100, 7, 'D'], [lupai_x, lupai_y - 200, 7, 'D']]
    #
    # target_path_05 = [[lupai_x, lupai_y + 40, 9 - gap * 4, 'D'], [lupai_x, lupai_y + 10, 7, 'D'], [lupai_x, lupai_y - 10 + step * 4, 7, 'P'],
    #                   [lupai_x, lupai_y - 100, 7, 'D'], [lupai_x, lupai_y - 200, 7, 'D']]

    # 红方
    # target_path_01 = [[node_1[0] + 20, node_1[1] + 50, 10, 'D'], [node_1[0] + 40, node_1[1] + 100, 10, 'P']]
    #
    # target_path_02 = [[node_2[0], node_2[1] - 50, 10, 'D'], [node_2[0], node_2[1] - 200, 10, 'P']]
    #
    # target_path_03 = [[node_3[0], node_3[1] - 50, 10, 'D'], [node_3[0], node_3[1] - 200, 10, 'P']]
    #
    # target_path_04 = [[node_4[0], node_4[1] - 50, 10, 'D'], [node_4[0], node_4[1] - 200, 10, 'P']]
    #
    # target_path_05 = [[node_5[0], node_5[1] - 50, 10, 'D'], [node_5[0], node_5[1] - 200, 10, 'P']]


    # 蓝方
    target_path_01 = [[node_1[0] + 20, node_1[1] - 50, 10, 'D'], [node_1[0] + 30, node_1[1] - 100, 10, 'P']]

    target_path_02 = [[node_2[0], node_2[1] + 50, 10, 'D'], [node_2[0], node_2[1] + 100, 10, 'P']]

    target_path_03 = [[node_3[0], node_3[1] + 50, 10, 'D'], [node_3[0], node_3[1] + 100, 10, 'P']]

    target_path_04 = [[node_4[0], node_4[1] + 50, 10, 'D'], [node_4[0], node_4[1] + 100, 10, 'P']]

    target_path_05 = [[node_5[0], node_5[1] + 50, 10, 'D'], [node_5[0], node_5[1] + 100, 10, 'P']]

    target_paths = [target_path_01, target_path_02, target_path_03, target_path_04, target_path_05]
    print(target_paths)

    return target_paths


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__
    )
    argparser.add_argument(
        '-i', '--address',
        default='127.0.0.1'
    )
    argparser.add_argument(
        '-p', '--port',
        type=int,
        default=2000
    )
    argparser.add_argument(
        '-n', '--number',
        type=int,
        default=10
    )
    argparser.add_argument(
        '-s', '--subject',
        type=int,
        default=1
    )
    argparser.add_argument(
        '-t', '--team',
        type=int,
        default=0
    )
    args = argparser.parse_args()
    try:
        client = SwarmAEClient(ue_ip=args.address.strip(), ue_port=args.port)
        timestamp, world, code = client.get_world()

        # 无人车
        vehicles = []
        number = 5
        for i in range(number):
            num = i + 1
            frame_timestamp, sw_node, sw_code = client.register_node(
                '四轮车', str(num), num,
                int(round(time.time() * 1000))
            )
            if sw_code == 200:
                vehicles.append(sw_node)
                location = sw_node.get_location()
                print(f'{sw_node.get_node_id()} {location[0]} {location[1]} {location[1]}')

        # 无人机
        uavs = []
        number = 10
        for i in range(number):
            num = i + 5 + 1
            frame_timestamp, sw_node, sw_code = client.register_node(
                '四旋翼', str(num), num,
                int(round(time.time() * 1000))
            )
            if sw_code == 200:
                uavs.append(sw_node)
                location = sw_node.get_location()
                print(f'{sw_node.get_node_id()} {location[0]} {location[1]} {location[1]}')

        node_name, node_no, _, node_type, node_model, color = uavs[0].get_node_info()
        if node_no not in [11, 21]:
            print('请设置运行参数 -t ， 0 表示红队， 1 表示蓝队，示例： -t=0')
            return

        global team_flag
        team_flag = args.team
        target_paths = create_path(client, vehicles)

        # # # 更改小车顺序
        # lupai_x = lupai_location.x
        # lupai_y = lupai_location.y
        #
        # # def distance_from_x(vehicle):
        # #     return abs(lupai_x - vehicle.get_location()[0])
        #
        # def distance_from_x(vehicle):
        #     x = vehicle.get_location()[0]
        #     y = vehicle.get_location()[1]
        #     return math.sqrt((x - lupai_x) ** 2 + (y - lupai_y) ** 2)
        #
        # sorted_vehicles = sorted(vehicles, key=distance_from_x)
        # for vehicle in sorted_vehicles:
        #     print("Vehicle {}".format(vehicle.get_node_id()))

        # # 车辆控制
        timestamp, world, code = client.get_world()
        control_object_01 = Vehicle_control(vehicles[0], target_paths[0])
        control_object_02 = Vehicle_control(vehicles[1], target_paths[1])
        control_object_03 = Vehicle_control(vehicles[2], target_paths[2])
        control_object_04 = Vehicle_control(vehicles[3], target_paths[3])
        control_object_05 = Vehicle_control(vehicles[4], target_paths[4])

        def vcontrol_1():
            while True:
                control_object_01.run_step()
                time.sleep(0.01)

        def vcontrol_2():
            while True:
                control_object_02.run_step()
                time.sleep(0.01)

        def vcontrol_3():
            while True:
                control_object_03.run_step()
                time.sleep(0.01)

        def vcontrol_4():
            while True:
                control_object_04.run_step()
                time.sleep(0.01)

        def vcontrol_5():
            while True:
                control_object_05.run_step()
                time.sleep(0.01)

        def all_control():
            count = 0
            while True:
                time.sleep(0.01)
                global target_mission
                if sum(target_mission) == 5 and count == 0:
                    time.sleep(0.1)
                    target_mission = [0, 0, 0, 0, 0]
                    count += 1

                if sum(target_mission) == 5 and count == 1:
                    time.sleep(0.1)
                    target_mission = [0, 0, 0, 0, 0]
                    count += 1

        t1 = threading.Thread(target=vcontrol_1)
        t2 = threading.Thread(target=vcontrol_2)
        t3 = threading.Thread(target=vcontrol_3)
        t4 = threading.Thread(target=vcontrol_4)
        t5 = threading.Thread(target=vcontrol_5)
        t6 = threading.Thread(target=all_control)

        t1.start()
        # t2.start()
        # t3.start()
        # t4.start()
        # t5.start()
        t6.start()

    finally:
        print('main')


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('done.')

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


class Steer_PID_controller(object):
    def __init__(self, node_car, k_p=0.010, k_i=0.0008, k_d=0.001, d_t=0.05):  # 手动调节参数k_p、k_i、k_d
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
        angle_n = self.vehicle.get_attitude()[0]
        # print("target_angle:"+str(target_angle)+"    n_angle:"+str(angle_n))   #日志
        self.target_angle = target_angle  # 目标

        error = self.target_angle - angle_n  # 误差
        self.error_buffer.append(error)  # 将新的角度误差放入缓存区，如果缓存区满了，最左边的溢出，整体数据左移一位，新的数据加在最右边
        print("now_angle:  " + str(angle_n) + "  target_angle:  " + str(target_angle) + "  error: " + str(error))
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
        print("steer:" + str(steer))
        return steer


class Parking_PID_controller(object):  # 纵向控制
    def __init__(self, node_car, k_p=0.165, k_i=5.10, k_d=10.186, d_t=0.01):  # 手动调节参数k_p、k_i、k_d
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
        self.error_buffer = deque(maxlen=90)  # 设置一个误差缓存区，用于积分项和差分项的计算
        self.error_threshold = 1  # 设定一个阈值

    def PID_control(self, target_position):
        """
        函数：计算PID控制的输出
        return: u
        """
        location = self.vehicle.get_location()  # self.vehicle.get_velocity()

        self.target_position = target_position  # 目标速度D

        error = math.sqrt(
            (self.target_position[0] - location[0]) ** 2 + (self.target_position[1] - location[1]) ** 2
        )  # 距离差

        print("error: " + str(error))

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

        pid_speed = self.k_p * error + self.k_i * integral_error + self.k_d * derivative_error

        print(" pid_speed :" + str(pid_speed))
        return pid_speed, error


class Longitudinal_PID_controller(object):  # 纵向控制
    def __init__(self, node_car, k_p=0.190, k_i=1.04, k_d=0.007, d_t=0.01):  # 手动调节参数k_p、k_i、k_d
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
        self.error_threshold = 1  # 设定一个阈值

    def PID_control(self, target_speed):
        """
        函数：计算PID控制的输出
        return: u
        """
        v = self.vehicle.get_velocity()  # self.vehicle.get_velocity()
        v_now = v[3]
        if (v[0] < 0):  # 反向速度解析
            v_now = 0 - v[3]

        self.target_speed = target_speed  # 目标速度D

        error = self.target_speed - v_now  # 速度误差

        print("now_speed:  " + str(v_now) + "  target_speed:  " + str(target_speed) + "  error: " + str(error))

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
        print("UUUUUU:" + str(u))
        return u, v_now


class Vehicle_control(object):
    def __init__(self, ego_vehicle, pathway, world):
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
        self.world = world
        self.flag = 0
        self.Lateral_control = Steer_PID_controller(ego_vehicle)
        self.Longitudinal_control = Longitudinal_PID_controller(ego_vehicle)
        self.Parking_control = Parking_PID_controller(ego_vehicle)

    def angle_mode_count(self, end):
        # 计算目标方向的角度     1 为前进  -1 为倒车
        start = (self.vehicle.get_location()[0], self.vehicle.get_location()[1])
        target_angle_rad = math.atan2(end[1] - start[1], end[0] - start[0])
        target_angle_deg = math.degrees(target_angle_rad)
        print(target_angle_deg)
        if -90 < target_angle_deg and target_angle_deg < 90:
            return target_angle_deg, 1
        elif target_angle_deg <= -90:
            return 180 - abs(target_angle_deg), -1
        elif target_angle_deg >= 90:
            return target_angle_deg - 180, -1

    def run_step(self):
        """
        函数：计算横向控制与纵向控制的流程
        """
        # # 如果路径已经走完，则重新生成新的路径
        # if self.Path_index >= len(self.Path) and self.flag == 0:
        #     new_line = avoid_obstacles(self.vehicle, self.world)
        #     self.Path += new_line
        #     print('new line', self.Path)
        #     self.flag += 1

        target_point = self.Path[self.Path_index]
        target_angle, driver_mode = self.angle_mode_count(target_point)

        if target_point[3] == 'D':
            if abs(target_point[0] - (self.vehicle.get_location()[0])) < 6:
                self.Path_index += 1
            target_speed = target_point[2] * driver_mode  # 计算需要控制的最大目标速度

        # 停车车速控制
        if target_point[3] == 'P':
            pid_speed, error = self.Parking_control.PID_control(target_point)
            target_speed = min(target_point[2], pid_speed) * driver_mode
            global target_mission
            if target_mission[int(self.vehicle.get_node_info()[1]) - 1] == 0:  # 跳过停车点，等待下次放行任务
                self.Path_index += 1
                lock.acquire()
                target_mission[int(self.vehicle.get_node_info()[1] - 1)] = -1
                lock.release()

            elif target_mission[
                int(self.vehicle.get_node_info()[1] - 1)] == -1 and error < 0.83:  # 初始为-1，自己完成后改为1，放行改为0
                lock.acquire()
                target_mission[int(self.vehicle.get_node_info()[1] - 1)] = 1
                lock.release()

        steer = self.Lateral_control.PID_control(target_angle)  # 返回横向控制的转角
        throttle, v_now = self.Longitudinal_control.PID_control(target_speed)  # 返回纵向控制的油门

        # 横向控制限定范围
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

    # 获取小车位置信息
    x, y, z, frame_timestamp = vehicles[0].get_location()

    # 计算小车与每个路牌的距离
    distances = [math.sqrt((x - lupai.transform.location.x) ** 2 + (y - lupai.transform.location.y) ** 2) for
                 lupai in lupais_SM]

    # 找到最近的路牌
    min_distance = min(distances)
    min_index = distances.index(min_distance)
    nearest_lupai = lupais_SM[min_index]  # 获取最近的障碍物信息
    lupai_location = nearest_lupai.transform.location
    print('lupai_location', lupai_location)

    lupai_x = lupai_location.x + 8
    lupai_y = lupai_location.y

    step = 5
    gap = 1.4

    target_path_01 = [[lupai_x, lupai_y + 40, 9, 'D'], [lupai_x, lupai_y + 10, 7, 'D'], [lupai_x, lupai_y - 10, 7, 'P'],
                      [lupai_x, lupai_y - 50, 10, 'D'], [lupai_x, lupai_y - 100, 10, 'D']]

    target_path_02 = [[lupai_x, lupai_y + 40, 9 - gap, 'D'], [lupai_x, lupai_y + 10, 7, 'D'], [lupai_x, lupai_y - 10 + step, 7, 'P'],
                      [lupai_x, lupai_y - 50, 9, 'D'], [lupai_x, lupai_y - 100, 10, 'D']]

    target_path_03 = [[lupai_x, lupai_y + 40, 9 - gap * 2, 'D'], [lupai_x, lupai_y + 10, 7, 'D'], [lupai_x, lupai_y - 10 + step * 2, 7, 'P'],
                      [lupai_x, lupai_y - 50, 8, 'D'],  [lupai_x, lupai_y - 100, 10, 'D']]

    target_path_04 = [[lupai_x, lupai_y + 40, 9 - gap * 3, 'D'], [lupai_x, lupai_y + 10, 7, 'D'], [lupai_x, lupai_y - 10 + step * 3, 7, 'P'],
                      [lupai_x, lupai_y - 50, 7, 'D'], [lupai_x, lupai_y - 100, 10, 'D']]

    target_path_05 = [[lupai_x, lupai_y + 40, 9 - gap * 4, 'D'], [lupai_x, lupai_y + 10, 7, 'D'], [lupai_x, lupai_y - 10 + step * 4, 7, 'P'],
                      [lupai_x, lupai_y - 50, 6, 'D'], [lupai_x, lupai_y - 100, 10, 'D']]

    target_paths = [target_path_01, target_path_02, target_path_03, target_path_04, target_path_05]
    print(target_paths)

    return target_paths, lupai_location


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

        target_paths, lupai_location = create_path(client, vehicles)

        # # 更改小车顺序
        lupai_x = lupai_location.x
        lupai_y = lupai_location.y

        # def distance_from_x(vehicle):
        #     return abs(lupai_x - vehicle.get_location()[0])

        def distance_from_x(vehicle):
            x = vehicle.get_location()[0]
            y = vehicle.get_location()[1]
            return math.sqrt((x - lupai_x) ** 2 + (y - lupai_y) ** 2)

        sorted_vehicles = sorted(vehicles, key=distance_from_x)
        for vehicle in sorted_vehicles:
            print("Vehicle {}".format(vehicle.get_node_id()))

        # # 车辆控制
        timestamp, world, code = client.get_world()
        control_object_01 = Vehicle_control(sorted_vehicles[0], target_paths[0], world)
        control_object_02 = Vehicle_control(sorted_vehicles[1], target_paths[1], world)
        control_object_03 = Vehicle_control(sorted_vehicles[2], target_paths[2], world)
        control_object_04 = Vehicle_control(sorted_vehicles[3], target_paths[3], world)
        control_object_05 = Vehicle_control(sorted_vehicles[4], target_paths[4], world)

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
                    time.sleep(1)

                    lock.acquire()
                    target_mission[0] = 0
                    lock.release()
                    time.sleep(0.01)

                    lock.acquire()
                    target_mission[2] = 0
                    lock.release()
                    time.sleep(0.01)

                    lock.acquire()
                    target_mission[1] = 0
                    lock.release()
                    time.sleep(0.01)

                    lock.acquire()
                    target_mission[3] = 0
                    lock.release()
                    time.sleep(0.01)

                    lock.acquire()
                    target_mission[4] = 0
                    lock.release()
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
        t2.start()
        t3.start()
        t4.start()
        t5.start()
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

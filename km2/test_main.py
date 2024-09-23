import argparse
import concurrent
import math
import random
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import numpy as np

# 线程锁
lock = threading.Lock()

# 车列表，用于后续控制
nodes = []
target_mission = [-1, -1, -1, -1]


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
    def __init__(self, ego_vehicle, pathway, node_no, world):
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
        self.node_no = node_no
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
        # 如果路径已经走完，则重新生成新的路径
        if self.Path_index >= len(self.Path) and self.flag == 0:
            new_line = avoid_obstacles(self.vehicle, self.world)
            self.Path += new_line
            print('new line', self.Path)
            self.flag += 1

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
            if target_mission[int(self.node_no) - 1] == 0:  # 跳过停车点，等待下次放行任务
                self.Path_index += 1
                lock.acquire()
                target_mission[int(self.node_no - 1)] = -1
                lock.release()

            elif target_mission[int(self.node_no - 1)] == -1 and error < 0.83:  # 初始为-1，自己完成后改为1，放行改为0
                lock.acquire()
                target_mission[int(self.node_no - 1)] = 1
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


def avoid_obstacles(node, world):
    # 获取所有障碍物
    objects = world.get_environment_objects()
    obstacles = list(filter(lambda x: x.name.startswith('SM_Obstacle'), objects))

    # 获取小车位置信息
    x, y, z, frame_timestamp = node.get_location()

    # 初始化路径列表
    new_line = []

    # 循环处理每个障碍物
    while obstacles:
        # 在 每次计算最近障碍物之前，检测一遍临时障碍物，并更新障碍物列表
        temp_obstacle = world.get_targets()
        print('temp_obstacle', temp_obstacle)

        if len(temp_obstacle) > 0:
            print('temp_obstacle_location:==================================', temp_obstacle[0].get('location'))
            # obstacles.extend(temp_obstacle)

        # 计算小车与每个障碍物的距离
        distances = [math.sqrt((x - obstacle.transform.location.x) ** 2 + (y - obstacle.transform.location.y) ** 2) for
                     obstacle in obstacles]

        # 找到最近的障碍物
        min_distance = min(distances)
        min_index = distances.index(min_distance)
        nearest_obstacle = obstacles[min_index]  # 获取最近的障碍物信息
        obstacle_location = nearest_obstacle.transform.location
        print('obstacle_location', obstacle_location)

        # 判断当前车和障碍物的关系，并生成避障路径
        direction = None
        if y >= obstacle_location.y:
            direction = (obstacle_location.x, obstacle_location.y + 12)
        else:
            direction = (obstacle_location.x, obstacle_location.y - 12)
        res = format_point(direction)
        new_line.append(res)

        # 移除已处理的障碍物
        obstacles.pop(min_index)

    last_element = new_line[-1]
    final_last = np.array(
        [last_element[0] + 350, last_element[1], last_element[2] + 5, last_element[3]], dtype=object
    ).tolist()
    new_line.append(final_last)

    print('==============', new_line)
    return new_line


def format_point(bezier_point, speed=10):
    return np.array([bezier_point[0], bezier_point[1], speed, 'D'], dtype=object).tolist()


def sdk_get_map(client, entity_name):
    # sdk读取环境信息，需要 1.0.1 sdk 才支持
    timestamp, world, code = client.get_world()
    objects = world.get_environment_objects()

    return list(filter(lambda x: (x.name.startswith(entity_name)), objects))


def create_path(client, node_start_location, new_nodes):
    SM_Wall_Single = sdk_get_map(client, 'SM_Wall_Single')  # 两面墙位置
    SM_Obstacles = sdk_get_map(client, 'SM_Obstacle')  # 障碍物位置

    obstacles = []
    for obj in SM_Obstacles:
        obs_loc = obj.transform.location
        obs = (obs_loc.x, obs_loc.y, obs_loc.z)
        obstacles.append(obs)

    sorted_positions = sorted(obstacles, key=lambda pos: pos[0])
    min_obs = sorted_positions[0]

    s_1_x = node_start_location[0] + 274
    s_1_y = node_start_location[1] + 5

    wall_1_extent = SM_Wall_Single[0].bounding_box.extent
    wall_1_x = SM_Wall_Single[0].transform.location.x  # 火力掩护区地址
    wall_1_y = SM_Wall_Single[0].transform.location.y

    wall_2_extent = SM_Wall_Single[1].bounding_box.extent
    wall_2_x = SM_Wall_Single[1].transform.location.x  # 敌方火力区地址
    wall_2_y = SM_Wall_Single[1].transform.location.y

    obs_x = min_obs[0]  # 区域三
    obs_y = min_obs[1]

    node_1 = new_nodes[0].get_location()
    node_2 = new_nodes[1].get_location()
    node_3 = new_nodes[2].get_location()
    node_4 = new_nodes[3].get_location()
    step = 5.6


    target_path_01 = [[node_1[0] + 20, node_1[1] - 13, 5, 'D'], [node_1[0] + 80, node_1[1] - 11, 7, 'P'], [wall_1_x - 164, wall_1_y, 15, 'D'],
                      [wall_1_x - 70, wall_1_y, 13, 'D'], [wall_1_x - 15, wall_1_y, 6, 'P'], [wall_1_x - 70, wall_1_y, 7, 'D'],
                      [wall_2_x - 35, wall_2_y, 7, 'D'], [wall_2_x - 15, wall_2_y + 15, 12, 'D'],
                      [wall_2_x + 285, wall_2_y + 15, 15, 'D'], [obs_x - 160, obs_y - 15, 11, 'D']]

    target_path_02 = [[node_2[0] + 20, node_2[1] - 13, 5, 'D'], [node_1[0] + 80 - step, node_1[1] - 11, 7, 'P'], [wall_1_x - 164 - step, wall_1_y, 15, 'D'],
                      [wall_1_x - 70 - step, wall_1_y, 13, 'D'], [wall_1_x - 15 - step, wall_1_y, 6, 'P'], [wall_1_x - 70 - step, wall_1_y, 7, 'D'],
                      [wall_2_x - 35 - step, wall_2_y, 7, 'D'], [wall_2_x - 15 - step, wall_2_y + 15, 12, 'D'],
                      [wall_2_x + 285 - step, wall_2_y + 15, 15, 'D'], [obs_x - 160, obs_y - 15, 9, 'D']]

    target_path_03 = [[node_3[0] + 20, node_3[1] - 13, 5, 'D'], [node_1[0] + 80, node_1[1] - 11 + step, 7, 'P'], [wall_1_x - 164, wall_1_y + step, 15, 'D'],
                      [wall_1_x - 70, wall_1_y + step, 13, 'D'], [wall_1_x - 15, wall_1_y + step, 6, 'P'], [wall_1_x - 70, wall_1_y + step, 7, 'D'],
                      [wall_2_x - 35, wall_2_y + step, 7, 'D'], [wall_2_x - 15, wall_2_y + 15 + step, 12, 'D'],
                      [wall_2_x + 285, wall_2_y + 15 + step, 15, 'D'], [obs_x - 160, obs_y - 15, 10, 'D']]

    target_path_04 = [[node_4[0] + 20, node_4[1] - 13, 5, 'D'], [node_1[0] + 80 - step, node_1[1] - 11 + step, 7, 'P'], [wall_1_x - 164 - step, wall_1_y + step, 15, 'D'],
                      [wall_1_x - 70 - step, wall_1_y + step, 13, 'D'], [wall_1_x - 15 - step, wall_1_y + step, 6, 'P'], [wall_1_x - 70 - step, wall_1_y + step, 7, 'D'],
                      [wall_2_x - 35 - step, wall_2_y + step, 7, 'D'], [wall_2_x - 15 - step, wall_2_y + 15 + step, 12, 'D'],
                      [wall_2_x + 285 - step, wall_2_y + 15 + step, 15, 'D'], [obs_x - 160, obs_y - 15, 7, 'D']]
    target_paths = [target_path_01, target_path_02, target_path_03, target_path_04]
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
        default=5
    )
    argparser.add_argument(
        '-s', '--subject',
        type=int,
        default=1
    )
    args = argparser.parse_args()
    ue_ip = args.address.strip()
    ue_port = args.port
    num = args.number
    try:
        # 1. 首先引入sdk
        from swarmae.SwarmAEClient import SwarmAEClient

        # 2. 连接UE
        client = SwarmAEClient(ue_ip=ue_ip, ue_port=ue_port)
        # 注册车辆
        for id in range(num):
            # node_no 从1开始编号
            timestamp, node, code = client.register_node(
                node_type="四轮车", node_name="节点" + str(id + 1), node_no=id + 1,
                frame_timestamp=int(round(time.time() * 1000))
            )
            if code == 200:
                # 获取信息
                node_name, node_no, _, node_type, node_model, color = node.get_node_info()
                nodes.append(node)

        def control_vehicle(node, command):
            node.control_vehicle(command)
            time.sleep(0.01)

        def set_vehicle_brake(node, command):
            node.apply_control(0, 0, command, 0.88, 1)
            time.sleep(0.01)

        def apply_control(node, command):
            node.apply_control(0, 0, command, 0.99, command)
            time.sleep(0.01)

        def start_control_vehicle(node, command):
            node.apply_control(command, 0, 0, 0, 1)
            time.sleep(0.01)

        node_start_location = nodes[0].get_location()

        # 创建线程列表
        threads = []
        # 为每个节点创建并启动一个线程
        for node in nodes:
            thread = threading.Thread(target=start_control_vehicle, args=(node, 0.7))
            threads.append(thread)
            thread.start()
            time.sleep(0.01)
        # 等待所有线程完成
        for thread in threads:
            thread.join()
        time.sleep(3.5)


        new_threads = []
        for node in nodes:
            thread = threading.Thread(target=set_vehicle_brake, args=(node, 1))
            new_threads.append(thread)
            thread.start()
            time.sleep(0.01)
        # 等待所有线程完成
        for thread in new_threads:
            thread.join()
        time.sleep(2) 

        break_flag = False   
        while True:
            time.sleep(0.1)
            for node in nodes:
                if int(node.get_health()[0]) < 100:  #
                    break_flag = True
                    break
            if break_flag:
                break

        print("guet_start============")
        # 重新设置所有车辆的编号
        new_nodes = []
        for i, node in enumerate(nodes):
            if int(node.get_health()[0]) == 100:  # 正式测试时需要使用这个 条件
                new_nodes.append(node)
                print(node)

        
        # 将所有正常车 先减速5s
        new_nodes_threads = []
        for node in new_nodes:
            thread = threading.Thread(target=apply_control, args=(node, 1))
            new_nodes_threads.append(thread)
            thread.start()
            time.sleep(0.01)
        # 等待所有线程完成
        for thread in new_nodes_threads:
            thread.join()
        time.sleep(2)


        # 将速度拉下来，让小车保持距离
        speed = 0.38
        # 创建一个线程池
        with ThreadPoolExecutor(max_workers=len(new_nodes)) as executor:
            # 创建一个字典来存储 future 对象和它们对应的速度
            futures = {}
            for node in new_nodes:
                throttle_value = speed
                futures[executor.submit(control_vehicle, node, throttle_value)] = throttle_value
                speed -= 0.082

            # 等待所有线程完成
            for future in concurrent.futures.as_completed(futures):
                throttle_value = futures[future]
                time.sleep(1.5)
        time.sleep(1.5)

        with ThreadPoolExecutor(max_workers=len(new_nodes)) as executor:
            # 创建一个字典来存储 future 对象和它们对应的速度
            futures = {}
            for node in new_nodes:
                futures[executor.submit(control_vehicle, node, 0.1)] = throttle_value

            # 等待所有线程完成
            for future in concurrent.futures.as_completed(futures):
                throttle_value = futures[future]
                time.sleep(1)
        time.sleep(1)

        # 输出路径点和详细信息格式为[x,y,speed,D/P] D目标点后继续前进 P为目标的需要停止
        # target_path_01 = [[840, 446, 5, 'P'], [930, 439, 10, 'D'], [1110, 417, 10, 'D'],
        #                   [1210, 417, 10, 'D'],
        #                   [1257, 416, 6, 'P'], [1210, 417, 6, 'D'], [1230, 439, 6, 'D'], [1250, 450, 6, 'D'],
        #                   [1350, 450, 6, 'P'], [1600, 437, 10, 'D']]
        #
        # target_path_02 = [[834.7, 446, 5, 'P'], [925, 439, 10, 'D'], [1105, 417, 10, 'D'],
        #                   [1205, 417, 10, 'D'],
        #                   [1252, 416, 6, 'P'], [1205, 417, 6, 'D'], [1225, 439, 6, 'D'], [1245, 450, 6, 'D'],
        #                   [1345, 450, 6, 'P'], [1600, 437, 10, 'D']]
        #
        # target_path_03 = [[840, 451, 5, 'P'], [930, 444, 10, 'D'], [1110, 422, 10, 'D'],
        #                   [1210, 422, 10, 'D'],
        #                   [1257, 421, 6, 'P'], [1210, 422, 6, 'D'], [1230, 444, 6, 'D'], [1250, 455, 6, 'D'],
        #                   [1350, 455, 6, 'P'], [1600, 437, 10, 'D']]
        #
        # target_path_04 = [[834.7, 451, 5, 'P'], [925, 444, 10, 'D'], [1105, 422, 10, 'D'],
        #                   [1205, 422, 10, 'D'],
        #                   [1252, 421, 6, 'P'], [1205, 422, 6, 'D'], [1225, 444, 6, 'D'], [1245, 455, 6, 'D'],
        #                   [1345, 455, 6, 'P'], [1600, 437, 10, 'D']]

        target_paths = create_path(client, node_start_location, new_nodes)
        # # 车辆控制
        timestamp, world, code = client.get_world()
        control_object_01 = Vehicle_control(new_nodes[0], target_paths[0], 1, world)
        control_object_02 = Vehicle_control(new_nodes[1], target_paths[1], 2, world)
        control_object_03 = Vehicle_control(new_nodes[2], target_paths[2], 3, world)
        control_object_04 = Vehicle_control(new_nodes[3], target_paths[3], 4, world)

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

        def all_control():
            count = 0
            while True:
                time.sleep(0.01)
                global target_mission
                if sum(target_mission) == 4 and count == 0:
                    
                    time.sleep(2)  # 任务1完成6秒后出发

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
                    count += 1

                if sum(target_mission) == 4 and count == 1:
                    
                    time.sleep(0.1)
                    target_mission = [0, 0, 0, 0]
                    count += 1

                # if sum(target_mission) == 4 and count == 2:
                #     count += 1
                #     time.sleep(1)  # 任务1完成6秒后出发

                #     lock.acquire()
                #     target_mission[0] = 0
                #     lock.release()
                #     time.sleep(2.5)

                #     lock.acquire()
                #     target_mission[2] = 0
                #     lock.release()
                #     time.sleep(2)

                #     lock.acquire()
                #     target_mission[1] = 0
                #     lock.release()
                #     time.sleep(2.5)

                #     lock.acquire()
                #     target_mission[3] = 0
                #     lock.release()

        t1 = threading.Thread(target=vcontrol_1)
        t2 = threading.Thread(target=vcontrol_2)
        t3 = threading.Thread(target=vcontrol_3)
        t4 = threading.Thread(target=vcontrol_4)
        t5 = threading.Thread(target=all_control)

        t1.start()
        t2.start()
        t3.start()
        t4.start()
        t5.start()
    finally:
        print('main() end')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('__main__ end')

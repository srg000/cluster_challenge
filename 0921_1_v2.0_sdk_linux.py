import time
import math
from collections import deque
import threading
import argparse

# 线程锁
lock = threading.Lock()

# 车列表，用于后续控制
nodes = []
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
        self.error_buffer = deque(maxlen=100)  # 设置一个误差缓存区，用于积分项和差分项的计算
        self.error_threshold = 1  # 设定一个阈值

    def PID_control(self, target_position):
        """
        函数：计算PID控制的输出
        return: u
        """
        location = self.vehicle.get_location()  # self.vehicle.get_velocity()

        self.target_position = target_position  # 目标速度D

        error = math.sqrt(
            (self.target_position[0] - location[0]) ** 2 + (self.target_position[1] - location[1]) ** 2)  # 距离差

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
    def __init__(self, ego_vehicle, pathway):
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
        target_point = self.Path[self.Path_index]
        target_angle, driver_mode = self.angle_mode_count(target_point)

        if target_point[3] == 'D':
            if abs(target_point[0] - (self.vehicle.get_location()[0])) < 10:
                self.Path_index += 1
            target_speed = target_point[2] * driver_mode  # 计算需要控制的最大目标速度

        # 停车车速控制
        elif target_point[3] == 'P':
            pid_speed, error = self.Parking_control.PID_control(target_point)
            target_speed = min(target_point[2], pid_speed) * driver_mode
            global target_mission
            if target_mission[int(self.vehicle.get_node_info()[1]) - 1] == 0:  # 跳过停车点，等待下次放行任务
                self.Path_index += 1
                lock.acquire()
                target_mission[int(self.vehicle.get_node_info()[1] - 1)] = -1
                lock.release()

            elif target_mission[int(self.vehicle.get_node_info()[1] - 1)] == -1 and error < 0.4:  # 初始为-1，自己完成后改为1，放行改为0
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

    entitys = list(filter(lambda x: (x.name.startswith(entity_name)), objects))

    print(f"entitys len: {len(entitys)}")
    # for entity in entitys:
    #     print(entity)
    return entitys


def create_path(client):
    '''
    Mesh(id=9137305488417451985, name=BP_Bridge_SM_0, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=3947073371958556415, name=BP_Bridge_SM_1, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=15341302823975162039, name=BP_Bridge_SM_2, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=16323050085299573552, name=BP_Bridge_SM_3, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=7415580299229501886, name=BP_Bridge_SM_4, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=9065901915005755346, name=BP_Bridge_SM_5, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=9000246658315599348, name=BP_Bridge_SM_6, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=6076405299895700080, name=BP_Bridge_SM_7, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=7701859443096489116, name=BP_Bridge_SM_8, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=16538407641240465198, name=BP_Bridge_SM_9, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=5879398340418169428, name=BP_Bridge_SM_10, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=4846309383460322403, name=BP_Bridge_SM_11, transform=Transform(Location(x=-515.309021, y=447.157715, z=484.545074), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-515.309021, y=447.157715, z=484.545074), Extent(x=4.000000, y=3.101149, z=0.575867), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))

    Mesh(id=5148036365721985073, name=SM_SpeedBump_0_SM_0, transform=Transform(Location(x=-1072.719971, y=439.799988, z=486.446045), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-1072.719971, y=439.799988, z=486.446045), Extent(x=0.200000, y=30.000010, z=0.010477), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=1793548211848906098, name=SM_SpeedBump_1_SM_0, transform=Transform(Location(x=-1062.719971, y=439.799988, z=486.437958), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-1062.719971, y=439.799988, z=486.437958), Extent(x=0.200000, y=30.000010, z=0.010477), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=12841089146197709455, name=SM_SpeedBump_2_SM_0, transform=Transform(Location(x=-562.719971, y=439.799988, z=486.436096), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-562.719971, y=439.799988, z=486.436096), Extent(x=0.200000, y=30.000010, z=0.010477), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=12730791208815567714, name=SM_SpeedBump_3_SM_0, transform=Transform(Location(x=-362.720001, y=439.799988, z=486.436249), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=-362.720001, y=439.799988, z=486.436249), Extent(x=0.200000, y=30.000010, z=0.010477), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    Mesh(id=11908609790605142444, name=SM_SpeedBump_4_SM_0, transform=Transform(Location(x=137.279999, y=439.799988, z=486.443298), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=137.279999, y=439.799988, z=486.443298), Extent(x=0.200000, y=30.000010, z=0.010477), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
    '''

    target_path_01 = []
    target_path_02 = []
    target_path_03 = []
    target_path_04 = []
    target_path_05 = []
    BP_Bridges = sdk_get_map(client, 'BP_Bridge')
    SM_SpeedBumps = sdk_get_map(client, 'SM_SpeedBump')

    s_1_x = SM_SpeedBumps[1].transform.location.x  # 出发点坐标
    s_1_y = SM_SpeedBumps[1].transform.location.y

    s_2_x = SM_SpeedBumps[2].transform.location.x  # 坑洼路坐标
    s_2_y = SM_SpeedBumps[2].transform.location.y

    b_x = BP_Bridges[0].transform.location.x  # 桥的起始坐标，桥长约65m  按80m
    b_y = BP_Bridges[0].transform.location.y

    s_3_x = SM_SpeedBumps[3].transform.location.x  # 列队区域
    s_3_y = SM_SpeedBumps[3].transform.location.y

    s_4_x = SM_SpeedBumps[4].transform.location.x  # 冲线点
    s_4_y = SM_SpeedBumps[4].transform.location.y

    #                              队形点前                      队形点                              坑洼路前80            坑洼路                 坑洼路到桥头前                   桥头                      桥中                          下桥                列队开始                      队列形成前                           队列形成                                   冲线
    target_path_01 = [[s_1_x + 41.4, s_1_y - 5.0, 10, 'D'], [s_1_x + 61.4, s_1_y - 5.0, 5, 'P'],
                      [s_2_x - 80, b_y, 17, 'D'], [s_2_x, b_y, 10, 'D'], [b_x - 20, b_y, 10.0, 'D'],
                      [b_x, b_y, 10.4, 'D'], [b_x + 35, b_y, 10.4, 'D'], [b_x + 110, b_y, 6, 'D'],
                      [s_3_x, s_3_y, 10, 'D'], [s_3_x + 60.0, s_3_y + 5.0, 12, 'D'],
                      [s_3_x + 80.0, s_3_y + 5.0, 6, 'P'], [s_4_x + 80.0, s_3_y + 5.0, 17, 'D']]
    target_path_02 = [[s_1_x + 45.7, s_1_y - 2.5, 10, 'D'], [s_1_x + 65.7, s_1_y - 2.5, 5, 'P'],
                      [s_2_x - 80, b_y, 17, 'D'], [s_2_x, b_y, 10, 'D'], [b_x - 20, b_y, 11.5, 'D'],
                      [b_x, b_y, 11.6, 'D'], [b_x + 35, b_y, 11.6, 'D'], [b_x + 110, b_y, 6, 'D'],
                      [s_3_x, s_3_y, 10, 'D'], [s_3_x + 64.3, s_3_y + 2.5, 12, 'D'],
                      [s_3_x + 84.3, s_3_y + 2.5, 6, 'P'], [s_3_x + 84.3, s_3_y + 2.5, 17, 'D']]
    target_path_03 = [[s_1_x + 50.0, s_1_y + 0.0, 10, 'D'], [s_1_x + 70.0, s_1_y + 0.0, 5, 'P'],
                      [s_2_x - 80, b_y, 17, 'D'], [s_2_x, b_y, 10, 'D'], [b_x - 20, b_y, 12.0, 'D'],
                      [b_x, b_y, 12.0, 'D'], [b_x + 35, b_y, 12.0, 'D'], [b_x + 110, b_y, 6, 'D'],
                      [s_3_x, s_3_y, 10, 'D'], [s_3_x + 64.3, s_3_y - 2.5, 12, 'D'],
                      [s_3_x + 84.3, s_3_y - 2.5, 6, 'P'], [s_3_x + 84.3, s_3_y - 2.5, 17, 'D']]
    target_path_04 = [[s_1_x + 45.7, s_1_y + 2.5, 10, 'D'], [s_1_x + 65.7, s_1_y + 2.5, 5, 'P'],
                      [s_2_x - 80, b_y, 17, 'D'], [s_2_x, b_y, 10, 'D'], [b_x - 20, b_y, 11.0, 'D'],
                      [b_x, b_y, 11.2, 'D'], [b_x + 35, b_y, 11.2, 'D'], [b_x + 110, b_y, 6, 'D'],
                      [s_3_x, s_3_y, 10, 'D'], [s_3_x + 60.0, s_3_y - 5.0, 12, 'D'],
                      [s_3_x + 80.0, s_3_y - 5.0, 6, 'P'], [s_3_x + 80.0, s_3_y - 5.0, 17, 'D']]
    target_path_05 = [[s_1_x + 41.4, s_1_y + 5.0, 10, 'D'], [s_1_x + 61.4, s_1_y + 5.0, 5, 'P'],
                      [s_2_x - 80, b_y, 17, 'D'], [s_2_x, b_y, 10, 'D'], [b_x - 20, b_y, 10.5, 'D'],
                      [b_x, b_y, 10.8, 'D'], [b_x + 35, b_y, 10.8, 'D'], [b_x + 110, b_y, 6, 'D'],
                      [s_3_x, s_3_y, 10, 'D'], [s_3_x + 60.0, s_3_y + 0.0, 12, 'D'],
                      [s_3_x + 80.0, s_3_y + 0.0, 6, 'P'], [s_3_x + 80.0, s_3_y + 0.0, 17, 'D']]

    target_paths = [target_path_01, target_path_02, target_path_03, target_path_04, target_path_05]
    print(target_paths)

    return target_paths


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-i', '--address',
        default='127.0.0.1')
    argparser.add_argument(
        '-p', '--port',
        type=int,
        default=2000)
    argparser.add_argument(
        '-n', '--number',
        type=int,
        default=5)
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

        # 输出路径点和详细信息格式为[x,y,speed,D/P] D目标点后继续前进 P为目标的需要停止
        target_path_01 = [[-1008.3, 442, 25, 'P'], [-808.3, 442, 25, 'D'], [-680, 446.2, 25, 'D'], [-650, 446, 25, 'D'],
                          [-520, 446.86, 10.2, 'D'], [-450, 446.86, 14, 'D'], [-425, 446.8, 9, 'D'],
                          [-360, 446, 12, 'D'], [-300, 441, 6, 'P'], [200, 441, 25, 'P']]
        target_path_02 = [[-1004.3, 444.5, 25, 'P'], [-804.3, 444.5, 25, 'D'], [-680, 446.2, 25, 'D'],
                          [-620, 446, 25, 'D'], [-520, 446.86, 10.4, 'D'], [-450, 446.86, 14.75, 'D'],
                          [-425, 446.8, 9.75, 'D'], [-360, 446, 12, 'D'], [-297.5, 443.5, 6, 'P'],
                          [200, 443.5, 25, 'P']]
        target_path_03 = [[-1000, 447, 25, 'P'], [-800, 447, 25, 'D'], [-680, 446.2, 25, 'D'], [-600, 446, 25, 'D'],
                          [-520, 446.86, 10.5, 'D'], [-450, 446.86, 15, 'D'], [-425, 446.8, 10, 'D'],
                          [-360, 446, 12, 'D'], [-300, 446, 6, 'P'], [200, 446, 25, 'P']]
        target_path_04 = [[-1004.3, 449.5, 25, 'P'], [-804.3, 449.5, 25, 'D'], [-680, 446.2, 25, 'D'],
                          [-640, 446, 25, 'D'], [-520, 446.86, 10.3, 'D'], [-450, 446.86, 14.5, 'D'],
                          [-425, 446.8, 9.5, 'D'], [-360, 446, 12, 'D'], [-297.5, 448.5, 6, 'P'], [200, 448.5, 25, 'P']]
        target_path_05 = [[-1008.3, 452, 25, 'P'], [-808.3, 452, 25, 'D'], [-680, 446.2, 25, 'D'], [-650, 446, 25, 'D'],
                          [-520, 446.86, 10.0, 'D'], [-450, 446.86, 13.75, 'D'], [-425, 446.8, 8.75, 'D'],
                          [-360, 446, 12, 'D'], [-300, 451, 6, 'P'], [200, 451, 25, 'P']]
        target_paths = create_path(client)

        # 车辆控制a
        control_object_01 = Vehicle_control(nodes[0], target_paths[0])
        control_object_02 = Vehicle_control(nodes[1], target_paths[1])
        control_object_03 = Vehicle_control(nodes[2], target_paths[2])
        control_object_04 = Vehicle_control(nodes[3], target_paths[3])
        control_object_05 = Vehicle_control(nodes[4], target_paths[4])

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
                print(target_mission)
                if sum(target_mission) == 5 and count == 0:
                    count = 1
                    time.sleep(2)  # 任务1完成6秒后出发
                    lock.acquire()
                    target_mission[2] = 0
                    lock.release()
                    time.sleep(1.5)
                    lock.acquire()
                    target_mission[1] = 0
                    lock.release()
                    time.sleep(1.5)
                    lock.acquire()
                    target_mission[3] = 0
                    lock.release()
                    time.sleep(1.5)
                    lock.acquire()
                    target_mission[0] = 0
                    lock.release()
                    time.sleep(2.5)
                    lock.acquire()
                    target_mission[4] = 0
                    lock.release()
                if sum(target_mission) == 5 and count == 1:
                    time.sleep(2)
                    target_mission = [0, 0, 0, 0, 0]

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
        print('main() end')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('__main__ end')

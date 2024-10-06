# from get_node_xyz import get_map
import time
import math
from collections import deque
import threading
import argparse

# 线程锁
lock = threading.Lock()

# 车列表，用于后续控制
nodes = []
target_mission = [-1, -1, -1, -1]


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
        if (v[0] < 0):  # 反向速度解析
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

        target_angle_rad = math.atan2(end[1] - start[1], end[0] - start[0])
        target_angle_deg = math.degrees(target_angle_rad)
        if -90 < target_angle_deg and target_angle_deg < 90:
            return target_angle_deg, 1
        elif target_angle_deg <= -90:
            return 180 - abs(target_angle_deg), -1
        elif target_angle_deg >= 90:
            return target_angle_deg - 180, -1

    def avoid_barrier(self, start, end, move_distance=8, avoid_distance=7):
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

        target_point = self.Path[self.Path_index]

        attitude_node = self.vehicle.get_attitude()

        target_angle, driver_mode = self.angle_mode_count(start, target_point)

        # 巡航攻击
        len_Attack = len(self.Attack_target)
        if len_Attack > self.Attack_index:
            attack_1 = self.Attack_target[self.Attack_index]
            if attack_1[0] - start[0] < 120:  # 进入瞄准距离
                attack_angle, need_attack = self.angle_mode_count(start, attack_1)
                if need_attack > 0:
                    self.Weapon.set_weapon_angle(attack_angle - attitude_node[0], 0 - attitude_node[1],
                                                 time.time())  # 瞄准
                    if attack_1[0] - start[0] < 55 and abs(attitude_node[1]) < 1:  # 进入攻击距离
                        if abs(self.vehicle.get_attitude()[1]) < 1:
                            if abs(attack_angle) > 8:
                                self.vehicle.gun_fire(1)
                                self.Attack_index += 1

        if target_point[3] == 'D':
            if abs(target_point[0] - (start[0])) < 7:
                self.Path_index += 1
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

        elif target_point[3] == 'S':
            pid_speed, error = self.Parking_control.PID_control(target_point)
            target_speed = min(target_point[2], abs(pid_speed)) * driver_mode
            if error < 2 and abs(attitude_node[1]) < 0.7:
                self.vehicle.gun_fire(1)
                self.Path_index += 1

        # 停车车速控制
        if target_point[3] == 'P':
            pid_speed, error = self.Parking_control.PID_control(target_point)
            target_speed = min(target_point[2], abs(pid_speed)) * driver_mode

            node_id = self.vehicle.get_node_info()[1]
            if target_mission[int(node_id) - 1] == 0:  # 收到放行指令，跳过停车点，等待下次放行任务
                self.Path_index += 1
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


'''
顺序是否会优化？
Mesh(id=5148036365721985073,  name=SM_SpeedBump_0_SM_0, transform=Transform(Location(x=2838.849854, y=439.799988, z=486.413025), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=2838.849854, y=439.799988, z=486.413025), Extent(x=0.200000, y=30.000010, z=0.010477), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=12841089146197709455, name=SM_SpeedBump_2_SM_0, transform=Transform(Location(x=3348.849854, y=439.799988, z=486.425720), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3348.849854, y=439.799988, z=486.425720), Extent(x=0.200000, y=30.000010, z=0.010477), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=12730791208815567714, name=SM_SpeedBump_3_SM_0, transform=Transform(Location(x=3848.849854, y=439.799988, z=486.420624), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3848.849854, y=439.799988, z=486.420624), Extent(x=0.200000, y=30.000010, z=0.010477), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=11908609790605142444, name=SM_SpeedBump_4_SM_0, transform=Transform(Location(x=4348.850098, y=439.799988, z=486.278778), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=4348.850098, y=439.799988, z=486.278778), Extent(x=0.200000, y=30.000010, z=0.010477), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=6016886745726211886,  name=SM_SpeedBump_5_SM_0, transform=Transform(Location(x=2848.849854, y=439.799988, z=486.415588), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=2848.849854, y=439.799988, z=486.415588), Extent(x=0.200000, y=30.000010, z=0.010477), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))

Mesh(id=15019441286765478737, name=BP_Barricade_0_SM_0, transform=Transform(Location(x=3087.445801, y=446.928894, z=486.205109), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3087.784912, y=447.073517, z=486.967102), Extent(x=1.248666, y=5.216498, z=0.882122), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=5514574699792589318, name=BP_Barricade_1_SM_0, transform=Transform(Location(x=3081.604736, y=428.761108, z=485.725433), Rotation(pitch=0.000000, yaw=179.999969, roll=0.000000)), bounding_box=BoundingBox(Location(x=3081.265625, y=428.616516, z=486.487396), Extent(x=1.248666, y=5.216498, z=0.882122), Rotation(pitch=0.000000, yaw=179.999969, roll=0.000000)))

Mesh(id=5183812273768705477, name=SM_Wall_0_SM_0, transform=Transform(Location(x=3528.699951, y=424.399994, z=487.669983), Rotation(pitch=0.000000, yaw=-89.999992, roll=0.000000)), bounding_box=BoundingBox(Location(x=3528.699951, y=424.399994, z=487.669983), Extent(x=14.099655, y=14.599854, z=1.250000), Rotation(pitch=0.000000, yaw=-89.999992, roll=0.000000)))
Mesh(id=8265387829885102873, name=SM_Wall_1_SM_0, transform=Transform(Location(x=3701.000000, y=455.149994, z=487.669983), Rotation(pitch=0.000000, yaw=-89.999992, roll=0.000000)), bounding_box=BoundingBox(Location(x=3701.000000, y=455.149994, z=487.669983), Extent(x=14.099655, y=14.599854, z=1.250000), Rotation(pitch=0.000000, yaw=-89.999992, roll=0.000000)))

Mesh(id=7485865046866888074, name=BP_WeaponCache_0_SM_0, transform=Transform(Location(x=4160.653809, y=460.942810, z=486.262787), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=4160.609375, y=461.136230, z=488.148804), Extent(x=1.145350, y=1.370213, z=1.936607), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=9275796581878164931, name=BP_WeaponCache_1_SM_0, transform=Transform(Location(x=4099.684082, y=420.789917, z=486.243530), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=4099.639648, y=420.983337, z=488.129547), Extent(x=1.145350, y=1.370213, z=1.936607), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))

Mesh(id=17620253248515795660, name=BP_AmmoCrat_0_SM_0, transform=Transform(Location(x=4034.105469, y=455.870453, z=490.561890), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=4034.341553, y=455.876617, z=488.831512), Extent(x=1.351969, y=1.508344, z=2.609704), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=16618970377060419071, name=BP_AmmoCrat_3_SM_0, transform=Transform(Location(x=4229.321289, y=429.761475, z=490.633545), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=4229.557129, y=429.767639, z=488.903168), Extent(x=1.351969, y=1.508344, z=2.609704), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))


Mesh(id=15949429296614610557, name=SM_Obstacle10_SM_0, transform=Transform(Location(x=3083.403076, y=461.315063, z=486.758392), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3083.403076, y=461.315063, z=486.758392), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=13562345242973926320, name=SM_Obstacle11_SM_0, transform=Transform(Location(x=3088.302979, y=461.215057, z=486.766357), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3088.302979, y=461.215057, z=486.766357), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=11308910329120574677, name=SM_Obstacle2_SM_0, transform=Transform(Location(x=3080.533691, y=421.334991, z=486.678864), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3080.533691, y=421.334991, z=486.678864), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=921498749482781364, name=SM_Obstacle3_SM_0, transform=Transform(Location(x=3085.433594, y=421.334991, z=486.794556), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3085.433594, y=421.334991, z=486.794556), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=5770966702205269610, name=SM_Obstacle4_SM_0, transform=Transform(Location(x=3090.333740, y=421.334991, z=486.794556), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3090.333740, y=421.334991, z=486.794556), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=3069289917226213753, name=SM_Obstacle5_22_SM_0, transform=Transform(Location(x=3080.926514, y=457.338440, z=486.398071), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3080.926514, y=457.338440, z=486.398071), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=17896910226124288197, name=SM_Obstacle6_SM_0, transform=Transform(Location(x=3085.826416, y=457.338440, z=486.517487), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3085.826416, y=457.338440, z=486.517487), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=18081418042765819075, name=SM_Obstacle7_SM_0, transform=Transform(Location(x=3090.726562, y=457.338440, z=486.794556), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3090.726562, y=457.338440, z=486.794556), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=14126064192684982292, name=SM_Obstacle8_SM_0, transform=Transform(Location(x=3073.603027, y=461.515076, z=486.398071), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3073.603027, y=461.515076, z=486.398071), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=15332933868660624542, name=SM_Obstacle9_SM_0, transform=Transform(Location(x=3078.503174, y=461.415070, z=486.412659), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3078.503174, y=461.415070, z=486.412659), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=11735562380373224665, name=SM_Obstacle_0_SM_0, transform=Transform(Location(x=4099.184082, y=423.261719, z=487.395966), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=4099.184082, y=423.261719, z=487.395966), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=18151766189295390836, name=SM_Obstacle_1_SM_0, transform=Transform(Location(x=3721.000000, y=455.149994, z=487.669983), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3721.000000, y=455.149994, z=487.669983), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=9996095152822011655, name=SM_Obstacle_10_SM_0, transform=Transform(Location(x=3548.699951, y=424.399994, z=487.500000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3548.699951, y=424.399994, z=487.500000), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=18120448531977676323, name=SM_Obstacle_16_SM_0, transform=Transform(Location(x=3075.633789, y=421.334991, z=486.277008), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=3075.633789, y=421.334991, z=486.277008), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=14019246619065813112, name=SM_Obstacle_2_SM_0, transform=Transform(Location(x=4034.105469, y=452.558441, z=487.546356), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=4034.105469, y=452.558441, z=487.546356), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=11548218219658376741, name=SM_Obstacle_3_SM_0, transform=Transform(Location(x=4227.921387, y=432.289764, z=487.395966), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=4227.921387, y=432.289764, z=487.395966), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))
Mesh(id=5496871880815395459, name=SM_Obstacle_5_SM_0, transform=Transform(Location(x=4159.453613, y=459.042969, z=487.395966), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)), bounding_box=BoundingBox(Location(x=4159.453613, y=459.042969, z=487.395966), Extent(x=0.600000, y=0.519615, z=1.125000), Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000)))

'''


def path_create(client):
    SM_SpeedBumps = sdk_get_map(client, 'SM_SpeedBump')
    BP_Barricades = sdk_get_map(client, "BP_Barricade")
    SM_Walls = sdk_get_map(client, "SM_Wall")
    BP_WeaponCaches = sdk_get_map(client, "BP_WeaponCache")
    BP_AmmoCrats = sdk_get_map(client, "BP_AmmoCrat")
    SM_Obstacles = sdk_get_map(client, 'SM_Obstacle')

    # print(BP_Barricade)
    # print(SM_Wall)
    # print(BP_WeaponCache)
    # print(BP_AmmoCrat)
    # print(SM_Obstacle)
    BP_3_ALL = BP_WeaponCaches + BP_AmmoCrats
    BP_all_location = []
    for bp_1 in BP_3_ALL:
        BP_all_location.append([bp_1.transform.location.x, bp_1.transform.location.y])
    BP_all_location = sorted(BP_all_location, key=lambda x: x[0])
    print(BP_all_location)
    Obstacles = []
    SpeedBumps = []
    for Obstacle in SM_Obstacles:
        Obstacles.append([Obstacle.transform.location.x, Obstacle.transform.location.y])
    for SpeedBump in SM_SpeedBumps:
        SpeedBumps.append([SpeedBump.transform.location.x, SpeedBump.transform.location.y])

    SpeedBumps = sorted(SpeedBumps, key=lambda x: x[0])

    area_1_Obstacle = [item for item in Obstacles if SpeedBumps[1][0] <= item[0] <= SpeedBumps[2][0]]
    area_2_Obstacle = [item for item in Obstacles if SpeedBumps[2][0] <= item[0] <= SpeedBumps[3][0]]
    area_3_Obstacle = [item for item in Obstacles if SpeedBumps[3][0] <= item[0] <= SpeedBumps[4][0]]

    s_1_x = SpeedBumps[1][0]  # 出发点坐标
    s_1_y = SpeedBumps[1][1]

    t_1_x = BP_Barricades[0].transform.location.x  # 攻击目标一坐标
    t_1_y = BP_Barricades[0].transform.location.y
    t_1_z = BP_Barricades[0].transform.location.z

    t_2_x = BP_Barricades[1].transform.location.x  # 攻击目标二坐标
    t_2_y = BP_Barricades[1].transform.location.y
    t_2_z = BP_Barricades[1].transform.location.z

    s_2_x = SpeedBumps[2][0]  # 坑洼路结束坐标铺装路开始
    s_2_y = SpeedBumps[2][1]

    w_1_x = SM_Walls[0].transform.location.x  # 掩体一坐标
    w_1_y = SM_Walls[0].transform.location.y

    w_2_x = SM_Walls[1].transform.location.x  # 掩体二坐标
    w_2_y = SM_Walls[1].transform.location.y

    s_3_x = SpeedBumps[3][0]  # 铺装路结束坐标沙地路开始
    s_3_y = SpeedBumps[3][1]

    bp_1_x = BP_all_location[0][0]  # x一坐标
    bp_1_y = BP_all_location[0][1]

    bp_2_x = BP_all_location[1][0]  # x二坐标
    bp_2_y = BP_all_location[1][1]

    bp_3_x = BP_all_location[2][0]  # x三坐标
    bp_3_y = BP_all_location[2][1]

    bp_4_x = BP_all_location[3][0]  # x四坐标
    bp_4_y = BP_all_location[3][1]

    s_4_x = SpeedBumps[4][0]  # 终点线
    s_4_y = SpeedBumps[4][0]

    if w_1_x > w_2_x:
        w_xmax = w_1_x
        w_ymax = w_1_y
        w_xmin = w_2_x
        w_ymin = w_2_y
    else:
        w_xmax = w_2_x
        w_ymax = w_2_y
        w_xmin = w_1_x
        w_ymin = w_1_y

    if t_1_x < t_2_x:
        t_f_x = t_1_x
        t_f_y = t_1_y
        t_f_z = t_1_z
        t_s_x = t_2_x
        t_s_y = t_2_y
        t_s_z = t_2_z
    else:
        t_f_x = t_2_x
        t_f_y = t_2_y
        t_f_z = t_2_z
        t_s_x = t_1_x
        t_s_y = t_1_y
        t_s_z = t_1_z
    if w_ymin > s_2_y:  # 说明障碍2靠右边，先往左靠
        lr_mode = -1
    else:
        lr_mode = 1

    target_03_path = [[BP_all_location[0][0] - 100, s_2_y, 14, 'D']]
    for bp in BP_all_location:
        if s_2_y - 5 < bp[1] and bp[1] < s_2_y + 5:
            target_03_path.append([bp[0], bp[y] + 6, 6, 'D'])
        else:
            target_03_path.append([bp[0], s_2_y, 6, 'D'])
    target_03_path.append([s_4_x + 50, s_2_y, 14, 'D'])
    print(target_03_path)
    #                                                      障碍物1前                   障碍物1攻击处                   障碍物2前                  障碍物2攻击处                                                     处于铺装路前                      掩体1前停车                     掩体中停车                 掩体中倒车出来                        掩体1外前                                                    过掩体1                              线外停车
    target_path_01 = [[t_f_x - 100, t_f_y, 8, 'D'], [t_f_x - 60, t_f_y, 7, 'D'], [t_f_x - 30, t_f_y, 5, 'S'],
                      [t_s_x - 60, t_s_y, 7, 'D'], [t_s_x - 30, t_s_y, 5, 'S'], [t_s_x + 30, t_s_y, 10, 'P'],
                      [s_2_x - 20, w_ymax, 10, 'D'], [w_xmax - 22, w_ymax, 10, 'P'], [w_xmax, w_ymax, 10, 'P'],
                      [w_xmax - 50, s_2_y, 10, 'D'], [w_xmax - 18, s_2_y + (-lr_mode * 15), 17, 'D'],
                      [w_xmax + 18, s_2_y + (-lr_mode * 15), 17, 'D'], [s_3_x - 10, s_3_y, 17, 'P']]
    #                         障碍物1前                   障碍物2处                        铺装路前                     掩体2前停车                    掩体2中停车                     掩体2中倒出                     掩体2外中间                                      掩体2外上角                                 掩体1外前                                      过掩体1                             线前停
    target_path_02 = [[t_f_x - 60, t_f_y, 4, 'P'], [t_s_x, s_2_y, 10, 'D'], [s_2_x - 20, w_ymin, 10, 'D'],
                      [w_xmin - 22, w_ymin, 10, 'P'], [w_xmin, w_ymin, 17, 'P'], [w_xmin - 50, s_2_y, 17, 'D'],
                      [w_xmin - 12, s_2_y + (lr_mode * 15), 17, 'D'], [w_xmin + 18, s_2_y + (lr_mode * 15), 17, 'D'],
                      [w_xmax + 18, s_2_y + (-lr_mode * 15), 17, 'D'], [w_xmax + 18, s_2_y + (-lr_mode * 15), 17, 'D'],
                      [s_3_x - 30, s_3_y, 17, 'P']]
    #                         障碍物1前                   障碍物2处                         铺装路前                    掩体2外上角                                      掩体1外前                                   过掩体1                                 线前停车
    target_path_03 = [[t_f_x - 95, t_f_y, 4, 'P'], [t_s_x, t_s_y, 9, 'D'], [s_2_x - 20, s_2_y, 10, 'D'],
                      [w_xmin + 18, s_2_y + (lr_mode * 15), 17, 'P'], [w_xmax - 18, s_2_y + (-lr_mode * 15), 17, 'D'],
                      [w_xmax + 18, s_2_y + (-lr_mode * 15), 17, 'D'], [s_3_x + 40, s_3_y, 17, 'P']]
    #                         障碍物1前                   障碍物2处                         铺装路前                    掩体2外中间                                      掩体2外上角                                 掩体1外前                                      过掩体1                             线前停车
    target_path_04 = [[t_f_x - 125, t_f_y, 4, 'P'], [t_s_x, t_s_y, 8, 'D'], [s_2_x - 20, s_2_y, 10, 'D'],
                      [w_xmin - 12, s_2_y + (lr_mode * 15), 17, 'P'], [w_xmin + 18, s_2_y + (lr_mode * 15), 17, 'D'],
                      [w_xmax - 18, s_2_y + (-lr_mode * 15), 17, 'D'], [w_xmax + 18, s_2_y + (-lr_mode * 15), 17, 'D'],
                      [s_3_x + 10, s_3_y, 17, 'P']]

    all_path = [target_path_01 + target_03_path, target_path_02 + target_03_path, target_path_03 + target_03_path,
                target_path_04 + target_03_path]

    BP_location = [[BP_all_location[2]], [BP_all_location[3]], [BP_all_location[0]], [BP_all_location[1]]]
    print(BP_location)
    return all_path, BP_location


def sdk_get_map(client, entity_name):
    # sdk读取环境信息，需要 1.0.1 sdk 才支持
    timestamp, world, code = client.get_world()
    objects = world.get_environment_objects()

    entitys = list(filter(lambda x: (x.name.startswith(entity_name)), objects))

    print(f"entitys len: {len(entitys)}")
    for entity in entitys:
        print(entity)
    return entitys


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

        '''
        [['BP_Barricade_0', 3087.7848975372312, -447.07353057861326, 486.9671220779419], ['BP_Barricade_1', 3081.2657284413976, -428.6165089160966, 486.4874345779419]]
        [['SM_Wall_0', 3528.700119045008, -424.4000122070286, 487.67000022888186], ['SM_Wall_1', 3701.0001190450075, -455.1500122070286, 487.67000022888186]]
        [['BP_WeaponCache_0', 4160.616912260056, -461.1040254211426, 487.07110996246337], ['BP_WeaponCache_1', 4099.647224760056, -421.42308792114255, 487.0518521499634]]
        [['BP_AmmoCrat_0', 4034.23685546875, -450.861875, 487.0463818359375], ['BP_AmmoCrat_3', 4229.45248046875, -430.99234375, 486.9831787109375]]
        '''
        all_path, BP_all_location = path_create(client)
        # all_path = [[[3087.44580078125, 436.92889404296875, 8, 'D'], [3328.849853515625, 439.79998779296875, 10, 'D'],
        #              [3516.699951171875, 449.79998779296875, 17, 'D'], [3546.699951171875, 449.79998779296875, 17, 'D'],
        #              [3719.0, 434.79998779296875, 17, 'D'], [3858.849853515625, 439.79998779296875, 17, 'P']],
        #             [],
        #             [],
        #             []
        #             ]
        attacklists = [[[3087.7848975372312, 447.07353057861326]],
                       [],
                       [],
                       []
                       ]
        # 车辆控制   按照距离第一个点的Y  坐标点顺序排序
        control_object_01 = Vehicle_control(nodes[0], all_path[0], BP_all_location[0], nodes[0].get_weapon()[1])
        control_object_02 = Vehicle_control(nodes[1], all_path[1], BP_all_location[1], nodes[1].get_weapon()[1])
        control_object_03 = Vehicle_control(nodes[2], all_path[2], BP_all_location[2], nodes[2].get_weapon()[1])
        control_object_04 = Vehicle_control(nodes[3], all_path[3], BP_all_location[3], nodes[3].get_weapon()[1])

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

        def all_control():  # 先放行完成  再进行任务数加一
            count = 0
            while True:
                time.sleep(0.1)
                global target_mission
                # 调试放行
                if sum(target_mission) == 4 and count == 0:  # 攻击完成
                    print("---------------------------攻击完成")
                    print(target_mission)
                    time.sleep(1)
                    target_mission = [0, 0, 0, 0]
                    count += 1
                if sum(target_mission) == 4 and count == 1:  # 掩体前停车完成
                    print("---------------------------掩体前停车完成")
                    print(target_mission)
                    target_mission = [0, 0, 0, 0]
                    count += 1
                if target_mission[0] == 1 and target_mission[1] == 1 and count == 2:  # 掩体前停车完成
                    print("---------------------------掩体内停车完成")
                    print(target_mission)
                    time.sleep(1)
                    target_mission[0] = 0
                    target_mission[1] = 0
                    count += 1
                if sum(target_mission) == 4 and count == 3:  # 掩体前停车完成
                    print("---------------------------掩体内停车完成")
                    print(target_mission)
                    target_mission[2] = 0
                    time.sleep(5)
                    target_mission[3] = 0
                    time.sleep(5)
                    target_mission[0] = 0
                    time.sleep(5)
                    target_mission[1] = 0
                    count += 1

        t1 = threading.Thread(target=vcontrol_1)
        t2 = threading.Thread(target=vcontrol_2)
        t3 = threading.Thread(target=vcontrol_3)
        t4 = threading.Thread(target=vcontrol_4)

        t6 = threading.Thread(target=all_control)

        t1.start()
        time.sleep(7)
        t2.start()
        time.sleep(7)
        t3.start()
        time.sleep(7)
        t4.start()

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

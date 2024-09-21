import numpy as np
import time
import math
from collections import deque
import threading
import argparse
from models.SwarmAEVehicle import SwarmAEVehicle
from concurrent.futures import ThreadPoolExecutor
import concurrent
import bezierUtil

# 线程锁
lock = threading.Lock()

# 车列表，用于后续控制
nodes = []
vehicles = []
target_angle = 0


class Steer_controller(object):
    def __init__(self, node_car, k_p=0.015, k_i=0.002, k_d=0.002, d_t=0.05):  # 手动调节参数k_p、k_i、k_d
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


class Longitudinal_PID_controller(object):  # 纵向控制
    def __init__(self, node_car, k_p=0.190, k_i=1.04, k_d=0.09, d_t=0.01):  # 手动调节参数k_p、k_i、k_d
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
        self.error_buffer = deque(maxlen=90)  # 设置一个误差缓存区，用于积分项和差分项的计算
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
        self.max_brake = 1  # 最大刹车 = 1，最小刹车 = 0
        self.max_steer = 1  # 最大转角 = 1
        self.min_steer = -1  # 最小转角 = 1
        self.Lateral = None
        self.Path = pathway
        self.Path_index = 0  # 路径起始点
        self.Lateral_control = Steer_controller(ego_vehicle)
        # elif controller_type == "LQR_controller":  # 采用LQR横向控制
        #     self.Lateral = "LQR_controller"
        #     self.Lateral_control = Lateral_LQR_controller(ego_vehicle, vehicle_para, pathway)
        self.Longitudinal_control = Longitudinal_PID_controller(ego_vehicle)

    def angle_mode_count(self, end):
        # 计算目标方向的角度     1 为前进  -1 为倒车
        start = (self.vehicle.get_location()[0], self.vehicle.get_location()[1])
        target_angle_rad = math.atan2(end[1] - start[1], end[0] - start[0])
        target_angle_deg = math.degrees(target_angle_rad)
        print(target_angle_deg)
        if -90 < target_angle_deg and target_angle_deg < 90:
            return target_angle_deg, 1
        elif target_angle_deg < -90:
            return 180 - abs(target_angle_deg), -1
        elif target_angle_deg > 90:
            return target_angle_deg - 180, -1

    def run_step(self):
        """
        函数：计算横向控制与纵向控制的流程
        """
        if self.Path_index >= len(self.Path):
            return False
        target_point = self.Path[self.Path_index]
        if abs(target_point[0] - (self.vehicle.get_location()[0])) < 10:
            self.Path_index += 1
        target_angle, Driver_mode = self.angle_mode_count(target_point)

        steer_r = self.Lateral_control.PID_control(target_angle)  # 返回横向控制的转角
        acceleration_r, v_now = self.Longitudinal_control.PID_control(target_point[2] * Driver_mode)  # 返回纵向控制的加速度
        # 横向控制限定范围
        if v_now < 0:  # 速度向后时，反向控制方向
            steer_r = 0 - steer_r
        if steer_r >= 0:
            self.vehicle.set_vehicle_steer(min(self.max_steer, steer_r))
        else:
            self.vehicle.set_vehicle_steer(max(self.min_steer, steer_r))
        time.sleep(0.1)
        # 纵向控制限定范围
        if acceleration_r >= 0:
            self.vehicle.control_vehicle(min(self.max_throttle, acceleration_r))
            time.sleep(0.1)
            self.vehicle.set_vehicle_brake(0)
            time.sleep(0.1)
        else:
            self.vehicle.control_vehicle(min(self.max_throttle, acceleration_r))
            time.sleep(0.1)
            self.vehicle.set_vehicle_brake(0)
            time.sleep(0.1)
            # self.vehicle.set_vehicle_brake(max(self.max_brake, acceleration_r))
        time.sleep(0.02)


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
                # 创建无人车实例
                vehicle = SwarmAEVehicle(node)
                vehicles.append(vehicle)

        leader_index = num // 2  # 选择中间车辆作为领导者
        leader_vehicle = vehicles[leader_index]

        # 让领导者车辆先行
        leader_vehicle.move_forward()
        time.sleep(0.25)  # 让领导者车辆先行

        # 1、箭形编队行进
        arrow_formation(vehicles)
        time.sleep(20)  # 等待一段时间以形成队形

        # 2、编队竖线阵型
        # 2.1、将跟随者小车的速度拉下来
        order = [1, 3, 0, 4]
        speed = 0.75
        # 创建一个线程池
        with ThreadPoolExecutor(max_workers=len(order)) as executor:
            # 创建一个字典来存储 future 对象和它们对应的速度
            futures = {}
            for i in order:
                if i != 3:
                    throttle_value = speed
                    futures[executor.submit(move_vehicle, vehicles[i], throttle_value)] = throttle_value
                else:
                    throttle_value = 0.3
                    futures[executor.submit(move_vehicle, vehicles[i], throttle_value)] = throttle_value
                speed -= 0.22  # 更新速度

            # 等待所有线程完成
            for future in concurrent.futures.as_completed(futures):
                throttle_value = futures[future]
                time.sleep(6)  # 等待 1 秒，模拟每个车辆移动的时间间隔
        time.sleep(6)

        # 生成路径点
        # 获取到桥头的位置
        timestamp, world, code = client.get_world()
        objects = world.get_environment_objects()
        # 桥信息
        bridges = list(filter(lambda x: (x.name.startswith('BP_Bridge')), objects))
        transform = bridges[0].transform
        bridge_location = (transform.location.x - 100, transform.location.y - 1)

        # 2.2、将每辆无人车移动到目标位置
        orderList = [2, 1, 3, 0, 4]
        threads = []

        # 启动线程
        for i in orderList:
            thread = threading.Thread(target=control_vehicle, args=(i, vehicles, bridge_location))
            threads.append(thread)
            thread.start()

        # 等待所有线程完成
        for thread in threads:
            thread.join()

    finally:
        print('main() end')


def control_vehicle(i, vehicles, bridge_location):
    node = vehicles[i].node
    target_path = create_move_points(node, bridge_location)

    # 车控制路线
    control_object = Vehicle_control(node, target_path)

    while True:
        # x, y, z, frame_timestamp = node.get_location()
        # if abs(x - bridge_location[0]) < 2 and abs(y - bridge_location[1]) < 2:
        #     break

        result = control_object.run_step()
        if not result:
            break
        time.sleep(0.10)


# 定义一个函数，用于执行车辆的移动操作
def move_vehicle(vehicle, throttle):
    vehicle.move_forward(throttle=throttle)


def arrow_formation(vehicles):
    """
    箭形编队行进
    :param vehicles:
    :return:
    """
    from concurrent.futures import ThreadPoolExecutor
    # 创建一个线程池
    with ThreadPoolExecutor(max_workers=len(vehicles)) as executor:
        # 将每个车辆的移动操作作为任务提交给线程池
        executor.map(move_vehicle, vehicles, [0.45, 0.82, 1, 0.82, 0.45])
    # 让上方设置的速度持续一段时间，实现箭形编队行进
    time.sleep(2)

    # 所有车辆同时加速
    with ThreadPoolExecutor(max_workers=len(vehicles)) as executor:
        # 将每个车辆的移动操作作为任务提交给线程池
        executor.map(move_vehicle, vehicles, [1, 1, 1, 1, 1])
    time.sleep(0.5)  # 稍微延迟以模拟实时处理
    print('arrow_formation() end')


def create_move_points(node, bridge_location):
    control_points = [
        bridge_location,  # 桥头位置
        (bridge_location[0] - 250, bridge_location[1]),  # 假设的中间控制点
        (node.get_location()[0], node.get_location()[1])  # 小车实时位置
    ]
    curve_points = bezierUtil.get_bezier_curve_points_2d(control_points, num_points=10)
    target_path = bezierUtil.format_points(curve_points, bridge_location)

    print(target_path)
    return target_path


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('__main__ end')

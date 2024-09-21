import time

import numpy as np


class SwarmAEVehicle:
    def __init__(self, node):
        """
        初始化无人车对象
        :param node: 无人车节点信息
        """
        self.node = node
        self.brake_value = 0  # 刹车力度，范围从 0 到 1
        self.m_brake = False  # 是否启用手刹, True 表示启用, False 表示不启用。
        self.steer_value = 0  # 转向角度，范围从 -1 到 1。
        self.throttle_value = 0  # 油门力度，范围从 -1 到 1。
        self.gear = 1  # 档位
        self.hand_brake = False  # 与 m_brake 类似，控制手刹状态。

    def move_forward(self, throttle=1):
        """
        移动到正前方
        """
        # 确保小车是朝向正前方的
        self.adjust_car()
        self.node.set_vehicle_steer(steer=0)
        time.sleep(0.1)
        self.node.control_vehicle(throttle=throttle)
        time.sleep(0.1)

    def move_left_forward(self, steer=-0.5):
        """
        移动到左前方
        """
        self.node.set_vehicle_steer(steer=steer)
        time.sleep(0.8)
        self.node.control_vehicle(throttle=0.8)
        time.sleep(3)
        # 调整小车，将车身调正
        self.adjust_car()

    def move_right_forward(self, steer=0.5):
        """
        移动到右前方
        """
        self.node.set_vehicle_steer(steer=steer)
        time.sleep(0.8)
        self.node.control_vehicle(throttle=0.8)
        time.sleep(3)
        # 调整小车，将车身调正
        self.adjust_car()

    def move_backward(self):
        """
        移动到正后方
        """
        self.node.set_vehicle_steer(steer=0)
        time.sleep(0.1)
        self.node.control_vehicle(throttle=-1)
        time.sleep(0.1)

    def move_left_backward(self, steer=-0.5):
        """
        移动到左后方
        """
        self.node.set_vehicle_steer(steer=steer)
        time.sleep(0.1)
        self.node.control_vehicle(throttle=-1)
        time.sleep(0.1)

    def move_right_backward(self, steer=0.5):
        """
        移动到右后方
        """
        self.node.set_vehicle_steer(steer=steer)
        time.sleep(0.1)
        self.node.control_vehicle(throttle=-1)
        time.sleep(0.1)

    def stop(self):
        """
        停止移动, 刹车需要先把control_vehicle输入0
        """
        self.node.set_vehicle_steer(steer=0)
        time.sleep(0.1)
        self.node.set_vehicle_brake(brake=1)
        time.sleep(0.1)

    def adjust_car(self):
        """
        调整小车，将车身调正
        """
        while abs(self.node.get_attitude()[0]) >= 0.09:
            raw = self.node.get_attitude()[0]
            print(raw)
            self.node.set_vehicle_steer(steer=-(raw / 90))
            time.sleep(0.1)
            self.node.control_vehicle(throttle=0.7)
            time.sleep(0.3)

    def move_to_position(self, target_position, duration=1):
        """
        移动小车到指定位置
        :param target_position: 目标位置 (x, y, z)
        :param duration: 移动持续时间 (秒)
        """
        # 获取小车当前位置
        x, y, z, frame_timestamp = self.node.get_location()
        current_position = (x, y, z)

        while current_position != target_position:
            print(f"当前位置: {current_position}, 目标位置: {target_position}")

            # 当 无人车移动到和 目标位置 同一y坐标时，角度回正
            if abs(current_position[1] - target_position[1]) < 0.45:
                print("回正")
                self.move_forward()
                break

            # 计算目标方向向量
            target_vector = np.array(target_position) - np.array(current_position)

            # 计算转向角度
            steer_angle = np.arctan2(target_vector[1], target_vector[0])

            # 转换转向角度到 [-1, 1] 范围
            steer_angle_normalized = steer_angle / (2 * np.pi)

            # 设置转向角度
            self.node.set_vehicle_steer(steer_angle_normalized)
            time.sleep(0.1)
            self.node.control_vehicle(throttle=0.4)
            time.sleep(duration)

            x, y, z, frame_timestamp = self.node.get_location()
            current_position = (x, y, z)

    def move_to_position_test(self, target_position, duration=2):
        """
        移动小车到指定位置
        :param target_position: 目标位置 (x, y, z)
        :param duration: 移动持续时间 (秒)
        """
        # 获取小车当前位置和朝向
        x, y, z, frame_timestamp = self.node.get_location()
        current_position = np.array([x, y, z])
        current_yaw = self.node.get_attitude()[0]  # 假设 yaw 是车辆朝向的角度

        while np.linalg.norm(current_position - np.array(target_position)) > 0.5:  # 设置一个接近阈值
            print(f"当前位置: {current_position}, 目标位置: {target_position}")

            # 计算目标方向向量
            target_vector = np.array(target_position) - current_position
            target_vector[2] = 0  # 忽略 z 轴的差异

            # 计算目标方向与车辆当前朝向之间的角度差
            target_yaw = np.arctan2(target_vector[1], target_vector[0])
            yaw_diff = target_yaw - current_yaw
            yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi  # 将角度差限制在 [-π, π]

            # 转换转向角度到 [-1, 1] 范围
            steer_angle_normalized = yaw_diff / np.pi

            # 设置转向角度，并限制在 [-1, 1] 范围内
            steer_angle_normalized = np.clip(steer_angle_normalized, -1, 1)
            self.node.set_vehicle_steer(steer_angle_normalized)

            # 控制车辆前进
            self.node.control_vehicle(throttle=1)
            time.sleep(duration)

            # 更新当前位置和朝向
            x, y, z, frame_timestamp = self.node.get_location()
            current_position = np.array([x, y, z])
            current_yaw = self.node.get_attitude()[0]  # 假设 yaw 是车辆朝向的角度

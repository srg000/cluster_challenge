import math
import time

import numpy as np


def move_to_position_test(node, target_position, duration=2):
    """
    移动小车到指定位置
    :param target_position: 目标位置 (x, y, z)
    :param duration: 移动持续时间 (秒)
    """
    # 获取小车当前位置和朝向
    x, y, z, frame_timestamp = node.get_location()
    current_position = np.array([x, y, z])
    current_yaw = node.get_attitude()[0]  # 假设 yaw 是车辆朝向的角度

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
        node.set_vehicle_steer(steer_angle_normalized)

        # 控制车辆前进
        node.control_vehicle(throttle=1)
        time.sleep(duration)

        # 更新当前位置和朝向
        x, y, z, frame_timestamp = node.get_location()
        current_position = np.array([x, y, z])
        current_yaw = node.get_attitude()[0]  # 假设 yaw 是车辆朝向的角度


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
            print('temp_obstacle_location:==================================',temp_obstacle[0].get('location'))
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
        [last_element[0] + 350, last_element[1], last_element[2] + 7, last_element[3]], dtype=object
    ).tolist()
    new_line.append(final_last)

    print('==============', new_line)
    return new_line


def bezier_curve_2d(points, t):
    """
    计算二维贝塞尔曲线上的点

    :param points: 控制点列表（二维坐标）
    :param t: 参数 t，范围 [0, 1]
    :return: 二维贝塞尔曲线上的点
    """
    n = len(points) - 1
    point = np.zeros(2)
    for i, p in enumerate(points):
        binomial_coeff = np.math.factorial(n) / (np.math.factorial(i) * np.math.factorial(n - i))
        point += binomial_coeff * (1 - t) ** (n - i) * t ** i * np.array(p)
    return point


def get_bezier_curve_points_2d(control_points, num_points=10):
    """
    获取二维贝塞尔曲线上的一系列点

    :param control_points: 控制点列表（二维坐标）
    :param num_points: 要生成的点的数量
    :return: 曲线上的点列表
    """
    curve_points = []
    for i in range(num_points):
        t = i / (num_points - 1)
        curve_point = bezier_curve_2d(control_points, t)
        curve_points.append(curve_point)
    return curve_points


# 示例使用二维贝塞尔曲线
control_points = [
    (-507.238892, 440.169159),  # 桥头位置
    # 你可以添加更多的控制点来定义曲线的形状
    (-700, 450),  # 假设的中间控制点
    (-854.2210693359375, 446.54058837890625)  # 小车实时位置
]

bezier_points = get_bezier_curve_points_2d(control_points, num_points=5)


def format_point(bezier_point, speed=10):
    return np.array([bezier_point[0], bezier_point[1], speed, 'D'], dtype=object).tolist()


def format_points(bezier_points, speed=6):
    formatted_points = []
    target_path = []
    # 打印二维贝塞尔曲线上的点
    for point in bezier_points:
        formatted_point = np.array([point[0], point[1], speed, 'D'], dtype=object).tolist()
        formatted_points.append(formatted_point)

    for point in formatted_points[::-1]:
        target_path.append(point)
    return target_path

# formatted_points = []
# target_path = []
# # 打印二维贝塞尔曲线上的点
# for point in bezier_points:
#     speed = 12 if point[0] <= -650 else 6
#     formatted_point = np.array([point[0], point[1], speed, 'D'], dtype=object).tolist()
#     formatted_points.append(formatted_point)
#
# for point in formatted_points[::-1]:
#     target_path.append(point)
# print(target_path)

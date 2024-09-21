#!/usr/bin/env python

import argparse
import time


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

        # sdk读取环境信息，需要 1.0.1 sdk 才支持
        timestamp, world, code = client.get_world()
        objects = world.get_environment_objects()

        # 两面墙位置
        wall = list(filter(lambda x: (x.name.startswith('SM_Wall_Single')), objects))

        # 拿到火力掩护区位置信息 和 墙体的长宽高
        print(wall[0])  # 火力掩护区
        print(wall[1])  # 敌方
        transform = wall[0].transform
        bounding_box = wall[0].bounding_box
        Wall_1_location = (transform.location.x - 12, transform.location.y - 3)
        wall_1_size = (bounding_box.extent.x, bounding_box.extent.y)
        print(f"wall_1_location: {Wall_1_location}, wall_1_size: {wall_1_size}")

        # 拿到敌方火里位置信息 和 墙体的长宽高
        transform_2 = wall[1].transform
        bounding_box_2 = wall[1].bounding_box
        Wall_2_location = (transform_2.location.x, transform_2.location.y)
        wall_2_size = (bounding_box_2.extent.x, bounding_box_2.extent.y)
        print(f"Wall_2_location: {Wall_2_location}, wall_2_size: {wall_2_size}")\


        # 获取4个固定障碍物
        obstacles = list(filter(lambda x: (x.name.startswith('SM_Obstacle')), objects))
        for i in range(len(obstacles)):
            print(obstacles[i])


        # 获取临时障碍物
        temp_obstacle = world.get_targets()
        print(temp_obstacle)


    finally:
        print('main() end')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('__main__ end')

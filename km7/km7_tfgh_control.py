#!/usr/bin/env python

import argparse
import random
import threading
import time
from os import times
import keyboard

from swarmae.SwarmAEClient import SwarmAEClient


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
        print(node_no)
        if node_no not in [11, 21]:
            print('请设置运行参数 -t ， 0 表示红队， 1 表示蓝队，示例： -t=0')
            return

        while True:

            print(type(vehicles[0].get_node_info()))
            print(vehicles[0].get_node_info())
            if keyboard.is_pressed('t'):
                # speed = current_speed  [3]*3.6
                # 0.8 55
                vehicles[0].control_vehicle(1)
            elif not keyboard.is_pressed('g'):
                vehicles[0].control_vehicle(0)

            if keyboard.is_pressed('g'):
                vehicles[0].control_vehicle(-1)
            elif not keyboard.is_pressed('t'):
                vehicles[0].control_vehicle(0)

            if keyboard.is_pressed('k'):
                vehicles[0].control_vehicle(0)
                vehicles[0].set_vehicle_brake(1)
                print("brake", end='')
            elif not keyboard.is_pressed('k'):
                vehicles[0].set_vehicle_brake(0)

            if keyboard.is_pressed('f'):
                vehicles[0].set_vehicle_steer(-1)
            elif not keyboard.is_pressed('h'):
                vehicles[0].set_vehicle_steer(0)

            if keyboard.is_pressed('h'):
                vehicles[0].set_vehicle_steer(1)
            elif not keyboard.is_pressed('f'):
                vehicles[0].set_vehicle_steer(0)

            time.sleep(0.01)

    finally:
        print('main')


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('done.')

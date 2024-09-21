#!/usr/bin/env python
import argparse
import time
import keyboard
import math
import threading

# 初始化速度变量
current_speed = 0.0

# 线程锁
lock = threading.Lock()

# 车列表，用于后续控制
nodes = []


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

        # location_thread = threading.Thread(target=get_speed)
        # location_thread.daemon = Truesd
        # location_thread.start()

        while True:
            print(type(nodes[1].get_node_info()))
            print(nodes[1].get_node_info())
            if keyboard.is_pressed('t'):
                # speed = current_speed  [3]*3.6
                # 0.8 55
                nodes[0].control_vehicle(1)
            elif not keyboard.is_pressed('g'):
                nodes[0].control_vehicle(0)

            if keyboard.is_pressed('g'):
                nodes[0].control_vehicle(-1)
            elif not keyboard.is_pressed('t'):
                nodes[0].control_vehicle(0)

            if keyboard.is_pressed('k'):
                nodes[0].control_vehicle(0)
                nodes[0].set_vehicle_brake(1)
                print("brake", end='')
            elif not keyboard.is_pressed('k'):
                nodes[0].set_vehicle_brake(0)

            if keyboard.is_pressed('f'):
                nodes[0].set_vehicle_steer(-1)
            elif not keyboard.is_pressed('h'):
                nodes[0].set_vehicle_steer(0)

            if keyboard.is_pressed('h'):
                nodes[0].set_vehicle_steer(1)
            elif not keyboard.is_pressed('f'):
                nodes[0].set_vehicle_steer(0)

            time.sleep(0.01)

    finally:
        print('main() end')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('__main__ end')

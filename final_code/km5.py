#!/usr/bin/env python

import argparse
import time
import math

DOORS = []
#获取障碍物位于无人机的方位(s结尾的表示无人机坐标，t结尾的表示障碍物坐标)
def getTargetDes(xs,ys,zs,xt,yt,zt):
    xd = xs - xt;
    yd = ys - yt
    zd = zs - zt
    des = "障碍物位于无人机的: "
    if(xd > 0):
        des = des + "后方"
    else:
        des = des + "前方"
    if(yd > 0):
        des = des + "," + "左方"
    else:
        des = des + ","  +"右方"
    if (zd > 0):
        des = des + "," + "下方"
    else:
        des = des + "," + "上方"
    return (des,xd,yd,zd);
#计算两点之间的距离(s结尾的表示无人机坐标，t结尾的表示障碍物坐标)
def calcDistance(xs,ys,zs,xt,yt,zt):
    xd = xs - xt;
    yd = ys - yt
    zd = zs - zt
    D = math.sqrt(xd*xd + yd*yd +  zd * zd)
    return D

#将无人机移动到指定的目标点（x,y,z,v是目标点的坐标和速度）
def moveFlyerTotarget(node,x,y,z,v):
    flag = 0
    xs, ys, zs, frame_timestamp = node.get_location()
    node.control_kinetic_simply_global(x, y, z, v, frame_timestamp)
    while True:
        xs, ys, zs, frame_timestamp = node.get_location()
        # d = calcDistance(xs,ys,zs,x,y,z);
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        if (v3 != 0):   #速度不为0并且距离大于两米认为飞机正在飞行
            flag = 1
            print("moveFlyerTotarget飞机正在飞行" +  "," + str(v3))
            hs = node.detect_hostile_list()
            print("moveFlyerTotarget障碍物的数量{}".format(len(hs)))
            for h in hs:
                print("moveFlyerTotarget障碍物:")
                print(h[1].location)
                x = h[1].location.x
                y = h[1].location.y
                z = h[1].location.z
                print((x, y, z))
            # time.sleep(0.001)

        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1 ):  #速度为0并且距离小于两米认为飞机到达目的地
            flag = 0
            print("moveFlyerTotarget飞机到达目标点"  + "," + str(v3))
            break



def moveToDoor1():
    pass


def moveCrossDoor1():
    pass


def moveToDoor2():
    pass

def moveCrossDoor2():
    pass






# print(DOORS)
# exit()
# final_x,final_y,final_z = 2093.00005859375,-110.19094848632812,501.50537109375
V = 30
flag_first = 0;
def move(node,Door1_adress,Door2_adress,Final_X,y_offset,num):
    global flag_first,V
    # print(nodes[0].get_weapon()[1].get_weapon_ammo())
    time.sleep(num * 10)  # 每辆无人机每隔8秒起飞
    flag = 0
    node.control_kinetic_simply_global(Door1_adress[0] - 10 ,Door1_adress[1] , Door1_adress[2],30, node.get_location()[3])
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, Door1_adress[0] - 10, Door1_adress[1], Door1_adress[2]);
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            if (num == 0 and flag_first == 0):  # 第一辆飞机开始起飞
                flag_first = 1
                time.sleep(10)


            print("去a的飞机开始起飞")
            hs = node.detect_hostile_list()
            print("去a的障碍物的数量{}".format(len(hs)))
            for h in hs:
                print("去a的障碍物:")
                print(h[1].location)
                x = h[1].location.x
                y = h[1].location.y
                z = h[1].location.z
                print((x, y, z))
            # time.sleep(0.001)

        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1 and d <= 3):
            flag = 0
            print("飞机到达A点")
            break
    moveFlyerTotarget(node, Door1_adress[0], Door1_adress[1], Door1_adress[2], 50)
    flag = 0
    node.control_kinetic_simply_global(Door2_adress[0] - 20, Door2_adress[1], Door2_adress[2], 50, node.get_location()[3])
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        # d = calcDistance(xs, ys, zs, DOORS[1][0] - 10, DOORS[1][1], DOORS[1][2]+40);
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            print("飞机开始起飞去b")
            hs = node.detect_hostile_list()
            print("去b的路上障碍物的数量{}".format(len(hs)))
            for h in hs:
                print("障碍物2:")
                print(h[1].location)
                x = h[1].location.x
                y = h[1].location.y
                z = h[1].location.z
                print((x, y, z))
            # time.sleep(0.001)

        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
            flag = 0
            print("飞机到达B点")
            break
    moveFlyerTotarget(node, Door2_adress[0], Door2_adress[1], Door2_adress[2], 50)

    # while True:
    #     v = node.get_velocity()
    #     v3 = math.floor(abs(v[3]))
    #     v0 = math.floor(abs(v[0]))
    #     v1 = math.floor(abs(v[1]))
    #     v2 = math.floor(abs(v[2]))
    #     time.sleep(0.001)
    #     if (v3 != 0):
    #         flag = 1
    #         print("飞机开始起飞")
    #
    #     if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
    #         print("飞机到达终点")
    #
    #         flag = 0
    #         break
    node.control_kinetic_simply_global(Final_X, Door2_adress[1] + y_offset, Door2_adress[2], V, node.get_location()[3])



# def move_to_fianl(node,y_offset):
#     global final_x,final_y,final_z,V
#
#     flag = 0
#     node.control_kinetic_simply_global(final_x+200, final_y+y_offset- y, final_z, V, node.get_location()[3])
#     while True:
#         v = node.get_velocity()
#         v3 = math.floor(abs(v[3]))
#         v0 = math.floor(abs(v[0]))
#         v1 = math.floor(abs(v[1]))
#         v2 = math.floor(abs(v[2]))
#         if (v3 != 0):
#             flag = 1
#             print("飞机开始起飞")
#
#         if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
#             print("飞机到达终点")
#
#             flag = 0
#             break
#

import threading
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
        default=10)
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
        # 列表，用于后续控制
        nodes = []

        # 1. 首先引入sdk
        from swarmae.SwarmAEClient import SwarmAEClient
        # 2. 连接UE
        client = SwarmAEClient(ue_ip=ue_ip, ue_port=ue_port)

        # sdk读取环境信息，需要 1.0.1 sdk 才支持
        timestamp, world, code = client.get_world()
        objects = world.get_environment_objects()
        # 线信息
        BP_Trees = list(filter(lambda x: (x.name.startswith('BP_Trees')), objects))
        print(BP_Trees)
        print("\n", BP_Trees[0].transform.location.x)
        Start_X = BP_Trees[0].transform.location.x
        Final_X = Start_X + 2003  # 获取不到终点线，但是可以获取起点线，并且规定赛道2000m，设置最终点于起点线+2003

        # 门1，门2的坐标信息
        BP_DoorInst = list(filter(lambda x: (x.name.startswith('BP_DoorInst')), objects))
        print(BP_DoorInst)
        Door2_adress = (BP_DoorInst[0].transform.location.x, BP_DoorInst[0].transform.location.y, BP_DoorInst[0].transform.location.z)
        print(Door2_adress)
        Door1_adress = (BP_DoorInst[1].transform.location.x, BP_DoorInst[1].transform.location.y, BP_DoorInst[1].transform.location.z)
        print(Door1_adress)


        # 注册车辆
        for id in range(num):
            # node_no 从1开始编号
            timestamp, node, code = client.register_node(
                node_type="四旋翼", node_name="节点" + str(id+1), node_no=id+1, frame_timestamp=int(round(time.time() * 1000))
            )
            if code == 200:
                # 获取信息
                node_name, node_no, _, node_type, node_model, color = node.get_node_info()
                nodes.append(node)



        init_num = 0;
        for node in nodes:
            init_offset_y = -40 + init_num * 10
            init_num = init_num + 1
            # 第一辆飞机
            if (init_num == 1):
                t = threading.Thread(target=move, args=(node,Door1_adress,Door2_adress,Final_X, init_offset_y,init_num - 1))
                t.start()

            # 开始等待第一辆飞机起飞
            while flag_first == 0:
                time.sleep(0.001)
                print("当前第一辆飞机是否起飞" + str(flag_first))
                pass
            # 第一辆飞机已经起飞
            if (flag_first == 1 and init_num >= 2):
                t = threading.Thread(target=move, args=(node,Door1_adress,Door2_adress,Final_X, init_offset_y,init_num - 1))
                t.start()
            time.sleep(0.001)
            # if
            # t = threading.Thread(target=move_to_target, args=(node, init_offset_x, init_offset_y, init_num - 1))
            # t.start()
            # move_to_target(node, init_offset_x, init_offset_y)
        t.join()  # 等待所有飞机完成编队







    finally:
        print('main() end')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('__main__ end')
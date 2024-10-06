#!/usr/bin/env python

import argparse
import time
import math


# from fbx_reader import Scenario
#
# scenario = Scenario()
# Waypoints = []
# # 以名字筛选静态资源
# nodes = scenario.get_nodes_by_regex(r'Waypoint')
#
# for name in nodes.keys():
#     node = nodes[name]
#     if not node.is_mesh():
#         continue
#     # door.get_transform_info()
#     vertices = node.get_vertices()
#     trans = node.get_global_transform_matrix()
#     v_g = []
#     if len(vertices) == 0:
#         continue
#     for v in vertices:
#         # print(v[0], v[1], v[2])
#         v_g.append(trans.MultT(v))
#         ## print(v_g[-1][0], v_g[-1][1], v_g[-1][2])
#         ## print()
#
#     xmin = v_g[0][0]
#     xmax = v_g[0][0]
#     ymin = v_g[0][1]
#     ymax = v_g[0][1]
#     zmin = v_g[0][2]
#     zmax = v_g[0][2]
#
#     for v in v_g:
#         xmin = xmin if v[0] > xmin else v[0]
#         xmax = xmax if v[0] < xmax else v[0]
#         ymin = ymin if v[1] > ymin else v[1]
#         ymax = ymax if v[1] < ymax else v[1]
#         zmin = zmin if v[2] > zmin else v[2]
#         zmax = zmax if v[2] < zmax else v[2]
#
#     # 节点的中心点坐标（已换算成UE中单位）
#     x = (xmax + xmin) / 2 / 100
#     y = (ymin + ymax) / 2 / 100
#     z = (zmin + zmax) / 2 / 100
#     Waypoints.append([x,-y,z])
#     print(f"{name}: {[xmin, xmax, ymin, ymax, zmin, zmax]}, vertices len: {len(vertices)}\n"
#           f"x: {x}, y: {y}, z: {z}")
#
# A_X,A_Y,A_Z = Waypoints[0]
# B_X,B_Y,B_Z = Waypoints[1]
A_X,A_Y,A_Z = (0,0,0)
A_X,A_Y,A_Z = (0,0,0)
V = 50
# x_first, y_first, z_first = 0,0,0
# flag_first = 0  #只有第一辆飞机开始飞的情况下才打开其他线程
# flag_first2 = 0
import  time
def calcDistance(xs,ys,zs,xt,yt,zt):
    xd = xs - xt
    yd = ys - yt
    zd = zs - zt
    D = math.sqrt(xd*xd + yd*yd)
    return D
def move_to_targetA(node, targetA_adress,x_offset, y_offset, z_offset ,num):
    global A_X, A_Y, A_Z
    # time.sleep(num * 16)  # 每辆无人机每隔5秒起飞
    # x, y, z, frame_timestamp = node.get_location()
    A_X, A_Y, A_Z = targetA_adress
    print("第{}两无人机起飞".format(num))
    print("第{}两无人机x_offset={},y_offset={},z_offset={}".format(num, x_offset, y_offset, z_offset))
    # x, y, z, frame_timestamp = node.get_location()
    flag = 0
    # 第一辆  x+10是在前方，y-10是在左方

    # 无人机同时起飞到目标高度，但彼此有高度差
    node.control_kinetic_simply(0, 0, 25+z_offset, V, node.get_location()[3])

    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1

            print("飞机正在调整高度")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
            flag = 0
            print("飞机到达高度点，出发前往A点")
            break

    node.control_kinetic_simply_global(A_X, A_Y, A_Z+25+z_offset, V, node.get_location()[3])
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        time.sleep(0.001)
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, A_X , A_Y, A_Z+25+z_offset)
        print("路径{}: 距离A:{}".format("A", d))
        if (v3 != 0):
            flag = 1

            print("飞机正在飞往A点")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
            if (d >= 2):
                node.control_kinetic_simply_global(A_X, A_Y, A_Z+25+z_offset, V, node.get_location()[3])
            else:
                flag = 0
                print("飞机到达A点")
                break

    node.control_kinetic_simply_global(A_X + x_offset + 50 , A_Y + y_offset + 50 , A_Z+25, V, node.get_location()[3])  #
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        time.sleep(0.001)
        # xs, ys, zs, frame_timestamp = node.get_location()
        # d = calcDistance(xs, ys, zs, A_X + x_offset + 50, A_Y + y_offset + 50, A_Z + 25 + z_offset)

        if (v3 != 0):
            flag = 1
            print("飞机开始编队")

        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0  and flag == 1):
            print("飞机到达编队点")

            flag = 0
            break


def move_to_target(node, x_offset, y_offset,num):
    global x_first, y_first, z_first,flag_first,A_X,A_Y,A_Z
    time.sleep(num * 16)  #每辆无人机每隔5秒起飞

    print("第{}两无人机起飞".format(num))
    print("第{}两无人机x_offset={},y_offset={}".format(num,x_offset,y_offset))
    x, y, z, frame_timestamp = node.get_location()
    flag = 0
    # 第一辆  x+10是在前方，y-10是在左方
    node.control_kinetic_simply(0, 0, 25, V, node.get_location()[3])


    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        if (v3 != 0):
            flag = 1
            if(num == 0 and flag_first == 0):  #第一辆飞机开始起飞
                flag_first = 1
                time.sleep(10)
            print("飞机开始起飞")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
            flag = 0
            print("飞机到达高度点，出发前往A点")
            break


    node.control_kinetic_simply(A_X - x, A_Y- y, A_Z, V, node.get_location()[3])
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        if (v3 != 0):
            flag = 1
            print("飞机开始起飞")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
            flag = 0
            print("飞机到达A点")
            break

    node.control_kinetic_simply(A_X+x_offset+50 - x, A_Y+y_offset+50- y, A_Z, V, node.get_location()[3])#
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        if (v3 != 0):
            flag = 1
            print("飞机开始起飞")

        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
            print("飞机到达编队点")
           # if(flag_first==0):
           #     x, y, z, frame_timestamp = node.get_location()
           #     x_first,y_first,z_first = x,y,z        #以第一个飞机编队坐标为基准点，后续无人机在这个点击的基础上进行
           #     flag_first = 1
            # x, y, z, frame_timestamp = nodes[0].get_location()
            # current_position = (x, y, z)
            # print(current_position)
            # flag_first = 1
            flag = 0
            break


def move_to_targetB(node ,targetB_adress,Final_X,z_offset , num):
    global B_X,B_Y,B_Z
    B_X,B_Y,B_Z = targetB_adress
    print("第{}两无人机起飞".format(num))
    print("第{}两无人机x_offset={},y_offset={},z_offset={}".format(num, 0, 0, z_offset))
    flag = 0
    node.control_kinetic_simply_global(node.get_location()[0], node.get_location()[1], node.get_location()[2]+5 + z_offset, V, node.get_location()[3])
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        time.sleep(0.001)


        if (v3 != 0):
            flag = 1

            print("飞机正在调整高度")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
            flag = 0
            print("飞机到达高度点，出发前往B点")
            break

    # node.control_kinetic_simply_global(B_X - 20, B_Y, B_Z + 26 + z_offset, V, node.get_location()[3])
    #
    # while True:
    #     v = node.get_velocity()
    #     v3 = math.floor(abs(v[3]))
    #     v0 = math.floor(abs(v[0]))
    #     v1 = math.floor(abs(v[1]))
    #     v2 = math.floor(abs(v[2]))
    #     time.sleep(0.001)
    #     # xs, ys, zs, frame_timestamp = node.get_location()
    #     # d = calcDistance(xs, ys, zs, B_X, B_Y, B_Z + 26 + z_offset)
    #     # print("路径{}: 距离:{}".format("B", d))
    #     if (v3 != 0):
    #         flag = 1
    #         # node.control_kinetic_simply_global(B_X, B_Y, B_Z + 26 + z_offset, V, node.get_location()[3])
    #         print("飞机正在飞往B点前20m")
    #     if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
    #         # print("飞机:{} 当前坐标:{} B点坐标{} ".format(num, node.get_location(), (B_X, B_Y, B_Z)))
    #
    #
    #         flag = 0
    #         print("飞机到达B点")
    #         break


    node.control_kinetic_simply_global(B_X - 20, B_Y, B_Z+25+z_offset, V, node.get_location()[3])

    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        time.sleep(0.001)
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, B_X, B_Y, B_Z+25+z_offset)
        print("路径{}: 距离:{}".format("B", d))
        if (v3 != 0):
            flag = 1
            # node.control_kinetic_simply_global(B_X, B_Y, B_Z + 26 + z_offset, V, node.get_location()[3])
            print("飞机正在飞往B点")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0  and d<30 and flag == 1):
            flag = 0
            print("飞机到达B点")
            break
            print("飞机:{} 当前坐标:{} B点坐标{} ".format(num, node.get_location(), (B_X, B_Y, B_Z)))
            # if(d >= 2):
            #     pass
            #     # print("飞机:{} 当前坐标:{} B点坐标{} ".format(num, node.get_location(),(B_X, B_Y, B_Z)))
            # else:
            #     flag = 0
            #     print("飞机到达B点")
            #     break

    node.control_kinetic_simply_global(Final_X , B_Y, B_Z+26+z_offset, V, node.get_location()[3])
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, B_X, B_Y, B_Z + 26 + z_offset)
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            print("飞机开始起飞")
            print("飞机:{} 距离：{} 当前坐标:{} B点坐标{} ".format(num, d , node.get_location(), (B_X, B_Y, B_Z)))
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
            print("飞机到达终点")

            flag = 0
            break

import threading
def main():
    global flag_first
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
        Final_X = Start_X + 2003 #获取不到终点线，但是可以获取起点线，并且规定赛道2000m，设置最终点于起点线+2003

        Waypoints = list(filter(lambda x: (x.name.startswith('Waypoint')), objects))
        print(Waypoints)
        targetA_adress = (Waypoints[0].transform.location.x,Waypoints[0].transform.location.y,Waypoints[0].transform.location.z)
        print(targetA_adress)
        targetB_adress = (Waypoints[1].transform.location.x,Waypoints[1].transform.location.y,Waypoints[1].transform.location.z)
        print(targetB_adress)

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



        # print(nodes[0].get_weapon()[1].get_weapon_ammo())

        init_offset_x = 0
        init_offset_y = 0
        init_offset_z = 0
        init_num = 0
        init_num1 = 0
        init_num2 = 0



        t = None
        for node in nodes:
            # 计算偏移量，用于排队
            init_offset_x = 0
            init_offset_y = 0
            init_offset_z = init_num*7
            if(init_num<5):
                init_offset_x = init_num*10
            elif(init_num >= 5):
                init_offset_x = init_num1*10
                init_offset_y = -10
                init_num1 = init_num1 + 1
            init_num = init_num + 1


            t = threading.Thread(target=move_to_targetA,args=(node,targetA_adress,init_offset_x,init_offset_y,init_offset_z,init_num - 1))
            t.start()
            time.sleep(0.001)

        t.join() #等待所有飞机完成编队

        time.sleep(10)

        t = None
        init_num3 = 0
        # 依次前往B点
        for node in nodes:
            # move_to_targetB(node,x_local[init_num3],y_local[init_num3])
            init_offset_z = init_num3*6
            init_num3 = init_num3 + 1

            t = threading.Thread(target=move_to_targetB, args=(node, targetB_adress,Final_X ,init_offset_z ,init_num3 -1))
            t.start()
            time.sleep(0.001)

        t.join()  # 等待所有飞机完成编队

        # while True:
        #     time.sleep(1)




    finally:
        print('main() end')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('__main__ end')
#!/usr/bin/env python

import argparse
import time
import math,threading

client1 = None;
flag_first = 0;
def getH():
    # global client1
    #
    # world = client1.get_world()[1]
    # targets = world.get_targets()
    # for t in targets:
    #     print("目标生命数值{}".format(t["health_point"]))
    #
    # print("*" * 50)
    pass

def calcDistance(xs,ys,zs,xt,yt,zt):
    xd = xs - xt;
    yd = ys - yt
    zd = zs - zt
    D = math.sqrt(xd*xd + yd*yd +  zd * zd)
    return D

def P1_P4_0_2(node ,PATH ,ids ,Final_X ,z_offset, num):
    global flag_first
    time.sleep(10 * num)
    flag = 0
    node.control_kinetic_simply_global(node.get_location()[0], PATH["p1"][1], node.get_location()[2] + 50, 50, time.time())
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, node.get_location()[0], PATH["p1"][1], node.get_location()[2] + 50)
        print("路径{}: 距离:{}".format("p1_y", d))
        time.sleep(0.001)
        if (v3 != 0):
            if (num == 0 and flag_first == 0):  # 第一辆飞机开始起飞
                flag_first = 1
                #time.sleep(10)
            flag = 1
            getH()
            #print("飞机正在出发前往p1_y")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d<100 and flag == 1):
            flag = 0
            print("飞机到达p1_y")
            break
    node.control_kinetic_simply_global(PATH["p1"][0] - 10, PATH["p1"][1],
                                           PATH["p1"][2] + 50, 50, time.time())


    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, PATH["p1"][0] - 10, PATH["p1"][1], PATH["p1"][2] + 50)
        print("路径{}: 距离:{}".format("p1", d))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            # print(normal_targets[1][1])
            print("飞机正在出发前往P1")
            print("ids长度：", len(ids))
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 10 and flag == 1):
            flag = 0
            print("飞机到达P1")
            break

    # node为注册的无人机
    # 获取车辆武器
    weapon = node.get_weapon()[1]
    # 瞄准目标
    yaw,pitch,roll,heading,frame_timestamp = node.get_attitude()

    weapon.set_weapon_angle(yaw, pitch, time.time())
    # 无人机开火
    # 获取打击目标id,具体坐标信息等可查看具体get_targets方法，这里仅展示id获取部分
    # world = client.get_world()[1]
    # targets = world.get_targets()
    # ids = []
    # for target in targets:
    #     ids.append(target["id"])
    # weapon类方法missle_fire负责瞄准打击目标，参数为打击目标的id（通过get_targets方法获得）
    print("ids长度：",len(ids))
    weapon.missile_fire(ids[3])
    # node类方法gun_fire负责开火，参数为开火次数，注意规则中荷载武器上限
    node.gun_fire(1)
    print(num,"号无人机已开火")


    node.control_kinetic_simply_global(PATH["p4"][0] - 10, PATH["p4"][1],
                                       PATH["p4"][2] + 50, 50, time.time())
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, PATH["p4"][0] - 10, PATH["p4"][1], PATH["p4"][2] + 50)
        print("路径{}: 距离:{}".format("p4", d))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            # print(normal_targets[1][1])
            getH()
            #print("飞机正在出发前往P4")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 10 and flag == 1):
            flag = 0
            print("飞机到达P4")
            break

    # node为注册的无人机
    # 获取车辆武器
    weapon = node.get_weapon()[1]
    # 瞄准目标
    yaw, pitch, roll, heading, frame_timestamp = node.get_attitude()

    weapon.set_weapon_angle(yaw, pitch, time.time())
    # 无人机开火
    # 获取打击目标id,具体坐标信息等可查看具体get_targets方法，这里仅展示id获取部分
    # world = client.get_world()[1]
    # targets = world.get_targets()
    # ids = []
    # for target in targets:
    #     ids.append(target["id"])
    # weapon类方法missle_fire负责瞄准打击目标，参数为打击目标的id（通过get_targets方法获得）
    weapon.missile_fire(ids[4])
    # node类方法gun_fire负责开火，参数为开火次数，注意规则中荷载武器上限
    node.gun_fire(1)
    getH()

    print(num, "号无人机已开火")

    node.control_kinetic_simply_global(Final_X, PATH["p4"][1] , PATH["p4"][2] + z_offset, 50,
                                           node.get_location()[3])


def P2_P5_3_4(node ,PATH ,ids ,Final_X ,z_offset, num):
    global flag_first
    time.sleep(10 * num)
    flag = 0
    node.control_kinetic_simply_global(node.get_location()[0], PATH["p2"][1], node.get_location()[2] + 50, 50,
                                       time.time())
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, node.get_location()[0], PATH["p2"][1], node.get_location()[2] + 50)
        print("路径{}: 距离:{}".format("p2_y", d))
        time.sleep(0.001)
        if (v3 != 0):
            if (num == 0 and flag_first == 0):  # 第一辆飞机开始起飞
                flag_first = 1
                # time.sleep(10)
            flag = 1
            getH()
            # print("飞机正在出发前往p2_y")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 100 and flag == 1):
            flag = 0
            print("飞机到达p2_y")
            break
    node.control_kinetic_simply_global(PATH["p2"][0] - 10, PATH["p2"][1],
                                       PATH["p2"][2] + 50, 50, time.time())

    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, PATH["p2"][0] - 10, PATH["p2"][1], PATH["p2"][2] + 50)
        print("路径{}: 距离:{}".format("p2", d))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            # print(normal_targets[1][1])
            print("飞机正在出发前往P2")
            print("ids长度：", len(ids))
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 10 and flag == 1):
            flag = 0
            print("飞机到达P2")
            break

    # node为注册的无人机
    # 获取车辆武器
    weapon = node.get_weapon()[1]
    # 瞄准目标
    yaw, pitch, roll, heading, frame_timestamp = node.get_attitude()

    weapon.set_weapon_angle(yaw, pitch, time.time())
    # 无人机开火
    # 获取打击目标id,具体坐标信息等可查看具体get_targets方法，这里仅展示id获取部分
    # world = client.get_world()[1]
    # targets = world.get_targets()
    # ids = []
    # for target in targets:
    #     ids.append(target["id"])
    # weapon类方法missle_fire负责瞄准打击目标，参数为打击目标的id（通过get_targets方法获得）
    print("ids长度：", len(ids))
    weapon.missile_fire(ids[6])
    # node类方法gun_fire负责开火，参数为开火次数，注意规则中荷载武器上限
    node.gun_fire(1)
    print(num, "号无人机已开火")

    node.control_kinetic_simply_global(PATH["p5"][0] - 10, PATH["p5"][1],
                                       PATH["p5"][2] + 50, 50, time.time())
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, PATH["p5"][0] - 10, PATH["p5"][1], PATH["p5"][2] + 50)
        print("路径{}: 距离:{}".format("p5", d))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            # print(normal_targets[1][1])
            getH()
            # print("飞机正在出发前往p5")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 10 and flag == 1):
            flag = 0
            print("飞机到达p5")
            break

    # node为注册的无人机
    # 获取车辆武器
    weapon = node.get_weapon()[1]
    # 瞄准目标
    yaw, pitch, roll, heading, frame_timestamp = node.get_attitude()

    weapon.set_weapon_angle(yaw, pitch, time.time())
    # 无人机开火
    # 获取打击目标id,具体坐标信息等可查看具体get_targets方法，这里仅展示id获取部分
    # world = client.get_world()[1]
    # targets = world.get_targets()
    # ids = []
    # for target in targets:
    #     ids.append(target["id"])
    # weapon类方法missle_fire负责瞄准打击目标，参数为打击目标的id（通过get_targets方法获得）
    weapon.missile_fire(ids[7])
    # node类方法gun_fire负责开火，参数为开火次数，注意规则中荷载武器上限
    node.gun_fire(1)
    getH()

    print(num, "号无人机已开火")

    node.control_kinetic_simply_global(Final_X, PATH["p5"][1], PATH["p5"][2] + z_offset, 50,
                                       node.get_location()[3])

def P3_P6_5(node ,PATH ,ids ,Final_X ,z_offset, num):
    global flag_first
    time.sleep(10 * num)
    flag = 0
    node.control_kinetic_simply_global(node.get_location()[0], PATH["p3"][1], node.get_location()[2] + 50, 50,
                                       time.time())
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, node.get_location()[0], PATH["p3"][1], node.get_location()[2] + 50)
        print("路径{}: 距离:{}".format("p3_y", d))
        time.sleep(0.001)
        if (v3 != 0):
            if (num == 0 and flag_first == 0):  # 第一辆飞机开始起飞
                flag_first = 1
                # time.sleep(10)
            flag = 1
            getH()
            # print("飞机正在出发前往p3_y")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 100 and flag == 1):
            flag = 0
            print("飞机到达p3_y")
            break
    node.control_kinetic_simply_global(PATH["p3"][0] - 10, PATH["p3"][1],
                                       PATH["p3"][2] + 50, 50, time.time())

    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, PATH["p3"][0] - 10, PATH["p3"][1], PATH["p3"][2] + 50)
        print("路径{}: 距离:{}".format("p3", d))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            # print(normal_targets[1][1])
            print("飞机正在出发前往p3")
            print("ids长度：", len(ids))
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 10 and flag == 1):
            flag = 0
            print("飞机到达p3")
            break

    # node为注册的无人机
    # 获取车辆武器
    weapon = node.get_weapon()[1]
    # 瞄准目标
    yaw, pitch, roll, heading, frame_timestamp = node.get_attitude()

    weapon.set_weapon_angle(yaw, pitch, time.time())
    # 无人机开火
    # 获取打击目标id,具体坐标信息等可查看具体get_targets方法，这里仅展示id获取部分
    # world = client.get_world()[1]
    # targets = world.get_targets()
    # ids = []
    # for target in targets:
    #     ids.append(target["id"])
    # weapon类方法missle_fire负责瞄准打击目标，参数为打击目标的id（通过get_targets方法获得）
    print("ids长度：", len(ids))
    weapon.missile_fire(ids[1])
    # node类方法gun_fire负责开火，参数为开火次数，注意规则中荷载武器上限
    node.gun_fire(1)
    print(num, "号无人机已开火")

    node.control_kinetic_simply_global(PATH["p6"][0] - 10, PATH["p6"][1],
                                       PATH["p6"][2] + 50, 50, time.time())
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, PATH["p6"][0] - 10, PATH["p6"][1], PATH["p6"][2] + 50)
        print("路径{}: 距离:{}".format("p6", d))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            # print(normal_targets[1][1])
            getH()
            # print("飞机正在出发前往p6")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 10 and flag == 1):
            flag = 0
            print("飞机到达p6")
            break

    # node为注册的无人机
    # 获取车辆武器
    weapon = node.get_weapon()[1]
    # 瞄准目标
    yaw, pitch, roll, heading, frame_timestamp = node.get_attitude()

    weapon.set_weapon_angle(yaw, pitch, time.time())
    # 无人机开火
    # 获取打击目标id,具体坐标信息等可查看具体get_targets方法，这里仅展示id获取部分
    # world = client.get_world()[1]
    # targets = world.get_targets()
    # ids = []
    # for target in targets:
    #     ids.append(target["id"])
    # weapon类方法missle_fire负责瞄准打击目标，参数为打击目标的id（通过get_targets方法获得）
    weapon.missile_fire(ids[9])
    # node类方法gun_fire负责开火，参数为开火次数，注意规则中荷载武器上限
    node.gun_fire(1)
    getH()

    print(num, "号无人机已开火")

    node.control_kinetic_simply_global(Final_X, PATH["p6"][1], PATH["p6"][2] + z_offset, 50,
                                       node.get_location()[3])

def P6_5_6(node ,PATH ,ids ,Final_X ,z_offset, num):
    global flag_first
    time.sleep(10 * num)
    flag = 0
    node.control_kinetic_simply_global(node.get_location()[0], PATH["p6"][1], node.get_location()[2] + 50, 50,
                                       time.time())
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, node.get_location()[0], PATH["p6"][1], node.get_location()[2] + 50)
        print("路径{}: 距离:{}".format("p6_y", d))
        time.sleep(0.001)
        if (v3 != 0):
            if (num == 0 and flag_first == 0):  # 第一辆飞机开始起飞
                flag_first = 1
                # time.sleep(10)
            flag = 1
            getH()
            # print("飞机正在出发前往p6_y")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 100 and flag == 1):
            flag = 0
            print("飞机到达p6_y")
            break
    node.control_kinetic_simply_global(PATH["p6"][0] - 10, PATH["p6"][1],
                                       PATH["p6"][2] + 50, 50, time.time())

    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, PATH["p6"][0] - 10, PATH["p6"][1], PATH["p6"][2] + 50)
        print("路径{}: 距离:{}".format("p6", d))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            # print(normal_targets[1][1])
            print("飞机正在出发前往p6")
            print("ids长度：", len(ids))
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 10 and flag == 1):
            flag = 0
            print("飞机到达p6")
            break

    # node为注册的无人机
    # 获取车辆武器
    weapon = node.get_weapon()[1]
    # 瞄准目标
    yaw, pitch, roll, heading, frame_timestamp = node.get_attitude()

    weapon.set_weapon_angle(yaw, pitch, time.time())
    # 无人机开火
    # 获取打击目标id,具体坐标信息等可查看具体get_targets方法，这里仅展示id获取部分
    # world = client.get_world()[1]
    # targets = world.get_targets()
    # ids = []
    # for target in targets:
    #     ids.append(target["id"])
    # weapon类方法missle_fire负责瞄准打击目标，参数为打击目标的id（通过get_targets方法获得）
    print("ids长度：", len(ids))
    weapon.missile_fire(ids[9])
    # node类方法gun_fire负责开火，参数为开火次数，注意规则中荷载武器上限
    node.gun_fire(1)
    print(num, "号无人机已开火")



    node.control_kinetic_simply_global(Final_X, PATH["p6"][1], PATH["p6"][2] + z_offset, 50,
                                       node.get_location()[3])

def P7_7_9(node ,PATH ,ids ,Final_X ,z_offset , num):
    global flag_first
    time.sleep(10 * num)
    flag = 0
    node.control_kinetic_simply_global(node.get_location()[0], PATH["p4"][1], node.get_location()[2] + 50, 50,
                                       time.time())
    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, node.get_location()[0], PATH["p4"][1], node.get_location()[2] + 50)
        print("路径{}: 距离:{}".format("p4_y", d))
        time.sleep(0.001)
        if (v3 != 0):
            if (num == 0 and flag_first == 0):  # 第一辆飞机开始起飞
                flag_first = 1
                # time.sleep(10)
            flag = 1
            getH()
            # print("飞机正在出发前往p4_y")
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 100 and flag == 1):
            flag = 0
            print("飞机到达p4_y")
            break
    node.control_kinetic_simply_global(PATH["p4"][0] , PATH["p4"][1],
                                       PATH["p4"][2] + 50, 50, time.time())

    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, PATH["p4"][0] , PATH["p4"][1], PATH["p4"][2] + 50)
        print("路径{}: 距离:{}".format("p4", d))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            # print(normal_targets[1][1])
            print("飞机正在出发前往p4")
            print("ids长度：", len(ids))
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 10 and flag == 1):
            flag = 0
            print("飞机到达p4")
            break

    node.control_kinetic_simply_global(PATH["p7"][0], PATH["p7"][1],
                                       PATH["p7"][2] + 50, 50, time.time())

    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, PATH["p7"][0], PATH["p7"][1], PATH["p7"][2] + 50)
        print("路径{}: 距离:{}".format("p7", d))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            # print(normal_targets[1][1])
            print("飞机正在出发前往p7")
            print("ids长度：", len(ids))
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 10 and flag == 1):
            flag = 0
            print("飞机到达p7")
            break

    # node为注册的无人机
    # 获取车辆武器
    weapon = node.get_weapon()[1]
    # 瞄准目标
    yaw, pitch, roll, heading, frame_timestamp = node.get_attitude()

    weapon.set_weapon_angle(yaw, pitch, time.time())
    # 无人机开火
    # 获取打击目标id,具体坐标信息等可查看具体get_targets方法，这里仅展示id获取部分
    # world = client.get_world()[1]
    # targets = world.get_targets()
    # ids = []
    # for target in targets:
    #     ids.append(target["id"])
    # weapon类方法missle_fire负责瞄准打击目标，参数为打击目标的id（通过get_targets方法获得）
    print("ids长度：", len(ids))
    weapon.missile_fire(ids[8])
    # node类方法gun_fire负责开火，参数为开火次数，注意规则中荷载武器上限
    node.gun_fire(1)
    print(num, "号无人机已开火")

    node.control_kinetic_simply_global(PATH["p8"][0] , PATH["p8"][1],
                                       PATH["p8"][2] + 50, 50, time.time())

    while True:
        v = node.get_velocity()
        v3 = math.floor(abs(v[3]))
        v0 = math.floor(abs(v[0]))
        v1 = math.floor(abs(v[1]))
        v2 = math.floor(abs(v[2]))
        xs, ys, zs, frame_timestamp = node.get_location()
        d = calcDistance(xs, ys, zs, PATH["p8"][0] , PATH["p8"][1], PATH["p8"][2] + 50)
        print("路径{}: 距离:{}".format("p8", d))
        time.sleep(0.001)
        if (v3 != 0):
            flag = 1
            # print(normal_targets[1][1])
            print("飞机正在出发前往p8")
            print("ids长度：", len(ids))
        if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d < 10 and flag == 1):
            flag = 0
            print("飞机到达p8")
            break

    # node为注册的无人机
    # 获取车辆武器
    weapon = node.get_weapon()[1]
    # 瞄准目标
    yaw, pitch, roll, heading, frame_timestamp = node.get_attitude()

    weapon.set_weapon_angle(yaw, pitch, time.time())
    # 无人机开火
    # 获取打击目标id,具体坐标信息等可查看具体get_targets方法，这里仅展示id获取部分
    # world = client.get_world()[1]
    # targets = world.get_targets()
    # ids = []
    # for target in targets:
    #     ids.append(target["id"])
    # weapon类方法missle_fire负责瞄准打击目标，参数为打击目标的id（通过get_targets方法获得）
    print("ids长度：", len(ids))
    weapon.missile_fire(ids[5])
    # node类方法gun_fire负责开火，参数为开火次数，注意规则中荷载武器上限
    node.gun_fire(1)
    print(num, "号无人机已开火")

    node.control_kinetic_simply_global(Final_X, PATH["p8"][1], PATH["p8"][2] + z_offset, 50,
                                       node.get_location()[3])


def main():
    global client1,flag_first
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
        client1 = client
        # sdk读取环境信息，需要 1.0.1 sdk 才支持
        timestamp, world, code = client.get_world()
        objects = world.get_environment_objects()
        # 线信息
        BP_Trees = list(filter(lambda x: (x.name.startswith('BP_Trees')), objects))
        print("BP_Trees")
        print(BP_Trees[0].transform.location.x)
        Start_X = BP_Trees[0].transform.location.x
        Final_X = Start_X + 2003  # 获取不到终点线，但是可以获取起点线，并且规定赛道2000m，设置最终点于起点线+2003

        # 普通目标、坚固目标、重点目标的坐标信息
        normal_targets = []
        solid_targets = []
        key_targets = []
        Dynamic_BP_WaterTower = list(filter(lambda x: (x.name.startswith('Dynamic_BP_WaterTower')), objects))
        # print(Dynamic_BP_WaterTower)
        for i in range(len(Dynamic_BP_WaterTower)):
            x, y, z = Dynamic_BP_WaterTower[i].transform.location.x, Dynamic_BP_WaterTower[i].transform.location.y, \
            Dynamic_BP_WaterTower[i].transform.location.z
            normal_targets.append([x, y, z])
            # print(normal_targets[i])
        print("normal_targets:")
        print(normal_targets)

        Dynamic_BP_Pillbox = list(filter(lambda x: (x.name.startswith('Dynamic_BP_Pillbox')), objects))
        # print(Dynamic_BP_Pillbox)
        for i in range(len(Dynamic_BP_Pillbox)):
            x, y, z = Dynamic_BP_Pillbox[i].transform.location.x, Dynamic_BP_Pillbox[i].transform.location.y, \
                Dynamic_BP_Pillbox[i].transform.location.z
            solid_targets.append([x, y, z])
            # print(solid_targets[i])
        print("solid_targets:")
        print(solid_targets)

        Dynamic_BP_RadarDish = list(filter(lambda x: (x.name.startswith('Dynamic_BP_RadarDish')), objects))
        # print(Dynamic_BP_RadarDish)
        for i in range(len(Dynamic_BP_RadarDish)):
            x, y, z = Dynamic_BP_RadarDish[i].transform.location.x, Dynamic_BP_RadarDish[i].transform.location.y, \
                Dynamic_BP_RadarDish[i].transform.location.z
            key_targets.append([x, y, z])
            # print(key_targets[i])
        print("key_targets:")
        print(key_targets)

        world = client.get_world()[1]
        targets = world.get_targets()
        ids = []
        locations = []
        for target in targets:
            ids.append(target["id"])
            locations.append(target["location"])

        Y_Nofly = []
        num1 = 0;
        for location in locations:
            if(num1 >= 10):
                Y_Nofly.append(location.y)
            num1 = num1 + 1;

        Y_Max = max(Y_Nofly)

        PATH = {
            "p1" : key_targets[0],
            "p2" : solid_targets[0],
            "p3" : normal_targets[1],
            "p4" : key_targets[1],
            "p5" : solid_targets[1],
            "p6": solid_targets[3],
            "p7" : solid_targets[2],
            "p8": key_targets[2]
        }

        print(PATH)




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

        num = 0;
        t = None;
        for node in nodes:
             z_offset = num*5
             if num < 3:
                 #P1_P4_0_2(node,PATH,ids,Final_X,z_offset,num)
                 # 第一辆飞机
                 if (num == 0):
                     t = threading.Thread(target=P1_P4_0_2,
                                          args=(node,PATH,ids,Final_X,z_offset,num))
                     t.start()

                 # 开始等待第一辆飞机起飞
                 while flag_first == 0:
                     time.sleep(0.001)
                     print("当前第一辆飞机是否起飞" + str(flag_first))
                     pass
                 # 第一辆飞机已经起飞
                 if (flag_first == 1 and num >= 1):
                     t = threading.Thread(target=P1_P4_0_2,
                                          args=(node,PATH,ids,Final_X,z_offset,num))
                     t.start()
             elif num == 3 or num ==4:
                 t = threading.Thread(target=P2_P5_3_4,
                                      args=(node, PATH, ids, Final_X, z_offset, num))
                 t.start()
             elif num == 5:
                 t = threading.Thread(target=P3_P6_5,
                                      args=(node, PATH, ids, Final_X, z_offset, num))
                 t.start()
             elif num == 6:
                 t = threading.Thread(target=P6_5_6,
                                      args=(node, PATH, ids, Final_X, z_offset, num))
                 t.start()
             elif num == 7 or num == 8 or num == 9:
                 t = threading.Thread(target=P7_7_9,
                                      args=(node, PATH, ids, Final_X, z_offset, num))
                 t.start()

             num = num + 1
        t.join()
        #P1_P4_0_2(nodes[0],PATH,ids,Final_X,0,0);



        # flag = 0

        # nodes[0].control_kinetic_simply_global(nodes[0].get_location()[0], key_targets[0][1],
        #                                        nodes[0].get_location()[2] + 50, 50, time.time())
        # while True:
        #     v = nodes[0].get_velocity()
        #     v3 = math.floor(abs(v[3]))
        #     v0 = math.floor(abs(v[0]))
        #     v1 = math.floor(abs(v[1]))
        #     v2 = math.floor(abs(v[2]))
        #     time.sleep(0.001)
        #     if (v3 != 0):
        #         flag = 1
        #         print(normal_targets[1][1])
        #         print("飞机正在调整高度")
        #     if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
        #         flag = 0
        #         print("飞机到达高度点，出发前往y")
        #         break
        #
        # for key in PATH.keys():
        #     location = PATH[key]
        #     nodes[0].control_kinetic_simply_global(location[0], location[1],
        #                                            location[2] + 50, 50, time.time())
        #     while True:
        #         v = nodes[0].get_velocity()
        #         v3 = math.floor(abs(v[3]))
        #         v0 = math.floor(abs(v[0]))
        #         v1 = math.floor(abs(v[1]))
        #         v2 = math.floor(abs(v[2]))
        #         xs, ys, zs, frame_timestamp = nodes[0].get_location()
        #         d = calcDistance(xs, ys, zs, location[0], location[1], location[2] + 50 )
        #         print("路径{}: 距离:{}".format(key,d))
        #         time.sleep(0.001)
        #         if (v3 != 0):
        #             flag = 1
        #             print(normal_targets[1][1])
        #             print("飞机正在调整高度")
        #         if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and d<10 and flag == 1):
        #             flag = 0
        #             print("飞机到达高度点，出发前往y")
        #             break



       # print(nodes[0].get_weapon()[1].get_weapon_ammo())



        # nodes[0].control_kinetic_simply_global(normal_targets[1][0]-50,normal_targets[1][1],normal_targets[1][2]+25,50,time.time())
        # while True:
        #     v = nodes[0].get_velocity()
        #     v3 = math.floor(abs(v[3]))
        #     v0 = math.floor(abs(v[0]))
        #     v1 = math.floor(abs(v[1]))
        #     v2 = math.floor(abs(v[2]))
        #     time.sleep(0.001)
        #     if (v3 != 0):
        #         flag = 1
        #         print(normal_targets[1][1])
        #         print("飞机正在调整高度")
        #     if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
        #         flag = 0
        #         print("飞机到达高度点，出发前往normal_targets")
        #         break
        # nodes[0].control_kinetic_simply_global(normal_targets[0][0] + 10, normal_targets[0][1],
        #                                        normal_targets[0][2] + 25, 50, time.time())
        # while True:
        #     v = nodes[0].get_velocity()
        #     v3 = math.floor(abs(v[3]))
        #     v0 = math.floor(abs(v[0]))
        #     v1 = math.floor(abs(v[1]))
        #     v2 = math.floor(abs(v[2]))
        #     time.sleep(0.001)
        #     if (v3 != 0):
        #         flag = 1
        #         print(normal_targets[1][1])
        #         print("飞机正在飞往normal 1")
        #     if (v0 == 0 and v1 == 0 and v2 == 0 and v3 == 0 and flag == 1):
        #         flag = 0
        #         print("飞机到达normal 1")
        #         break
        # # node为注册的无人机
        # # 获取车辆武器
        # weapon = nodes[0].get_weapon()[1]
        # # 瞄准目标
        # yaw,pitch,roll,heading,frame_timestamp = nodes[0].get_attitude()
        #
        # weapon.set_weapon_angle(yaw, pitch, time.time())
        # # 无人机开火
        # # 获取打击目标id,具体坐标信息等可查看具体get_targets方法，这里仅展示id获取部分
        # world = client.get_world()[1]
        # targets = world.get_targets()
        # ids = []
        # for target in targets:
        #     ids.append(target["id"])
        # # weapon类方法missle_fire负责瞄准打击目标，参数为打击目标的id（通过get_targets方法获得）
        # weapon.missile_fire(ids[0])
        # # node类方法gun_fire负责开火，参数为开火次数，注意规则中荷载武器上限
        # nodes[0].gun_fire(1)
        # print("无人机已开火")

    finally:
        print('main() end')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('__main__ end')
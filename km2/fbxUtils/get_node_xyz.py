from fbx_reader import Scenario

scenario = Scenario()

# 以名字筛选静态资源
nodes = scenario.get_nodes_by_regex(r'SM_Wall_Single*')

for name in nodes.keys():
    node = nodes[name]
    if not node.is_mesh():
        continue
    # door.get_transform_info()
    vertices = node.get_vertices()
    trans = node.get_global_transform_matrix()
    v_g = []
    if len(vertices) == 0:
        continue
    for v in vertices:
        # print(v[0], v[1], v[2])
        v_g.append(trans.MultT(v))
        ## print(v_g[-1][0], v_g[-1][1], v_g[-1][2])
        ## print()

    xmin = v_g[0][0]
    xmax = v_g[0][0]
    ymin = v_g[0][1]
    ymax = v_g[0][1]
    zmin = v_g[0][2]
    zmax = v_g[0][2]

    for v in v_g:
        xmin = xmin if v[0] > xmin else v[0]
        xmax = xmax if v[0] < xmax else v[0]
        ymin = ymin if v[1] > ymin else v[1]
        ymax = ymax if v[1] < ymax else v[1]
        zmin = zmin if v[2] > zmin else v[2]
        zmax = zmax if v[2] < zmax else v[2]

    # 节点的中心点坐标（已换算成UE中单位）
    x = (xmax + xmin) / 2 / 100
    y = (ymin + ymax) / 2 / 100
    z = (zmin + zmax) / 2 / 100
    print(f"{name}: {[xmin, xmax, ymin, ymax, zmin, zmax]}, vertices len: {len(vertices)}\n"
          f"x: {x}, y: {y}, z: {z}")
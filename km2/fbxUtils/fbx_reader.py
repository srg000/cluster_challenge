import pdb
import sys

import fbx
from FbxCommon import *
from timeit import timeit
from re import compile
import pdb


class NodeWrapper:
    def __init__(self, fbx_node, fbx_type, father):
        self.cur_node = fbx_node
        self.cur_type = fbx_type
        self.father = father
        self.children = []

    def is_mesh(self):
        attr = self.cur_node.GetNodeAttribute()
        if attr is None:
            return False
        elif attr.GetAttributeType() == fbx.FbxNodeAttribute.eMesh:
            return True
        else:
            return False

    def add_child(self, child):
        self.children.append(child)

    def get_vertices(self):
        if self.cur_type is None \
                or self.cur_type != fbx.FbxNodeAttribute.eMesh:
            return None
        else:
            return self.cur_node.GetMesh().GetControlPoints()

    def get_transform_info(self):
        print("/--------------------------------------------------------------")
        T = self.cur_node.GetGeometricTranslation(fbx.FbxNode.eDestinationPivot)
        print("| ", "GeometricTranslation-Destination", T)
        R = self.cur_node.GetGeometricRotation(fbx.FbxNode.eDestinationPivot)
        print("| ", "GetGeometricRotation-Destination", R)
        S = self.cur_node.GetGeometricScaling(fbx.FbxNode.eDestinationPivot)
        print("| ", "GetGeometricScaling-Destination", S)

        print("|")

        T = self.cur_node.GetGeometricTranslation(fbx.FbxNode.eSourcePivot)
        print("| ", "GeometricTranslation-Source", T)
        R = self.cur_node.GetGeometricRotation(fbx.FbxNode.eSourcePivot)
        print("| ", "GetGeometricRotation-Source", R)
        S = self.cur_node.GetGeometricScaling(fbx.FbxNode.eSourcePivot)
        print("| ", "GetGeometricScaling-Source", S)

        print("|")

        x = self.cur_node.LclTranslation.Get()
        print("| ", "LclTranslation", list(x))

        x = self.cur_node.LclRotation.Get()
        print("| ", "LclRotation", list(x))

        x = self.cur_node.LclScaling.Get()
        print("| ", "LclScaling", list(x))

        x = self.cur_node.ScalingOffset.Get()
        print("| ", "ScalingOffset", list(x))

        print("|")

        x = self.cur_node.EvaluateGlobalTransform()
        print("| ", "EvaluateGlobalTransform")
        for s in x:
            print("| ", s[0], s[1], s[2], s[3])
        print("|")

        x = self.cur_node.EvaluateLocalTransform()
        print("| ", "EvaluateLocalTransform")
        for s in x:
            print("| ", s[0], s[1], s[2], s[3])
        print("\\______________________________________________________________")

    def get_global_transform_matrix(self):
        return self.cur_node.EvaluateGlobalTransform()


class Scenario:
    def __init__(self, map_name="CE_L2.fbx"):
        self.manager, self.scene = InitializeSdkObjects()
        self.res = LoadScene(self.manager, self.scene, map_name)

        self.root = None
        self.elements = {}

        self.traversal()

    def traversal(self, a_node=None):
        if a_node is None:
            fbx_node = self.scene.GetRootNode()
            attr = fbx_node.GetNodeAttribute()
            a_type = None
            if attr is not None:
                a_type = attr.GetAttributeType()

            a_node = NodeWrapper(fbx_node, attr, None)
            self.root = a_node
            self.elements[fbx_node.GetName()] = a_node

        for each in range(a_node.cur_node.GetChildCount()):
            child = a_node.cur_node.GetChild(each)
            attr = child.GetNodeAttribute()
            c_type = None
            if attr is not None:
                c_type = attr.GetAttributeType()
            c_node = NodeWrapper(child, c_type, a_node)
            a_node.add_child(c_node)
            self.elements[child.GetName()] = c_node
            if c_type is not None and c_type == fbx.FbxNodeAttribute.eMesh:
                pass
            else:
                self.traversal(c_node)

    def get_nodes_by_regex(self, regex: str):
        node_list = {}
        reg = compile(regex)
        for name, node in self.elements.items():
            if reg.match(name) is not None:
                node_list[name] = node
        return node_list

    def get_vertices_by_name(self, obj_name):
        obj = None
        if obj_name in self.elements.keys():
            obj = self.elements[obj_name]
        else:
            return None
        return obj.get_vertices()


def ProcessMesh(pMesh):
    if (pMesh == None):
        return

    # 获得mesh的构成顶点 = polygonVertices
    # 获得三角面的数量 = triangleCount

    lNormals = pMesh.GetElementNormal()
    triangleCount = pMesh.GetPolygonCount()
    polygonVertices = pMesh.GetPolygonVertices()
    vertexCounter = 0

    # 逐顶点去遍历
    for i in range(triangleCount):
        for j in range(3):
            ctrlPointIndex = polygonVertices[i * 3 + j]
            vertex = pMesh.GetControlPointAt(ctrlPointIndex)
            vertexCounter = vertexCounter + 1

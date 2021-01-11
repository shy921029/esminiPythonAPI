# -*- encoding: utf-8 -*-

"""
@File: esminiRMLibData.py
@Description: python data structure definitions for roadmanagerdll.hpp
@Date: 2021/01/07 17:19:00
@Author: ysh
@version: 1.0
"""


from ctypes import *


class PositionData(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("h", c_float),
        ("p", c_float),
        ("r", c_float),
        ("hRelative", c_float),
        ("roadId", c_int),
        ("landId", c_int),
        ("laneOffset", c_float),
        ("s", c_float),
    ]


class RoadLaneInfo(Structure):
    _fields_ = [
        ("pos", c_float * 3),
        ("heading", c_float),
        ("pitch", c_float),
        ("roll", c_float),
        ("width", c_float),
        ("curvature", c_float),
        ("speed_limit", c_float),
    ]


class RoadProbeInfo(Structure):
    _fields_ = [
        ("road_lane_info", RoadLaneInfo),
        ("relative_pos", c_float * 3),
        ("relative_h", c_float),
    ]


class PositionDiff(Structure):
    _fields_ = [("ds", c_float), ("dt", c_float), ("dLaneId", c_int)]

# -*- encoding: utf-8 -*-

"""
@File: esminiLibData.py
@Description: python data structure definitions for esminiLib.hpp
@Date: 2021/01/07 17:16:18
@Author: ysh
@version: 1.0
"""


from ctypes import *


class ScenarioObjectState(Structure):
    _fields_ = [
        ("id", c_int),          # Automatically generated unique object id
        ("model_id", c_int),    # Id to control what 3D model to represent the vehicle - see carModelsFiles_[] in scenarioenginedll.cpp
        ("ctrl_type", c_int),   # 0: DefaultController 1: External. Further values see Controller:: Type enum
        ("timestamp", c_float),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("h", c_float),
        ("p", c_float),
        ("r", c_float),
        ("roadId", c_int),
        ("t", c_float),
        ("laneId", c_int),
        ("laneOffset", c_float),
        ("s", c_float),
        ("speed", c_float),
        ("centerOffsetX", c_float),
        ("centerOffsetY", c_float),
        ("centerOffsetZ", c_float),
        ("width", c_float),
        ("length", c_float),
        ("height", c_float),
    ]


class RoadInfo(Structure):
    _fields_ = [
        ("global_pos_x", c_float),  # target position, in global coordinate system
        ("global_pos_y", c_float),
        ("global_pos_z", c_float),
        ("local_pos_x", c_float),   # target position, relative vehicle (pivot position object) coordinate system
        ("local_pos_y", c_float),  
        ("local_pos_z", c_float),  
        ("angle", c_float),         # heading angle to target from and relatove to vehicle (pivot position)
        ("road_heading", c_float),  # road heading at steering target point
        ("road_pitch", c_float),
        ("road_roll", c_float),
        ("trail_heading", c_float), # trail heading (only when used for trail lookups, else equals road_heading)
        ("curvature", c_float),     # road curvature at steering target point
        ("speed_limit", c_float),   # speed limit given by OpenDRIVE type entry
    ]


class LaneBoundaryId(Structure):
    _fields_ = [
        ("far_left_lb_id", c_int),
        ("left_lb_id", c_int),
        ("right_lb_id", c_int),
        ("far_right_lb_id", c_int),
    ]


class SimpleVehicleState(Structure):
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("h", c_float),
        ("p", c_float),
        ("speed", c_float),
    ]


class Parameter(Structure):
    _fields_ = [
        ("name", c_char_p),
        ("value", c_void_p),
    ]
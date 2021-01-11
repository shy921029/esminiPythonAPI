# -*- encoding: utf-8 -*-

"""
@File: esminiRMLibFunctions.py
@Description: python functions for esminiRMLib.hpp
@Date: 2021/01/07 17:28:09
@Author: ysh
@version: 1.0
"""


from ctypes import *
from esminiRMLibData import (
    PositionData,
    RoadLaneInfo,
    RoadProbeInfo,
    PositionDiff,
)


class RoadManager:
    """
    python class for roadmanager.
    """

    def __init__(self, lib_path: str, odrFilename: str):
        """
        Args:
            lib_path: path to libesminiRMLib.so.
            odrFilename: path to a xodr file.
        """
        self.rm = CDLL(lib_path)
        self._rm_init(odrFilename)

    def _rm_init(self, odrFilename: str) -> int:
        """
        Initialize the roadmanager.
        """
        res = self.rm.RM_Init(bytes(odrFilename.encode()))
        return res

    def close(self) -> int:
        res = self.rm.RM_Close()
        return res

    def createPosition(self) -> int:
        """
        Create a position object.

        Returns:
            Handle to the position object, to use for operations
        """
        res = self.rm.RM_CreatePosition()
        return res

    def getNrOfPositions(self) -> int:
        """
        Get the number of created position objects.

        Returns:
            Number of created position objects
        """
        res = self.rm.RM_GetNrOfPositions()
        return res

    def deletePosition(self, handle: int) -> int:
        """
        Delete one or all position object(s).

        Args:
            hande - Handle to the position object. Set -1 to delete all.

        Returns:
            0 if succesful, -1 if specified position(s) could not be deleted
        """
        res = self.rm.RM_DeletePosition(handle)
        return res

    def getNumberOfRoads(self) -> int:
        """
        Get the total number fo roads in the road network of the currently loaded OpenDRIVE file.

        Returns:
            Number of roads
        """
        res = self.rm.RM_GetNumberOfRoads()
        return res

    def getIdOfRoadFromIndex(self, index: int) -> int:
        """
        Get the Road ID of the road with specified index. E.g. if there are 4 roads, index 3 means the last one.

        Args:
            index - The index of the road

            Returns:
            The ID of the road
        """
        res = self.rm.RM_GetIdOfRoadFromIndex(index)
        return res

    def getRoadLength(self, roadId: int) -> float:
        """
        Get the lenght of road with specified ID

        Args:
            roadId - The road ID

        Returns:
            The length of the road if ID exists, else 0.0
        """
        self.rm.RM_GetRoadLength.restype = c_float  #! 声明返回数据类型
        res = self.rm.RM_GetRoadLength(roadId)
        return res

    def getRoadNumberOfLanes(self, roadId: int, s: float) -> int:
        """
        Get the number of drivable lanes of specified road.

        Args:
            roadId - The road ID
            s - The distance along the road at what point to check number of lanes (which can vary along the road)

        Returns:
            The number of drivable lanes
        """
        res = self.rm.RM_GetRoadNumberOfLanes(roadId, c_float(s))
        return res

    def getLaneIdByIndex(self, roadId: int, laneIndex: int, s: float) -> int:
        """
        Get the ID of the lane given by index.

        Args:
            roadId - The road ID
                laneIndex - The index of the lane
                s - The distance along the road at what point to look up the lane ID

        Returns:
            The lane ID
        """
        res = self.rm.RM_GetLaneIdByIndex(roadId, laneIndex, c_float(s))
        return res

    def setLanePosition(
        self,
        handle: int,
        roadId: int,
        laneId: int,
        laneOffset: float,
        s: float,
        align: bool,
    ) -> int:
        """
        Set position from road coordinates, world coordinates being calculated.

        Args:
            handle - Handle to the position object
                roadId - Road specifier
                laneId - Lane specifier
                laneOffset - Offset from lane center
                s - Distance along the specified road
                align - If true the heading will be reset to the lane driving direction (typically only at initialization)

        Returns:
            0 if successful, -1 if not
        """
        res = self.rm.RM_SetLanePosition(
            handle, roadId, laneId, c_float(laneOffset), c_float(s), align
        )
        return res

    def setS(self, handle: int, s: float) -> int:
        """
        Set s (distance) part of a lane position, world coordinates being calculated.

            Args:
            handle - Handle to the position object
                s - Distance along the specified road

        Returns:
            0 if successful, -1 if not
        """
        res = self.rm.RM_SetS(handle, c_float(s))
        return res

    def setWorldPosition(
        self, handle: int, x: float, y: float, z: float, h: float, p: float, r: float
    ) -> int:
        """
        Set position from world coordinates, road coordinates being calculated.

        Args:
            handle - Handle to the position object
            x - cartesian coordinate x value
            y - cartesian coordinate y value
            z - cartesian coordinate z value
            h - rotation heading value
            p - rotation pitch value
            r - rotation roll value

        Returns:
            0 if successful, -1 if not
        """
        res = self.rm.RM_SetWorldPosition(
            handle,
            c_float(x),
            c_float(y),
            c_float(z),
            c_float(h),
            c_float(p),
            c_float(r),
        )
        return res

    def setWorldXYHPosition(self, handle: int, x: float, y: float, h: float) -> int:
        """
        Set position from world X, Y and heading coordinates; Z, pitch and road coordinates being calculated.

        Args:
            handle - Handle to the position object
            x - cartesian coordinate x value
            y - cartesian coordinate y value
            h - rotation heading value

        Returns:
            0 if successful, -1 if not
        """
        res = self.rm.RM_SetWorldXYHPosition(handle, c_float(x), c_float(y), c_float(h))
        return res

    def positionMoveForward(self, handle: int, dist: float, strategy: int) -> int:
        """
        Move position forward along the road. Choose way randomly though any junctions.

        Args:
            handle - Handle to the position object
            dist - Distance (meter) to move
            strategy - How to move in a junction where multiple route options appear, see Junction::JunctionStrategyType

        Returns:
            0 if successful, -1 if not
        """
        res = self.rm.RM_PositionMoveForward(handle, c_float(dist), strategy)
        return res

    def getPositionData(self, handle: int, positionData: PositionData) -> int:
        """
        Get the fields of the position of specified index.

        Args:
            handle - Handle to the position object
            data - Struct to fill in the values

        Returns:
            0 if successful, -1 if not
        """
        res = self.rm.RM_GetPositionData(handle, byref(positionData))
        return res

    def getSpeedLimit(self, handle: int) -> int:
        """
        Retrieve current speed limit (at current road, s-value and lane) based on ODR type elements or nr of lanes.

        Args:
            handle - Handle to the position object

        Returns:
            0 if successful, -1 if not
        """
        res = self.rm.RM_GetSpeedLimit(handle)
        return res

    def getLaneInfo(
        self,
        handle: int,
        lookahead_distance: float,
        roadLaneInfo: RoadLaneInfo,
        lookAheadMode: int,
    ) -> int:
        """
        Retrieve lane information from the position object (at current road, s-value and lane).

        Args:
            handle - Handle to the position object
            lookahead_distance - The distance, along the road, to the point of interest
            roadLaneInfo - Struct including all result values, see RM_RoadLaneInfo typedef
            lookAheadMode - Measurement strategy: Along reference lane, lane center or current lane offset, AT_LANE_CENTER = 0, AT_ROAD_CENTER = 1, AT_CURRENT_LATERAL_OFFSET = 2
        
        Returns:
            0 if successful, -1 if not
        """
        res = self.rm.RM_GetLaneInfo(
            handle, c_float(lookahead_distance), byref(roadLaneInfo), lookAheadMode
        )
        return res

    def getProbeInfo(
        self,
        handle: int,
        lookahead_distance: float,
        roadProbeInfo: RoadProbeInfo,
        lookAheadMode: int,
    ) -> int:
        """
        As RM_GetLaneInfo plus relative location of point of interest (probe) from current position.

        Args:
            handle - Handle to the position object from which to measure
            lookahead_distance - The distance, along the road to the probe (point of interest)
            roadProbeInfo - Struct including all result values, see RM_RoadProbeInfo typedef
            lookAheadMode - Measurement strategy: Along reference lane, lane center or current lane offset, AT_LANE_CENTER = 0, AT_ROAD_CENTER = 1, AT_CURRENT_LATERAL_OFFSET = 2

        Returns:
            0 if successful, -1 if not
        """
        res = self.rm.RM_GetProbeInfo(
            handle, c_float(lookahead_distance), byref(roadProbeInfo), lookAheadMode
        )
        return res

    def subtractAFromB(self, handleA: int, handleB: int, pos_diff: PositionDiff) -> int:
        """
        Find out the difference between two position objects, i.e. delta distance (long and lat) and delta laneId.

        Args:
            handleA - Handle to the position object from which to measure
            handleB - Handle to the position object to which the distance is measured
            pos_diff - Struct including all result values, see PositionDiff typedef

        Returns:
            true if a valid path between the road positions was found and calculations could be performed
        """
        res = self.rm.RM_SubtractAFromB(handleA, handleB, byref(pos_diff))
        return res

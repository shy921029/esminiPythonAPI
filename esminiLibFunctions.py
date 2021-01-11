# -*- encoding: utf-8 -*-

"""
@File: esminiLibFunctions.py
@Description: python API functions for esminiLib.hpp
@Date: 2021/01/07 13:43:14
@Author: ysh
@version: 1.0
"""

import platform
from ctypes import *
from esminiLibData import (
    ScenarioObjectState,
    RoadInfo,
    LaneBoundaryId,
    SimpleVehicleState,
    Parameter,
)


class ScenarioEngine:
    """
    python class for scenarioengine.
    """

    def __init__(self, path: str = "../bin/"):
        """
        Args:
            path (str): path to libesminiLib.so
        """
        sys = platform.system()
        lib = "libesminiLib.so"
        self.se = CDLL(path + lib)

    def addPath(self, path: str):
        """
        Add a search path for OpenDrive and 3D model files.

        Args:
            path (str): path to a directory

        Returns:
            0 if successful, -1 if not
        """
        res = self.se.SE_AddPath(bytes(path.encode()))
        return res

    def clearPaths(self):
        """
        Clear all search paths for OpenDrive and 3D model files.
        """
        self.se.SE_ClearPaths()

    def scenarioInit(
        self,
        oscFilename: str,
        disable_ctrls: int,
        use_viewer: int = 1,
        threads: int = 0,
        record: int = 0,
    ):
        """
        Initialize the scenario engine.

        Args:
            oscFilename (str): Path to the OpenSCEANRIO file.
            disable_ctrls (int): 1=Any controller will be disabled 0=Controllers applied according to OSC file.
            use_viewer (int): 0=no viewer, 1=use viewer.
            threads (int): 0=single thread, 1=viewer in a separate thread, parallel to scenario engine.
            record (int): Create recording for later playback, 0=no recording, 1=recording.

        Returns:
                res: 0 if successful, -1 if not
        """
        res = self.se.SE_Init(
            bytes(oscFilename.encode()),
            disable_ctrls,
            use_viewer,
            threads,
            record,
        )
        return res

    def scenarioInitWithString(
        self,
        oscAsXMLString: str,
        disable_ctrls: int,
        use_viewer: int = 1,
        threads: int = 0,
        record: int = 0,
    ):
        """
        Initialize the scenario engine with osc as XML string.

        Args:
            oscAsXMLString (str): OpenSCENARIO XML as string.
            disable_ctrls (int): 1=Any controller will be disabled 0=Controllers applied according to OSC file.
            use_viewer (int): 0=no viewer, 1=use viewer.
            threads (int): 0=single thread, 1=viewer in a separate thread, parallel to scenario engine.
            record (int): Create recording for later playback, 0=no recording, 1=recording.

        Returns:
                res: 0 if successful, -1 if not
        """
        res = self.se.SE_InitWithString(
            bytes(oscAsXMLString.encode()),
            disable_ctrls,
            use_viewer,
            threads,
            record,
        )
        return res

    def stepDT(self, dt: float):
        """
        Step the simulation forward with specified timestep.

        Args:
            dt: time step in seconds.
        Returns:
            0 if successful, -1 if not
        """
        res = self.se.SE_StepDT(c_float(dt))
        return res

    def step(self):
        """
        Step the simulation forward.
        Time step will be elapsed system (world) time since last step.
        Useful for interactive/realtime use cases.

        Returns:
            res: 0 if successful, -1 if not
        """
        res = self.se.SE_Step()
        return res

    def close(self):
        """
        Stop simulation gracefully.
        Two purposes: 1. Release memory and 2. Prepare for next simulation, e.g. reset object lists.
        """
        self.se.SE_Close()

    def getSimulationTime(self):
        """
        Get simulation time in sceonds.

        Returns:
            time (float)
        """
        self.se.SE_GetSimulationTime.restype = c_float
        time = self.se.SE_GetSimulationTime()
        return time

    def getQuitFlag(self):
        """
        Get the bool value of the end of the scenario.

        Returns:
            1 if quit, 0 if running
        """
        res = self.se.SE_GetQuitFlag()
        return res

    def getODRFilename(self):
        """
        Get name of currently referred and loaded OpenDRIVE file.

        Args:
            None

        Returns:
            filename as string (const, since it's allocated and handled by esmini).
        """
        self.se.SE_GetODRFilename.restype = c_char_p
        odrFilename = self.se.SE_GetODRFilename()
        return odrFilename

    def getSceneGraphFilename(self):
        """
        Get name of currently referred and loaded SceneGraph file.

        Args:
            None

        Returns:
            filename as string (const, since it's allocated and handled by esmini).
        """
        self.se.SE_GetSceneGraphFilename.restype = c_char_p
        filename = self.se.SE_GetSceneGraphFilename()
        return filename

    def setParameter(self, parameter: Parameter):
        """
        Set value of named parameter.

        Args:
            parameter (Parameter): Struct object including name of parameter and pointer to value, see SE_Parameter declaration.

        Returns:
            0 if successful, -1 if not
        """
        res = self.se.SE_SetParameter(parameter)
        return res

    def getParameter(self, parameter: Parameter):
        """
        Get value of named parameter. The value within the parameter struct will be filled in.

        Args:
            parameter: struct object, see SE_Parameter declaration.

        Returns:
            0 if successful, -1 if not
        """
        res = self.se.GetParameter(byref(parameter))
        return res

    def getODRManager(self):
        """
        Todo
        """

    def reportObjectPos(
        self,
        objId: int,
        timestamp: float,
        x: float,
        y: float,
        z: float,
        h: float,
        p: float,
        r: float,
        speed: float,
    ):
        """
        Report object position in cartesian coordinates.

        Args:
            objId: object id
            timestamp： time
            x
            y
            z
            h: heading angle, rad
            p: pitch angle, rad
            r: roll angle, rad
            speed: m/s
        Returns:
            res: 0 if successful, -1 if not
        """
        timestamp = c_float(timestamp)
        x = c_float(x)
        y = c_float(y)
        z = c_float(z)
        h = c_float(h)
        p = c_float(p)
        r = c_float(r)
        speed = c_float(speed)
        res = self.se.SE_ReportObjectPos(objId, timestamp, x, y, z, h, p, r, speed)
        return res

    def reportObjectRoadPos(
        self,
        objId: int,
        timestamp: float,
        roadId: int,
        laneId: int,
        laneOffset: float,
        s: float,
        speed: float,
    ):
        """
        Transport object to lane pos.
        Control mode of object need to be external.

        Args:
            objId: object id
            timestamp
            roadId
            laneId
            laneOffset: Lateral offset from center of specified lane.
            s: Longitudinal distance of the position along the specified road.
            speed: Speed in forward direction (s axis) of the enitity.
        Returns:
            res: 0 if successful, -1 if not
        """
        timestamp = c_float(timestamp)
        laneOffset = c_float(laneOffset)
        s = c_float(s)
        speed = c_float(speed)
        res = self.se.SE_ReportObjectRoadPos(
            objId, timestamp, roadId, laneId, laneOffset, s, speed
        )
        return res

    def reportObjectSpeed(self, objId: int, speed: float):
        """
        Report object longitudinal speed. Useful for an external longitudinal controller.

        Args:
            objId: Id of the object.
            speed: Speed in forward direction of the object.

        Returns:
            0 if successful, -1 if not
        """
        res = self.se.SE_ReportObjectSpeed(objId, c_float(speed))
        return res

    def reportObjectLateralPosition(self, objId: int, t: float):
        """
        Report object lateral position relative road centerline. Useful for an external lateral controller.

        Args:
            objId: Id of the object.
            t: Lateral position.

        Returns:
            0 if successful, -1 if not
        """
        res = self.se.SE_ReportObjectLateralPosition(objId, c_float(t))
        return res

    def reportObjectLateralLanePosition(
        self, objId: int, laneId: int, laneOffset: float
    ):
        """
        Report object lateral position by lane id and lane offset. Useful for an external lateral controller.

        Args:
            objId: Id of the object.
            laneId: Id of the lane.
            laneOffset: Lateral offset from center of specified lane.

        Returns:
            0 if successful, -1 if not
        """
        res = self.se.SE_ReportObjectLateralLanePosition(
            objId, laneId, c_float(laneOffset)
        )
        return res

    def getNumberOfObjects(self):
        """
        Get the number of entities in the current scenario.

        Returns:
            num (int): num of entities.
        """
        num = self.se.SE_GetNumberOfObjects()
        return num

    def getObjectState(self, objId: int):
        """
        Get state of specified object.

        Args:
            objId: object id.

        Returns:
            state: ScenarioObjectState
        """
        state = ScenarioObjectState()
        self.se.SE_GetObjectState(objId, byref(state))
        return state

    def getObjectName(self, index: int):
        """
        Get the name of specified object.

        Args:
            index: Index of the object. Note: not ID

        Returns:
            name
        """
        self.se.SE_GetObjectName.restype = c_char_p
        name = self.se.SE_GetObjectName(index)
        return name

    def objectHasGhost(self, index: int):
        """
        Check whether an object has a ghost (special purpose lead vehicle).

        Args:
            index: Index of the object. Note: not ID

        Returns:
            1 if has ghost, 0 if not
        """
        res = self.se.SE_ObjectHasGhost(index)
        return res

    def getObjectGhostState(self, index: int):
        """
        Get the state of specified object's ghost (special purpose lead vehicle).

        Args:
            index: Index of the object. Note: not ID

        Returns:
            res: 0 if successful, -1 if not
            state of ghost.
        """
        state = ScenarioObjectState()
        res = self.se.SE_GetObjectGhostState(index, byref(state))
        return res, state

    def getRoadInfoAtDistance(self, objId, lookahead_distance, lookAheadMode):
        """
        Get information suitable for driver modeling of a point at a specified distance from object along the road ahead

        Args:
            objId: Id of the object from which to measure
            lookahead_distance: The distance, along the road, to the point
            lookAheadMode: Measurement strategy: Along 0=lane center, 1=road center (ref line) or 2=current lane offset.

        Returns:
            res: 0 if successful, -1 if not
            roadInfo: RoadInfo
        """
        lookahead_distance = c_float(lookahead_distance)
        roadInfo = RoadInfo()
        res = self.se.SE_GetRoadInfoAtDistance(
            objId, lookahead_distance, byref(roadInfo), lookAheadMode
        )
        return res, roadInfo

    def addObjectSensor(
        self,
        object_id: int,
        x: float,
        y: float,
        z: float,
        h: float,
        rangeNear: float,
        rangeFar: float,
        fovH: float,
        maxObj: int,
    ):
        """
        Create an ideal object sensor and attach to specified vehicle.

        Args:
            object_id(int): Handle to the object to which the sensor should be attached.
            x(float): Position x coordinate of the sensor in vehicle local coordinates.
            y(float): Position y coordinate of the sensor in vehicle local coordinates.
            z(float): Position z coordinate of the sensor in vehicle local coordinates.
            h(float): heading of the sensor in vehicle local coordinates.
            rangeNear(float): Near value of the sensor depth range.
            rangeFar(float): Far value of the sensor depth range.
            fovH(float): Horizontal field of view, in degrees.
            maxObj(int): Maximum number of objects theat the sensor can track.

        Returns:
            Sensor ID (Global index of sensor), -1 if unsucessful.
        """
        x = c_float(x)
        y = c_float(y)
        z = c_float(z)
        h = c_float(h)
        rangeNear = c_float(rangeNear)
        rangeFar = c_float(rangeFar)
        fovH = c_float(fovH)
        sensorId = self.se.SE_AddObjectSensor(
            object_id, x, y, z, h, rangeNear, rangeFar, fovH, maxObj
        )
        return sensorId

    def fetchSensorObjectList(self, sensorId: int, maxHits: int = 100):
        """
        Fetch list of identified objects from a sensor.

        Args:
            sensorId: Handle (index) to the sensor.

        Returns:
            nHits: Number of identified objects, -1 if unsuccesful.
            objList: c_int_array of object id.
        """
        objList = (c_int * maxHits)()
        nHits = self.se.SE_FetchSensorObjectList(sensorId, objList)
        return nHits, objList

    def registerObjectCallback(self, objId, callback, user_data):
        """ todo """

    """ OSI interface, need to be completed, do not know how to use right now """

    def openOSISocket(self, ipAddress: str):
        """
        Send OSI packages over UDP to specified IP address.

        Args:
            ipAddress

        Returns:
            res: 0 if successful, -1 if not
        """
        ipAddress = str(ipAddress)
        res = self.se.SE_OpenOSISocket(bytes(ipAddress.encode()))
        return res

    def disableOSIFile(self):
        """
        Switch off logging to OSI files
        """
        self.se.SE_DisableOSIFile()

    def enableOSIFile(self, filename=""):
        """
        Switch on logging to OSI files.

        Args:
            filename: Optional filename, including path, Set "" to use default.
        """
        self.se.SE_EnableOSIFile(bytes(filename.encode()))

    def getOSIRoadLane(self, size: int, objId: int):
        """
        The SE_GetOSIRoadLane function returns a char array
        containing the osi Lane information/message of the lane
        where the object with object_id is, serialized to a string
        """
        res = self.se.SE_GetOSIRoadLane(byref(c_int(size)), c_int(objId))
        return res

    def getOSILaneBoundary(self, size: int, globalId: int):
        """"""
        res = self.se.SE_GetOSILaneBoundary(byref(c_int(size)), c_int(globalId))
        return res

    def getOSILaneBoundaryIds(self, objId: int, ids: LaneBoundaryId):
        """"""
        self.se.SE_GetOSILaneBoundaryIds(c_int(objId), byref(ids))

    def osiFileOpen(self):
        """
        Create and open osi file.
        """
        self.se.SE_OSIFileOpen()

    def osiFileWrite(self):
        """"""
        self.se.SE_OSIFileWrite()
    
    """ viewer """

    def viewerShowFeature(self, featureType: int, enable: bool):
        self.se.SE_ViewerShowFeature(featureType, enable)

    """ 
    ----- vehicle interface ----- 
    """

    def simpleVehicleCreate(self, x: float, y: float, h: float, length: float):
        """
        Create an instance of a simplistic vehicle based on a 2D bicycle kincematic model

        Args:
            x: Initial position X world coordinate
            y: Initial position Y world coordinate
            h: Initial heading
            length: Length of the vehicle

        Returns:
            Handle to the created object
        """
        self.se.SE_SimpleVehicleCreate.restype = c_void_p
        handle = self.se.SE_SimpleVehicleCreate(
            c_float(x), c_float(y), c_float(h), c_float(length)
        )
        return handle

    def simpleVehicleDelete(self, handle):
        """
        Delete an instance of the simplistic vehicle model

        Args:
            Handle to the created object.
        """
        self.se.SE_SimpleVehicleDelete(c_void_p(handle))

    def simpleVehicleControlBinary(self, handle, dt, throttle: int, steering: int):
        """
        Control the speed and steering with discreet [-1, 0, 1] values, suitable for keyboard control (e.g. up/none/down).
        The function also steps the vehicle model, updating its position according to motion state and timestep.

        Args:
            dt: timesStep (s)
            throttle: Longitudinal control, -1: brake, 0: none, +1: accelerate
            steering: Lateral control, -1: left, 0: straight, 1: right

        Returns:
            None
        """
        self.se.SE_SimpleVehicleControlBinary(c_void_p(handle), c_double(dt), throttle, steering)

    def simpleVehicleControlAnalog(self, handle, dt, throttle: float, steering: float):
        """
        Control the speed and steering with floaing values in the range [-1, 1], suitable for driver models.
        The function also steps the vehicle model, updating its position according to motion state and timestep.

        Args:
            dt: timesStep (s)
            throttle: Longitudinal control, -1: maximum brake, 0: no acceleration, +1: maximum acceleration
            steering: Lateral control, -1: max left, 0: straight, 1: max right

        Returns:
            None
        """
        self.se.SE_SimpleVehicleControlAnalog(
            c_void_p(handle), c_double(dt), c_double(throttle), c_double(steering)
        )

    def simpleVehicleSetMaxSpeed(self, handle, speed: float):
        """
        Set maximum vehicle speed.
        
        Args:
            speed: Maximum speed (m/s)
        
        Returns:
            None
        """
        self.se.SE_SimpleVehicleSetMaxSpeed(c_void_p(handle), c_float(speed))
    
    def simpleVehicleGetState(self, handle):
        """
        Get current state of the vehicle. Typically called after Control has been applied.

        Args:
            handle: handle of vehicle
        
        Returns:
            state: SimpleVehicleState
        """
        state = SimpleVehicleState()
        self.se.SE_SimpleVehicleGetState(c_void_p(handle), byref(state))
        return state

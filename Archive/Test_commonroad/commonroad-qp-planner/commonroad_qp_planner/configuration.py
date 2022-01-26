import enum
import numpy as np
from typing import Union

# commonroad-io
from commonroad.common.validity import is_real_number
from commonroad.scenario.lanelet import LaneletNetwork

# commonroad-curvilinear-coordinate-system
import commonroad_dc.pycrccosy as pycrccosy

VEHICLE_ID = int
TIME_IDX = int


class ReferencePoint(enum.Enum):
    """
    The reference point for the ego vehicle to describe the kinematics of the ego vehicle.
    If we choose the rear axis, the slip angle can be assumed to be zero at low speeds and generally
    neglected at high speeds.
    """
    CENTER = "center"
    REAR = "rear"


class PlanningConfigurationVehicle:
    """ Class which holds all necessary vehicle-specific parameters for the
    trajectory planning."""

    def __init__(self):
        """
        Default settings.
        """
        self.vehicle_id = -1
        self.min_speed_x = -np.inf
        self.max_speed_x = +np.inf
        self.min_speed_y = -np.inf
        self.max_speed_y = +np.inf
        self.a_max_x = 12.0
        self.a_min_x = -10.0
        self.a_max_y = 6.0
        self.a_min_y = -6.0
        self.a_max = 15.0
        self.j_min_x = -15.0
        self.j_max_x = 15.0
        self.j_min_y = -10.0
        self.j_max_y = 10.0
        self.desired_speed = 0.0
        self.wheelbase = 3.0
        self.length = 4.5
        self.width = 2.0
        self._curvilinear_coordinate_system = None
        self._reference_path = None
        self._reference_point = ReferencePoint.REAR
        self._lanelet_network = None

    @property
    def vehicle_id(self) -> VEHICLE_ID:
        """ Unique ID of the vehicle, e.g, the planning problem ID from CommonRoad."""
        return self._vehicle_id

    @vehicle_id.setter
    def vehicle_id(self, vehicle_id: VEHICLE_ID):
        assert (type(vehicle_id) is VEHICLE_ID), '<PlanConfiguration/vehicle_id> Expected type int; ' \
                                                 'Got type %s instead.' % (type(vehicle_id))
        self._vehicle_id = vehicle_id

    @property
    def min_speed_x(self) -> float:
        """ Minimum speed of the vehicle in longitudinal direction."""
        return self._min_speed_x

    @min_speed_x.setter
    def min_speed_x(self, min_speed_x: float):
        assert (type(min_speed_x) is float), '<PlanConfiguration/min_speed_x> Expected type float; ' \
                                             'Got type %s instead.' % (type(min_speed_x))
        self._min_speed_x = min_speed_x

    @property
    def max_speed_x(self) -> float:
        """ Maximum speed of the vehicle in longitudinal direction."""
        return self._max_speed_x

    @max_speed_x.setter
    def max_speed_x(self, max_speed_x: float):
        assert (type(max_speed_x) is float), '<PlanConfiguration/max_speed_x> Expected type float; ' \
                                             'Got type %s instead.' % (type(max_speed_x))
        self._max_speed_x = max_speed_x

    @property
    def min_speed_y(self) -> float:
        """ Minimum speed of the vehicle in lateral direction."""
        return self._min_speed_y

    @min_speed_y.setter
    def min_speed_y(self, min_speed_y: float):
        assert (type(min_speed_y) is float), '<PlanConfiguration/min_speed_y> Expected type float; ' \
                                             'Got type %s instead.' % (type(min_speed_y))
        self._min_speed_y = min_speed_y

    @property
    def max_speed_y(self) -> float:
        """ Maximum speed of the vehicle in lateral direction."""
        return self._max_speed_y

    @max_speed_y.setter
    def max_speed_y(self, max_speed_y: float):
        assert (type(max_speed_y) is float), '<PlanConfiguration/max_speed_y> Expected type float; ' \
                                             'Got type %s instead.' % (type(max_speed_y))
        self._max_speed_y = max_speed_y

    @property
    def a_max_x(self) -> float:
        """ Maximum acceleration of the vehicle in longitudinal direction."""
        return self._a_max_x

    @a_max_x.setter
    def a_max_x(self, a_max_x: float):
        assert (type(a_max_x) is float), '<PlanConfiguration/a_max_x> Expected type float; ' \
                                         'Got type %s instead.' % (type(a_max_x))
        self._a_max_x = a_max_x

    @property
    def a_min_x(self) -> float:
        """ Minimum acceleration of the vehicle in longitudinal direction."""
        return self._a_min_x

    @a_min_x.setter
    def a_min_x(self, a_min_x: float):
        assert (type(a_min_x) is float), '<PlanConfiguration/a_min_x> Expected type float; ' \
                                         'Got type %s instead.' % (type(a_min_x))
        self._a_min_x = a_min_x

    @property
    def a_max_y(self) -> float:
        """ Maximum acceleration of the vehicle in lateral direction."""
        return self._a_max_y

    @a_max_y.setter
    def a_max_y(self, a_max_y: float):
        assert (type(a_max_y) is float), '<PlanConfiguration/a_max_y> Expected type float; ' \
                                         'Got type %s instead.' % (type(a_max_y))
        self._a_max_y = a_max_y

    @property
    def a_min_y(self) -> float:
        """ Minimum acceleration of the vehicle in lateral direction."""
        return self._a_min_y

    @a_min_y.setter
    def a_min_y(self, a_min_y: float):
        assert (type(a_min_y) is float), '<PlanConfiguration/a_min_y> Expected type float; ' \
                                         'Got type %s instead.' % (type(a_min_y))
        self._a_min_y = a_min_y

    @property
    def a_max(self) -> float:
        """ Maximum overall acceleration of the vehicle."""
        return self._a_max

    @a_max.setter
    def a_max(self, a_max: float):
        assert (type(a_max) is float), '<PlanConfiguration/a_max> Expected type float; ' \
                                       'Got type %s instead.' % (type(a_max))
        self._a_max = a_max

    @property
    def j_min_x(self) -> float:
        """ Minimum jerk in longitudinal direction."""
        return self._j_min_x

    @j_min_x.setter
    def j_min_x(self, j_min_x: float):
        assert isinstance(j_min_x, float), '<PlanConfiguration/j_min_x> Expected type float; ' \
                                           'Got type %s instead.' % (type(j_min_x))
        self._j_min_x = j_min_x

    @property
    def j_max_x(self) -> float:
        """ Maximum jerk of the vehicle in longitudinal direction."""
        return self._j_max_x

    @j_max_x.setter
    def j_max_x(self, j_max_x: float):
        assert isinstance(j_max_x, float), '<PlanConfiguration/j_max_x> Expected type float; ' \
                                           'Got type %s instead.' % (type(j_max_x))
        self._j_max_x = j_max_x

    @property
    def j_min_y(self) -> float:
        """ Minimum jerk in lateral direction."""
        return self._j_min_y

    @j_min_y.setter
    def j_min_y(self, j_min_y: float):
        assert isinstance(j_min_y, float), '<PlanConfiguration/j_min_y> Expected type float; ' \
                                           'Got type %s instead.' % (type(j_min_y))
        self._j_min_y = j_min_y

    @property
    def j_max_y(self) -> float:
        """ Maximum jerk in lateral direction."""
        return self._j_max_y

    @j_max_y.setter
    def j_max_y(self, j_max_y: float):
        assert isinstance(j_max_y, float), '<PlanConfiguration/j_max_y> Expected type float; ' \
                                           'Got type %s instead.' % (type(j_max_y))
        self._j_max_y = j_max_y

    @property
    def desired_speed(self) -> float:
        """ Desired speed of the vehicle for trajectory planning."""
        return self._desired_speed

    @desired_speed.setter
    def desired_speed(self, desired_speed: float):
        assert isinstance(desired_speed, float), '<PlanConfiguration/desired_speed> Expected type float; ' \
                                                 'Got type %s instead.' % (type(desired_speed))
        self._desired_speed = desired_speed

    @property
    def wheelbase(self) -> float:
        """
        The vehicle’s shape is approximated with three circles with equal radius. The centers of the first and third
        circle coincides with the rear and front axle, respectively. The distance between the first and third center
        is the wheelbase."""
        return self._wheelbase

    @wheelbase.setter
    def wheelbase(self, wheelbase: float):
        assert (type(wheelbase) is float), '<PlanConfiguration/wheelbase> ' \
                                           'Expected type float; Got type %s instead.' % (type(wheelbase))
        self._wheelbase = wheelbase

    @property
    def length(self) -> float:
        """ Length of the vehicle."""
        return self._length

    @length.setter
    def length(self, length: float):
        assert is_real_number(length), '<PlanConfiguration/length>: argument "length" is not a real number. ' \
                                       'length = {}'.format(length)
        self._length = length

    @property
    def width(self) -> float:
        """ Width of the vehicle."""
        return self._width

    @width.setter
    def width(self, width: float):
        assert is_real_number(width), '<PlanConfiguration/width>: argument "width" is not a real number. ' \
                                      'width = {}'.format(width)
        self._width = width

    @property
    def curvilinear_coordinate_system(self) -> pycrccosy.CurvilinearCoordinateSystem:
        """ Curvilinear coordinate system for the reachable set computation in curvilinear coordinates."""
        if self._curvilinear_coordinate_system is None:
            self._curvilinear_coordinate_system = pycrccosy.CurvilinearCoordinateSystem(self.reference_path)

        return self._curvilinear_coordinate_system

    @curvilinear_coordinate_system.setter
    def curvilinear_coordinate_system(
            self, curvilinear_coordinate_system: pycrccosy.CurvilinearCoordinateSystem):
        assert (isinstance(curvilinear_coordinate_system, pycrccosy.CurvilinearCoordinateSystem)), \
            '<PlanConfiguration/curvilinear_coordinate_system> Expected type ' \
            'pycrccosy.PolylineCoordinateSystem; Got type %s instead.' % (type(curvilinear_coordinate_system))
        self._curvilinear_coordinate_system = curvilinear_coordinate_system

    @property
    def reference_path(self):
        """ Reference path of the vehicle for the generation of a curvilinear coordinate system or trajectory
        planning. The reference path must be given as polyline."""
        return self._reference_path

    @reference_path.setter
    def reference_path(self, reference_path: np.ndarray):
        assert isinstance(reference_path, np.ndarray) and reference_path.ndim == 2 \
               and len(reference_path) > 1 and len(reference_path[0, :]) == 2, \
               '<PlanConfiguration/reference_path>: Provided reference is not valid. reference = {}'. \
               format(reference_path)
        self._reference_path = reference_path

    @property
    def reference_point(self) -> ReferencePoint:
        return self._reference_point

    @reference_point.setter
    def reference_point(self, reference_point: ReferencePoint):
        assert isinstance(reference_point, ReferencePoint), \
            '<ReachSetConfiguration/reference_point>: argument reference_point of wrong type. Expected type: %s. ' \
            'Got type: %s' % (ReferencePoint, type(reference_point))
        self._reference_point = reference_point

    @property
    def lanelet_network(self) -> Union[None, LaneletNetwork]:
        """ The part of the lanelet network of the scenario, the vehicle is allowed or should drive on."""
        return self._lanelet_network

    @lanelet_network.setter
    def lanelet_network(self, lanelet_network: LaneletNetwork):
        assert isinstance(lanelet_network, LaneletNetwork), '<PlanConfiguration/lanelet_network>: argument ' \
                                                            'lanelet_network of wrong type. Expected type: %s. ' \
                                                            'Got type: %s.' % (LaneletNetwork, type(lanelet_network))
        self._lanelet_network = lanelet_network




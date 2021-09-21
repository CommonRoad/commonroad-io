__author__ = "Sebastian Maierhofer"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["CAR@TUM"]
__version__ = "2021.3"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Release"

import enum
from typing import List, Optional, Set, Tuple

import numpy as np

import commonroad.geometry.transform
from commonroad.common.validity import *
from commonroad.visualization.drawable import IDrawable
from commonroad.visualization.param_server import ParamServer
from commonroad.visualization.renderer import IRenderer

TRAFFIC_SIGN_VALIDITY_START = {'WARNING_DANGER_SPOT', 'WARNING_RIGHT_BEFORE_LEFT', 'WARNING_STEEP_HILL_DOWNWARDS',
                               'WARNING_SLIPPERY_ROAD', 'WARNING_CONSTRUCTION_SITE', 'WARNING_CROSSING_CYCLIST',
                               'WARNING_ANIMAL_CROSSING_RIGHT', 'RAILWAY', 'PRIORITY_OPPOSITE_DIRECTION',
                               'TURN_RIGHT_AHEAD', 'TURN_LEFT_AHEAD', 'ONEWAY_RIGHT', 'ONEWAY_LEFT',
                               'PRESCRIBED_PASSING_LEFT', 'PRESCRIBED_PASSING_RIGHT', 'BIKEWAY', 'SIDEWALK',
                               'PEDESTRIAN_ZONE_START', 'BICYCLE_ROAD_START', 'BUS_LANE', 'BAN_ALL_VEHICLES',
                               'BAN_CARS', 'BAN_TRUCKS', 'BAN_BICYCLE', 'BAN_MOTORCYCLE', 'BAN_BUS', 'BAN_PEDESTRIAN',
                               'BAN_CAR_TRUCK_BUS_MOTORCYCLE', 'BAN_VEHICLES_CARRYING_DANGEROUS_GOODS', 'NO_ENTRY',
                               'MAX_WEIGHT', 'MAX_WIDTH', 'MAX_HEIGHT', 'MAX_LENGTH', 'MAX_SPEED',
                               'MAX_SPEED_ZONE_START', 'MIN_SPEED',  'NO_OVERTAKING_START',
                               'NO_OVERTAKING_TRUCKS_START', 'TRAFFIC_CALMED_AREA_START', 'PRIORITY_OVER_ONCOMING',
                               'TOWN_SIGN', 'TUNNEL', 'INTERSTATE_START', 'HIGHWAY_START', 'PEDESTRIANS_CROSSING'}

TRAFFIC_SIGN_WITH_ADDITIONAL_VALUE = {'MAX_WEIGHT', 'MAX_WIDTH', 'MAX_HEIGHT', 'MAX_LENGTH', 'MAX_SPEED',
                                      'MAX_SPEED_ZONE_START', 'MIN_SPEED', 'ADDITION_VALID_FOR_X_METERS',
                                      'ADDITION_VALID_IN_X_KILOMETERS', 'ADDITION_TIME_PERIOD_PERMITTED'}

LEFT_HAND_TRAFFIC = {'AUS', 'JPN', 'HKG', 'IND', 'JEY', 'IMN', 'IRL', 'JAM',
                     'KEN', 'MLT', 'MYS', 'NPL', 'NZL', 'ZAF', 'SGP', 'THA',
                     'GBR', 'IDN', 'MAC', 'PAK', 'CYP'}


@enum.unique
class SupportedTrafficSignCountry(enum.Enum):
    GERMANY = 'DEU'
    USA = "USA"
    CHINA = "CHN"
    SPAIN = "ESP"
    RUSSIA = "RUS"
    ARGENTINA = "ARG"
    BELGIUM = "BEL"
    FRANCE = "FRA"
    GREECE = "GRC"
    CROATIA = "HRV"
    ITALY = "ITA"
    PUERTO_RICO = "PRI"
    ZAMUNDA = "ZAM"  # default


@enum.unique
class TrafficSignIDGermany(enum.Enum):
    # default traffic sign IDs (similar to German IDs)
    WARNING_DANGER_SPOT = '101'
    WARNING_RIGHT_BEFORE_LEFT = '102'
    WARNING_LEFT_CURVE = '103-10'
    WARNING_RIGHT_CURVE = '103-20'
    WARNING_STEEP_HILL_DOWNWARDS = '108'
    WARNING_SLIPPERY_ROAD = '114'
    WARNING_CONSTRUCTION_SITE = '123'
    WARNING_TRAFFIC_QUEUES_LIKELY = '124'
    WARNING_ONCOMING_TRAFFIC = '125'
    WARNING_TRAFFIC_LIGHTS_AHEAD = '131'
    WARNING_PEDESTRIANS_RIGHT = '133-10'
    WARNING_PEDESTRIANS_LEFT = '133-20'
    WARNING_CROSSING_CYCLIST = '138'
    WARNING_ANIMAL_CROSSING_RIGHT = '142-10'
    WARNING_LOOSE_GRAVEL = '145-50'
    RAILWAY = '201'
    YIELD = '205'
    STOP = '206'
    PRIORITY_OPPOSITE_DIRECTION = '208'
    TURN_RIGHT_AHEAD = '209-10'
    TURN_LEFT_AHEAD = '209-20'
    KEEP_STRAIGHT_AHEAD = '209-30'
    PRESCRIBED_DIRECTION_RIGHT = '211-20'
    ROUNDABOUT = '215'
    ONEWAY_RIGHT = '220-10'
    ONEWAY_LEFT = '220-20'
    PRESCRIBED_PASSING_LEFT = '222-10'
    PRESCRIBED_PASSING_RIGHT = '222-20'
    DO_NOT_DRIVE_ON_SHOULDER_LANE = '223.2'
    DO_NOT_DRIVE_ON_SHOULDER_LANE_2_LANE = '223.2-50'
    DO_NOT_DRIVE_ON_SHOULDER_LANE_3_LANE = '223.2-51'
    BUS_STOP = '224-50'
    BIKEWAY = '237'
    SIDEWALK = '239'
    PEDESTRIAN_AND_BICYCLE_ROAD = '240'
    PEDESTRIAN_ZONE_START = '242.1'
    PEDESTRIAN_ZONE_END = '242.2'
    BICYCLE_ROAD_START = '244.1'
    BICYCLE_ROAD_END = '244.2'
    BUSLANE = '245'
    BAN_ALL_VEHICLES = '250'
    BAN_CARS = '251'
    BAN_TRUCKS = '253'
    BAN_BICYCLE = '254'
    BAN_MOTORCYCLE = '255'
    BAN_BUS = '257-54'
    BAN_PEDESTRIAN = '259'
    BAN_CAR_TRUCK_BUS_MOTORCYCLE = '260'
    BAN_VEHICLES_CARRYING_DANGEROUS_GOODS = '261'
    MAX_WEIGHT = '262'
    MAX_WIDTH = '264'
    MAX_HEIGHT = '265'
    MAX_LENGTH = '266'
    NO_ENTRY = '267'
    ENVIRONMENTAL_ZONE_START = '270.1'
    ENVIRONMENTAL_ZONE_END = '270.2'
    U_TURN = '272'
    MAX_SPEED = '274'
    MAX_SPEED_ZONE_START = '274.1'
    MAX_SPEED_ZONE_END = '274.2'
    MIN_SPEED = '275'
    NO_OVERTAKING_START = '276'
    NO_OVERTAKING_TRUCKS_START = '277'
    MAX_SPEED_END = '278'
    NO_OVERTAKING_END = '280'
    NO_OVERTAKING_TRUCKS_END = '281'
    ALL_MAX_SPEED_AND_OVERTAKING_END = '282'
    NO_STOP_START_RIGHT = '283-10'
    NO_STOP_CENTER_RIGHT = '283-30'
    RESTRICTED_STOP_CENTER_RIGHT = '286-30'
    RIGHT_OF_WAY = '301'
    PRIORITY = '306'
    PRIORITY_OVER_ONCOMING = '308'
    TOWN_SIGN = '310'
    TOWN_SIGN_BACK = '311'
    PARKING_AREA = '314'
    PARKING_AREA_LEFT = '314-10'
    PARKING_AREA_RIGHT = '314-20'
    PARKING_AREA_RIGHT_LEFT = '314-30'
    TRAFFIC_CALMED_AREA_START = '325.1'
    TRAFFIC_CALMED_AREA_END = '325.2'
    TUNNEL = '327'
    EMERYGECNY_STOP = '328'
    INTERSTATE_START = '330.1'
    INTERSTATE_END = '330.2'
    HIGHWAY_START = '331.1'
    HIGHWAY_END = '331.2'
    HIGHWAY_EXIT_WITH_PLACE_NAME = '332'
    EXIT_ROUTE = '332.1'
    HIGHWAY_EXIT = '333'
    EXIT_BUILT_UP = '333-21'
    EXIT_GENERAL = '333-22'
    PEDESTRIANS_CROSSING = '350'
    WATER_PROTECTION_ZONE = '354'
    TRAFFIC_ASSISTANTS = '356'
    DEAD_END = '357'
    POLICE = '363'
    EMERGENCY_CALL_STATION = '365-51'
    GAS_STATION = '365-52'
    CAMP_AND_CARAVAN_SITE = '365-60'
    ATTRACTION_POINT = '386.1'
    TOURISTIC_ROUTE = '386.2'
    NEARBY_ATTRACTION_POINT = '386.3'
    HIGHWAY_INTERSECTION = '406-50'
    DIRECTION_ARROW_SIGN_MULTI = '418-20'
    DIRECTION_ARROW_SIGN_SINGLE = '419-20'
    DIRECTION_SIGN_CONSOLIDATED = '434-50'
    EXPRESSWAY_ARROW_DIRECTION = '430-20'
    ARROW_SIGN_POST_POINT_OF_INTEREST_LEFT = '432-10'
    STATION = '432-20'
    GUIDE_SIGN_TABLE = '434'
    ADVANCE_DIRECTION = '438'
    DIRECTIONS_SIGN = '439'
    EXPRESSWAY_ENTRANCE_DIRECTIONS = '440'
    INTERSTATE_ANNOUNCEMENT = '448'
    INTERSTATE_ADVANCE_DIRECTION = '449'
    HIGHWAY_EXIT_AHEAD_100_METER = '450-50'
    HIGHWAY_EXIT_AHEAD_200_METER = '450-51'
    HIGHWAY_EXIT_AHEAD_300_METER = '450-52'
    EXPRESSWAY_EXIT_100_METRES = '450-53'
    EXPRESSWAY_EXIT_200_METRES = '450-54'
    EXPRESSWAY_EXIT_300_METRES = '450-55'
    INTERSTATE_DISTANCE = '453'
    DETOUR_SKETCH = '458'
    DETOUR_STRAIGHT = '455.1-30'
    DETOUR_ON_DEMAND_LEFT = '460-10'
    DETOUR_ON_DEMAND_GET_IN_LEFT_LANE = '460-12'
    DETOUR_ON_DEMAND_ANNOUNCEMENT_RIGHT = '460-20'
    DETOUR_ON_DEMAND_RIGHT = '460-21'
    DETOUR_ON_DEMAND_GET_IN_RIGHT_LANE = '460-22'
    DETOUR_ON_DEMAND_STRAIGHTFORWARD = '460-30'
    TRANSITION_3_LEFT_2_TRANSITIONED = '501-15'
    TRANSITION_1_LEFT_1_STRAIGHT = '501-16'
    LANE_BOARD_NO_OPPOSITE_TWO_LANES = '521-30'
    THREE_LANES_NO_ONCOMING_LANES = '521-31'
    FOUR_LANES_NO_ONCOMING_LANES = '521-32'
    FIVE_LANES_NO_ONCOMING_LANES = '521-33'
    LANE_BOARD_3_LANES_NO_OPPOSITE_WITH_SIGNS = '525'
    NARROWING_LANES_1_LANE_FROM_RIGHT = '531-10'
    NARROWING_LANES_1_LANE_FROM_LEFT = '531-20'
    NARROWING_LANES_2_LANES_PLUS_1_LEFT = '531-21'
    FOUR_LANES_NO_ONCOMING_TRAFFIC_TWO_RIGHT_LANES_TURN_RIGHT = '533-22'
    MERGING_LANES_1_LANE_PLUS_1_LANE_RIGHT = '550-20'
    BARRIER = '600-35'
    BARRIER_GATE_100_800 = '600-30'
    BARRIER_GATE_100_1200 = '600-31'
    BARRIER_GATE_100_1600 = '600-32'
    BARRIER_GATE_250_1600 = '600-34'
    BARRIER_GATE = '600-38'
    ROAD_WARNING_POST_SCRAPER_BEACON_RIGHT = '605-10'
    ROAD_WARNING_POST_SCRAPER_BEACON_LEFT = '605-20'
    ROAD_WARNING_POST_ARROW_BEACON_RIGHT = '605-11'
    DIRECTION_SIGN_LEFT_SINGLE = '625-10'
    DIRECTION_SIGN_LEFT_SMALL = '625-11'
    DIRECTION_SIGN_LEFT_MEDIUM = '625-12'
    DIRECTION_SIGN_LEFT_LARGE = '625-13'
    DIRECTION_SIGN_RIGHT_SINGLE = '625-20'
    DIRECTION_SIGN_RIGHT_SMALL = '625-21'
    DIRECTION_SIGN_RIGHT_MEDIUM = '625-22'
    DIRECTION_SIGN_RIGHT_LARGE = '625-23'
    WARNING_PANEL_RIGHT = '626-10'
    WARNING_PANEL_LEFT = '626-20'
    WARNING_PANEL_STRAIGHT_BROAD = '626-30'
    WARNING_PANEL_STRAIGHT_HIGH = '626-31'
    GUIDE_SILL_WITH_GUIDE_BEACON_RIGHT = '628-10'
    GUIDE_RAIL_WITH_GUIDE_BEACON_RIGHT = '629-10'
    GUIDE_PANEL_WITH_GUIDE_BEACON_RIGHT = '629-20'
    GREEN_ARROW = '720'
    ADDITION_LEFT_DIRECTION = '1000'
    ADDITION_LEFT_DIRECTION_1 = '1000-10'
    ADDITION_LEFT_DIRECTION_DANGER_POINT = '1000-11'
    ADDITION_RIGHT_DIRECTION_1 = '1000-20'
    ADDITION_RIGHT_DIRECTION_DANGER_POINT = '1000-21'
    ADDITION_BOTH_DIRECTIONS_HORIZONTAL = '1000-30'
    ADDITION_BOTH_DIRECTIONS_VERTICAL = '1000-31'
    ADDITION_VALID_FOR_X_METERS = '1001-30'
    ADDITION_VALID_FOR_X_KILOMETERS = '1001-31'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD = '1002-10'
    ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD = '1002-11'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-12'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD = '1002-13'
    ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-14'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD = '1002-20'
    ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD = '1002-21'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-22'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD = '1002-23'
    ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-24'
    ADDITION_VALID_IN_X_METERS = '1004-30'
    ADDITION_VALID_IN_X_KILOMETERS = '1004-31'
    ADDITION_VALID_IN_400_METRES = '1004-33'
    ADDITION_VALID_IN_600_METRES = '1004-34'
    ADDITION_VALID_IN_2_KILOMETERS = '1004-35'
    ADDITION_OIL_ON_ROAD = '1006-30'
    ADDITION_SMOKE = '1006-31'
    ADDITION_LOOSE_GRAVEL = '1006-32'
    ADDITION_BUILDING_SITE_EXIT = '1006-33'
    ADDITION_DAMAGED_ROAD = '1006-34'
    ADDITION_DIRTY_ROAD = '1006-35'
    ADDITION_DANGER_OF_COLLISION = '1006-36'
    ADDITION_TOAD_MIGRATION = '1006-37'
    ADDITION_DANGER_OF_CONGESTION = '1006-38'
    ADDITION_RESTRICTED_VIEW_DUE_TO_TREES = '1006-39'
    DANGER_INDICATION_SMOKE = '1007-31'
    ADDITION_CHILDREN_PLAYING_ON_ROAD = '1010-10'
    ADDITION_WINTER_SPORTS_ALLOWED = '1010-11'
    ADDITION_TRAILERS_ALLOWED_TO_PARK_MORE_THAN_14_DAYS = '1010-12'
    ADDITION_CARAVANS_ALLOWED_TO_PARK_MORE_THAN_14_DAYS = '1010-13'
    ADDITION_ROLLING_HIGHWAY = '1010-14'
    ADDITION_LOADING_AREA = '1012-30'
    ADDITION_END = '1012-31'
    ADDITION_GET_OFF_BICYCLES = '1012-32'
    ADDITION_NO_MOPEDS = '1012-33'
    ADDITION_GREEN_WAVE_AT_KM_H = '1012-34'
    ADDITION_STOP_HERE_AT_RED = '1012-35'
    ADDITION_NOISE_CONTROL = '1012-36'
    ADDITION_INFLOW_REGULATION = '1012-37'
    ADDITION_SECONDARY_LANE = '1012-38'
    ADDITION_SCHOOL = '1012-50'
    ADDITION_KINDERGARTEN = '1012-51'
    ADDITION_RETIREMENT_HOME = '1012-52'
    ADDITION_HOSPITAL = '1012-53'
    ADDITION_RESIDENTS_PERMITTED = '1020-30'
    ADDITION_BICYCLES_PERMITTED = '1022-10'
    ADDITION_CARS_PERMITTED = '1024-10'
    ADDITION_AGRICULTURE_PERMITTED = '1026-36'
    ADDITION_FOREST_PERMITTED = '1026-37'
    ADDITION_AGRICULTURE_FOREST_PERMITTED = '1026-38'
    ADDITION_GREEN_STICKER_PERMITTED = '1031-52'
    ADDITION_TIME_PERIOD_PERMITTED = '1040-30'
    ADDITION_MOTOR_VEHICLES_ALLOWED_MASS_3_5_TONS = '1048-12'
    ADDITION_MIN_MASS_3_5_TONS = '1049-13'
    ADDITION_NO_WATER_POLLUTANTS_LOADED = '1052-31'
    ALLOWED_MASS_7_5_TONS = '1053-33'
    ADDITION_VALID_ON_SHOULDER = '1053-34'
    ADDITION_VALID_WHEN_WET = '1053-35'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDZamunda(enum.Enum):  # default traffic sign IDs (similar to German IDs)
    WARNING_DANGER_SPOT = '101'
    WARNING_RIGHT_BEFORE_LEFT = '102'
    WARNING_STEEP_HILL_DOWNWARDS = '108'
    WARNING_SLIPPERY_ROAD = '114'
    WARNING_CONSTRUCTION_SITE = '123'
    WARNING_CROSSING_CYCLIST = '138'
    WARNING_ANIMAL_CROSSING_RIGHT = '142-10'
    RAILWAY = '201'
    YIELD = '205'
    STOP = '206'
    PRIORITY_OPPOSITE_DIRECTION = '208'
    TURN_RIGHT_AHEAD = '209-10'
    TURN_LEFT_AHEAD = '209-20'
    ROUNDABOUT = '215'
    ONEWAY_RIGHT = '220-10'
    ONEWAY_LEFT = '220-20'
    PRESCRIBED_PASSING_LEFT = '222-10'
    PRESCRIBED_PASSING_RIGHT = '222-20'
    BIKEWAY = '237'
    SIDEWALK = '239'
    PEDESTRIAN_ZONE_START = '242.1'
    PEDESTRIAN_ZONE_END = '242.2'
    BICYCLE_ROAD_START = '244.1'
    BICYCLE_ROAD_END = '244.2'
    BUSLANE = '245'
    BAN_ALL_VEHICLES = '250'
    BAN_CARS = '251'
    BAN_TRUCKS = '253'
    BAN_BICYCLE = '254'
    BAN_MOTORCYCLE = '255'
    BAN_BUS = '257-54'
    BAN_PEDESTRIAN = '259'
    BAN_CAR_TRUCK_BUS_MOTORCYCLE = '260'
    BAN_VEHICLES_CARRYING_DANGEROUS_GOODS = '261'
    MAX_WEIGHT = '262'
    MAX_WIDTH = '264'
    MAX_HEIGHT = '265'
    MAX_LENGTH = '266'
    NO_ENTRY = '267'
    U_TURN = '272'
    MAX_SPEED = '274'
    MAX_SPEED_ZONE_START = '274.1'
    MAX_SPEED_ZONE_END = '274.2'
    MIN_SPEED = '275'
    NO_OVERTAKING_START = '276'
    NO_OVERTAKING_TRUCKS_START = '277'
    MAX_SPEED_END = '278'
    NO_OVERTAKING_TRUCKS_END = '281'
    ALL_MAX_SPEED_AND_OVERTAKING_END = '282'
    RIGHT_OF_WAY = '301'
    PRIORITY = '306'
    PRIORITY_OVER_ONCOMING = '308'
    TOWN_SIGN = '310'
    TRAFFIC_CALMED_AREA_START = '325.1'
    TRAFFIC_CALMED_AREA_END = '325.2'
    TUNNEL = '327'
    INTERSTATE_START = '330.1'
    INTERSTATE_END = '330.2'
    HIGHWAY_START = '331.1'
    HIGHWAY_END = '331.2'
    EXIT_BUILT_UP = '333-21'
    EXIT_GENERAL = '333-22'
    PEDESTRIANS_CROSSING = '350'
    DEAD_END = '357'
    DIRECTION_SIGN_LEFT_SINGLE = '625-10'
    DIRECTION_SIGN_LEFT_SMALL = '625-11'
    DIRECTION_SIGN_LEFT_MEDIUM = '625-12'
    DIRECTION_SIGN_LEFT_LARGE = '625-13'
    DIRECTION_SIGN_RIGHT_SINGLE = '625-20'
    DIRECTION_SIGN_RIGHT_SMALL = '625-21'
    DIRECTION_SIGN_RIGHT_MEDIUM = '625-22'
    DIRECTION_SIGN_RIGHT_LARGE = '625-23'
    WARNING_PANEL_RIGHT = '626-10'
    WARNING_PANEL_LEFT = '626-20'
    WARNING_PANEL_STRAIGHT_BROAD = '626-30'
    WARNING_PANEL_STRAIGHT_HIGH = '626-31'
    GREEN_ARROW = '720'
    ADDITION_LEFT_DIRECTION_1 = '1000-10'
    ADDITION_LEFT_DIRECTION_DANGER_POINT = '1000-11'
    ADDITION_RIGHT_DIRECTION_1 = '1000-20'
    ADDITION_RIGHT_DIRECTION_DANGER_POINT = '1000-21'
    ADDITION_BOTH_DIRECTIONS_HORIZONTAL = '1000-30'
    ADDITION_BOTH_DIRECTIONS_VERTICAL = '1000-31'
    ADDITION_VALID_FOR_X_METERS = '1001-30'
    ADDITION_VALID_FOR_X_KILOMETERS = '1001-31'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD = '1002-10'
    ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD = '1002-11'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-12'
    ADDITION_LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD = '1002-13'
    ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-14'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD = '1002-20'
    ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD = '1002-21'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD = '1002-22'
    ADDITION_RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD = '1002-23'
    ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD = '1002-24'
    ADDITION_VALID_IN_X_METERS = '1004-30'
    ADDITION_VALID_IN_X_KILOMETERS = '1004-31'
    ADDITION_RESIDENTS_PERMITTED = '1020-30'
    ADDITION_BICYCLES_PERMITTED = '1022-10'
    ADDITION_CARS_PERMITTED = '1024-10'
    ADDITION_AGRICULTURE_PERMITTED = '1026-36'
    ADDITION_FOREST_PERMITTED = '1026-37'
    ADDITION_AGRICULTURE_FOREST_PERMITTED = '1026-38'
    ADDITION_TIME_PERIOD_PERMITTED = '1040-30'
    ALLOWED_MASS_7_5_TONS = '1053-33'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDUsa(enum.Enum):
    MAX_SPEED = 'R2-1'
    U_TURN = 'R3-4'
    ROAD_WORK_AHEAD = 'CW20-1'
    NO_PARKING_ANY_TIME = 'R7-1'
    NO_STANDING = 'R7-4'
    TOW_AWAY_ZONE = 'R7-201a'
    ONE_WAY_LEFT = 'R6-1L'
    ONE_WAY_RIGHT = 'R6-1R'
    DO_NOT_ENTER = 'R5-1'
    NO_LEFT_TURN = 'R3-2'
    RIGHT_TURN_ONLY = 'R3-5R'
    TURN_ONLY_LANES = 'R3-8b'
    NO_RIGHT_TURN = 'R3-1'
    KEEP_RIGHT = 'R4-7'
    SIGNAL_AHEAD = 'W3-3'
    LOADING_ZONE = 'R8-3gP'
    NO_PARKING = 'R8-3'
    LEFT_TURN_ONLY = 'R3-5L'
    NO_STRAIGHT_THROUGH = 'R3-27'
    LEFT_REVERSE_TURN = 'W1-3L'
    PEDESTRIAN_WARNING = 'W11-2'
    DIAGONAL_DOWNWARD_LEFT_ARROW = 'M6-2aL'
    RIGHT_LANE_ENDS = 'W4-2R'
    RESERVED_HANDICAP_PARKING = 'R7-8'
    NO_PARKING_BUS_STOP = 'R7-107'
    ON_PAVEMENT = 'R8-3C'
    DO_NOT_BLOCK_INTERSECTION = 'R10-7'
    WARNING_ARROW_LEFT = 'W1-6L'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDChina(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''


@enum.unique
class TrafficSignIDSpain(enum.Enum):
    YIELD = 'r1'
    STOP = 'r2'
    BAN_ALL_VEHICLES = 'r100'
    NO_ENTRY = 'r101'
    BAN_TRUCKS = 'r106'
    MAX_WEIGHT = 'r107'
    MAX_HEIGHT = 'r205'
    MAX_SPEED = 'r301'
    NO_OVERTAKING_START = 'r305'
    NO_STOPPING = 'r307'
    NO_WAITING = 'r308'
    PEDESTRIANS_CROSSING = 's13'
    UNKNOWN = ''


@enum.unique
class TrafficSignIDRussia(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficSignIDArgentina(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficSignIDBelgium(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficSignIDFrance(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficSignIDGreece(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficSignIDCroatia(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''


@enum.unique
class TrafficSignIDItaly(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''


@enum.unique
class TrafficSignIDPuertoRico(enum.Enum):
    MAX_SPEED = '274'  # TODO: change to actual ID
    UNKNOWN = ''  # TODO: add actual IDs


@enum.unique
class TrafficLightDirection(enum.Enum):
    """
    Enum for all the possible directions for a traffic signal
    """
    RIGHT = "right"
    STRAIGHT = "straight"
    LEFT = "left"
    LEFT_STRAIGHT = "leftStraight"
    STRAIGHT_RIGHT = "straightRight"
    LEFT_RIGHT = "leftRight"
    ALL = "all"


@enum.unique
class TrafficLightState(enum.Enum):
    """
    Enum for the possible types of traffic light in signals
    """
    RED = "red"
    YELLOW = "yellow"
    RED_YELLOW = "redYellow"
    GREEN = "green"
    INACTIVE = "inactive"


class TrafficSignElement:
    """ Class which represents a collection of traffic signs at one position"""
    def __init__(self, traffic_sign_element_id: Union[TrafficSignIDZamunda, TrafficSignIDUsa, TrafficSignIDSpain,
                                                      TrafficSignIDGermany, TrafficSignIDChina, TrafficSignIDRussia],
                 additional_values: List[str] = []):
        """

        :param traffic_sign_element_id: ID of traffic sign element (must be element of a traffic sign element enum)
        :param additional_values: list of additional values of a traffic sign element, e.g. velocity, time, city name
        """
        self._traffic_sign_element_id = traffic_sign_element_id
        self._additional_values = additional_values

    @property
    def traffic_sign_element_id(self) -> enum:
        return self._traffic_sign_element_id

    @property
    def additional_values(self) -> List[str]:
        return self._additional_values

    def __eq__(self, other: 'TrafficSignElement'):
        if self.traffic_sign_element_id == other.traffic_sign_element_id \
                and self.additional_values == other.additional_values:
            return True
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(str(self. _traffic_sign_element_id) + str(self.additional_values))

    def __str__(self):
        return f"Sign Element with id {self._traffic_sign_element_id} and values {self._additional_values} "

    def __repr__(self):
        return f"Sign Element with id {self._traffic_sign_element_id} and values {self._additional_values} "


class TrafficSign(IDrawable):
    """Class to represent a traffic sign"""

    def __init__(self, traffic_sign_id: int,
                 traffic_sign_elements: List[TrafficSignElement],
                 first_occurrence: Set[int], position: np.ndarray,
                 virtual: bool = False):
        """
        :param traffic_sign_id: ID of traffic sign
        :param traffic_sign_elements: list of traffic sign elements
        :param first_occurrence: lanelet ID where traffic sign first appears
        :param position: position of traffic sign
        :param virtual: boolean indicating if this traffic sign is also
        placed there in the real environment or it
        is added for other reasons (e.g., completeness of scenario)
        """
        self._traffic_sign_id = traffic_sign_id
        self._position = position
        self._traffic_sign_elements = traffic_sign_elements
        self._virtual = virtual
        self._first_occurrence = first_occurrence

    @property
    def traffic_sign_id(self) -> int:
        return self._traffic_sign_id

    @property
    def position(self) -> Union[None,np.ndarray]:
        return self._position

    @property
    def traffic_sign_elements(self) -> List[TrafficSignElement]:
        return self._traffic_sign_elements

    @property
    def virtual(self) -> bool:
        return self._virtual

    @property
    def first_occurrence(self) -> Set[int]:
        return self._first_occurrence

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a traffic sign

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation, 2), '<TrafficSign/translate_rotate>: argument translation is ' \
                                                      'not a vector of real ' \
                                                      'numbers of length 2.'
        assert is_real_number(
            angle), '<TrafficSign/translate_rotate>: argument angle must be a ' \
                    'scalar. ' \
                    'angle = %s' % angle
        assert is_valid_orientation(
                angle), '<TrafficSign/translate_rotate>: argument angle must ' \
                        'be ' \
                        'within the ' \
                        'interval [-2pi, 2pi]. angle = %s' % angle
        self._position = commonroad.geometry.transform.translate_rotate(
                np.array([self._position]), translation, angle)[0]

    def __str__(self):
        return f"Sign At {self._position} with {self._traffic_sign_elements} "

    def draw(self, renderer: IRenderer,
             draw_params: Union[ParamServer, dict, None] = None,
             call_stack: Optional[Tuple[str, ...]] = tuple()):
        renderer.draw_traffic_light_sign(self, draw_params, call_stack)


class TrafficLightCycleElement:
    """Class to represent a traffic light cycle"""
    def __init__(self, state: TrafficLightState, duration: int):
        """
        :param state: state of a traffic light cycle element
        :param duration: duration of traffic light cycle element
        """
        self._state = state
        self._duration = duration

    @property
    def state(self) -> TrafficLightState:
        return self._state

    @property
    def duration(self) -> int:
        return self._duration


class TrafficLight(IDrawable):
    """ Class to represent a traffic light"""

    def __init__(self, traffic_light_id: int,
                 cycle: List[TrafficLightCycleElement], position: np.ndarray,
                 time_offset: int = 0,
                 direction: TrafficLightDirection = TrafficLightDirection.ALL,
                 active: bool = True):
        """
        :param traffic_light_id: ID of traffic light
        :param cycle: list of traffic light cycle elements
        :param time_offset: offset of traffic light cycle
        :param position: position of traffic light
        :param direction: driving directions for which the traffic light is valid
        :param active: boolean indicating if traffic light is currently active
        """
        self._traffic_light_id = traffic_light_id
        if len(cycle) == 0:
            self._cycle = get_default_cycle()
        else:
            self._cycle = cycle
        self._time_offset = time_offset
        self._position = position
        self._direction = direction
        self._active = active

    @property
    def traffic_light_id(self) -> int:
        return self._traffic_light_id

    @property
    def cycle(self) -> List[TrafficLightCycleElement]:
        return self._cycle

    def get_state_at_time_step(self, time_step: int) -> TrafficLightState:
        time_step_mod = ((time_step - self.time_offset) % (self.cycle_init_timesteps[-1] - self.time_offset))\
                        + self.time_offset
        i_cycle = np.argmax(time_step_mod < self.cycle_init_timesteps) - 1
        return self.cycle[i_cycle].state

    @property
    def cycle_init_timesteps(self):
        if not hasattr(self, '_cycle_init_timesteps'):
            durations = [cycle_el.duration for cycle_el in self._cycle]
            self._cycle_init_timesteps = np.cumsum(durations) + self.time_offset
            self._cycle_init_timesteps = np.insert(self._cycle_init_timesteps, 0, self.time_offset)
        return self._cycle_init_timesteps

    @property
    def time_offset(self) -> int:
        return self._time_offset

    @property
    def position(self) -> np.ndarray:
        return self._position

    @property
    def direction(self) -> TrafficLightDirection:
        return self._direction

    @direction.setter
    def direction(self, direction: TrafficLightDirection):
        self._direction = direction

    @property
    def active(self) -> bool:
        return self._active

    def translate_rotate(self, translation: np.ndarray, angle: float):
        """
        This method translates and rotates a traffic light

        :param translation: The translation given as [x_off,y_off] for the x and y translation
        :param angle: The rotation angle in radian (counter-clockwise defined)
        """

        assert is_real_number_vector(translation,
                                     2), '<TrafficLight/translate_rotate>: ' \
                                         'argument translation is ' \
                                         'not a vector of real numbers of ' \
                                         'length 2.'
        assert is_real_number(
                angle), '<TrafficLight/translate_rotate>: argument angle must ' \
                        'be ' \
                        'a scalar. ' \
                        'angle = %s' % angle
        assert is_valid_orientation(
                angle), '<TrafficLight/translate_rotate>: argument angle must ' \
                        'be ' \
                        'within the ' \
                        'interval [-2pi, 2pi]. angle = %s' % angle
        self._position = commonroad.geometry.transform.translate_rotate(
                np.array([self._position]), translation, angle)[0]

    def draw(self, renderer: IRenderer,
             draw_params: Union[ParamServer, dict, None] = None,
             call_stack: Optional[Tuple[str, ...]] = tuple()):
        renderer.draw_traffic_light_sign(self, draw_params, call_stack)


def get_default_cycle():
    """
    Defines default traffic light cycle in case no cycle is provided

    _:returns traffic light cycle element
    """
    cycle = [(TrafficLightState.RED, 60),
             (TrafficLightState.RED_YELLOW, 10),
             (TrafficLightState.GREEN, 60),
             (TrafficLightState.YELLOW, 10)]
    cycle_element_list = [TrafficLightCycleElement(state[0], state[1]) for state in cycle]
    return cycle_element_list

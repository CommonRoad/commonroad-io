from datetime import datetime
import enum
import warnings
import re
import iso3166
from typing import Union, List, Optional
from numpy import double
from commonroad.common.util import Time
from commonroad import SCENARIO_VERSION, SUPPORTED_COMMONROAD_VERSIONS


class ScenarioID:
    benchmark_id_pattern = re.compile(r"(?P<cooperative>C-)?(?P<country_id>[A-Z]{3})_(?P<map_name>[a-zA-Z0-9]+)-("
                                      r"?P<map_id>[1-9][0-9]*)(_(?P<configuration_id>[1-9][0-9]*)(_("
                                      r"?P<prediction_type>[STPI])(?P<prediction_ids>(-([1-9][0-9]*))+))?)?")

    def __init__(self, cooperative: bool = False, country_id: str = "ZAM", map_name: str = "Test", map_id: int = 1,
                 configuration_id: Union[None, int] = None, obstacle_behavior: Union[None, str] = None,
                 prediction_id: Union[None, int, List[int]] = None, scenario_version: str = SCENARIO_VERSION):
        """
        Implements the scenario ID as specified in the scenario documentation
        (see https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/tree/master/documentation)
        Example for benchmark ID C-USA_US101-33_2_T-1
        :param cooperative: True if scenario contains cooperative planning problem sets with multiple planning problems
        :param country_id: three-letter ID according
        :param map_name: name of the map (e.g. US101)
        :param map_id: index of the map (e.g. 33)
        :param configuration_id: enumerates initial configuration of vehicles on the map (e.g. 2)
        :param obstacle_behavior: describes how behavior of surrounding vehicles is defined;
        interactive (I) or prediction type (S, T)
        :param prediction_id: enumerates different predictions for the same initial configuration (e.g. 1)
        :param scenario_version: scenario version identifier (e.g. 2020a)
        """
        assert scenario_version in SUPPORTED_COMMONROAD_VERSIONS, 'Scenario_version {} not supported.'.format(
                scenario_version)
        self.scenario_version: str = scenario_version
        self.cooperative: bool = cooperative
        self._country_id = None
        self._map_name = None
        self.country_id: str = country_id
        self.map_name = map_name
        self.map_id: int = map_id
        is_map = configuration_id is None and obstacle_behavior is None and prediction_id is None
        has_prediction = obstacle_behavior is not None or prediction_id is not None

        # Obstacle behavior has to be defined if a prediction id is given. We can't recover from this!
        # prediction id is not None => obstacle_behavior is not None
        assert prediction_id is None or obstacle_behavior is not None, "Prediction id was given, but obstacle " \
                                                                       "behavior undefined!"
        if not is_map:
            if has_prediction:
                prediction_id = prediction_id or 1
            configuration_id = configuration_id or 1
        self.obstacle_behavior: Union[None, str] = obstacle_behavior
        self.configuration_id: Union[None, int] = configuration_id
        self.prediction_id: Union[None, int, List[int]] = prediction_id

        # Validate object
        assert self.obstacle_behavior in [None, "S", "T", "P",
                                          "I"], f"Unsupported prediction type '{obstacle_behavior}'! " \
                                                f"Available prediction types: S, T, P, I"
        assert self.map_id > 0, f"Map id {configuration_id} <= 0!"
        assert is_map or self.configuration_id > 0, f"Configuration id {configuration_id} <= 0!"
        prediction_id = prediction_id if isinstance(prediction_id, list) else [prediction_id]
        assert not has_prediction or all(p > 0 for p in prediction_id), f"Prediction id {configuration_id} <= 0!"

    def __eq__(self, other):
        if not isinstance(other, ScenarioID):
            warnings.warn(f"Inequality between ScenarioID {repr(self)} and different type {type(other)}")
            return False

        id_eq = self.cooperative == other.cooperative and self.country_id == other.country_id and \
            self.map_name == other.map_name and self.map_id == other.map_id and \
            self.configuration_id == other.configuration_id and self.obstacle_behavior == other.obstacle_behavior and \
            self.prediction_id == other.prediction_id and self.scenario_version == other.scenario_version

        return id_eq

    def __hash__(self):
        return hash((self.cooperative, self.country_id, self.map_name, self.map_id, self.configuration_id,
                     self.obstacle_behavior, self.prediction_id, self.scenario_version))

    def __str__(self):
        if self.obstacle_behavior is not None:
            prediction_id = self.prediction_id
            if not isinstance(self.prediction_id, list):
                prediction_id = [prediction_id]
            prediction = "-".join([self.obstacle_behavior] + [str(s) for s in prediction_id])
        else:
            prediction = None
        map_ = self.map_name if self.map_id is None else f"{self.map_name}-{self.map_id}"
        parts = [self.country_id, map_, self.configuration_id, prediction]
        scenario_id = "_".join([str(p) for p in parts if p is not None])
        if self.cooperative is True:
            scenario_id = "C-" + scenario_id

        return scenario_id

    @property
    def map_name(self):
        return self._map_name

    @map_name.setter
    def map_name(self, map_name: str):
        pattern = "[^a-zA-Z0-9]"
        cleaned_map_name = re.sub(pattern, "", map_name)
        self._map_name = cleaned_map_name

    @property
    def country_id(self):
        return self._country_id

    @country_id.setter
    def country_id(self, country_id: str):
        if country_id is None:
            self._country_id = 'ZAM'
        elif country_id in iso3166.countries_by_alpha3 or country_id == 'ZAM':
            self._country_id = country_id
        else:
            raise ValueError('Country ID {} is not in the ISO-3166 three-letter format. '.format(country_id))

    @property
    def prediction_type(self):
        warnings.warn("prediction_type renamed to obstacle_behavior", DeprecationWarning)
        return self.obstacle_behavior

    @property
    def country_name(self):
        if self.country_id == "ZAM":
            return "Zamunda"
        else:
            return iso3166.countries_by_alpha3[self.country_id].name

    @classmethod
    def from_benchmark_id(cls, benchmark_id: str, scenario_version: str):
        """
        Create ScenarioID from benchmark_id and scenario_version in the XML header.
        :param benchmark_id: scenario ID provided as a string
        :param scenario_version: scenario format version (e.g. 2020a)
        :return:
        """
        match = ScenarioID.benchmark_id_pattern.fullmatch(benchmark_id)
        if match is None:
            warnings.warn('Not a valid scenario ID: ' + benchmark_id)
            return ScenarioID(cooperative=False, map_name=benchmark_id, map_id=1)

        # extract sub IDs from string
        cooperative = match["cooperative"] is not None

        country_id = match["country_id"]
        map_name = match["map_name"]
        map_id = int(match["map_id"])

        configuration_id = int(match["configuration_id"]) if match["configuration_id"] is not None else None
        prediction_type = match["prediction_type"]
        prediction_id = match["prediction_ids"]
        if prediction_id is not None:
            prediction_id = [int(pid) for pid in prediction_id.split("-")[1:]]
            if len(prediction_id) == 1:
                prediction_id = prediction_id[0]

        return ScenarioID(cooperative, country_id, map_name, map_id, configuration_id, prediction_type, prediction_id,
                          scenario_version)


class FileInformation:
    """
    Class that contains meta information regarding the files
    """

    def __init__(self, date: Optional[Time] = None, author: str = "", affiliation: str = "",
                 source: str = "", license_name: str = "", license_text: str = None):
        """
        :param date: date of the scenario
        :param author: authors of the CommonRoad scenario
        :param affiliation: institution of the authors
        :param source: source of the scenario, e.g. generated by a map converter and a traffic simulator
        :param license_name: license name of the scenario
        :param license_text: license text of the scenario
        """
        if date is None:
            time = datetime.now()
            self._date = Time(time.hour, time.minute, time.day, time.month, time.year)
        else:
            self._date = date
        self._author = author
        self._affiliation = affiliation
        self._source = source

        self._license_name = license_name
        if license_text is None:
            self._license_text = ""
        else:
            self._license_text = license_text

    def __eq__(self, other):
        if not isinstance(other, FileInformation):
            warnings.warn(f"Inequality between FileInformation {repr(self)} and different type {type(other)}")
            return False

        return self._license_name == other.license_name and self._license_text == other.license_text and \
            self._date == other.date and self._author == other.author and self._affiliation == other.affiliation and \
            self._source == other.source

    def __hash__(self):
        return hash((self._date, self._author, self._affiliation, self._source, self._license_name, self._license_text))

    @property
    def license_name(self) -> str:
        """ license name of the scenario."""
        return self._license_name

    @license_name.setter
    def license_name(self, license_name: str):
        self._license_name = license_name

    @property
    def license_text(self) -> Union[None, str]:
        """ license text of the scenario."""
        return self._license_text

    @license_text.setter
    def license_text(self, license_text: Union[None, str]):
        if license_text is None:
            self._license_name = ""
        else:
            self._license_text = license_text

    @property
    def date(self) -> Time:
        """ date of the scenario."""
        return self._date

    @date.setter
    def date(self, date: Time):
        self._date = date

    @property
    def author(self) -> str:
        """ authors of the CommonRoad scenario."""
        return self._author

    @author.setter
    def author(self, author: str):
        self._author = author

    @property
    def affiliation(self) -> str:
        """ institution of the authors."""
        return self._affiliation

    @affiliation.setter
    def affiliation(self, affiliation: str):
        self._affiliation = affiliation

    @property
    def source(self) -> str:
        """ source of the scenario, e.g. generated by a map converter and a traffic simulator."""
        return self._source

    @source.setter
    def source(self, source: str):
        self._source = source


class MapMetaInformation:
    """
    Class that contains meta information for a map
    """

    def __init__(self, scenario_id: ScenarioID = ScenarioID(), file_information: FileInformation = FileInformation()):
        """
        :param scenario_id: CommonRoad scenario ID containing only map information
        :param file_information: CommonRoad file information
        """
        self._scenario_id = scenario_id
        self._file_information = file_information

    def __eq__(self, other):
        if not isinstance(other, ScenarioMetaInformation):
            warnings.warn(f"Inequality between ScenarioMetaInformation {repr(self)} and different type {type(other)}")
            return False

        return self._scenario_id == other.scenario_id and self._file_information == other.file_information

    def __hash__(self):
        return hash((self._scenario_id, self._file_information))

    @property
    def scenario_id(self) -> ScenarioID:
        """ CommonRoad scenario ID of the scenario."""
        return self._scenario_id

    @scenario_id.setter
    def scenario_id(self, scenario_id: ScenarioID):
        self._scenario_id = scenario_id

    @property
    def file_information(self) -> FileInformation:
        """ File information of the scenario."""
        return self._file_information

    @file_information.setter
    def file_information(self, file_information: FileInformation):
        self._file_information = file_information

    @property
    def complete_map_name(self) -> str:
        """ Creates map name as string"""
        return f"{self._scenario_id.country_id}_{self._scenario_id.map_name}-{self._scenario_id.map_id}"


class ScenarioMetaInformation:
    """
    Class that contains meta information for a scenario
    """

    def __init__(self, scenario_id: ScenarioID, file_information: FileInformation, time_step_size: double):
        """
        :param scenario_id: CommonRoad scenario ID
        :param time_step_size: global time step size of the time-discrete scenario
        :param file_information: CommonRoad file information
        """
        self._scenario_id = scenario_id
        self._time_step_size = time_step_size
        self._file_information = file_information

    def __eq__(self, other):
        if not isinstance(other, ScenarioMetaInformation):
            warnings.warn(f"Inequality between ScenarioMetaInformation {repr(self)} and different type {type(other)}")
            return False

        return self._scenario_id == other.scenario_id and self._file_information == other.file_information and \
            self._time_step_size == other.time_step_size

    def __hash__(self):
        return hash((self._scenario_id, self._file_information, self._time_step_size))

    @property
    def scenario_id(self) -> ScenarioID:
        """ CommonRoad scenario ID of the scenario."""
        return self._scenario_id

    @scenario_id.setter
    def scenario_id(self, scenario_id: ScenarioID):
        self._scenario_id = scenario_id

    @property
    def file_information(self) -> FileInformation:
        """ File information of the scenario."""
        return self._file_information

    @file_information.setter
    def file_information(self, file_information: FileInformation):
        self._file_information = file_information

    @property
    def time_step_size(self) -> double:
        """ global time step size of the time-discrete scenario."""
        return self._time_step_size

    @time_step_size.setter
    def time_step_size(self, time_step_size: double):
        self._time_step_size = time_step_size


@enum.unique
class TimeOfDay(enum.Enum):
    """ Enum containing all possible time of days."""
    NIGHT = "night"
    SUNSET = "sunset"
    AFTERNOON = "afternoon"
    NOON = "noon"
    MORNING = "morning"
    UNKNOWN = "unknown"


@enum.unique
class Weather(enum.Enum):
    """ Enum containing all possible weathers."""
    CLEAR = "clear"
    LIGHT_RAIN = "light_rain"
    MID_RAIN = "mid_rain"
    HEAVY_RAIN = "heavy_rain"
    FOG = "fog"
    SNOW = "snow"
    HAIL = "hail"
    CLOUDY = "cloudy"
    UNKNOWN = "unknown"


@enum.unique
class Underground(enum.Enum):
    """ Enum containing all possible undergrounds."""
    WET = "wet"
    CLEAN = "clean"
    DIRTY = "dirty"
    DAMAGED = "damaged"
    SNOW = "snow"
    ICE = "ice"
    UNKNOWN = "unknown"


class Environment:
    """
    Class which describes the environment where a scenario takes place as specified in the CommonRoad specification.
    """

    def __init__(self, time: Time = None, time_of_day: TimeOfDay = None, weather: Weather = None,
                 underground: Underground = None):
        """
        Constructor of an environment object

        :param time: time in hours
        :param time_of_day: current time of day, i.e., day or night
        :param weather: weather information, e.g., sunny
        :param underground: underground information, e.g., ice
        """
        self._time = time
        self._time_of_day = time_of_day
        self._weather = weather
        self._underground = underground

    def __eq__(self, other):
        if not isinstance(other, Environment):
            return False

        return self._time == other.time and self._time_of_day == other.time_of_day and \
            self._weather == other.weather and self._underground == other.underground

    def __hash__(self):
        return hash((self._time, self._time_of_day, self._weather, self._underground))

    @property
    def time(self) -> Time:
        return self._time

    @time.setter
    def time(self, time: Time):
        self._time = time

    @property
    def time_of_day(self) -> TimeOfDay:
        return self._time_of_day

    @time_of_day.setter
    def time_of_day(self, time_of_day: TimeOfDay):
        self._time_of_day = time_of_day

    @property
    def weather(self) -> Weather:
        return self._weather

    @weather.setter
    def weather(self, weather: Weather):
        self._weather = weather

    @property
    def underground(self) -> Underground:
        return self._underground

    @underground.setter
    def underground(self, underground: Underground):
        self._underground = underground


class GeoTransformation:
    """
    Class which describes the transformation from geodetic to projected Cartesian coordinates according to the
    CommonRoad specification
    """

    def __init__(self, geo_reference: str = None, x_translation: float = None, y_translation: float = None,
                 z_rotation: float = None, scaling: float = None):
        """
        Constructor of a location object

        :param geo_reference: proj-string describing transformation from geodetic to projected Cartesian coordinates
        :param x_translation: translation value for x-coordinates
        :param y_translation: translation value for y-coordinates
        :param z_rotation: rotation value around origin
        :param scaling: multiplication value of x- and y-coordinates
        """
        self.geo_reference = geo_reference
        self.x_translation = x_translation
        self.y_translation = y_translation
        self.z_rotation = z_rotation
        self.scaling = scaling

    def __eq__(self, other):
        if not isinstance(other, GeoTransformation):
            return False

        return self._geo_reference == other.geo_reference and self._x_translation == other.x_translation and \
            self._y_translation == other.y_translation and self._z_rotation == other.z_rotation and \
            self._scaling == other.scaling

    def __hash__(self):
        return hash((self._geo_reference, self._x_translation, self._y_translation, self._z_rotation, self._scaling))

    @property
    def geo_reference(self) -> str:
        return self._geo_reference

    @property
    def x_translation(self) -> float:
        return self._x_translation

    @property
    def y_translation(self) -> float:
        return self._y_translation

    @property
    def z_rotation(self) -> float:
        return self._z_rotation

    @property
    def scaling(self) -> float:
        return self._scaling

    @geo_reference.setter
    def geo_reference(self, geo_reference):
        self._geo_reference = geo_reference if geo_reference is not None else 0

    @x_translation.setter
    def x_translation(self, x_translation):
        self._x_translation = x_translation if x_translation is not None else 0

    @y_translation.setter
    def y_translation(self, y_translation):
        self._y_translation = y_translation if y_translation is not None else 0

    @z_rotation.setter
    def z_rotation(self, z_rotation):
        self._z_rotation = z_rotation if z_rotation is not None else 0

    @scaling.setter
    def scaling(self, scaling):
        self._scaling = scaling if scaling is not None else 1


class Location:
    """
    Class which describes a location according to the CommonRoad specification.
    """

    def __init__(self, geo_name_id: int = -999, gps_latitude: float = 999, gps_longitude: float = 999,
                 geo_transformation: GeoTransformation = None, environment: Environment = None):
        """
        Constructor of a location object

        :param geo_name_id: GeoName ID
        :param gps_latitude: GPS latitude coordinate
        :param gps_longitude: GPS longitude coordinate
        :param geo_transformation: description of geometric transformation during scenario generation
        :param environment: environmental information, e.g. weather
        """
        self._geo_name_id = geo_name_id
        self._gps_latitude = gps_latitude
        self._gps_longitude = gps_longitude
        self._geo_transformation = geo_transformation
        self._environment = environment

    def __eq__(self, other):
        if not isinstance(other, Location):
            return False

        return self._geo_name_id == other.geo_name_id and self._gps_latitude == other.gps_latitude and \
            self._gps_longitude == other.gps_longitude and self._geo_transformation == other.geo_transformation and \
            self._environment == other.environment

    def __hash__(self):
        return hash((self._geo_name_id, self._gps_latitude, self._gps_longitude, self._geo_transformation,
                     self._environment))

    @property
    def geo_name_id(self) -> int:
        return self._geo_name_id

    @geo_name_id.setter
    def geo_name_id(self, geo_name_id: int):
        self._geo_name_id = geo_name_id

    @property
    def gps_latitude(self) -> float:
        return self._gps_latitude

    @gps_latitude.setter
    def gps_latitude(self, gps_latitude: float):
        self._gps_latitude = gps_latitude

    @property
    def gps_longitude(self) -> float:
        return self._gps_longitude

    @gps_longitude.setter
    def gps_longitude(self, gps_longitude: float):
        self._gps_longitude = gps_longitude

    @property
    def geo_transformation(self) -> GeoTransformation:
        return self._geo_transformation

    @geo_transformation.setter
    def geo_transformation(self, geo_transformation: GeoTransformation):
        self._geo_transformation = geo_transformation

    @property
    def environment(self) -> Environment:
        return self._environment

    @environment.setter
    def environment(self, environment: Environment):
        self._environment = environment

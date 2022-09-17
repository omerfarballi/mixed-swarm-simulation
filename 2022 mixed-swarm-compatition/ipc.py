import mmap
import os
import struct
import tempfile
import time

# ================================================ PUBLIC CONSTANTS ================================================

MOTOR_STATE__STOPPED = 1
MOTOR_STATE__STARTED = 2
MOTOR_STATE__STOPPING = 3
MOTOR_STATE__STARTING = 4

RESULT__SUCCESS = 0
RESULT__ERROR_UNSPECIFIED = 1
RESULT__ERROR_TRY_AGAIN_LATER = 2
RESULT__ERROR_VEHICLE_IN_MOTION = 3
RESULT__ERROR_VEHICLE_TOO_FAST = 4
RESULT__ERROR_MOTOR_IS_NOT_STARTED = 5
RESULT__ERROR_MOTOR_IS_NOT_STOPPED = 6
RESULT__ERROR_OUT_OF_FUEL = 7
RESULT__ERROR_STATION_NOT_FOUND = 8
RESULT__ERROR_ALREADY_ATTACHED_FOR_THIS_SUPPLY_TYPE = 9
RESULT__ERROR_STATION_NOT_SERVING_SUPPLY_TYPE = 10
RESULT__ERROR_STATION_OUT_OF_RANGE = 11
RESULT__ERROR_STATION_OUT_OF_CAPACITY = 12
RESULT__ERROR_STATION_NOT_ATTACHED_YET = 13
RESULT__ERROR_TARGET_LOCATION_IS_TOO_FAR = 14
RESULT__ERROR_TARGET_LOCATION_IS_NOT_REACHABLE = 15
RESULT__ERROR_VICTIM_ID_IS_NOT_VALID = 16
RESULT__ERROR_VEHICLE_ID_IS_NOT_VALID = 17
RESULT__ERROR_VICTIM_IS_NOT_CALLABLE = 18
RESULT__ERROR_VICTIM_IS_TOO_FAR_TO_CALL = 19
RESULT__ERROR_VICTIM_IS_NOT_LOADABLE = 20
RESULT__ERROR_VICTIM_IS_TOO_FAR_TO_LOAD = 21
RESULT__ERROR_VEHICLE_PASSENGER_CAPACITY_REACHED = 22
RESULT__ERROR_CANNOT_LOAD_TO_INACTIVE_VEHICLE = 23
RESULT__ERROR_VICTIM_IS_NOT_LOADED = 24
RESULT__ERROR_DROPPING_POINT_IS_NOT_WALKABLE = 25
RESULT__ERROR_DROPPING_POINT_IS_TOO_FAR = 26
RESULT__ERROR_VICTIM_MUST_BE_UNLOADED_TO_ARRIVE = 27
RESULT__ERROR_VICTIM_MUST_BE_UNLOADED_TO_HALT = 28
RESULT__ERROR_DESTINATION_ID_IS_NOT_VALID = 29
RESULT__ERROR_DESTINATION_IS_TOO_FAR = 30

CRASH_CAUSE__UNSPECIFIED = 0
CRASH_CAUSE__VEHICLE_COLLISION_A = 1
CRASH_CAUSE__VEHICLE_COLLISION_B = 2
CRASH_CAUSE__SPEED_VIOLATION = 3
CRASH_CAUSE__BACKWARD_FAILURE = 4
CRASH_CAUSE__OUT_OF_FUEL = 5
CRASH_CAUSE__ACTION_ON_SUPPLY = 6

VICTIM_STATE__STANDING = 0
VICTIM_STATE__CALLING = 1
VICTIM_STATE__ON_BOARD = 2
VICTIM_STATE__ARRIVING = 3

VEHICLE_TYPE__UGV = 1
VEHICLE_TYPE__USV = 2

EVC_COMMAND__NO_OP = 0
EVC_COMMAND__CALL = 1
EVC_COMMAND__LOAD = 2
EVC_COMMAND__UNLOAD = 3
EVC_COMMAND__SEND = 4
EVC_COMMAND__STOP = 5

# ================================================ PRIVATE CONSTANTS ================================================

_FILE_NAME = "KARMASIM_MMAP_FILE_1.bin"
_FILE_SIZE = 256 * 1024 * 1024

_B_SURFACE_EDGE_DATA = 0
_B_ROAD_NODE = 1
_B_ROAD_EDGE = 2
_B_UGV_DATA = 3
_B_USV_DATA = 4
_B_FIRE_DATA = 5
_B_STATION_DATA = 6
_B_DESTINATION_DATA = 7
_B_VICTIM_DATA = 8
_B_GAME_DATA = 9
_B_SURFACE_INFO = 10
_B_CMD_DEBUG_STATES = 11
_B_CMD_UGV = 12
_B_CMD_USV = 13
_B_CMD_VICTIM = 14
_B_CMD_DEBUG_SPHERE = 15
_B_CMD_DEBUG_TEXT = 16
_B_SCORING = 17
_B_CLIENT_CONTROL = 18

_BLOCK_SIZE_OF = [8, 12, 44, 182, 153, 36, 61, 28, 64, 12, 44, 6, 35, 33, 12, 29, 125, 28, 8]
_BLOCK_MAX_COUNT = [16384, 1024, 1024, 512, 512, 512, 256, 256, 256, 1, 1, 1, 16384, 16384, 16384, 4096, 4096, 1, 1]
_BLOCK_TOTAL_SIZE = [131076, 12292, 45060, 93188, 78340, 18436, 15620, 7172, 16388, 16, 48, 10, 573444, 540676, 196612,
                     118788, 512004, 32, 12]
_BLOCK_START_INDEX = [0, 131076, 143368, 188428, 281616, 359956, 378392, 394012, 401184, 417572, 417588, 417636, 417646,
                      991090, 1531766, 1728378, 1847166, 2359170, 2359202]


# ================================================ PRIVATE UTILITIES ================================================

class _SharedMemory:
    _file_name = None
    _file_size = None
    _file_path = None
    _file_obj = None
    _mmf = None

    def __init__(self, file_name, file_size):
        self._file_name = file_name
        self._file_size = file_size
        self._file_path = os.path.join(tempfile.gettempdir(), self._file_name)

        self._file_obj = open(self._file_path, mode="ab+")
        self._mmf = mmap.mmap(self._file_obj.fileno(), length=self._file_size, access=mmap.ACCESS_WRITE)

    def get_bool(self, pos):
        return self._mmf[pos] != 0

    def get_int8(self, pos):
        return self._mmf[pos]

    def get_uint8(self, pos):
        return struct.unpack("<B", self._mmf[pos:pos + 1])[0]

    def get_int16(self, pos):
        return struct.unpack("<h", self._mmf[pos:pos + 2])[0]

    def get_uint16(self, pos):
        return struct.unpack("<H", self._mmf[pos:pos + 2])[0]

    def get_int32(self, pos):
        return struct.unpack("<l", self._mmf[pos:pos + 4])[0]

    def get_uint32(self, pos):
        return struct.unpack("<L", self._mmf[pos:pos + 4])[0]

    def get_int64(self, pos):
        return struct.unpack("<q", self._mmf[pos:pos + 8])[0]

    def get_uint64(self, pos):
        return struct.unpack("<Q", self._mmf[pos:pos + 8])[0]

    def get_float32(self, pos):
        return struct.unpack("<f", self._mmf[pos:pos + 4])[0]

    def get_float64(self, pos):
        return struct.unpack("<d", self._mmf[pos:pos + 8])[0]

    def set_bool(self, pos, value):
        self._mmf[pos] = 1 if value else 0

    def set_int8(self, pos, value):
        self._mmf[pos] = value.to_bytes(1, byteorder='little', signed=True)

    def set_uint8(self, pos, value):
        self._mmf[pos:pos + 1] = value.to_bytes(1, byteorder='little', signed=False)

    def set_int16(self, pos, value):
        self._mmf[pos:pos + 2] = value.to_bytes(2, byteorder='little', signed=True)

    def set_uint16(self, pos, value):
        self._mmf[pos:pos + 2] = value.to_bytes(2, byteorder='little', signed=False)

    def set_int32(self, pos, value):
        self._mmf[pos:pos + 4] = value.to_bytes(4, byteorder='little', signed=True)

    def set_uint32(self, pos, value):
        self._mmf[pos:pos + 4] = value.to_bytes(4, byteorder='little', signed=False)

    def set_int64(self, pos, value):
        self._mmf[pos:pos + 8] = value.to_bytes(8, byteorder='little', signed=True)

    def set_uint64(self, pos, value):
        self._mmf[pos:pos + 8] = value.to_bytes(8, byteorder='little', signed=False)

    def set_float32(self, pos, value):
        self._mmf[pos:pos + 4] = bytearray(struct.pack("f", value))

    def set_float64(self, pos, value):
        self._mmf[pos:pos + 8] = bytearray(struct.pack("d", value))

    def set_bytes(self, pos, array):
        self._mmf[pos:pos + len(array)] = array


# ================================================ PRIVATE VARIABLES ================================================

_mem = _SharedMemory(_FILE_NAME, _FILE_SIZE)


# ================================================ PRIVATE FUNCTIONS ================================================

def _read_block_length(block_index):
    return _mem.get_int32(_BLOCK_START_INDEX[block_index])


def _write_block_length(block_index, count):
    return _mem.set_int32(_BLOCK_START_INDEX[block_index], count)


def _map_to_index(block_index, element_index, value_index):
    return _BLOCK_START_INDEX[block_index] + 4 + element_index * _BLOCK_SIZE_OF[block_index] + value_index


def _read_int32(block_index, element_index, value_index):
    return _mem.get_int32(_map_to_index(block_index, element_index, value_index))


def _read_bool(block_index, element_index, value_index):
    return _mem.get_bool(_map_to_index(block_index, element_index, value_index))


def _read_float32(block_index, element_index, value_index):
    return _mem.get_float32(_map_to_index(block_index, element_index, value_index))


def _read_vector2(block_index, element_index, value_index):
    return [_mem.get_float32(_map_to_index(block_index, element_index, value_index)),
            _mem.get_float32(_map_to_index(block_index, element_index, value_index + 4))]


def _read_vector3(block_index, element_index, value_index):
    return [_mem.get_float32(_map_to_index(block_index, element_index, value_index)),
            _mem.get_float32(_map_to_index(block_index, element_index, value_index + 4)),
            _mem.get_float32(_map_to_index(block_index, element_index, value_index + 8))]


def _write_uint8(block_index, element_index, value_index, value):
    _mem.set_uint8(_map_to_index(block_index, element_index, value_index), value)


def _write_int32(block_index, element_index, value_index, value):
    _mem.set_int32(_map_to_index(block_index, element_index, value_index), value)


def _write_int64(block_index, element_index, value_index, value):
    _mem.set_int64(_map_to_index(block_index, element_index, value_index), value)


def _write_bool(block_index, element_index, value_index, value):
    _mem.set_bool(_map_to_index(block_index, element_index, value_index), value)


def _write_float32(block_index, element_index, value_index, value):
    _mem.set_float32(_map_to_index(block_index, element_index, value_index), value)


def _write_vector2(block_index, element_index, value_index, value):
    _mem.set_float32(_map_to_index(block_index, element_index, value_index), value[0])
    _mem.set_float32(_map_to_index(block_index, element_index, value_index + 4), value[1])


def _write_vector3(block_index, element_index, value_index, value):
    _mem.set_float32(_map_to_index(block_index, element_index, value_index), value[0])
    _mem.set_float32(_map_to_index(block_index, element_index, value_index + 4), value[1])
    _mem.set_float32(_map_to_index(block_index, element_index, value_index + 8), value[2])


def _write_fstring(block_index, element_index, value_index, value):
    _mem.set_bytes(_map_to_index(block_index, element_index, value_index), value.encode('utf-8'))


# ================================================ PUBLIC FUNCTIONS ================================================

def perform_heartbeat():
    ClientControl.set_last_time(int(time.time() * 1000))


# ================================================ ACCESSORS ================================================

class RoadNode:
    @staticmethod
    def item_count(): return _read_block_length(_B_ROAD_NODE)

    @staticmethod
    def position(node_at_index): return _read_vector3(_B_ROAD_NODE, node_at_index, 0)


class RoadEdge:
    @staticmethod
    def item_count(): return _read_block_length(_B_ROAD_EDGE)

    @staticmethod
    def start_node(edge_at_index): return _read_int32(_B_ROAD_EDGE, edge_at_index, 0)

    @staticmethod
    def end_node(edge_at_index): return _read_int32(_B_ROAD_EDGE, edge_at_index, 4)

    @staticmethod
    def start_tangent(edge_at_index): return _read_vector3(_B_ROAD_EDGE, edge_at_index, 8)

    @staticmethod
    def end_tangent(edge_at_index): return _read_vector3(_B_ROAD_EDGE, edge_at_index, 20)

    @staticmethod
    def length(edge_at_index): return _read_float32(_B_ROAD_EDGE, edge_at_index, 32)

    @staticmethod
    def sharpness(edge_at_index): return _read_float32(_B_ROAD_EDGE, edge_at_index, 36)

    @staticmethod
    def speed_limit(edge_at_index): return _read_float32(_B_ROAD_EDGE, edge_at_index, 40)


class UgvData:
    @staticmethod
    def item_count(): return _read_block_length(_B_UGV_DATA)

    @staticmethod
    def id(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 0)

    @staticmethod
    def is_active(ugv_at_index): return _read_bool(_B_UGV_DATA, ugv_at_index, 4)

    @staticmethod
    def motor_state(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 5)

    @staticmethod
    def position(ugv_at_index): return _read_vector3(_B_UGV_DATA, ugv_at_index, 9)

    @staticmethod
    def velocity(ugv_at_index): return _read_vector3(_B_UGV_DATA, ugv_at_index, 21)

    @staticmethod
    def fuel_value(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 33)

    @staticmethod
    def water_value(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 37)

    @staticmethod
    def attached_fuel_station(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 41)

    @staticmethod
    def attached_water_station(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 45)

    @staticmethod
    def edge(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 49)

    @staticmethod
    def time(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 53)

    @staticmethod
    def turn_choice(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 57)

    @staticmethod
    def throttle(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 61)

    @staticmethod
    def brake(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 65)

    @staticmethod
    def forward_gear(ugv_at_index): return _read_bool(_B_UGV_DATA, ugv_at_index, 69)

    @staticmethod
    def victim_count(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 70)

    @staticmethod
    def fire_hose_pressure(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 74)

    @staticmethod
    def fire_hose_direction(ugv_at_index): return _read_vector3(_B_UGV_DATA, ugv_at_index, 78)

    @staticmethod
    def motor_start_time(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 90)

    @staticmethod
    def motor_stop_time(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 94)

    @staticmethod
    def respawn_time(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 98)

    @staticmethod
    def fuel_capacity(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 102)

    @staticmethod
    def fuel_solace(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 106)

    @staticmethod
    def fuel_consumption_add(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 110)

    @staticmethod
    def fuel_consumption_vel_multiply(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 114)

    @staticmethod
    def fuel_consumption_throttle_multiply(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 118)

    @staticmethod
    def water_capacity(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 122)

    @staticmethod
    def drag_add(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 126)

    @staticmethod
    def drag_factor(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 130)

    @staticmethod
    def forward_acceleration_factor(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 134)

    @staticmethod
    def backward_acceleration_factor(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 138)

    @staticmethod
    def brake_factor(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 142)

    @staticmethod
    def fire_hose_range_min(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 146)

    @staticmethod
    def fire_hose_range_max(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 150)

    @staticmethod
    def passenger_capacity(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 154)

    @staticmethod
    def last_motor_result(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 158)

    @staticmethod
    def last_gear_result(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 162)

    @staticmethod
    def last_fuel_attach_result(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 166)

    @staticmethod
    def last_water_attach_result(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 170)

    @staticmethod
    def last_crash_time(ugv_at_index): return _read_float32(_B_UGV_DATA, ugv_at_index, 174)

    @staticmethod
    def last_crash_cause(ugv_at_index): return _read_int32(_B_UGV_DATA, ugv_at_index, 178)


class UsvData:
    @staticmethod
    def item_count(): return _read_block_length(_B_USV_DATA)

    @staticmethod
    def id(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 0)

    @staticmethod
    def is_active(usv_at_index): return _read_bool(_B_USV_DATA, usv_at_index, 4)

    @staticmethod
    def motor_state(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 5)

    @staticmethod
    def position(usv_at_index): return _read_vector3(_B_USV_DATA, usv_at_index, 9)

    @staticmethod
    def velocity(usv_at_index): return _read_vector3(_B_USV_DATA, usv_at_index, 21)

    @staticmethod
    def target_x(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 33)

    @staticmethod
    def target_z(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 37)

    @staticmethod
    def fuel_value(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 41)

    @staticmethod
    def water_value(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 45)

    @staticmethod
    def attached_fuel_station(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 49)

    @staticmethod
    def attached_water_station(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 53)

    @staticmethod
    def victim_count(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 57)

    @staticmethod
    def fire_hose_pressure(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 61)

    @staticmethod
    def fire_hose_direction(usv_at_index): return _read_vector3(_B_USV_DATA, usv_at_index, 65)

    @staticmethod
    def motor_start_time(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 77)

    @staticmethod
    def motor_stop_time(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 81)

    @staticmethod
    def respawn_time(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 85)

    @staticmethod
    def fuel_capacity(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 89)

    @staticmethod
    def fuel_solace(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 93)

    @staticmethod
    def fuel_consumption_add(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 97)

    @staticmethod
    def fuel_consumption_vel_multiply(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 101)

    @staticmethod
    def water_capacity(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 105)

    @staticmethod
    def fire_hose_range_min(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 109)

    @staticmethod
    def fire_hose_range_max(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 113)

    @staticmethod
    def passenger_capacity(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 117)

    @staticmethod
    def max_speed(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 121)

    @staticmethod
    def max_target_distance(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 125)

    @staticmethod
    def last_motor_result(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 129)

    @staticmethod
    def last_fuel_attach_result(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 133)

    @staticmethod
    def last_water_attach_result(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 137)

    @staticmethod
    def last_target_result(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 141)

    @staticmethod
    def last_crash_time(usv_at_index): return _read_float32(_B_USV_DATA, usv_at_index, 145)

    @staticmethod
    def last_crash_cause(usv_at_index): return _read_int32(_B_USV_DATA, usv_at_index, 149)


class FireData:
    @staticmethod
    def item_count(): return _read_block_length(_B_FIRE_DATA)

    @staticmethod
    def id(fire_at_index): return _read_int32(_B_FIRE_DATA, fire_at_index, 0)

    @staticmethod
    def location(fire_at_index): return _read_vector3(_B_FIRE_DATA, fire_at_index, 4)

    @staticmethod
    def radius(fire_at_index): return _read_float32(_B_FIRE_DATA, fire_at_index, 16)

    @staticmethod
    def growing_speed(fire_at_index): return _read_float32(_B_FIRE_DATA, fire_at_index, 20)

    @staticmethod
    def strength_factor(fire_at_index): return _read_float32(_B_FIRE_DATA, fire_at_index, 24)

    @staticmethod
    def maximum_strength_factor(fire_at_index): return _read_float32(_B_FIRE_DATA, fire_at_index, 28)

    @staticmethod
    def strength_factor_speed(fire_at_index): return _read_float32(_B_FIRE_DATA, fire_at_index, 32)


class StationData:
    @staticmethod
    def item_count(): return _read_block_length(_B_STATION_DATA)

    @staticmethod
    def id(station_at_index): return _read_int32(_B_STATION_DATA, station_at_index, 0)

    @staticmethod
    def location(station_at_index): return _read_vector3(_B_STATION_DATA, station_at_index, 4)

    @staticmethod
    def is_surface(station_at_index): return _read_bool(_B_STATION_DATA, station_at_index, 16)

    @staticmethod
    def range(station_at_index): return _read_float32(_B_STATION_DATA, station_at_index, 17)

    @staticmethod
    def vehicle_capacity(station_at_index): return _read_int32(_B_STATION_DATA, station_at_index, 21)

    @staticmethod
    def attached_vehicle_count(station_at_index): return _read_int32(_B_STATION_DATA, station_at_index, 25)

    @staticmethod
    def fuel_value(station_at_index): return _read_float32(_B_STATION_DATA, station_at_index, 29)

    @staticmethod
    def fuel_capacity(station_at_index): return _read_float32(_B_STATION_DATA, station_at_index, 33)

    @staticmethod
    def fuel_refill_speed(station_at_index): return _read_float32(_B_STATION_DATA, station_at_index, 37)

    @staticmethod
    def fuel_transfer_speed(station_at_index): return _read_float32(_B_STATION_DATA, station_at_index, 41)

    @staticmethod
    def water_value(station_at_index): return _read_float32(_B_STATION_DATA, station_at_index, 45)

    @staticmethod
    def water_capacity(station_at_index): return _read_float32(_B_STATION_DATA, station_at_index, 49)

    @staticmethod
    def water_refill_speed(station_at_index): return _read_float32(_B_STATION_DATA, station_at_index, 53)

    @staticmethod
    def water_transfer_speed(station_at_index): return _read_float32(_B_STATION_DATA, station_at_index, 57)


class DestinationData:
    @staticmethod
    def item_count(): return _read_block_length(_B_DESTINATION_DATA)

    @staticmethod
    def id(destination_at_index): return _read_int32(_B_DESTINATION_DATA, destination_at_index, 0)

    @staticmethod
    def location(destination_at_index): return _read_vector3(_B_DESTINATION_DATA, destination_at_index, 4)

    @staticmethod
    def current_capacity(destination_at_index): return _read_int32(_B_DESTINATION_DATA, destination_at_index, 16)

    @staticmethod
    def max_capacity(destination_at_index): return _read_int32(_B_DESTINATION_DATA, destination_at_index, 20)

    @staticmethod
    def refresh_time(destination_at_index): return _read_int32(_B_DESTINATION_DATA, destination_at_index, 24)


class VictimData:
    @staticmethod
    def item_count(): return _read_block_length(_B_VICTIM_DATA)

    @staticmethod
    def id(victim_at_index): return _read_int32(_B_VICTIM_DATA, victim_at_index, 0)

    @staticmethod
    def location(victim_at_index): return _read_vector3(_B_VICTIM_DATA, victim_at_index, 4)

    @staticmethod
    def victim_state(victim_at_index): return _read_int32(_B_VICTIM_DATA, victim_at_index, 16)

    @staticmethod
    def target_destination(victim_at_index): return _read_int32(_B_VICTIM_DATA, victim_at_index, 20)

    @staticmethod
    def remaining_life(victim_at_index): return _read_float32(_B_VICTIM_DATA, victim_at_index, 24)

    @staticmethod
    def target_pos(victim_at_index): return _read_vector2(_B_VICTIM_DATA, victim_at_index, 28)

    @staticmethod
    def current_vehicle_type(victim_at_index): return _read_int32(_B_VICTIM_DATA, victim_at_index, 36)

    @staticmethod
    def current_vehicle_id(victim_at_index): return _read_int32(_B_VICTIM_DATA, victim_at_index, 40)

    @staticmethod
    def max_speed(victim_at_index): return _read_float32(_B_VICTIM_DATA, victim_at_index, 44)

    @staticmethod
    def calling_range(victim_at_index): return _read_float32(_B_VICTIM_DATA, victim_at_index, 48)

    @staticmethod
    def loading_range(victim_at_index): return _read_float32(_B_VICTIM_DATA, victim_at_index, 52)

    @staticmethod
    def arriving_range(victim_at_index): return _read_float32(_B_VICTIM_DATA, victim_at_index, 56)

    @staticmethod
    def last_result(victim_at_index): return _read_int32(_B_VICTIM_DATA, victim_at_index, 60)


class SurfaceEdgeData:
    @staticmethod
    def item_count(): return _read_block_length(_B_SURFACE_EDGE_DATA)

    @staticmethod
    def cell_x(item_at_index): return _read_int32(_B_SURFACE_EDGE_DATA, item_at_index, 0)

    @staticmethod
    def cell_z(item_at_index): return _read_int32(_B_SURFACE_EDGE_DATA, item_at_index, 4)


class GameData:

    @staticmethod
    def time(): return _read_float32(_B_GAME_DATA, 0, 0)

    @staticmethod
    def game_duration(): return _read_float32(_B_GAME_DATA, 0, 4)

    @staticmethod
    def ignore_crashes(): return _read_bool(_B_GAME_DATA, 0, 8)

    @staticmethod
    def ignore_speed_limit(): return _read_bool(_B_GAME_DATA, 0, 9)

    @staticmethod
    def infinite_supply(): return _read_bool(_B_GAME_DATA, 0, 10)

    @staticmethod
    def ignore_lifetime(): return _read_bool(_B_GAME_DATA, 0, 11)


class Scoring:

    @staticmethod
    def vehicle_crash(): return _read_int32(_B_SCORING, 0, 0)

    @staticmethod
    def victim_lose(): return _read_int32(_B_SCORING, 0, 4)

    @staticmethod
    def victim_delivery(): return _read_int32(_B_SCORING, 0, 8)

    @staticmethod
    def fire_extinguish(): return _read_int32(_B_SCORING, 0, 12)

    @staticmethod
    def fire_penalty_period(): return _read_int32(_B_SCORING, 0, 16)

    @staticmethod
    def fire_penalty_factor(): return _read_float32(_B_SCORING, 0, 20)

    @staticmethod
    def max_fire_penalty(): return _read_int32(_B_SCORING, 0, 24)


class SurfaceInfo:

    @staticmethod
    def terrain_position(): return _read_vector3(_B_SURFACE_INFO, 0, 0)

    @staticmethod
    def terrain_size(): return _read_vector3(_B_SURFACE_INFO, 0, 12)

    @staticmethod
    def heightmap_resolution(): return _read_int32(_B_SURFACE_INFO, 0, 24)

    @staticmethod
    def heightmap_scale(): return _read_float32(_B_SURFACE_INFO, 0, 28)

    @staticmethod
    def heightmap_range(): return _read_float32(_B_SURFACE_INFO, 0, 32)

    @staticmethod
    def surface_level(): return _read_float32(_B_SURFACE_INFO, 0, 36)

    @staticmethod
    def target_bounds_margin(): return _read_float32(_B_SURFACE_INFO, 0, 40)


class CmdDebugStates:

    @staticmethod
    def set_ignore_crashes(value): _write_bool(_B_CMD_DEBUG_STATES, 0, 0, value)

    @staticmethod
    def set_ignore_speed_limit(value): _write_bool(_B_CMD_DEBUG_STATES, 0, 1, value)

    @staticmethod
    def set_infinite_supply(value): _write_bool(_B_CMD_DEBUG_STATES, 0, 2, value)

    @staticmethod
    def set_ignore_lifetime(value): _write_bool(_B_CMD_DEBUG_STATES, 0, 3, value)

    @staticmethod
    def set_show_road_nodes(value): _write_bool(_B_CMD_DEBUG_STATES, 0, 4, value)

    @staticmethod
    def set_show_road_limits(value): _write_bool(_B_CMD_DEBUG_STATES, 0, 5, value)


_write_block_length(_B_CMD_DEBUG_STATES, 1)


class ClientControl:

    @staticmethod
    def set_last_time(value): _write_int64(_B_CLIENT_CONTROL, 0, 0, value)


_write_block_length(_B_CLIENT_CONTROL, 1)


class CmdUgv:
    @staticmethod
    def _bounds_check(value):
        if value >= 16384 or value < 0:
            raise "argument out of range"

    @staticmethod
    def set_throttle(ugv_id, value):
        CmdUgv._bounds_check(ugv_id)
        _write_float32(_B_CMD_UGV, ugv_id, 0, value)

    @staticmethod
    def set_brake(ugv_id, value):
        CmdUgv._bounds_check(ugv_id)
        _write_float32(_B_CMD_UGV, ugv_id, 4, value)

    @staticmethod
    def set_backward_gear(ugv_id, value):
        CmdUgv._bounds_check(ugv_id)
        _write_bool(_B_CMD_UGV, ugv_id, 8, value)

    @staticmethod
    def set_turn_choice(ugv_id, value):
        CmdUgv._bounds_check(ugv_id)
        _write_uint8(_B_CMD_UGV, ugv_id, 9, value)

    @staticmethod
    def set_motor_running(ugv_id, value):
        CmdUgv._bounds_check(ugv_id)
        _write_bool(_B_CMD_UGV, ugv_id, 10, value)

    @staticmethod
    def set_fire_hose_direction(ugv_id, value):
        CmdUgv._bounds_check(ugv_id)
        _write_vector3(_B_CMD_UGV, ugv_id, 11, value)

    @staticmethod
    def set_fire_hose_pressure(ugv_id, value):
        CmdUgv._bounds_check(ugv_id)
        _write_float32(_B_CMD_UGV, ugv_id, 23, value)

    @staticmethod
    def set_attach_to_station_for_fuel(ugv_id, value):
        CmdUgv._bounds_check(ugv_id)
        _write_int32(_B_CMD_UGV, ugv_id, 27, value)

    @staticmethod
    def set_attach_to_station_for_water(ugv_id, value):
        CmdUgv._bounds_check(ugv_id)
        _write_int32(_B_CMD_UGV, ugv_id, 31, value)


_write_block_length(_B_CMD_UGV, 16384)


class CmdUsv:
    @staticmethod
    def _bounds_check(value):
        if value >= 16384 or value < 0:
            raise "argument out of range"

    @staticmethod
    def set_motor_running(usv_id, value):
        CmdUsv._bounds_check(usv_id)
        _write_bool(_B_CMD_USV, usv_id, 0, value)

    @staticmethod
    def set_attach_to_station_for_fuel(usv_id, value):
        CmdUsv._bounds_check(usv_id)
        _write_int32(_B_CMD_USV, usv_id, 1, value)

    @staticmethod
    def set_attach_to_station_for_water(usv_id, value):
        CmdUsv._bounds_check(usv_id)
        _write_int32(_B_CMD_USV, usv_id, 5, value)

    @staticmethod
    def set_fire_hose_direction(usv_id, value):
        CmdUsv._bounds_check(usv_id)
        _write_vector3(_B_CMD_USV, usv_id, 9, value)

    @staticmethod
    def set_fire_hose_pressure(usv_id, value):
        CmdUsv._bounds_check(usv_id)
        _write_float32(_B_CMD_USV, usv_id, 21, value)

    @staticmethod
    def set_target_x(usv_id, value):
        CmdUsv._bounds_check(usv_id)
        _write_float32(_B_CMD_USV, usv_id, 25, value)

    @staticmethod
    def set_target_z(usv_id, value):
        CmdUsv._bounds_check(usv_id)
        _write_float32(_B_CMD_USV, usv_id, 29, value)


_write_block_length(_B_CMD_USV, 16384)


class CmdVictim:
    @staticmethod
    def _bounds_check(value):
        if value >= 16384 or value < 0:
            raise "argument out of range"

    @staticmethod
    def set_command(victim_id, value):
        CmdVictim._bounds_check(victim_id)
        _write_int32(_B_CMD_VICTIM, victim_id, 0, value)

    @staticmethod
    def set_arg0(victim_id, value):
        CmdVictim._bounds_check(victim_id)
        _write_float32(_B_CMD_VICTIM, victim_id, 4, value)

    @staticmethod
    def set_arg1(victim_id, value):
        CmdVictim._bounds_check(victim_id)
        _write_float32(_B_CMD_VICTIM, victim_id, 8, value)


_write_block_length(_B_CMD_VICTIM, 16384)


class CmdDebugSphere:
    @staticmethod
    def _bounds_check(value):
        if value >= 4096 or value < 0:
            raise "argument out of range"

    @staticmethod
    def set_enabled(sphere_id, value):
        CmdDebugSphere._bounds_check(sphere_id)
        _write_bool(_B_CMD_DEBUG_SPHERE, sphere_id, 0, value)

    @staticmethod
    def set_position(sphere_id, value):
        CmdDebugSphere._bounds_check(sphere_id)
        _write_vector3(_B_CMD_DEBUG_SPHERE, sphere_id, 1, value)

    @staticmethod
    def set_color(sphere_id, value):
        CmdDebugSphere._bounds_check(sphere_id)
        _write_vector3(_B_CMD_DEBUG_SPHERE, sphere_id, 13, value)

    @staticmethod
    def set_radius(sphere_id, value):
        CmdDebugSphere._bounds_check(sphere_id)
        _write_float32(_B_CMD_DEBUG_SPHERE, sphere_id, 25, value)


_write_block_length(_B_CMD_DEBUG_SPHERE, 4096)


class CmdDebugText:
    @staticmethod
    def _bounds_check(value):
        if value >= 4096 or value < 0:
            raise "argument out of range"

    @staticmethod
    def set_enabled(text_id, value):
        CmdDebugText._bounds_check(text_id)
        _write_bool(_B_CMD_DEBUG_TEXT, text_id, 0, value)

    @staticmethod
    def set_position(text_id, value):
        CmdDebugText._bounds_check(text_id)
        _write_vector3(_B_CMD_DEBUG_TEXT, text_id, 1, value)

    @staticmethod
    def set_color(text_id, value):
        CmdDebugText._bounds_check(text_id)
        _write_vector3(_B_CMD_DEBUG_TEXT, text_id, 13, value)

    @staticmethod
    def set_text(text_id, value):
        CmdDebugText._bounds_check(text_id)
        _write_fstring(_B_CMD_DEBUG_TEXT, text_id, 25, value)


_write_block_length(_B_CMD_DEBUG_TEXT, 4096)

# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: static_obstacle.proto

import sys

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database

import commonroad.scenario_definition.protobuf_format.generated_scripts.obstacle_pb2 as obstacle__pb2
import commonroad.scenario_definition.protobuf_format.generated_scripts.util_pb2 as util__pb2

_b = sys.version_info[0] < 3 and (lambda x: x) or (lambda x: x.encode("latin1"))

# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
    name="static_obstacle.proto",
    package="commonroad",
    syntax="proto2",
    serialized_options=None,
    serialized_pb=_b(
        '\n\x15static_obstacle.proto\x12\ncommonroad\x1a\x0eobstacle.proto\x1a\nutil.proto"\xa1\x02\n\x0eStaticObstacle\x12\x1a\n\x12static_obstacle_id\x18\x01 \x02(\r\x12@\n\robstacle_type\x18\x02 \x02(\x0e\x32).commonroad.ObstacleTypeEnum.ObstacleType\x12 \n\x05shape\x18\x03 \x02(\x0b\x32\x11.commonroad.Shape\x12(\n\rinitial_state\x18\x04 \x02(\x0b\x32\x11.commonroad.State\x12\x35\n\x14initial_signal_state\x18\x05 \x01(\x0b\x32\x17.commonroad.SignalState\x12.\n\rsignal_series\x18\x06 \x03(\x0b\x32\x17.commonroad.SignalState'
    ),
    dependencies=[
        obstacle__pb2.DESCRIPTOR,
        util__pb2.DESCRIPTOR,
    ],
)


_STATICOBSTACLE = _descriptor.Descriptor(
    name="StaticObstacle",
    full_name="commonroad.StaticObstacle",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="static_obstacle_id",
            full_name="commonroad.StaticObstacle.static_obstacle_id",
            index=0,
            number=1,
            type=13,
            cpp_type=3,
            label=2,
            has_default_value=False,
            default_value=0,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="obstacle_type",
            full_name="commonroad.StaticObstacle.obstacle_type",
            index=1,
            number=2,
            type=14,
            cpp_type=8,
            label=2,
            has_default_value=False,
            default_value=0,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="shape",
            full_name="commonroad.StaticObstacle.shape",
            index=2,
            number=3,
            type=11,
            cpp_type=10,
            label=2,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="initial_state",
            full_name="commonroad.StaticObstacle.initial_state",
            index=3,
            number=4,
            type=11,
            cpp_type=10,
            label=2,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="initial_signal_state",
            full_name="commonroad.StaticObstacle.initial_signal_state",
            index=4,
            number=5,
            type=11,
            cpp_type=10,
            label=1,
            has_default_value=False,
            default_value=None,
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
        _descriptor.FieldDescriptor(
            name="signal_series",
            full_name="commonroad.StaticObstacle.signal_series",
            index=5,
            number=6,
            type=11,
            cpp_type=10,
            label=3,
            has_default_value=False,
            default_value=[],
            message_type=None,
            enum_type=None,
            containing_type=None,
            is_extension=False,
            extension_scope=None,
            serialized_options=None,
            file=DESCRIPTOR,
        ),
    ],
    extensions=[],
    nested_types=[],
    enum_types=[],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=66,
    serialized_end=355,
)

_STATICOBSTACLE.fields_by_name[
    "obstacle_type"
].enum_type = obstacle__pb2._OBSTACLETYPEENUM_OBSTACLETYPE
_STATICOBSTACLE.fields_by_name["shape"].message_type = util__pb2._SHAPE
_STATICOBSTACLE.fields_by_name["initial_state"].message_type = obstacle__pb2._STATE
_STATICOBSTACLE.fields_by_name["initial_signal_state"].message_type = obstacle__pb2._SIGNALSTATE
_STATICOBSTACLE.fields_by_name["signal_series"].message_type = obstacle__pb2._SIGNALSTATE
DESCRIPTOR.message_types_by_name["StaticObstacle"] = _STATICOBSTACLE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

StaticObstacle = _reflection.GeneratedProtocolMessageType(
    "StaticObstacle",
    (_message.Message,),
    dict(
        DESCRIPTOR=_STATICOBSTACLE,
        __module__="static_obstacle_pb2",
        # @@protoc_insertion_point(class_scope:commonroad.StaticObstacle)
    ),
)
_sym_db.RegisterMessage(StaticObstacle)


# @@protoc_insertion_point(module_scope)

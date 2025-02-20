# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: traffic_light.proto

import sys

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database

import commonroad.scenario_definition.protobuf_format.generated_scripts.util_pb2 as util__pb2

_b = sys.version_info[0] < 3 and (lambda x: x) or (lambda x: x.encode("latin1"))

# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()

DESCRIPTOR = _descriptor.FileDescriptor(
    name="traffic_light.proto",
    package="commonroad",
    syntax="proto2",
    serialized_options=None,
    serialized_pb=_b(
        '\n\x13traffic_light.proto\x12\ncommonroad\x1a\nutil.proto"j\n\x15TrafficLightStateEnum"Q\n\x11TrafficLightState\x12\x07\n\x03RED\x10\x00\x12\x0e\n\nRED_YELLOW\x10\x01\x12\t\n\x05GREEN\x10\x02\x12\n\n\x06YELLOW\x10\x03\x12\x0c\n\x08INACTIVE\x10\x04"\x97\x01\n\x19TrafficLightDirectionEnum"z\n\x15TrafficLightDirection\x12\t\n\x05RIGHT\x10\x00\x12\x0c\n\x08STRAIGHT\x10\x01\x12\x08\n\x04LEFT\x10\x02\x12\x11\n\rLEFT_STRAIGHT\x10\x03\x12\x12\n\x0eSTRAIGHT_RIGHT\x10\x04\x12\x0e\n\nLEFT_RIGHT\x10\x05\x12\x07\n\x03\x41LL\x10\x06"d\n\x0c\x43ycleElement\x12\x10\n\x08\x64uration\x18\x01 \x02(\r\x12\x42\n\x05\x63olor\x18\x02 \x02(\x0e\x32\x33.commonroad.TrafficLightStateEnum.TrafficLightState"\xf4\x01\n\x0cTrafficLight\x12\x18\n\x10traffic_light_id\x18\x01 \x02(\r\x12\x30\n\x0e\x63ycle_elements\x18\x02 \x03(\x0b\x32\x18.commonroad.CycleElement\x12#\n\x08position\x18\x03 \x01(\x0b\x32\x11.commonroad.Point\x12\x13\n\x0btime_offset\x18\x04 \x01(\r\x12N\n\tdirection\x18\x05 \x01(\x0e\x32;.commonroad.TrafficLightDirectionEnum.TrafficLightDirection\x12\x0e\n\x06\x61\x63tive\x18\x06 \x01(\x08'
    ),
    dependencies=[
        util__pb2.DESCRIPTOR,
    ],
)


_TRAFFICLIGHTSTATEENUM_TRAFFICLIGHTSTATE = _descriptor.EnumDescriptor(
    name="TrafficLightState",
    full_name="commonroad.TrafficLightStateEnum.TrafficLightState",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="RED", index=0, number=0, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="RED_YELLOW", index=1, number=1, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="GREEN", index=2, number=2, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="YELLOW", index=3, number=3, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="INACTIVE", index=4, number=4, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=72,
    serialized_end=153,
)
_sym_db.RegisterEnumDescriptor(_TRAFFICLIGHTSTATEENUM_TRAFFICLIGHTSTATE)

_TRAFFICLIGHTDIRECTIONENUM_TRAFFICLIGHTDIRECTION = _descriptor.EnumDescriptor(
    name="TrafficLightDirection",
    full_name="commonroad.TrafficLightDirectionEnum.TrafficLightDirection",
    filename=None,
    file=DESCRIPTOR,
    values=[
        _descriptor.EnumValueDescriptor(
            name="RIGHT", index=0, number=0, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="STRAIGHT", index=1, number=1, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="LEFT", index=2, number=2, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="LEFT_STRAIGHT", index=3, number=3, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="STRAIGHT_RIGHT", index=4, number=4, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="LEFT_RIGHT", index=5, number=5, serialized_options=None, type=None
        ),
        _descriptor.EnumValueDescriptor(
            name="ALL", index=6, number=6, serialized_options=None, type=None
        ),
    ],
    containing_type=None,
    serialized_options=None,
    serialized_start=185,
    serialized_end=307,
)
_sym_db.RegisterEnumDescriptor(_TRAFFICLIGHTDIRECTIONENUM_TRAFFICLIGHTDIRECTION)


_TRAFFICLIGHTSTATEENUM = _descriptor.Descriptor(
    name="TrafficLightStateEnum",
    full_name="commonroad.TrafficLightStateEnum",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[],
    extensions=[],
    nested_types=[],
    enum_types=[
        _TRAFFICLIGHTSTATEENUM_TRAFFICLIGHTSTATE,
    ],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=47,
    serialized_end=153,
)


_TRAFFICLIGHTDIRECTIONENUM = _descriptor.Descriptor(
    name="TrafficLightDirectionEnum",
    full_name="commonroad.TrafficLightDirectionEnum",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[],
    extensions=[],
    nested_types=[],
    enum_types=[
        _TRAFFICLIGHTDIRECTIONENUM_TRAFFICLIGHTDIRECTION,
    ],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=156,
    serialized_end=307,
)


_CYCLEELEMENT = _descriptor.Descriptor(
    name="CycleElement",
    full_name="commonroad.CycleElement",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="duration",
            full_name="commonroad.CycleElement.duration",
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
            name="color",
            full_name="commonroad.CycleElement.color",
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
    ],
    extensions=[],
    nested_types=[],
    enum_types=[],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=309,
    serialized_end=409,
)


_TRAFFICLIGHT = _descriptor.Descriptor(
    name="TrafficLight",
    full_name="commonroad.TrafficLight",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="traffic_light_id",
            full_name="commonroad.TrafficLight.traffic_light_id",
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
            name="cycle_elements",
            full_name="commonroad.TrafficLight.cycle_elements",
            index=1,
            number=2,
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
        _descriptor.FieldDescriptor(
            name="position",
            full_name="commonroad.TrafficLight.position",
            index=2,
            number=3,
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
            name="time_offset",
            full_name="commonroad.TrafficLight.time_offset",
            index=3,
            number=4,
            type=13,
            cpp_type=3,
            label=1,
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
            name="direction",
            full_name="commonroad.TrafficLight.direction",
            index=4,
            number=5,
            type=14,
            cpp_type=8,
            label=1,
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
            name="active",
            full_name="commonroad.TrafficLight.active",
            index=5,
            number=6,
            type=8,
            cpp_type=7,
            label=1,
            has_default_value=False,
            default_value=False,
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
    serialized_start=412,
    serialized_end=656,
)

_TRAFFICLIGHTSTATEENUM_TRAFFICLIGHTSTATE.containing_type = _TRAFFICLIGHTSTATEENUM
_TRAFFICLIGHTDIRECTIONENUM_TRAFFICLIGHTDIRECTION.containing_type = _TRAFFICLIGHTDIRECTIONENUM
_CYCLEELEMENT.fields_by_name["color"].enum_type = _TRAFFICLIGHTSTATEENUM_TRAFFICLIGHTSTATE
_TRAFFICLIGHT.fields_by_name["cycle_elements"].message_type = _CYCLEELEMENT
_TRAFFICLIGHT.fields_by_name["position"].message_type = util__pb2._POINT
_TRAFFICLIGHT.fields_by_name[
    "direction"
].enum_type = _TRAFFICLIGHTDIRECTIONENUM_TRAFFICLIGHTDIRECTION
DESCRIPTOR.message_types_by_name["TrafficLightStateEnum"] = _TRAFFICLIGHTSTATEENUM
DESCRIPTOR.message_types_by_name["TrafficLightDirectionEnum"] = _TRAFFICLIGHTDIRECTIONENUM
DESCRIPTOR.message_types_by_name["CycleElement"] = _CYCLEELEMENT
DESCRIPTOR.message_types_by_name["TrafficLight"] = _TRAFFICLIGHT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TrafficLightStateEnum = _reflection.GeneratedProtocolMessageType(
    "TrafficLightStateEnum",
    (_message.Message,),
    dict(
        DESCRIPTOR=_TRAFFICLIGHTSTATEENUM,
        __module__="traffic_light_pb2",
        # @@protoc_insertion_point(class_scope:commonroad.TrafficLightStateEnum)
    ),
)
_sym_db.RegisterMessage(TrafficLightStateEnum)

TrafficLightDirectionEnum = _reflection.GeneratedProtocolMessageType(
    "TrafficLightDirectionEnum",
    (_message.Message,),
    dict(
        DESCRIPTOR=_TRAFFICLIGHTDIRECTIONENUM,
        __module__="traffic_light_pb2",
        # @@protoc_insertion_point(class_scope:commonroad.TrafficLightDirectionEnum)
    ),
)
_sym_db.RegisterMessage(TrafficLightDirectionEnum)

CycleElement = _reflection.GeneratedProtocolMessageType(
    "CycleElement",
    (_message.Message,),
    dict(
        DESCRIPTOR=_CYCLEELEMENT,
        __module__="traffic_light_pb2",
        # @@protoc_insertion_point(class_scope:commonroad.CycleElement)
    ),
)
_sym_db.RegisterMessage(CycleElement)

TrafficLight = _reflection.GeneratedProtocolMessageType(
    "TrafficLight",
    (_message.Message,),
    dict(
        DESCRIPTOR=_TRAFFICLIGHT,
        __module__="traffic_light_pb2",
        # @@protoc_insertion_point(class_scope:commonroad.TrafficLight)
    ),
)
_sym_db.RegisterMessage(TrafficLight)


# @@protoc_insertion_point(module_scope)

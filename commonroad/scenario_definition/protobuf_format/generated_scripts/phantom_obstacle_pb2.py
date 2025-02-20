# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: phantom_obstacle.proto

import sys

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database

import commonroad.scenario_definition.protobuf_format.generated_scripts.obstacle_pb2 as obstacle__pb2

_b = sys.version_info[0] < 3 and (lambda x: x) or (lambda x: x.encode("latin1"))

# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
    name="phantom_obstacle.proto",
    package="commonroad",
    syntax="proto2",
    serialized_options=None,
    serialized_pb=_b(
        '\n\x16phantom_obstacle.proto\x12\ncommonroad\x1a\x0eobstacle.proto"Z\n\x0fPhantomObstacle\x12\x13\n\x0bobstacle_id\x18\x01 \x02(\r\x12\x32\n\nprediction\x18\x02 \x01(\x0b\x32\x1e.commonroad.SetBasedPrediction'
    ),
    dependencies=[
        obstacle__pb2.DESCRIPTOR,
    ],
)


_PHANTOMOBSTACLE = _descriptor.Descriptor(
    name="PhantomObstacle",
    full_name="commonroad.PhantomObstacle",
    filename=None,
    file=DESCRIPTOR,
    containing_type=None,
    fields=[
        _descriptor.FieldDescriptor(
            name="obstacle_id",
            full_name="commonroad.PhantomObstacle.obstacle_id",
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
            name="prediction",
            full_name="commonroad.PhantomObstacle.prediction",
            index=1,
            number=2,
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
    ],
    extensions=[],
    nested_types=[],
    enum_types=[],
    serialized_options=None,
    is_extendable=False,
    syntax="proto2",
    extension_ranges=[],
    oneofs=[],
    serialized_start=54,
    serialized_end=144,
)

_PHANTOMOBSTACLE.fields_by_name["prediction"].message_type = obstacle__pb2._SETBASEDPREDICTION
DESCRIPTOR.message_types_by_name["PhantomObstacle"] = _PHANTOMOBSTACLE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PhantomObstacle = _reflection.GeneratedProtocolMessageType(
    "PhantomObstacle",
    (_message.Message,),
    dict(
        DESCRIPTOR=_PHANTOMOBSTACLE,
        __module__="phantom_obstacle_pb2",
        # @@protoc_insertion_point(class_scope:commonroad.PhantomObstacle)
    ),
)
_sym_db.RegisterMessage(PhantomObstacle)


# @@protoc_insertion_point(module_scope)

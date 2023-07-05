#! /bin/bash

# generate map
protoc \
--proto_path=./common/ \
--proto_path=./map/ \
--python_out=../../commonroad/common/protobuf/map/ \
./map/*.proto

# generate dynamic
protoc \
--proto_path=./common/ \
--proto_path=./dynamic/ \
--python_out=../../commonroad/common/protobuf/dynamic/ \
./dynamic/*.proto

## generate scenario
protoc \
--proto_path=./common/ \
--proto_path=./scenario/ \
--python_out=../../commonroad/common/protobuf/scenario/ \
./scenario/*.proto

## generate rest
protoc \
--proto_path=./common/ \
--python_out=../../commonroad/common/protobuf/common/ \
./common/*.proto

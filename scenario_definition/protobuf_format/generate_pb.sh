#! /bin/bash

# generate map
protoc \
--proto_path=./definition_files/common/ \
--proto_path=./definition_files/map/ \
--python_out=../../commonroad/common/pb_scripts/map/ \
./definition_files/map/*.proto

# generate dynamic
protoc \
--proto_path=./definition_files/common/ \
--proto_path=./definition_files/dynamic/ \
--python_out=../../commonroad/common/pb_scripts/dynamic/ \
./definition_files/dynamic/*.proto

## generate scenario
protoc \
--proto_path=./definition_files/common/ \
--proto_path=./definition_files/scenario/ \
--python_out=../../commonroad/common/pb_scripts/scenario/ \
./definition_files/scenario/*.proto

## generate rest
protoc \
--proto_path=./definition_files/common/ \
--python_out=../../commonroad/common/pb_scripts/common/ \
./definition_files/common/*.proto

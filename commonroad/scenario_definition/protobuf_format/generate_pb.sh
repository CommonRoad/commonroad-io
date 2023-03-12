#! /bin/bash

# generate map
protoc \
--proto_path=./definition_files/ \
--proto_path=./definition_files/map/ \
--python_out=./generated_scripts/ \
./definition_files/map/commonroad_map.proto

# generate dynamic
protoc \
--proto_path=./definition_files/ \
--proto_path=./definition_files/map/ \
--proto_path=./definition_files/dynamic/ \
--python_out=./generated_scripts/ \
./definition_files/dynamic/commonroad_dynamic.proto

# generate scenario
protoc \
--proto_path=./definition_files/ \
--proto_path=./definition_files/scenario/ \
--proto_path=./definition_files/dynamic/ \
--python_out=./generated_scripts/ \
./definition_files/scenario/commonroad_scenario.proto
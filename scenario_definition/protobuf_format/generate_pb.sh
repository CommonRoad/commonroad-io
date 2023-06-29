#! /bin/bash

# generate map
protoc \
--proto_path=./ \
--python_out=../../ \
./commonroad/common/pb_scripts/map/*.proto

# generate dynamic
protoc \
--proto_path=./ \
--python_out=../../ \
./commonroad/common/pb_scripts/dynamic/*.proto

### generate scenario
protoc \
--proto_path=./ \
--python_out=../../ \
./commonroad/common/pb_scripts/scenario/*.proto

## generate rest
protoc \
--proto_path=./ \
--python_out=../../ \
./commonroad/common/pb_scripts/common/*.proto

#!/usr/bin/env bash
set -euo pipefail

PROTO_ROOT=.
OUT=../../commonroad/common/protobuf

rm -rf "$OUT"/common "$OUT"/map "$OUT"/dynamic "$OUT"/scenario
mkdir -p "$OUT"/{common,map,dynamic,scenario}

protoc -I "$PROTO_ROOT" --python_out="$OUT" \
  common/*.proto map/*.proto dynamic/*.proto scenario/*.proto

touch "$OUT/__init__.py"
touch "$OUT/common/__init__.py"
touch "$OUT/map/__init__.py"
touch "$OUT/dynamic/__init__.py"
touch "$OUT/scenario/__init__.py"

PY_PKG=commonroad.common.protobuf

find "$OUT" -name '*_pb2.py' -type f | while read -r f; do
  sed -i \
    -e "s/^from common import /from ${PY_PKG}.common import /" \
    -e "s/^from map import /from ${PY_PKG}.map import /" \
    -e "s/^from dynamic import /from ${PY_PKG}.dynamic import /" \
    -e "s/^from scenario import /from ${PY_PKG}.scenario import /" \
    "$f"
done

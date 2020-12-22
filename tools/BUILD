load("@rules_python//python:defs.bzl", "py_runtime_pair", "py_runtime")
py_runtime(
    name = "commonroad-py37",
    interpreter_path = "/home/luis/anaconda3/envs/commonroad-py37/bin/python",
    python_version = "PY3",
)

py_runtime_pair(
    name = "commonroad_py_pair",
    py3_runtime = ":commonroad-py37",
)

toolchain(
    name = "commonroad_py_toolchain",
    toolchain = ":commonroad_py_pair",
    toolchain_type = "@rules_python//python:toolchain_type",
)
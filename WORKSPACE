workspace(name = "commonroad_io")

all_content = """filegroup(name = "all", srcs = glob(["**"]), visibility = ["//visibility:public"])"""

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_python",
    sha256 = "b6d46438523a3ec0f3cead544190ee13223a52f6a6765a29eae7b7cc24cc83a0",
    url = "https://github.com/bazelbuild/rules_python/releases/download/0.1.0/rules_python-0.1.0.tar.gz",
)

# Register python interpreter
#register_toolchains("//:commonroad_py_toolchain")

#-------- External dependencies --------
# Eigen
http_archive(
    name = "eigen",
    build_file_content = """cc_library(
                                  name = 'eigen',
                                  srcs = [],
                                  includes = ['.'],
                                  hdrs = glob(['Eigen/**', 'unsupported/**']),
                                  visibility = ['//visibility:public'],
                              )""",
    sha256 = "4b1120abc5d4a63620a886dcc5d7a7a27bf5b048c8c74ac57521dd27845b1d9f",
    strip_prefix = "eigen-git-mirror-98e54de5e25aefc6b984c168fb3009868a93e217",
    urls = ["https://github.com/eigenteam/eigen-git-mirror/archive/98e54de5e25aefc6b984c168fb3009868a93e217.zip"],
)

# pybind
http_archive(
    name = "pybind11_bazel",
    strip_prefix = "pybind11_bazel-26973c0ff320cb4b39e45bc3e4297b82bc3a6c09",
    urls = ["https://github.com/pybind/pybind11_bazel/archive/26973c0ff320cb4b39e45bc3e4297b82bc3a6c09.zip"],
)

http_archive(
    name = "pybind11",
    build_file = "@pybind11_bazel//:pybind11.BUILD",
    strip_prefix = "pybind11-2.6.1",
    urls = ["https://github.com/pybind/pybind11/archive/v2.6.1.tar.gz"],
)

load("@pybind11_bazel//:python_configure.bzl", "python_configure")

python_configure(name = "local_config_python")

#-------- Commonroad modules --------
git_repository(
    name = "curvilinear",
    branch = "bazel",
    remote = "git@gitlab.lrz.de:cps/commonroad-curvilinear-coordinate-system.git",
)

git_repository(
    name = "stl_crmonitor",
    #path = "/home/luis/Documents/Promotion/Code/stl_crmonitor",
    branch = "bazel",
    remote = "git@gitlab.lrz.de:ge69xek/stl_crmonitor.git",
)

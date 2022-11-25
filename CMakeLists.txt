cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# set(ENV{IDF_MAINTAINER} 1)
# set(ENV{IDF_TOOLCHAIN} clang) # needs to install xtensa-clang first: idf_tools.py install xtensa-clang
# list(APPEND compile_options "-Wno-parentheses-equality" "-Wno-unused-function")

project(falsealarm)

# littlefs_create_partition_image(littlefs data)

cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# list(APPEND compile_options "--coverage")
# set(ENV{IDF_MAINTAINER} 1)
# set(ENV{IDF_TOOLCHAIN} clang) # needs to install xtensa-clang first: idf_tools.py install xtensa-clang
# list(APPEND compile_options "-Wno-dev")

# idf_build_set_property(COMPILE_OPTIONS "-fsanitize=undefined" "-fno-sanitize=shift-base" APPEND)
project(falsealarm)

# littlefs_create_partition_image(littlefs flash_data)

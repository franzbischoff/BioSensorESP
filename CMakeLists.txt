cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(ENV{IDF_MAINTAINER} 1)

# set(ENV{IDF_TOOLCHAIN} clang) # requires installing xtensa-clang (in development)
project(falsealarm)

# littlefs_create_partition_image(littlefs data)

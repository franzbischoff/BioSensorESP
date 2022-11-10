cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(ENV{IDF_MAINTAINER} 1)
# set(ENV{IDF_TOOLCHAIN} clang)

# set(ENV{CMAKE_C_COMPILER_ID} "Clang")
# set(CMAKE_C_COMPILER xtensa-esp32-elf-clang)
# set(CMAKE_CXX_COMPILER xtensa-esp32-elf-clang++)
# set(CMAKE_ASM_COMPILER xtensa-esp32-elf-clang)

# set(CMAKE_AR xtensa-esp32-elf-ar)
# set(CMAKE_RANLIB xtensa-esp32-elf-gcc-ranlib)
# set(CMAKE_OBJDUMP xtensa-esp32-elf-objdump)
project(falsealarm)

# littlefs_create_partition_image(littlefs data)

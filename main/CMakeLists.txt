# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.
#FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(SRCS "main.cpp" "biosensor.cpp")

add_definitions(-DRESPIN=19 -DMFIOPIN=18 -DCONFIG_DISABLE_HAL_LOCKS
                -D_POSIX_READER_WRITER_LOCKS -DESP_PLATFORM -DI2C_SDA=GPIO_NUM_21 -DI2C_SCL=GPIO_NUM_22
                -D_GNU_SOURCE)

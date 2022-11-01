cmake_minimum_required(VERSION 3.5)
idf_component_register(SRCS "max32664.cpp" INCLUDE_DIRS "include"
    PRIV_REQUIRES driver)
target_compile_definitions(${COMPONENT_LIB} PRIVATE TEST_MODE)

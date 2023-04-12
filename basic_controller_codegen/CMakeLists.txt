cmake_minimum_required(VERSION 3.4.1)
project(RflyPilot_Basic_Controller)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lm -lstdc++ -lrt -O3")


aux_source_directory(./ basic_controler_source)
SET(param_source
param_load_block_wrapper.cpp)

include_directories(
    ./
    ../src/simulink_utility/
    )
add_library(basic_controller_codegen ${param_source} ${basic_controler_source} )

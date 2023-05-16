cmake_minimum_required(VERSION 3.4.1)
project(RflyPilot_Controller)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lm -lstdc++ -lrt -O3 -finline-functions -I. -I./acado/qpoases -I./acado/qpoases/INCLUDE -I./acado/qpoases/SRC")
aux_source_directory(./ controler_source)
include_directories(
    ./      
)
add_library(controller_codegen ${controler_source})

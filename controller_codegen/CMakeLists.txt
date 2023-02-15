cmake_minimum_required(VERSION 3.4.1)
project(RflyPilot_Controller)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lm -lstdc++ -lrt -O3 -finline-functions -I. -I./acado/qpoases -I./acado/qpoases/INCLUDE -I./acado/qpoases/SRC")


aux_source_directory(./ controler_source)
SET(acado_source
    ./acado/acado_solver.c
    ./acado/acado_integrator.c
    ./acado/acado_auxiliary_functions.c
    ./acado/acado_qpoases_interface.cpp
    ./acado/qpoases/SRC/Bounds.cpp
    ./acado/qpoases/SRC/Constraints.cpp
    ./acado/qpoases/SRC/CyclingManager.cpp
    ./acado/qpoases/SRC/Indexlist.cpp
    ./acado/qpoases/SRC/MessageHandling.cpp
    ./acado/qpoases/SRC/QProblem.cpp
    ./acado/qpoases/SRC/QProblemB.cpp
    ./acado/qpoases/SRC/SubjectTo.cpp
    ./acado/qpoases/SRC/Utils.cpp
    ./acado/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)

include_directories(
    ./
    ../src/simulink_utility/
    ./acado/
    ./acado/qpoases/
    ./acado/qpoases/INCLUDE
    ./acado/qpoases/INCLUDE/EXTRAS   
    ./acado/qpoases/SRC/EXTRAS   
    ./acado/qpoases/SRC       
    )
add_library(controller_codegen ${controler_source}  ${acado_source})

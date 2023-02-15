cmake_minimum_required(VERSION 3.4.1)
project(RflyPilot_Estimator)
set(CMAKE_BUILD_TYPE "Release")
aux_source_directory(./ Estimator)
include_directories(
    ./
    ../src/simulink_utility/
    )
add_library(estimator_codegen ${Estimator})
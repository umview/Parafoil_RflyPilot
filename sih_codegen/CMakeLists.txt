cmake_minimum_required(VERSION 3.4.1)
project(RflyPilot_SIH)
set(CMAKE_BUILD_TYPE "Release")
aux_source_directory(./ SIH)
include_directories(
    ./
    ../src/simulink_utility/
    )
add_library(sih_codegen ${SIH})
cmake_minimum_required(VERSION 3.4.1)
project(RflyPilot_CONF)
set(CMAKE_BUILD_TYPE "Release")
SET(conf_source
src/conf.c
src/hash/hash.c
src/stack/stack.c
)
include_directories(
include
include/conf-c	
)
add_library(confc ${conf_source})
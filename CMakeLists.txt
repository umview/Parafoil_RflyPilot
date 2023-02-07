cmake_minimum_required(VERSION 3.4.1)
project(RflyPilot)
set(CMAKE_BUILD_TYPE "Release")

#set(CMAKE_CXX_COMPILER "/usr/bin/arm-linux-gnueabihf-g++")
#set(CMAKE_C_COMPILER "/usr/bin/arm-linux-gnueabihf-gcc")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lrt -O3")

add_subdirectory(src/3rdparty/sbus)

SET(simulink_utility
    src/simulink_utility/rt_nonfinite.cpp
    src/simulink_utility/rtGetInf.cpp
    src/simulink_utility/rtGetNaN.cpp
)
SET(source_file
    src/main.cpp
    src/system_utility/systime.cpp
    src/system_utility/system.cpp

)
include_directories(
                    src
                    #src/3rdparty/conf-c/
                    #src/3rdparty/conf-c/include
                    src/simulink_utility
                    src/system_utility
                    )


add_executable(controller.o  ${source_file}  ${simulink_utility} )
target_link_libraries(controller.o  libsbus)


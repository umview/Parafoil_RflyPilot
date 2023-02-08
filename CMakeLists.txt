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
    src/lib/math_function.cpp
    src/msg/msg_def.cpp
    src/calibration/accel_calib.cpp
    src/calibration/gyro_calib.cpp
    src/calibration/mag_calib.cpp
    src/calibration/ellipsoid_method.cpp
    src/calibration/sensor_calibration.cpp
    src/parameter/parameter_read.cpp
)

SET(drivers_source
    src/drivers
    src/drivers/px4lib/test_time.cpp
    src/drivers/posix/i2c.cpp
    src/drivers/posix/spi.cpp
    src/drivers/actuator/pca9685/pca9685.cpp
    src/drivers/rc/sbus/sbus_api.cpp
    src/drivers/imu/icm42688p/ICM42688P.cpp
    src/drivers/imu/gyroscope/PX4Gyroscope.cpp
    src/drivers/imu/accelerometer/PX4Accelerometer.cpp    
)
SET(drivers_include 
    src/drivers
    src/drivers/px4lib
    src/drivers/actuator/pca9685
    src/drivers/rc/sbus
    src/drivers/imu/icm42688p
    src/drivers/imu/accelerometer
    src/drivers/imu/gyroscope
    src/drivers/gps
    src/drivers/barometer
    src/drivers/barometer
    src/drivers/magmetometer

)
SET(thread_source
    #src/thread/sbus_rc_thread.cpp
)
include_directories(
                    config/
                    src
                    src/lib
                    src/calibration
                    ${drivers_include}
                    src/3rdparty/conf-c/
                    src/3rdparty/conf-c/include
                    src/simulink_utility
                    src/system_utility
                    src/thread
                    src/msg
                    src/parameter
                    )

link_directories(
                src/3rdparty/conf-c/lib
                )
add_executable(controller.o  
    ${source_file}  
    ${simulink_utility} 
    ${drivers_source}
    ${thread_source})
target_link_libraries(controller.o  libsbus conf)


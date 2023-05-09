set(CMAKE_CXX_COMPILER "/usr/bin/arm-linux-gnueabihf-g++")
set(CMAKE_C_COMPILER "/usr/bin/arm-linux-gnueabihf-gcc")

#set(CMAKE_CXX_COMPILER "/usr/bin/aarch64-linux-gnu-g++")
#set(CMAKE_C_COMPILER "/usr/bin/aarch64-linux-gnu-gcc")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lrt -O3")
add_subdirectory(src/3rdparty/sbus)
add_subdirectory(controller_codegen)
add_subdirectory(basic_controller_codegen)

add_subdirectory(sih_codegen)
add_subdirectory(estimator_codegen)
add_subdirectory(src/3rdparty/conf-c)



cmake_minimum_required(VERSION 3.4.1)
project(RflyPilot)
set(CMAKE_BUILD_TYPE "Release")

aux_source_directory(src/simulink_utility simulink_utility)


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
    src/scope/scope.cpp
    src/log/binlog.cpp
    src/console/console.cpp
    src/screen/screen.cpp
)


aux_source_directory(src/application app_source)

SET(drivers_source
    src/drivers
    src/drivers/px4lib/test_time.cpp
    src/drivers/posix/i2c.cpp
    src/drivers/posix/spi.cpp
    src/drivers/actuator/pca9685/pca9685.cpp
    src/drivers/actuator/fpga/actuator_fpga.cpp
    src/drivers/rc/sbus/sbus_api.cpp
    src/drivers/imu/icm42688p/ICM42688P.cpp
    src/drivers/imu/icm20689/icm20689.cpp
    src/drivers/imu/gyroscope/PX4Gyroscope.cpp
    src/drivers/imu/accelerometer/PX4Accelerometer.cpp  
    src/drivers/magnetometer/ist8310/ist8310.cpp
    src/drivers/magnetometer/qmc5883l/QMC5883L.cpp  
    src/drivers/barometer/ms5611/ms5611.cpp
    src/drivers/gps/gps_api.cpp

)
SET(drivers_include 
    src/drivers
    src/drivers/px4lib
    src/drivers/actuator/pca9685
    src/drivers/actuator/fpga
    src/drivers/rc/sbus
    src/drivers/imu/icm42688p
    src/drivers/imu/accelerometer
    src/drivers/imu/gyroscope
    src/drivers/imu/icm20689
    src/drivers/gps
    src/drivers/barometer
    src/drivers/barometer
    src/drivers/magnetometer/ist8310
    src/drivers/magnetometer/qmc5883l
    src/drivers/barometer/ms5611
    src/drivers/gps
)

include_directories(
                    config/
                    src
                    src/lib
                    src/calibration
                    ${drivers_include}
                    src/3rdparty/conf-c/include
                    src/simulink_utility
                    src/system_utility
                    src/application
                    src/msg
                    src/parameter
                    src/scope
                    src/console
                    src/screen
                    src/log
                    controller_codegen
                    estimator_codegen
                    sih_codegen
                    basic_controller_codegen
                    )

link_directories(
                )
add_executable(rflypilot
    ${source_file}  
    ${app_source}
    ${simulink_utility} 
    ${drivers_source}
    ${estimator_source}
    ${controller_source}
    ${basic_controller_codegen}
    ${sih_source})
target_link_libraries(rflypilot 
confc  
libsbus 
estimator_codegen
controller_codegen 
basic_controller_codegen
sih_codegen
rt)

add_custom_target(
    upload
    COMMAND "../tools/upload.sh"
)
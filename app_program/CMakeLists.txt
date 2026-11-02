cmake_minimum_required(VERSION 3.13)

# MUST be set before project() for the SDK to pick up the correct compiler
set(PICO_BOARD pico_w CACHE STRING "Board type")

add_definitions(-DTARGET_DIR=pico_rp2040)

# Set the project BEFORE pulling in the SDK
project(app_program C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in the Pico SDK AFTER project()
include(${CMAKE_CURRENT_LIST_DIR}/../pico_sdk_import.cmake)

# Initialise the SDK (this configures the toolchain)
pico_sdk_init()

# Gather all application source files
set(APP_SOURCES
    app_main.c
    
    # Subsystem 1
    subsystem1/subsystem1.c
    subsystem1/telemetry_frame.c
    subsystem1/wifi_telemetry.c
    
    # Subsystem 2
    subsystem2/encoder.c
    subsystem2/motion_control.c
    subsystem2/motor.c
    subsystem2/pid.c
    subsystem2/subsystem2.c
    
    # Subsystem 3
    subsystem3/barcode.c
    #subsystem3/line_follow.c
    subsystem3/line_sensor.c
    subsystem3/subsystem3.c
    
    # Subsystem 4
    subsystem4/drv_i2c1.c
    subsystem4/drv_imu.c
    subsystem4/hump.c
    subsystem4/imu.c
    subsystem4/subsystem4.c
    
    # Subsystem 5
    subsystem5/obstacle.c
    subsystem5/servo.c
    subsystem5/subsystem5.c
    subsystem5/ultrasonic.c
)

# Create the executable
add_executable(app_program
    ${APP_SOURCES}
)

# Add include directories for both the app and the kernel
target_include_directories(app_program PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../kernel
    ${CMAKE_CURRENT_LIST_DIR}/../lib
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${CMAKE_CURRENT_LIST_DIR}/../config
)

# Link the Pico SDK
target_link_libraries(app_program
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_timer
)

# Enable USB serial for tm_printf logs
pico_enable_stdio_usb(app_program 1)
pico_enable_stdio_uart(app_program 0)

# Generate the .uf2 file
pico_add_extra_outputs(app_program)
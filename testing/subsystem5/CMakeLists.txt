# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)

project(subsystem5_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(subsystem5_test
    #s5_u01_echo_voltage.c
    #s5_u02_u03_distance.c
    #s5_u05_u06_u07_detection.c
    s5_u04_servo.c
)

# Link the standard Pico libraries
target_link_libraries(subsystem5_test
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_timer
)

# Enable USB serial output so I can see the PASS/FAIL logs
pico_enable_stdio_usb(subsystem5_test 1)
pico_enable_stdio_uart(subsystem5_test 0)

# Create map/bin/hex/uf2 file
pico_add_extra_outputs(subsystem5_test)
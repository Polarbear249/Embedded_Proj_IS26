# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(motor_pid_test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

# add_executable(encoder_test encoder_test.c )

# pico_set_program_name(encoder_test "encoder_test")
# pico_set_program_version(encoder_test "0.1")

# # Modify the below lines to enable/disable output over UART/USB
# pico_enable_stdio_uart(encoder_test 0)
# pico_enable_stdio_usb(encoder_test 1)

# # Add the standard library to the build
# target_link_libraries(encoder_test
#         pico_stdlib)

# # Add the standard include files to the build
# target_include_directories(encoder_test PRIVATE
#         ${CMAKE_CURRENT_LIST_DIR}
# )

# # Add any user requested libraries
# target_link_libraries(encoder_test 
        
#         )

# pico_add_extra_outputs(encoder_test)


# Create the PID motor control test
add_executable(motor_pid_test motor_pid_test.c)

# Program information
pico_set_program_name(motor_pid_test "motor_pid_test")
pico_set_program_version(motor_pid_test "0.1")

# Enable USB serial output
pico_enable_stdio_uart(motor_pid_test 0)
pico_enable_stdio_usb(motor_pid_test 1)

# Link the libraries needed for motor PWM and encoders
target_link_libraries(motor_pid_test
    pico_stdlib
    hardware_pwm
    hardware_sync
)

# Generate UF2 firmware
pico_add_extra_outputs(motor_pid_test)

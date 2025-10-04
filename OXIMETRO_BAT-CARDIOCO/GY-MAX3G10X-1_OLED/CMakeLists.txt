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
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(GY-MAX3G10X-1 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(GY-MAX3G10X-1
    GY-MAX3G10X-1.cpp  # <--- CORRIGIDO AQUI!
    MAX30105.cpp
    heartRate.cpp
    spo2_algorithm.cpp
    inc/ssd1306_i2c.c 
)

pico_set_program_name(GY-MAX3G10X-1 "GY-MAX3G10X-1")
pico_set_program_version(GY-MAX3G10X-1 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(GY-MAX3G10X-1 0)
pico_enable_stdio_usb(GY-MAX3G10X-1 1)

# Add the standard library to the build
target_link_libraries(GY-MAX3G10X-1
        pico_stdlib
        hardware_i2c)

# Add the standard include files to the build
target_include_directories(GY-MAX3G10X-1 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(GY-MAX3G10X-1 
        
        )

pico_add_extra_outputs(GY-MAX3G10X-1)


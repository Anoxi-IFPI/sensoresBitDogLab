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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# Definir o BOARD como pico_w, pois você está usando um Raspberry Pi Pico W.
# No entanto, como você não quer usar Wi-Fi, garanta que o código Wi-Fi esteja removido do main.c.
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Sensor_VL53L0X C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Adiciona o seu arquivo principal (Sensor_VL53L0X.c) e o arquivo da biblioteca tof.c
add_executable(Sensor_VL53L0X
    Sensor_VL53L0X.c   # Seu arquivo principal
    tof.c              # O arquivo de implementação da biblioteca VL53L0X (tof.c)
)

pico_set_program_name(Sensor_VL53L0X "Sensor_VL53L0X")
pico_set_program_version(Sensor_VL53L0X "0.1")

# Habilita a saída serial via USB (printf) e desabilita UART.
pico_enable_stdio_uart(Sensor_VL53L0X 0)
pico_enable_stdio_usb(Sensor_VL53L0X 1)

# Linka as bibliotecas padrão e as que você precisa.
# Note que 'hardware_wifi' e 'cyw43_no_pins' foram removidos,
# pois você não quer usar a funcionalidade Wi-Fi neste projeto.
target_link_libraries(Sensor_VL53L0X
    pico_stdlib
    hardware_i2c      # Necessário para comunicação I2C
)

# Adiciona os diretórios de inclusão para o seu arquivo principal e para tof.h.
# Assumimos que 'tof.h' e 'tof.c' estão na mesma pasta que 'Sensor_VL53L0X.c'.
target_include_directories(Sensor_VL53L0X PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(Sensor_VL53L0X)
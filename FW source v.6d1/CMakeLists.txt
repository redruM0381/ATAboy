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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ATAboy C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ATAboy
        ATAboy.c
        ide_pio.c
        ide.c
        menus.c
        usb.c
        usb_descriptors.c
        config.c)

pico_set_program_name(ATAboy "ATAboy")
pico_set_program_version(ATAboy "0.6d")

# Generate PIO header
pico_generate_pio_header(ATAboy ${CMAKE_CURRENT_LIST_DIR}/ataboy.pio)

# No pico_stdio — all console I/O through TinyUSB CDC via queues
pico_enable_stdio_uart(ATAboy 0)
pico_enable_stdio_usb(ATAboy 0)

# Link libraries
target_link_libraries(ATAboy
        pico_stdlib
        pico_multicore
        hardware_flash
        hardware_sync
        hardware_pio
        hardware_dma
        hardware_clocks
        tinyusb_device
        tinyusb_board
        )

# Add the standard include files to the build
target_include_directories(ATAboy PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(ATAboy)


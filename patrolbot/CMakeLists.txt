cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME patrolbot)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/BSP/encoder/encoder.h
        Core/BSP/encoder/encoder.c
        Core/Algorithm/pid.h
        Core/Algorithm/pid.c
        Core/BSP/i2c_bus/i2c_bus.h
        Core/BSP/i2c_bus/i2c_bus.c
        Core/BSP/gy906/gy906.h
        Core/BSP/gy906/gy906.c
        Core/BSP/oled/oled.h
        Core/BSP/oled/oled.c
        Core/BSP/oled/font.h
        Core/BSP/oled/font.c
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        Core/BSP/motor/motor_driver.c
        Core/BSP/motor/motor_driver.h
        Core/BSP/encoder/encoder.c
        Core/BSP/encoder/encoder.h
        Core/Algorithm/pid.c
        Core/Algorithm/pid.h
        Core/BSP/i2c_bus/i2c_bus.c
        Core/BSP/i2c_bus/i2c_bus.h
        Core/BSP/gy906/gy906.c
        Core/BSP/gy906/gy906.h
        Core/BSP/oled/oled.c
        Core/BSP/oled/oled.h
        Core/BSP/oled/font.c
        Core/BSP/oled/font.h
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Core/BSP/motor
        Core/BSP/encoder
        Core/Algorithm
        Core/BSP/i2c_bus
        Core/BSP/gy906
        Core/BSP/oled
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

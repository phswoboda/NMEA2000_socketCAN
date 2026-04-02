cmake_minimum_required(VERSION 3.5...3.27)

# NMEA2000 SocketCAN Driver Library
# Provides Linux SocketCAN support for the NMEA2000 library

add_library(NMEA2000_SocketCAN STATIC
    NMEA2000_SocketCAN.cpp
)

target_include_directories(NMEA2000_SocketCAN PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/NMEA2000/src
)

target_link_libraries(NMEA2000_SocketCAN PUBLIC
    nmea2000
)

target_compile_features(NMEA2000_SocketCAN PUBLIC cxx_std_11)

# Linux-specific settings for SocketCAN
if(UNIX AND NOT APPLE)
    target_compile_definitions(NMEA2000_SocketCAN PRIVATE
        __linux__
    )
endif()
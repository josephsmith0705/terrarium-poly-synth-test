cmake_minimum_required(VERSION 3.50)
project(TerrariumPolySynth VERSION 1.0.0)

set(FIRMWARE_NAME TerrariumPolySynth)
set(GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)
set(PIANO_SAMPLE_HEADER ${GENERATED_DIR}/PianoASample.h)

add_custom_command(
    OUTPUT ${PIANO_SAMPLE_HEADER}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED_DIR}
    COMMAND python3 ${CMAKE_SOURCE_DIR}/util/embed_wav_header.py
            --input ${CMAKE_SOURCE_DIR}/PianoA.wav
            --output ${PIANO_SAMPLE_HEADER}
            --symbol PianoA_wav
            --target-rate 12000
            --max-seconds 1.8
    DEPENDS ${CMAKE_SOURCE_DIR}/PianoA.wav
            ${CMAKE_SOURCE_DIR}/util/embed_wav_header.py
    COMMENT "Embedding PianoA.wav as header"
)

add_custom_target(embed_piano_sample DEPENDS ${PIANO_SAMPLE_HEADER})

set(FIRMWARE_SOURCES
    main.cpp
    syscalls.c
    util/Led.h
    util/Led.cpp
    util/Terrarium.h
    util/Terrarium.cpp
)
set(LIBDAISY_DIR ${CMAKE_SOURCE_DIR}/lib/libDaisy)
include(${LIBDAISY_DIR}/cmake/default_build.cmake)

option(Q_BUILD_EXAMPLES "build Q library examples" OFF)
option(Q_BUILD_TEST "build Q library tests" OFF)
option(Q_BUILD_IO "build Q IO library" OFF)
add_subdirectory(lib/q)

add_subdirectory(lib/gcem)

target_include_directories(${FIRMWARE_NAME} PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(${FIRMWARE_NAME} PRIVATE ${GENERATED_DIR})
target_link_libraries(${FIRMWARE_NAME} PUBLIC libq gcem)
add_dependencies(${FIRMWARE_NAME} embed_piano_sample)

set_target_properties(${FIRMWARE_NAME} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
)

target_link_options(${FIRMWARE_NAME} PRIVATE
    -flto=auto
)

if(NOT PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
    # Git auto-ignore out-of-source build directory
    file(GENERATE OUTPUT .gitignore CONTENT "*")
endif()

cmake_minimum_required(VERSION 3.28)

if(NOT KVASIR_ROOT)
    if(DEFINED ENV{KVASIR_ROOT})
        set(KVASIR_ROOT $ENV{KVASIR_ROOT})
    else()
        include(FetchContent)
        FetchContent_Declare(
            Kvasir_SDK
            GIT_REPOSITORY https://github.com/kvasir-io/Kvasir_SDK.git
            GIT_TAG master)
        FetchContent_Declare(
            chip_atsamd21
            GIT_REPOSITORY https://github.com/kvasir-io/chip_atsamd21.git
            GIT_TAG master)
        FetchContent_Declare(
            kvasir_devices
            GIT_REPOSITORY https://github.com/kvasir-io/kvasir_devices.git
            GIT_TAG master)
        FetchContent_MakeAvailable(Kvasir_SDK chip_atsamd21 kvasir_devices)
        set(KVASIR_ROOT ${kvasir_sdk_SOURCE_DIR})
        set(CHIP_ROOT ${chip_atsamd21_SOURCE_DIR})
        set(KVASIR_DEVICES_ROOT ${kvasir_devices_SOURCE_DIR})
    endif()
endif()

include(${KVASIR_ROOT}/cmake/kvasir.cmake)

project(
    "water_mix"
    VERSION 0.1.0
    DESCRIPTION "Water mixing valve controller firmware using Kvasir SDK")

# util.cmake pulls in KVASIR_DEVICES_ROOT itself and defines kvasir::devices - this is why there is no
# add_subdirectory(kvasir_devices) here any more.
include(${KVASIR_ROOT}/cmake/util.cmake)

# Valve.hpp and Regulator.hpp test themselves in static_asserts that simulate minutes of the valve on its pipes, with
# mp-units quantities all the way: about 30 million constexpr steps, clang stops at one million by default.
add_compile_options(-fconstexpr-steps=100000000)

kvasir_executable_variants(
    ""
    SOURCES
    src/main.cpp
    LIBRARIES
    kvasir::devices
    OPTIMIZATION
    size)

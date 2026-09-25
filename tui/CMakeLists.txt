cmake_minimum_required(VERSION 3.28)

# The remote control for the valve: a host program, built on its own (`just tui`), not part of the firmware build. It
# talks Protocol.hpp to the socket uc_log_printer opens for the firmware's "control" duplex channel.
project(water_mix_tui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# aglio: the sibling checkout if it is there, as for the firmware; its own CMake brings glaze.
set(AGLIO_USE_ENCHANTUM OFF)
if(NOT AGLIO_ROOT)
    set(AGLIO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../aglio)
endif()
if(EXISTS ${AGLIO_ROOT}/CMakeLists.txt)
    add_subdirectory(${AGLIO_ROOT} ${CMAKE_BINARY_DIR}/aglio)
else()
    FetchContent_Declare(
        aglio
        GIT_REPOSITORY https://github.com/dominicpoeschko/aglio.git
        GIT_TAG master)
    FetchContent_MakeAvailable(aglio)
endif()

# The version uc_log_printer uses. `just tui` points FETCHCONTENT_SOURCE_DIR_FTXUI at the copy the firmware build has
# already fetched, so nothing is downloaded twice.
set(FTXUI_ENABLE_INSTALL OFF)
set(FTXUI_QUIET ON)
FetchContent_Declare(
    ftxui
    GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
    GIT_TAG v7.0.2)
FetchContent_MakeAvailable(ftxui)

# mp-units: Protocol.hpp speaks in quantities (src/Units.hpp). The pin kvasir_devices has; `just tui` points
# FETCHCONTENT_SOURCE_DIR_MP-UNITS at the firmware build's copy.
set(MP_UNITS_API_CONTRACTS
    "NONE"
    CACHE STRING "" FORCE) # otherwise mp-units wants gsl-lite
set(MP_UNITS_API_NATURAL_UNITS
    OFF
    CACHE BOOL "" FORCE)
set(MP_UNITS_BUILD_CXX_MODULES
    OFF
    CACHE BOOL "" FORCE)
set(MP_UNITS_BUILD_INSTALL
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(
    mp-units
    GIT_REPOSITORY https://github.com/mpusz/mp-units.git
    GIT_TAG v2.5.0
    GIT_SHALLOW ON
    SOURCE_SUBDIR src
    UPDATE_DISCONNECTED ON)
FetchContent_MakeAvailable(mp-units)
foreach(_t mp-units-core mp-units-systems)
    if(TARGET ${_t})
        get_target_property(_dirs ${_t} INTERFACE_INCLUDE_DIRECTORIES)
        if(_dirs)
            target_include_directories(${_t} SYSTEM INTERFACE ${_dirs})
        endif()
    endif()
endforeach()

add_executable(water_mix_tui main.cpp)
target_include_directories(water_mix_tui PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../src)
target_link_libraries(water_mix_tui PRIVATE aglio::aglio mp-units::mp-units ftxui::screen ftxui::dom ftxui::component)
# Where `just log` puts the printer's unix sockets: the tui looks there for duplex.0.sock when not told otherwise.
target_compile_definitions(water_mix_tui PRIVATE TUI_RTT_LOG_DIR="${CMAKE_CURRENT_LIST_DIR}/../build/rtt_log")
target_compile_options(water_mix_tui PRIVATE -Wall -Wextra)
# The headers' static_assert simulations: see the firmware's CMakeLists.txt.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(water_mix_tui PRIVATE -fconstexpr-steps=100000000)
else()
    target_compile_options(water_mix_tui PRIVATE -fconstexpr-ops-limit=4000000000 -fconstexpr-loop-limit=100000000)
endif()

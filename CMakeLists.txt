cmake_minimum_required(VERSION 3.16)
project(VmMsoOpenSDK VERSION 1.22 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(VMOPEN_BUILD_TESTS "Build test programs" ON)
option(VMOPEN_BUILD_GUI "Build the VimuLink Qt GUI" ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
find_package(Threads REQUIRED)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

add_library(vmcore STATIC
    src/core/analysis.cpp
    src/core/dds.cpp
    src/core/devinfo.cpp
    src/core/devtables.cpp
    src/core/io.cpp
    src/core/log.cpp
    src/core/osc.cpp
    src/core/stream.cpp
    src/core/usbdev.cpp
    src/core/usbmng.cpp
    src/api/context.cpp)
target_include_directories(vmcore PUBLIC include)
target_link_libraries(vmcore PUBLIC PkgConfig::LIBUSB Threads::Threads)

# single device API, drop-in for the vendor libvmmso.so
add_library(vmmso SHARED src/api/vmmso_single.cpp)
target_link_libraries(vmmso PRIVATE vmcore)
target_include_directories(vmmso PUBLIC include)
set_target_properties(vmmso PROPERTIES VERSION 1.22 SOVERSION 1.22)

# multi device API, drop-in for the vendor libvmmultmso.so
add_library(vmmultmso SHARED src/api/vmmultmso_multi.cpp)
target_link_libraries(vmmultmso PRIVATE vmcore)
target_include_directories(vmmultmso PUBLIC include)
set_target_properties(vmmultmso PROPERTIES VERSION 1.22 SOVERSION 1.22)

# DSP library used by the GUI, drop-in for the vendor libvmsignal.so
set(VMSIGNAL_INC include/vmsignal/inc)
add_library(vmsignal SHARED
    vmsignal/src/fft.cpp
    vmsignal/src/fftinfo.cpp
    vmsignal/src/filters.cpp
    vmsignal/src/fir.cpp
    vmsignal/src/iir.cpp
    vmsignal/src/jacobi.cpp
    vmsignal/src/suanfa.cpp
    vmsignal/src/window.cpp)
target_compile_definitions(vmsignal PRIVATE SIGNAL_SHARED_LIBRARY)
target_include_directories(vmsignal PUBLIC include ${VMSIGNAL_INC} ${VMSIGNAL_INC}/fft ${VMSIGNAL_INC}/fftcpp
    ${VMSIGNAL_INC}/fir ${VMSIGNAL_INC}/iir ${VMSIGNAL_INC}/jacobi ${VMSIGNAL_INC}/window ${VMSIGNAL_INC}/convol)
# like the vendor library, export everything declared in the headers
set_target_properties(vmsignal PROPERTIES VERSION 1.0 SOVERSION 1.0 CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF)

if(VMOPEN_BUILD_TESTS)
    add_executable(probe_single tests/probe_single.cpp)
    target_link_libraries(probe_single PRIVATE vmmso)
    add_executable(probe_multi tests/probe_multi.cpp)
    target_link_libraries(probe_multi PRIVATE vmmultmso)
    add_executable(sigtest tests/sigtest.cpp)
    target_link_libraries(sigtest PRIVATE vmsignal)
    target_compile_options(sigtest PRIVATE -Wno-unused-variable)
    add_executable(bench_fft tests/bench_fft.cpp)
    target_link_libraries(bench_fft PRIVATE vmsignal)
    add_executable(bench_calfreq tests/bench_calfreq.cpp)
    target_link_libraries(bench_calfreq PRIVATE vmmultmso)
endif()

include(GNUInstallDirs)
if(VMOPEN_BUILD_GUI)
    add_subdirectory(gui/VimuLink)
endif()

install(TARGETS vmmso vmmultmso vmsignal LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/VmMsoLib.h include/VmMultMsoLib.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(NOT APPLE)
    install(FILES udev/99-vmmso.rules DESTINATION lib/udev/rules.d OPTIONAL)
endif()

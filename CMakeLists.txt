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
set_target_properties(vmmso PROPERTIES VERSION 1.22 SOVERSION 1)

# multi device API, drop-in for the vendor libvmmultmso.so
add_library(vmmultmso SHARED src/api/vmmultmso_multi.cpp)
target_link_libraries(vmmultmso PRIVATE vmcore)
target_include_directories(vmmultmso PUBLIC include)
set_target_properties(vmmultmso PROPERTIES VERSION 1.22 SOVERSION 1)

if(VMOPEN_BUILD_TESTS)
    add_executable(probe_single tests/probe_single.cpp)
    target_link_libraries(probe_single PRIVATE vmmso)
endif()

include(GNUInstallDirs)
install(TARGETS vmmso vmmultmso LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/VmMsoLib.h include/VmMultMsoLib.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES udev/99-vmmso.rules DESTINATION lib/udev/rules.d OPTIONAL)

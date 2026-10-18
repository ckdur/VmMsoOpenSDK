# VimuLink GUI built against the open-source libvmmultmso and libvmsignal.
# Built from the top level CMakeLists (option VMOPEN_BUILD_GUI).
cmake_minimum_required(VERSION 3.16)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core Quick Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Quick Widgets QuickControls2 REQUIRED)

set(VIMULINK_SOURCES
    main.cpp
    fftcal.cpp fftcal.h
    vmusbwave.cpp vmusbwave.h
    filtercal.cpp filtercal.h
    plotrecord.cpp plotrecord.h
    defines.h arbwaves.h
    plot/Axis.cpp plot/Axis.h
    plot/PlotBasic.cpp plot/PlotBasic.h
    plot/PlotHelp.cpp plot/PlotHelp.h
    plot/SimplePlotBasic.cpp plot/SimplePlotBasic.h
    plot/VmSimplePlot.cpp plot/VmSimplePlot.h
    plot/plottype.h
    wavemng/AbsPhysicsBlockFile.cpp wavemng/AbsPhysicsBlockFile.h
    wavemng/AbsSummary.cpp wavemng/AbsSummary.h
    wavemng/BlockFile.cpp wavemng/BlockFile.h
    wavemng/BlockFileInfo.cpp wavemng/BlockFileInfo.h
    wavemng/EmsPhysicsBlockFile.cpp wavemng/EmsPhysicsBlockFile.h
    wavemng/HdPhysicsBlockFile.cpp wavemng/HdPhysicsBlockFile.h
    wavemng/MemSummary.cpp wavemng/MemSummary.h
    wavemng/plotpoints.cpp wavemng/plotpoints.h
    wavemng/points.h
    wavemng/SummaryInfo.cpp wavemng/SummaryInfo.h
    wavemng/TempFileMng.cpp wavemng/TempFileMng.h
    wavemng/TempFileMngRec.cpp wavemng/TempFileMngRec.h
    wavemng/wavefile.cpp wavemng/wavefile.h
    wavemng/wavemngdef.h
    wavemng/WaveTrack.cpp wavemng/WaveTrack.h
    wavemng/WaveTrackInfo.cpp wavemng/WaveTrackInfo.h
    filter/BasicFilter.cpp filter/BasicFilter.h
    filter/FreqFirFilter.cpp filter/FreqFirFilter.h
    filter/IirFilter.cpp filter/IirFilter.h
    filter/WindowFirFilter.cpp filter/WindowFirFilter.h
    vmmso.qrc)

if(QT_VERSION_MAJOR GREATER_EQUAL 6)
    qt_add_executable(VimuLink MANUAL_FINALIZATION ${VIMULINK_SOURCES})
else()
    add_executable(VimuLink ${VIMULINK_SOURCES})
endif()

target_include_directories(VimuLink PRIVATE . plot wavemng filter)
target_compile_options(VimuLink PRIVATE -w)   # upstream code, keep the build quiet
target_link_libraries(VimuLink PRIVATE
    Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt${QT_VERSION_MAJOR}::QuickControls2
    Qt${QT_VERSION_MAJOR}::Widgets vmmultmso vmsignal)
set_target_properties(VimuLink PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(VimuLink)
endif()

install(TARGETS VimuLink RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

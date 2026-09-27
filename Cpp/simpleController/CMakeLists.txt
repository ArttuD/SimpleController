cmake_minimum_required(VERSION 3.16)

project(simpleController VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

set(PROJECT_SOURCES
        main.cpp
        controller.cpp
        controller.h
        nidaqworker.cpp
        nidaqworker.h
        csvsaver.cpp
        csvsaver.h
        plotwidget.cpp
        plotwidget.h
        mainwindow.cpp
        mainwindow.h
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(simpleController
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET simpleController APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(simpleController SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(simpleController
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(simpleController PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

if(WIN32)
    target_link_libraries(simpleController PRIVATE winmm)
endif()

if(NOT WIN32)
    message(FATAL_ERROR "The native NI-DAQmx backend is currently configured for Windows.")
endif()

set(NIDAQMX_ROOT "$ENV{NIEXTCCOMPILERSUPP}" CACHE PATH "NI-DAQmx C API root")
if(NOT NIDAQMX_ROOT)
    set(NIDAQMX_ROOT "C:/Program Files (x86)/National Instruments/Shared/ExternalCompilerSupport/C")
endif()
find_path(NIDAQMX_INCLUDE_DIR NIDAQmx.h HINTS "${NIDAQMX_ROOT}/include" REQUIRED)
find_library(NIDAQMX_LIBRARY NAMES NIDAQmx nidaqmx HINTS "${NIDAQMX_ROOT}/lib64/msvc" REQUIRED)
target_include_directories(simpleController SYSTEM PRIVATE "${NIDAQMX_INCLUDE_DIR}")
target_link_libraries(simpleController PRIVATE "${NIDAQMX_LIBRARY}")
if(MINGW)
    target_compile_options(simpleController PRIVATE -fms-extensions)
endif()

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.simpleController)
endif()
set_target_properties(simpleController PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS simpleController
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(simpleController)
endif()

cmake_minimum_required(VERSION 3.16)

project(QtKittiVisualizer)

# Make sure PCL finds the installed Eigen3 and FLANN:
# FindEigen.cmake and FindFLANN.cmake
# were just copied from pcl-1.12.1/cmake/Modules
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

# You can alternatively include the modules from your PCL source directly:
#set(CMAKE_MODULE_PATH $ENV{HOME}/pcl-1.12.1/cmake/Modules)

#message(STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})


# VTK
# ===

# Hint were to find the VTK source code.
if (NOT DEFINED VTK_DIR)
    set(VTK_DIR $ENV{HOME}/vtk-9.1.0-build/)
endif()

find_package(VTK 9.1.0 REQUIRED COMPONENTS
  CommonCore
  FiltersSources
  GUISupportQt
  InteractionStyle
  RenderingCore
  RenderingOpenGL2
)

message(STATUS "VTK_QT_VERSION: " ${VTK_QT_VERSION})


# Qt
# ==

find_package(Qt5Widgets REQUIRED COMPONENTS
  Core
  Gui
  REQUIRED
)


# Boost
# =====

find_package(Boost 1.71 COMPONENTS
  program_options
  filesystem
  REQUIRED
)
message(STATUS "Boost_VERSION: " ${Boost_VERSION})
set(Boost_COMPONENTS_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
set(Boost_COMPONENTS_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})
set(Boost_COMPONENTS_LIBRARIES ${Boost_LIBRARIES})
include_directories(${Boost_COMPONENTS_INCLUDE_DIRS})
link_directories(${Boost_COMPONENTS_LIBRARY_DIRS})
add_definitions(${Boost_COMPONENTS_DEFINITIONS})


# PCL
# ===

# Hint were to find the PCL source code.
if (NOT DEFINED PCL_DIR)
    set(PCL_DIR $ENV{HOME}/pcl-1.12.1-build/)
endif()

find_package(PCL 1.12.1 REQUIRED)
message(STATUS "PCL_VERSION: " ${PCL_VERSION})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Target: qt-kitti-visualizer
# ===========================

set(PROJECT_BINARY_NAME qt-kitti-visualizer)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_SOURCE_DIR})

set(CPP_FILES
  KittiConfig.cpp
  KittiDataset.cpp
  main.cpp
  QtKittiVisualizer.cpp
)

add_executable(${PROJECT_BINARY_NAME}
  ${CPP_FILES}
)

target_link_libraries(${PROJECT_BINARY_NAME} PRIVATE
  ${PCL_LIBRARIES}
  Qt5::Widgets
  VTK::CommonCore
  VTK::FiltersSources
  VTK::GUISupportQt
  VTK::InteractionStyle
  VTK::RenderingCore
  VTK::RenderingOpenGL2
  ${Boost_COMPONENTS_LIBRARIES}
)

set_target_properties(${PROJECT_BINARY_NAME} PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
)

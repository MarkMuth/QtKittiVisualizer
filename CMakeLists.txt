# Resources:
#
# http://www.vtk.org/Wiki/VTK/Examples/Cxx/Qt/RenderWindowNoUiFile

cmake_minimum_required(VERSION 2.8)

if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()

project(QtKittiVisualizer)




if (MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS -DBOOST_ALL_NO_LIB -DBOOST_ALL_DYN_LINK -DBOOST_LOG_DYN_LINK)
endif()

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

message(STATUS "VTK_QT_VERSION: " ${VTK_QT_VERSION})
if(${VTK_VERSION} VERSION_GREATER "6" AND VTK_QT_VERSION VERSION_GREATER "4")
  message(STATUS "Using Qt5")
  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5Widgets REQUIRED QUIET)
else()
  message(STATUS "Using Qt4")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(PROJECT_BINARY_NAME qt-kitti-visualizer)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_SOURCE_DIR})

find_package(Boost 1.54 COMPONENTS program_options filesystem serialization REQUIRED)
set(Boost_COMPONENTS_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
set(Boost_COMPONENTS_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})
set(Boost_COMPONENTS_LIBRARIES ${Boost_LIBRARIES})
include_directories(${Boost_COMPONENTS_INCLUDE_DIRS})
link_directories(${Boost_COMPONENTS_LIBRARY_DIRS})
add_definitions(${Boost_COMPONENTS_DEFINITIONS})

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CPP_FILES
    KittiConfig.cpp
    KittiDataset.cpp
    main.cpp
    QtKittiVisualizer.cpp
)
set(WRAP_CPP_FILES QtKittiVisualizer.h)
set(WRAP_UI_FILES QtKittiVisualizer.ui)

if(${VTK_VERSION} VERSION_GREATER "6" AND VTK_QT_VERSION VERSION_GREATER "4")
  qt5_wrap_ui(PROJECT_FORMS_HEADERS ${WRAP_UI_FILES} )
  # CMAKE_AUTOMOC in ON so the MOC headers will be automatically wrapped.
  add_executable(${PROJECT_BINARY_NAME}
    ${CPP_FILES}
    ${PROJECT_FORMS_HEADERS}
    ${WRAP_CPP_FILES})
  qt5_use_modules(${PROJECT_BINARY_NAME} Core Gui)
  target_link_libraries(${PROJECT_BINARY_NAME}
      ${PCL_LIBRARIES}
      ${VTK_LIBRARIES}
      ${Boost_COMPONENTS_LIBRARIES})
else()
  QT4_WRAP_UI(PROJECT_FORMS_HEADERS ${WRAP_UI_FILES})
  QT4_WRAP_CPP(PROJECT_HEADERS_MOC ${WRAP_CPP_FILES})
  add_executable(${PROJECT_BINARY_NAME}
      ${CPP_FILES}
      ${PROJECT_FORMS_HEADERS}
      ${PROJECT_HEADERS_MOC})

  if(VTK_LIBRARIES)
    if(${VTK_VERSION} VERSION_LESS "6")
      target_link_libraries(${PROJECT_BINARY_NAME}
          ${PCL_LIBRARIES}
          ${VTK_LIBRARIES}
          QVTK
          ${Boost_COMPONENTS_LIBRARIES})
    else()
      target_link_libraries(${PROJECT_BINARY_NAME}
          ${PCL_LIBRARIES}
          ${VTK_LIBRARIES}
          ${Boost_COMPONENTS_LIBRARIES})
    endif()
  else()
    target_link_libraries(${PROJECT_BINARY_NAME}
        vtkHybrid
        QVTK
        vtkViews
        ${QT_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_COMPONENTS_LIBRARIES})
  endif()
endif()

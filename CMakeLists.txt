cmake_minimum_required(VERSION 2.8)

project(QtKittiVisualizer)

set(PROJECT_BINARY_NAME qt-kitti-visualizer)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

find_package(Boost 1.54 COMPONENTS program_options filesystem REQUIRED)
set(Boost_COMPONENTS_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
set(Boost_COMPONENTS_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})
set(Boost_COMPONENTS_LIBRARIES ${Boost_LIBRARIES})
include_directories(${Boost_COMPONENTS_INCLUDE_DIRS})
link_directories(${Boost_COMPONENTS_LIBRARY_DIRS})
add_definitions(${Boost_COMPONENTS_DEFINITIONS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(VTK REQUIRED)
set(VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

set(CPP_FILES
    KittiConfig.cpp
    KittiDataset.cpp
    main.cpp
    QtKittiVisualizer.cpp
)
set(WRAP_CPP_FILES QtKittiVisualizer.h)
set(WRAP_UI_FILES QtKittiVisualizer.ui)

qt4_wrap_cpp(PROJECT_HEADERS_MOC ${WRAP_CPP_FILES})
qt4_wrap_ui(PROJECT_FORMS_HEADERS ${WRAP_UI_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR})

add_executable(${PROJECT_BINARY_NAME}
    ${CPP_FILES}
    ${PROJECT_HEADERS_MOC}
    ${PROJECT_FORMS_HEADERS}
)
target_link_libraries(${PROJECT_BINARY_NAME}
    ${QT_LIBRARIES}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    ${Boost_COMPONENTS_LIBRARIES}
)

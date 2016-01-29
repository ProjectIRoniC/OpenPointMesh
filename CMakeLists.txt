cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(VBA)

find_package(Qt4 REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ../include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PROJECT_SOURCES main.cpp oni-to-pcd.cpp mainwindow.cpp errorMsgHandler.cpp CloudStitcher.cpp PCDRegistration.cpp MeshConstructor.cpp filesystemHelper.cpp)
set(project_HEADERS include/errorMsgHandler.h include/oni-to-pcd.h include/mainwindow.h include/CloudStitcher.h include/PCDRegistration.h include/MeshConstructor.h include/filesystemHelper.h include/OniProperties.h)
set(project_FORMS   src/mainwindow.ui)
set(VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)
set(PROJECT_EXECS src/main.cpp src/oni-to-pcd.cpp src/errorMsgHandler.cpp src/mainwindow.cpp src/CloudStitcher.cpp src/PCDRegistration.cpp src/MeshConstructor.cpp src/filesystemHelper.cpp)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})


ADD_EXECUTABLE  (vba ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC} 
				${PROJECT_EXECS} )
target_link_libraries(vba ${PCL_LIBRARIES} ${QT_LIBRARIES} ${VTK_LIBRARIES})


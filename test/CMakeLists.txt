CMAKE_MINIMUM_REQUIRED( VERSION 2.8 FATAL_ERROR )

####Executable Names
SET( MESH_CONSTRUCTOR_TEST "MeshConstructorTest" )
SET( CLOUD_STITCHER_TEST "CloudStitcherTest" )
SET( ONI_TO_PCD_TEST "OniToPcdTest" )


#### Package Dependencies
FIND_PACKAGE( Qt4 REQUIRED )
FIND_PACKAGE( VTK REQUIRED )
FIND_PACKAGE( PCL 1.7 REQUIRED )
FIND_PACKAGE( GTest REQUIRED )

#### Variables
FILE( GLOB OPENPOINTMESH_HEADERS "../include/*.h" )
SET( OPENPOINTMESH_SOURCES  "../src/AboutDialog.cpp"
                            "../src/AccuracyControlMenu.cpp"
                            "../src/CloudRegistrationFunctions.cpp"
                            "../src/CloudStitcher.cpp"
                            "../src/MeshConstructor.cpp"
                            "../src/PointCloudFilters.cpp"
                            "../src/errorMsgHandler.cpp"
                            "../src/filesystemHelper.cpp"
                            "../src/mainwindow.cpp"
                            "../src/oni-to-pcd.cpp"
                            "../src/viewer.cpp" )
FILE( GLOB OPENPOINTMESH_FORMS "../src/*.ui" )
SET( OPENPOINTMESH_INCLUDE_DIRS "../include" )
SET( VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK )

#### Qt4 Settings
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( OPENPOINTMESH_HEADERS_MOC ${OPENPOINTMESH_HEADERS} )
QT4_WRAP_UI( OPENPOINTMESH_FORMS_HEADERS ${OPENPOINTMESH_FORMS} )

#### Platform Specific Build Flags
IF( UNIX )
	MESSAGE( STATUS "Setting GCC flags" )
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -g -Wall" )
ELSE ()
	MESSAGE( FATAL_ERROR "You are on an unsupported platform! (Not UNIX)" )
ENDIF()
MESSAGE( STATUS "** CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}" )

#### Build Settings

INCLUDE_DIRECTORIES(	${PCL_INCLUDE_DIRS}
						${OPENPOINTMESH_INCLUDE_DIRS}
						${GTEST_INCLUDE_DIRS} )

LINK_DIRECTORIES( ${PCL_LIBRARY_DIRS} )

ADD_DEFINITIONS(	${PCL_DEFINITIONS}
					${QT_DEFINITIONS} )

ADD_EXECUTABLE(	${MESH_CONSTRUCTOR_TEST}
                "src/MeshConstructorTest.cpp"
				${OPENPOINTMESH_SOURCES}
				${OPENPOINTMESH_FORMS_HEADERS}
				${OPENPOINTMESH_HEADERS_MOC} )


target_link_libraries(	${MESH_CONSTRUCTOR_TEST}
						${PCL_LIBRARIES}
						${QT_LIBRARIES}
						${VTK_LIBRARIES}
						${GTEST_LIBRARIES}
						pthread )

ADD_EXECUTABLE(	${CLOUD_STITCHER_TEST}
                "src/CloudStitcherTest.cpp"
				${OPENPOINTMESH_SOURCES}
				${OPENPOINTMESH_FORMS_HEADERS}
				${OPENPOINTMESH_HEADERS_MOC} )

target_link_libraries(	${CLOUD_STITCHER_TEST}
						${PCL_LIBRARIES}
						${QT_LIBRARIES}
						${VTK_LIBRARIES}
						${GTEST_LIBRARIES}
						pthread )

ADD_EXECUTABLE(	${ONI_TO_PCD_TEST}
                "src/OniToPcdTest.cpp"
				${OPENPOINTMESH_SOURCES}
				${OPENPOINTMESH_FORMS_HEADERS}
				${OPENPOINTMESH_HEADERS_MOC} )

target_link_libraries(	${ONI_TO_PCD_TEST}
						${PCL_LIBRARIES}
						${QT_LIBRARIES}
						${VTK_LIBRARIES}
						${GTEST_LIBRARIES}
						pthread )



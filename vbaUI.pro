#-------------------------------------------------
#
# Project created by QtCreator 2015-10-08T15:45:14
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = vbaUI
TEMPLATE = app


SOURCES += src/main.cpp\
    src/mainwindow.cpp \
    build/include/moc_CloudStitcher.cxx \
    build/include/moc_errorMsgHandler.cxx \
    build/include/moc_mainwindow.cxx \
    build/include/moc_oni-to-pcd.cxx \
    build/include/moc_PCDRegistration.cxx \
    build/CMakeFiles/3.2.2/CompilerIdCXX/CMakeCXXCompilerId.cpp \
    build/CMakeFiles/feature_tests.cxx \
    src/cloud_registration.cpp \
    src/CloudStitcher.cpp \
    src/errorMsgHandler.cpp \
    src/oni-to-pcd.cpp \
    src/PCDRegistration.cpp \
    build/CMakeFiles/3.2.2/CompilerIdC/CMakeCCompilerId.c \
    build/CMakeFiles/feature_tests.c \
    src/MeshConstructor.cpp \
    build/CMakeFiles/3.4.0-rc2/CompilerIdCXX/CMakeCXXCompilerId.cpp \
    build/include/moc_CloudRegistrationFunctions.cxx \
    build/include/moc_filesystemHelper.cxx \
    build/include/moc_MeshConstructor.cxx \
    build/include/moc_PointCloudFilters.cxx \
    build/include/moc_viewer.cxx \
    src/CloudRegistrationFunctions.cpp \
    src/filesystemHelper.cpp \
    src/PointCloudFilters.cpp \
    build/CMakeFiles/3.4.0-rc2/CompilerIdC/CMakeCCompilerId.c

HEADERS  += include/mainwindow.h \
    build/ui_mainwindow.h \
    include/CloudStitcher.h \
    include/errorMsgHandler.h \
    include/oni-to-pcd.h \
    include/PCDRegistration.h \
    include/MeshConstructor.h \
    include/CloudRegistrationFunctions.h \
    include/filesystemHelper.h \
    include/PointCloudFilters.h \
    include/viewer.h \
    include/viewer.hxx \
    src/viewer.tem

FORMS    += src/mainwindow.ui

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += libpcl-all

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_io

INCLUDEPATH += $$PWD/../../../usr/include/pcl-1.7
DEPENDPATH += $$PWD/../../../usr/include/pcl-1.7

RESOURCES += \
    res/resources.qrc

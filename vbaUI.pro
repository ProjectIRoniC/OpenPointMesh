#-------------------------------------------------
#
# Project created by QtCreator 2015-10-08T15:45:14
#
#-------------------------------------------------

QT       += core gui
QT       +=testlib

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
    src/AboutDialog.cpp \
    src/AccuracyControlMenu.cpp \
    src/CloudRegistrationFunctions.cpp \
    src/filesystemHelper.cpp \
    src/PointCloudFilters.cpp \
    src/viewer.cpp \
    build/CMakeFiles/3.4.0-rc2/CompilerIdCXX/CMakeCXXCompilerId.cpp \
    build/CMakeFiles/feature_tests.cxx \
    build/include/moc_AboutDialog.cxx \
    build/include/moc_AccuracyControlMenu.cxx \
    build/include/moc_CloudRegistrationFunctions.cxx \
    build/include/moc_CloudStitcher.cxx \
    build/include/moc_debugger.cxx \
    build/include/moc_errorMsgHandler.cxx \
    build/include/moc_filesystemHelper.cxx \
    build/include/moc_mainwindow.cxx \
    build/include/moc_MeshConstructor.cxx \
    build/include/moc_oni-to-pcd.cxx \
    build/include/moc_PCDRegistration.cxx \
    build/include/moc_PointCloudFilters.cxx \
    build/include/moc_viewer.cxx \
    qtest/build/__/include/moc_AboutDialog.cxx \
    qtest/build/__/include/moc_AccuracyControlMenu.cxx \
    qtest/build/__/include/moc_CloudRegistrationFunctions.cxx \
    qtest/build/__/include/moc_CloudStitcher.cxx \
    qtest/build/__/include/moc_errorMsgHandler.cxx \
    qtest/build/__/include/moc_filesystemHelper.cxx \
    qtest/build/__/include/moc_mainwindow.cxx \
    qtest/build/__/include/moc_MeshConstructor.cxx \
    qtest/build/__/include/moc_oni-to-pcd.cxx \
    qtest/build/__/include/moc_PCDRegistration.cxx \
    qtest/build/__/include/moc_PointCloudFilters.cxx \
    qtest/build/CMakeFiles/3.4.0-rc2/CompilerIdCXX/CMakeCXXCompilerId.cpp \
    qtest/build/CMakeFiles/feature_tests.cxx \
    qtest/build/include/moc_debugger.cxx \
    qtest/build/include/moc_qtestsuite.cxx \
    qtest/build/include/moc_sharedData.cxx \
    qtest/build/include/moc_tst_MainWindow.cxx \
    qtest/src/main.cpp \
    qtest/src/qtestsuite.cpp \
    qtest/src/sharedData.cpp \
    qtest/src/tst_MainWindow.cpp \
    test/build/__/include/moc_AboutDialog.cxx \
    test/build/__/include/moc_AccuracyControlMenu.cxx \
    test/build/__/include/moc_CloudRegistrationFunctions.cxx \
    test/build/__/include/moc_CloudStitcher.cxx \
    test/build/__/include/moc_errorMsgHandler.cxx \
    test/build/__/include/moc_filesystemHelper.cxx \
    test/build/__/include/moc_mainwindow.cxx \
    test/build/__/include/moc_MeshConstructor.cxx \
    test/build/__/include/moc_oni-to-pcd.cxx \
    test/build/__/include/moc_PCDRegistration.cxx \
    test/build/__/include/moc_PointCloudFilters.cxx \
    test/build/__/include/moc_viewer.cxx \
    test/build/CMakeFiles/3.4.0-rc2/CompilerIdCXX/CMakeCXXCompilerId.cpp \
    test/build/CMakeFiles/feature_tests.cxx \
    test/src/main.cpp \
    build/CMakeFiles/3.4.0-rc2/CompilerIdC/CMakeCCompilerId.c \
    build/CMakeFiles/feature_tests.c \
    qtest/build/CMakeFiles/3.4.0-rc2/CompilerIdC/CMakeCCompilerId.c \
    qtest/build/CMakeFiles/feature_tests.c \
    test/build/CMakeFiles/3.4.0-rc2/CompilerIdC/CMakeCCompilerId.c \
    test/build/CMakeFiles/feature_tests.c

HEADERS  += include/mainwindow.h \
    build/ui_mainwindow.h \
    include/CloudStitcher.h \
    include/errorMsgHandler.h \
    include/oni-to-pcd.h \
    include/PCDRegistration.h \
    include/MeshConstructor.h \
    include/AccuracyControlMenu.h \
    include/CloudRegistrationFunctions.h \
    include/filesystemHelper.h \
    include/PointCloudFilters.h \
    include/viewer.h \
    include/viewer.hxx \
    include/debugger.h \
    build/ui_AboutDialog.h \
    build/ui_AccuracyControlMenu.h \
    build/ui_mainwindow.h \
    include/AboutDialog.h \
    qtest/build/ui_AboutDialog.h \
    qtest/build/ui_AccuracyControlMenu.h \
    qtest/build/ui_mainwindow.h \
    qtest/include/debugger.h \
    qtest/include/qtestsuite.h \
    qtest/include/sharedData.h \
    qtest/include/tst_MainWindow.h \
    qtest/ui_AboutDialog.h \
    qtest/ui_AccuracyControlMenu.h \
    qtest/ui_mainwindow.h

FORMS    += src/mainwindow.ui

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += libpcl-all

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lpcl_io

INCLUDEPATH += $$PWD/../../../usr/include/pcl-1.7
DEPENDPATH += $$PWD/../../../usr/include/pcl-1.7

RESOURCES += \
    res/resources.qrc


TEMPLATE = app
TARGET = LUPAE_V2
DEPENDPATH += . src
INCLUDEPATH += /home/wrabe/include
LIBS += -L/home/wrabe/lib -lusb -lftdi


OBJECTS_DIR = obj
MOC_DIR = moc


#Qt used libraries/functionalities
QT = core gui xml

# Input - Header
HEADERS 	+= src/mainwindow.h
HEADERS 	+= src/QExtAction.h
HEADERS 	+= src/panelHexViewer.h
HEADERS 	+= src/panelSetting.h
HEADERS 	+= src/panelLogView.h
HEADERS 	+= src/QAvrFuseDialog.h
HEADERS 	+= src/QFT232DeviceManager.h
HEADERS 	+= src/avrDeviceList.h
HEADERS 	+= src/constants.h
HEADERS 	+= src/types.h

# Input - Sources
SOURCES 	+= src/main.cpp
SOURCES 	+= src/mainwindow.cpp
SOURCES 	+= src/QExtAction.cpp
SOURCES 	+= src/panelHexViewer.cpp
SOURCES 	+= src/panelSetting.cpp
SOURCES 	+= src/panelLogView.cpp
SOURCES 	+= src/QAvrFuseDialog.cpp
SOURCES 	+= src/QFT232DeviceManager.cpp
SOURCES 	+= src/avrDeviceList.cpp

RESOURCES 	+= application.qrc

           

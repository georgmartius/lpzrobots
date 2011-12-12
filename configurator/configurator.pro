TEMPLATE = lib
TARGET = configurator
DEPENDPATH += . ./src/qconfigurable ./include/configurator
INCLUDEPATH += . ./src/qconfigurable ../selforg/include ./include/configurator
LIBS += -lselforg -L../selforg
#CONFIG += debug

QMAKE_CXXFLAGS += -Wno-deprecated -Wno-unused-parameter

CONFIG -= lib_bundle

OBJECTS_DIR = obj
MOC_DIR = moc

#Qt used libraries/functionalities
QT = core gui xml

#ecbrobots
# Input - Header
HEADERS   += src/QExtAction.h
HEADERS   += src/QConfigurator.h
HEADERS   += src/QLogViewWidget.h
HEADERS   += include/configurator/ConfiguratorProxy.h


# Input - Sources
SOURCES   += src/QExtAction.cpp
SOURCES   += src/QConfigurator.cpp
SOURCES   += src/QLogViewWidget.cpp
SOURCES   += src/ConfiguratorProxy.cpp


#-----------------------------------------------------------
#qconfigurable
# Input - Header
HEADERS   +=src/qconfigurable/QConfigurableWidget.h
HEADERS   +=src/qconfigurable/QAbstractConfigurableTileWidget.h
HEADERS   +=src/qconfigurable/QBoolConfigurableTileWidget.h
HEADERS   +=src/qconfigurable/QValConfigurableTileWidget.h
HEADERS   +=src/qconfigurable/QIntConfigurableTileWidget.h
HEADERS   +=src/qconfigurable/QDummyConfigurableTileWidget.h
HEADERS   +=src/qconfigurable/QConfigurableTileShowHideDialog.h
HEADERS   +=src/qconfigurable/QConfigurableSetBoundsDialog.h
HEADERS   +=src/qconfigurable/QConfigurableLoadSaveDialog.h
HEADERS   +=src/qconfigurable/QChangeNumberTileColumnsDialog.h
HEADERS   +=src/qconfigurable/QGridPos.h

# Input - Sources
SOURCES   +=src/qconfigurable/QConfigurableWidget.cpp
SOURCES   +=src/qconfigurable/QAbstractConfigurableTileWidget.cpp
SOURCES   +=src/qconfigurable/QBoolConfigurableTileWidget.cpp
SOURCES   +=src/qconfigurable/QValConfigurableTileWidget.cpp
SOURCES   +=src/qconfigurable/QIntConfigurableTileWidget.cpp
SOURCES   +=src/qconfigurable/QDummyConfigurableTileWidget.cpp
SOURCES   +=src/qconfigurable/QConfigurableTileShowHideDialog.cpp
SOURCES   +=src/qconfigurable/QConfigurableSetBoundsDialog.cpp
SOURCES   +=src/qconfigurable/QConfigurableLoadSaveDialog.cpp
SOURCES   +=src/qconfigurable/QChangeNumberTileColumnsDialog.cpp



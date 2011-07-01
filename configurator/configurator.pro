TEMPLATE = lib
TARGET = libconfigurator
DEPENDPATH += . ./qconfigurable
INCLUDEPATH += . /$(HOME)/include ./qconfigurable ../selforg/include
LIBS += -L$(HOME)/lib -lselforg_dbg -L../selforg
CONFIG += debug 
QMAKE_CXXFLAGS += -Wno-deprecated -Wno-unused-parameter

OBJECTS_DIR = obj
MOC_DIR = moc

#Qt used libraries/functionalities
QT = core gui xml

#ecbrobots
# Input - Header
HEADERS   += QExtAction.h
HEADERS   += QConfigurator.h
HEADERS   += QLogViewWidget.h


# Input - Sources
SOURCES   += QExtAction.cpp
SOURCES   += QConfigurator.cpp
SOURCES   += QLogViewWidget.cpp


#-----------------------------------------------------------
#qconfigurable
# Input - Header
HEADERS   +=qconfigurable/QConfigurableWidget.h
HEADERS   +=qconfigurable/QAbstractConfigurableTileWidget.h
HEADERS   +=qconfigurable/QBoolConfigurableTileWidget.h
HEADERS   +=qconfigurable/QValConfigurableTileWidget.h
HEADERS   +=qconfigurable/QIntConfigurableTileWidget.h
HEADERS   +=qconfigurable/QDummyConfigurableTileWidget.h
HEADERS   +=qconfigurable/QConfigurableTileShowHideDialog.h
HEADERS   +=qconfigurable/QConfigurableSetBoundsDialog.h
HEADERS   +=qconfigurable/QConfigurableLoadSaveDialog.h
HEADERS   +=qconfigurable/QChangeNumberTileColumnsDialog.h
HEADERS   +=qconfigurable/QGridPos.h

# Input - Header
SOURCES   +=qconfigurable/QConfigurableWidget.cpp
SOURCES   +=qconfigurable/QAbstractConfigurableTileWidget.cpp
SOURCES   +=qconfigurable/QBoolConfigurableTileWidget.cpp
SOURCES   +=qconfigurable/QValConfigurableTileWidget.cpp
SOURCES   +=qconfigurable/QIntConfigurableTileWidget.cpp
SOURCES   +=qconfigurable/QDummyConfigurableTileWidget.cpp
SOURCES   +=qconfigurable/QConfigurableTileShowHideDialog.cpp
SOURCES   +=qconfigurable/QConfigurableSetBoundsDialog.cpp
SOURCES   +=qconfigurable/QConfigurableLoadSaveDialog.cpp
SOURCES   +=qconfigurable/QChangeNumberTileColumnsDialog.cpp



#ecbrobots
# Input - Header
HEADERS   += QExtAction.h
HEADERS   += QECBRobotsWindow.h
HEADERS   += QLogViewWidget.h
HEADERS   += ECBCommunicationEvent.h
HEADERS   += ECBCommunicationData.h
HEADERS   += QECBCommunicator.h
HEADERS   += QECBManager.h
HEADERS   += ecb.h
HEADERS   += ecbrobot.h
HEADERS   += ecbagent.h
HEADERS   += QGlobalData.h


# Input - Sources
SOURCES   += QExtAction.cpp
SOURCES   += QECBRobotsWindow.cpp
SOURCES   += QLogViewWidget.cpp
SOURCES   += ECBCommunicationEvent.cpp
SOURCES   += ECBCommunicationData.cpp
SOURCES   += QECBCommunicator.cpp
SOURCES   += QECBManager.cpp
SOURCES   += ecb.cpp
SOURCES   += ecbrobot.cpp
SOURCES   += ecbagent.cpp
SOURCES   += QGlobalData.cpp

#messagedispatcher
# Input - Header
HEADERS   += messagedispatcher/QAbstractMessageDispatchServer.h
HEADERS   += messagedispatcher/QAbstractMessageClient.h
HEADERS   += messagedispatcher/constants.h
HEADERS   += messagedispatcher/types.h
HEADERS   += messagedispatcher/QMessageDispatchWindow.h
HEADERS   += messagedispatcher/QECBMessageDispatchServer.h
HEADERS   += messagedispatcher/QCommunicationChannel.h
HEADERS   += messagedispatcher/QFT232DeviceManager.h
HEADERS   += messagedispatcher/QCCHelper.h
HEADERS   += messagedispatcher/QExtByteArray.h
HEADERS   += messagedispatcher/QExtTimer.h
HEADERS   += messagedispatcher/QLog.h
HEADERS   += messagedispatcher/QMDSHelper.h
HEADERS   += messagedispatcher/QDNSDeviceWidget.h

# Input - Sources
SOURCES   += messagedispatcher/QECBMessageDispatchServer.cpp
SOURCES   += messagedispatcher/QMessageDispatchWindow.cpp
SOURCES   += messagedispatcher/QCommunicationChannel.cpp
SOURCES   += messagedispatcher/QFT232DeviceManager.cpp
SOURCES   += messagedispatcher/QCCHelper.cpp
SOURCES   += messagedispatcher/QExtByteArray.cpp
SOURCES   += messagedispatcher/QExtTimer.cpp
SOURCES   += messagedispatcher/QLog.cpp
SOURCES   += messagedispatcher/QMDSHelper.cpp
SOURCES   += messagedispatcher/QDNSDeviceWidget.cpp

#qconfigurable
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



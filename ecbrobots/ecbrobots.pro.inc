#ecbrobots
# Input - Header
HEADERS   += QExtAction.h
HEADERS 	+= QECBRobotsWindow.h
HEADERS 	+= QLogViewWidget.h
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
SOURCES 	+= QECBRobotsWindow.cpp
SOURCES 	+= QLogViewWidget.cpp
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
HEADERS 	+= messagedispatcher/QAbstractMessageClient.h
HEADERS 	+= messagedispatcher/constants.h
HEADERS 	+= messagedispatcher/types.h
HEADERS 	+= messagedispatcher/QMessageDispatchWindow.h
HEADERS   += messagedispatcher/QECBMessageDispatchServer.h
HEADERS   += messagedispatcher/QCommunicationChannel.h
HEADERS 	+= messagedispatcher/QFT232DeviceManager.h
HEADERS 	+= messagedispatcher/QCCHelper.h
HEADERS 	+= messagedispatcher/QExtByteArray.h
HEADERS 	+= messagedispatcher/QExtTimer.h
HEADERS  += messagedispatcher/QLog.h
HEADERS  += messagedispatcher/QMDSHelper.h

# Input - Sources
SOURCES   += messagedispatcher/QECBMessageDispatchServer.cpp
SOURCES 	+= messagedispatcher/QMessageDispatchWindow.cpp
SOURCES   += messagedispatcher/QCommunicationChannel.cpp
SOURCES 	+= messagedispatcher/QFT232DeviceManager.cpp
SOURCES 	+= messagedispatcher/QCCHelper.cpp
SOURCES 	+= messagedispatcher/QExtByteArray.cpp
SOURCES 	+= messagedispatcher/QExtTimer.cpp
SOURCES   += messagedispatcher/QLog.cpp
SOURCES   += messagedispatcher/QMDSHelper.cpp

#qconfigurable
HEADERS   +=qconfigurable/QConfigurableWidget.h
HEADERS   +=qconfigurable/QAbstractConfigurableLineWidget.h
HEADERS   +=qconfigurable/QBoolConfigurableLineWidget.h
HEADERS   +=qconfigurable/QValConfigurableLineWidget.h
HEADERS   +=qconfigurable/QIntConfigurableLineWidget.h
HEADERS   +=qconfigurable/QConfigurableTileShowHideDialog.h

SOURCES   +=qconfigurable/QConfigurableWidget.cpp
SOURCES   +=qconfigurable/QAbstractConfigurableLineWidget.cpp
SOURCES   +=qconfigurable/QBoolConfigurableLineWidget.cpp
SOURCES   +=qconfigurable/QValConfigurableLineWidget.cpp
SOURCES   +=qconfigurable/QIntConfigurableLineWidget.cpp
SOURCES   +=qconfigurable/QConfigurableTileShowHideDialog.cpp

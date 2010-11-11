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
#HEADERS 	+= messagedispatcher/QLogViewWidget.h


# Input - Sources
SOURCES   += messagedispatcher/QECBMessageDispatchServer.cpp
#SOURCES 	+= messagedispatcher/QECBMessageClient.cpp
SOURCES 	+= messagedispatcher/QMessageDispatchWindow.cpp
SOURCES   += messagedispatcher/QCommunicationChannel.cpp
SOURCES 	+= messagedispatcher/QFT232DeviceManager.cpp
#SOURCES 	+= messagedispatcher/QLogViewWidget.cpp

# Diese Datei wurde mit dem qmake-Manager von KDevelop erstellt. 
# ------------------------------------------- 
# Unterverzeichnis relativ zum Projektverzeichnis: ./src
# Das Target ist eine Anwendung:  ../bin/guilogger

HEADERS += guilogger.h \
           taggedcheckbox.h \
           taggedcombobox.h \
           channelrow.h \
           channelselectrow.h \
           filelogger.h \
           qdatasource.h \
           qserialreader.h \
           qpipereader.h \
           inifile.h \
           commlineparser.h 
SOURCES += guilogger.cpp \
           main.cpp \
           taggedcheckbox.cpp \
           taggedcombobox.cpp \
           channelrow.cpp \
           channelselectrow.cpp \
           filelogger.cpp \
           qserialreader.cpp \
           qpipereader.cpp \
           inifile.cpp 
TEMPLATE = app
CONFIG += debug \
warn_on \
thread \
qt
TARGET = bin/guilogger
#The following line was inserted by qt3to4
QT += qt3support

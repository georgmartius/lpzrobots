# Diese Datei wurde mit dem qmake-Manager von KDevelop erstellt. 
# ------------------------------------------- 
# Unterverzeichnis relativ zum Projektverzeichnis: ./src
# Das Target ist eine Anwendung:  ../bin/guilogger

HEADERS += guilogger.h \
           taggedcheckbox.h \
           channelrow.h \
           filelogger.h \
           qdatasource.h \
           qserialreader.h \
           qpipereader.h \
           inifile.h 
SOURCES += guilogger.cpp \
           main.cpp \
           taggedcheckbox.cpp \
           channelrow.cpp \
           filelogger.cpp \
           qserialreader.cpp \
           qpipereader.cpp \
           inifile.cpp 
TEMPLATE = app
CONFIG += debug \
warn_on \
thread \
qt
TARGET = ../bin/guilogger

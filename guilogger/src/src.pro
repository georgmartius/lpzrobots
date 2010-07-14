# Diese Datei wurde mit dem qmake-Manager von KDevelop erstellt. 
# ------------------------------------------- 
# Unterverzeichnis relativ zum Projektverzeichnis: ./src
# Das Target ist eine Anwendung:  ../bin/guilogger

HEADERS += \ 
           guilogger.h \
           gnuplot.h \
           filelogger.h \
           qdatasource.h \
           qserialreader.h \
           qpipereader.h \
           inifile.h \
           commlineparser.h \
           stl_adds.h \
           plotchannelstablemodel.h \
           plotinfo.h \
           channeldata.h \
    quickmp.h

SOURCES += \
           guilogger.cpp \
           main.cpp \
           gnuplot.cpp \
           filelogger.cpp \
           qserialreader.cpp \
           qpipereader.cpp \
           inifile.cpp \
           stl_adds.cpp \
           plotchannelstablemodel.cpp \
           plotinfo.cpp \
           channeldata.cpp
           

TEMPLATE = app
CONFIG += debug \
warn_on \
thread \
qt \
console
TARGET = bin/guilogger
target.path = /usr/bin
QT += qt3support
INSTALLS += target

SUBDIRS += src
TEMPLATE = subdirs 
QT += qt3support
CONFIG += warn_on \
          qt \
          thread \
          console 

#CONFIG += debug

CONFIG -= app_bundle

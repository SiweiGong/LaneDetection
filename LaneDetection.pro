TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    lanedetector.cpp
LIBS+=-L/usr/local/lib -lopencv_world

HEADERS += \
    lanedetector.h

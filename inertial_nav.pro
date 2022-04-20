QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    ./src/ECEF.cpp \
    ./src/main.cpp \
    ./src/mainwindow.cpp \
    ./src/navigation_info.cpp \
    ./src/update_trigger.cpp

HEADERS += \
    ./src/ECEF.h \
    ./src/mainwindow.h \
    ./src/navigation_info.h \
    ./src/update_trigger.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

unix:!macx: LIBS += -L$$PWD/lib/ -llsm9ds1

INCLUDEPATH += $$PWD/inc
DEPENDPATH += $$PWD/inc

RESOURCES += \
    images/img.qrc

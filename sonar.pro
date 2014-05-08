#-------------------------------------------------
#
# Project created by QtCreator 2011-07-01T09:24:59
#
#-------------------------------------------------

QT       += core gui

TARGET = sonar
TEMPLATE = app

SOURCES += main.cpp\
 mainwindow.cpp \
 sivia.cpp \
    repere.cpp

HEADERS += mainwindow.h \
 sivia.h \
    repere.h

FORMS += mainwindow.ui

IBEXHOME = /home/quentin/Documents/Ibex/IBEX_INSTALL_2_1/INSTALL

INCLUDEPATH += $$IBEXHOME/IBEX/include/ibex $$IBEXHOME/IBEX/include $$IBEXHOME/soplex-1.7.2/src -frounding-math -msse2 -mfpmath=sse

LIBS += -L$$IBEXHOME/IBEX/lib -L$$IBEXHOME/soplex-1.7.2/lib -libex -lsoplex -lprim

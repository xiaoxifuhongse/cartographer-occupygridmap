QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
INCLUDEPATH +=     /usr/include/boost \
					/usr/local/include/glog \
					/usr/include/eigen3 \
		/usr/local/ceres

INCLUDEPATH += /usr/local/include/opencv \
				/usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_core.so \
       /usr/local/lib/libopencv_highgui.so \
		/usr/local/lib/libopencv_imgproc.so

LIBS+=	/usr/lib/x86_64-linux-gnu/libboost_system.so  \
		/usr/lib/x86_64-linux-gnu/libboost_thread.so   \
		/usr/lib/x86_64-linux-gnu/libboost_timer.so   \
		/usr/local/lib/libglog.so \
		/usr/local/lib/libceres.a
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp \
    probability_values.cc \
    probability_grid.cc \
    grid_2d.cc

HEADERS += \
    probability_values.h \
    math1.h \
    port.h \
    make_unique.h \
    grid_2d.h \
    map_limits.h \
    xy_index.h \
    probability_grid.h

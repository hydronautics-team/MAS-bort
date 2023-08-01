QT += core network
QT += serialport

CONFIG += c++17 console
CONFIG -= app_bundle

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        Parser-BW-AH127C/AH127Cprotocol.cpp \
        cs_rov.cpp \
        kx_pult/kx_protocol.cpp \
        kx_pult/qkx_coeffs.cpp \
        kx_pult/qpiconfig.cpp \
        main.cpp \
        rov_model.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    Parser-BW-AH127C/AH127Cprotocol.h \
    cs_rov.h \
    kx_pult/kx_protocol.h \
    kx_pult/qkx_coeffs.h \
    kx_pult/qpiconfig.h \
    rov_model.h

cmake_minimum_required(VERSION 3.16)
project(Bort_MAS)

find_program(QT_MOC_EXECUTABLE moc REQUIRED)
add_executable(Qt5::moc IMPORTED)
set_property(TARGET Qt5::moc PROPERTY IMPORTED_LOCATION ${QT_MOC_EXECUTABLE})

find_package(Qt5 COMPONENTS Core SerialPort Network REQUIRED)

set(CMAKE_AUTOMOC ON)

add_executable(${PROJECT_NAME}
    main.cpp
    NIR/VMA_controller/vma_controller.cpp NIR/VMA_controller/vma_controller.h
    Parser-BW-AH127C/AH127Cprotocol.cpp Parser-BW-AH127C/AH127Cprotocol.h
    parserUWB/calibration.cpp parserUWB/protocoluwb.cpp parserUWB/trilatUWB.cpp
    cs_rov.cpp cs_rov.h
    kx_pult/configdata.cpp kx_pult/configdata.h
    kx_pult/kx_protocol.cpp kx_pult/kx_protocol.h
    kx_pult/qkx_coeffs.cpp kx_pult/qkx_coeffs.h
    kx_pult/qpiconfig.cpp kx_pult/qpiconfig.h
    protocol_bort_AUV/pc_protocol.h
    protocol_bort_AUV/protocol.h
    protocol_bort_AUV/udp_protocol.h
    rov_model.cpp rov_model.h
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt::Core
    Qt::Network
    Qt::SerialPort
    wiringPi
)

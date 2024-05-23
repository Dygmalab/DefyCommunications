add_library(Communications INTERFACE)

target_include_directories(Communications
        INTERFACE
        src)

target_sources(Communications
        INTERFACE
        src/device/keyScanner/CommunicationsKS.cpp
        src/Communications_protocol_rf.cpp
        )

target_link_libraries(Communications
        INTERFACE
        LEDTypes
        KeyScanner
        LEDManagement
        BatteryManagement
        SPI
        )

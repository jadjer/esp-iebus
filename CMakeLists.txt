cmake_minimum_required(VERSION 3.30.5)

set(INCLUDES
        include
)

set(SOURCES
        src/common.cpp

        src/Timer.cpp
        src/DriverRMT.cpp
        src/Processor.cpp
        src/Controller.cpp
        src/DriverGPIO.cpp
)

set(REQUIRES
        esp_driver_rmt
        esp_driver_gptimer
)

set(PRIV_REQUIRES
        esp_timer
        esp_driver_gpio
)

idf_component_register(INCLUDE_DIRS ${INCLUDES} SRCS ${SOURCES} REQUIRES ${REQUIRES} PRIV_REQUIRES ${PRIV_REQUIRES})

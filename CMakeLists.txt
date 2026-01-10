cmake_minimum_required(VERSION 3.30.5)

set(INCLUDES
        include
)

set(SOURCES
        src/common.cpp

        src/Driver.cpp
        src/Message.cpp
        src/Controller.cpp
)

set(REQUIRES)

set(PRIV_REQUIRES
        esp_timer
        esp_driver_gpio
)

idf_component_register(INCLUDE_DIRS ${INCLUDES} SRCS ${SOURCES} REQUIRES ${REQUIRES} PRIV_REQUIRES ${PRIV_REQUIRES})

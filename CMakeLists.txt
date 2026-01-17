cmake_minimum_required(VERSION 3.30.5)

set(INCLUDES
        include
)

set(SOURCES
        src/Timer.cpp
        src/Driver.cpp
        src/Message.cpp
        src/Processor.cpp
        src/Controller.cpp
)

set(REQUIRES
        esp_driver_gptimer
)

set(PRIV_REQUIRES
        esp_driver_gpio
)

idf_component_register(INCLUDE_DIRS ${INCLUDES} SRCS ${SOURCES} REQUIRES ${REQUIRES} PRIV_REQUIRES ${PRIV_REQUIRES})

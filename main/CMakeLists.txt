cmake_minimum_required(VERSION 3.5)

set(SOURCES
        ./main.cpp
        ./Bmx055Driver.cpp
        ./EspGPIO.cpp
        ./EspI2CMaster.cpp
        ./EspOledDriver.cpp 
        ./KalmanFilter.cpp
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS .
    REQUIRES
        # Add all ESP related components here
        esp_common

        #driver
        driver

        # FREERTOS
        freertos

        # eigen
        eigen

        # User libraries here
        
)

cmake_minimum_required(VERSION 3.5)

set(SOURCES
        ./main.cpp
        ./Bmx055Driver.cpp
        ./EspGPIO.cpp
        ./EspI2CMaster.cpp
        ./KalmanFilter.cpp
        ./EEPROMHandler.cpp
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

        # Flash storage
        nvs_flash

        esp_wifi

        esp_http_server

)

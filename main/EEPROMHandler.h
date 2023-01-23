#ifndef EEPROMHANDLER_H
#define EEPROMHANDLER_H
#include <esp_sleep.h>
#include <nvs_flash.h>
#include <nvs.h>
#define NVS_NAMESPACE "settings"
#define ARRAY_LENGTH 6

class EEPROMHandler {
    private:
        double settings[ARRAY_LENGTH]; // Settings Regression constants [0-2], gyro bias [3-5]
        nvs_handle handle;
    public:
        EEPROMHandler();
        ~EEPROMHandler();
        void write(double* array, size_t array_size);
        void read();
        double* getSettings();
};

#endif

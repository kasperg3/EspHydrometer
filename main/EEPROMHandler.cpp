#include "EEPROMHandler.h"


EEPROMHandler::EEPROMHandler() {
    esp_err_t err = nvs_flash_init();
    ESP_ERROR_CHECK(err);
}

EEPROMHandler::~EEPROMHandler() {
    nvs_close(handle);
}

void EEPROMHandler::write(double* array, size_t array_size) {
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    ESP_ERROR_CHECK(err);
    err = nvs_set_blob(handle, NVS_NAMESPACE, array, sizeof(double) * array_size);
    ESP_ERROR_CHECK(err);
    err = nvs_commit(handle);
    ESP_ERROR_CHECK(err);
}

void EEPROMHandler::read() {
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    ESP_ERROR_CHECK(err);
    size_t required_size;
    // err = nvs_get_blob(handle, NVS_NAMESPACE, NULL, &required_size);
    // ESP_ERROR_CHECK(err);
    err = nvs_get_blob(handle, NVS_NAMESPACE, settings, &required_size);
    ESP_ERROR_CHECK(err);
}

double* EEPROMHandler::getSettings() {
    return settings;
}

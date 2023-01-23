#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "EspGPIO.h"
#include "Bmx055Driver.h"
#include "EspI2CMaster.h"
#include "eigen3/Eigen/Dense"
#include "KalmanFilter.h"

static const float PI = 3.14159265359;
static const char *TAG = "MAIN";
using namespace Eigen;

void gpioTest(void *parameter)
{
    gpio_num_t gpioPin = GPIO_NUM_33;
    ESP_LOGI(TAG, "STATIN GPIO TEST: SETTING GPIO %i ", gpioPin);
    EspGPIO io = EspGPIO(gpioPin, GPIO_MODE_OUTPUT);
    // gpio_set_direction(gpioPin, GPIO_MODE_OUTPUT);

    int cnt = 0;
    while (1)
    {
        // ESP_LOGI(TAG, "cnt: %i", cnt++);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (cnt % 2)
        {
            // ESP_LOGI(TAG, "Pin %i OFF", gpioPin);
            io.clear();
        }
        else
        {
            // ESP_LOGI(TAG, "Pin %i ON", gpioPin);
            io.set();
        }
        cnt++;
    }
}

void imuTest()
{
    ESP_LOGI(TAG, "Starting i2c master...");
    EspI2CMaster i2cMaster = EspI2CMaster();
    ESP_LOGI(TAG, "Starting IMU Driver...");
    BMX055Driver imu = BMX055Driver(&i2cMaster);
    double accData[3] = {0};
    double gyrData[3] = {0};

    // State vector
    VectorXd x(6);
    x << 0, 0, 0, 0, 0, 0;

    // State transition matrix
    MatrixXd A(6, 6);
    A << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    // Measurement matrix
    MatrixXd H(3, 6);
    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0;

    // Process noise covariance matrix
    MatrixXd Q(6, 6);
    Q << 0.1, 0, 0, 0, 0, 0,
         0, 0.1, 0, 0, 0, 0,
         0, 0, 0.1, 0, 0, 0,
         0, 0, 0, 0.1, 0, 0,
         0, 0, 0, 0, 0.1, 0,
         0, 0, 0, 0, 0, 0.1;

    // Measurement noise covariance matrix
    MatrixXd R(3, 3);
    R << 0.1, 0, 0,
         0, 0.1, 0,
         0, 0, 0.1;

    // Initialize Kalman filter
    KalmanFilter kf(x, A, H, Q, R);


    while (true)
    {
        imu.getAccAngle(accData);
        imu.getGyroRelativeAngle(gyrData);

        // Update the state estimate
        kf.predict();
        kf.update(Vector3d(gyrData),Vector3d(accData));
        Eigen::Vector3d state = kf.state();
        
        // Print estimated roll, pitch, and yaw angles
        ESP_LOGI(TAG, "Estimated angles: x: %f , y: %f , z: %f", state[0] * 180 / PI, state[1] * 180 / PI, state[2] * 180 / PI);

        ESP_LOGI(TAG, "GYROMETER: x: %f , y: %f , z: %f  ", gyrData[0] * 180 / PI, gyrData[1] * 180 / PI, gyrData[2] * 180 / PI);
        ESP_LOGI(TAG, "ACCELEROMETER: x: %f , y: %f , z: %f  ", accData[0] * 180 / PI, accData[1] * 180 / PI, accData[2] * 180 / PI);
        vTaskDelay(10);
    }

    // TODO
    // Kalman filter
    // Calculate tilt wrt. gravity/horizontal
    // Create a polynominal regression from tilt to specific gravity
}

// # Work in progress
// * Github build check
// * Filters(kalman) for IMU driver
// * data logging/plotting remote
// * iot

extern "C" void app_main(void)
{
    imuTest();
}

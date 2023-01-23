#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "EspGPIO.h"
#include "Bmx055Driver.h"
#include "EspI2CMaster.h"
#include "eigen3/Eigen/Dense"
#include "KalmanFilter.h"
#include "EEPROMHandler.cpp"

static const char *TAG = "MAIN";
#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)

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

    imu.getAccAngle(accData);


    // State vector
    VectorXd x(6);
    x << accData[0], accData[1], accData[2], 0, 0, 0;
    // Measurement vector
    VectorXd y(6); 

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
        y << accData[0], accData[1],accData[2], gyrData[0],gyrData[1],gyrData[2];
        // Update the state estimate
        kf.predict();
        kf.update(y);
        Eigen::Vector3d state = kf.state();
        
        // Print estimated roll, pitch, and yaw angles
        ESP_LOGI(TAG, "Estimated angles: x: %f , y: %f , z: %f",  RAD_TO_DEG(state[0]), RAD_TO_DEG(state[1]),  RAD_TO_DEG(state[2]));

        ESP_LOGI(TAG, "GYROMETER: x: %f , y: %f , z: %f  ",  RAD_TO_DEG(gyrData[0]),  RAD_TO_DEG(gyrData[1]),  RAD_TO_DEG(gyrData[2]));
        ESP_LOGI(TAG, "ACCELEROMETER: x: %f , y: %f , z: %f  ",  RAD_TO_DEG(accData[0]),  RAD_TO_DEG(accData[1]),  RAD_TO_DEG(accData[2]));
        vTaskDelay(10);
    }

    // TODO
    // Use the IMU data to determine the angle of tilt
    //      IMU Bias correction should be a static value, do a measurement and set it as a constant
    //      IMU calibration: place on level surface and correct the error?
    // Store the datapoints using 
    // Create a polynominal regression from tilt to specific gravity
    // Create a matrix to hold the data points
    // MatrixXd data(N, 2);

    // // Fill the matrix with the x and y values
    // for (int i = 0; i < N; i++)
    // {
    //     data(i, 0) https://en.wikipedia.org/wiki/Numerical_methods_for_linear_least_squares= x[i];
    //     data(i, 1) = y[i];
    // }

    // // Perform polynomial regression
    // MatrixXd X(N, degree+1);
    // for (int i = 0; i < N; i++) {
    //     for (int j = 0; j < degree+1; j++)
    //         X(i,j) = pow(data(i,0),j);
    // }
    // MatrixXd X_T = X.transpose();
    // MatrixXd B = (X_T * X).inverse() * X_T * data.col(1);
    // cout << "The polynomial coefficients are:" << endl << B << endl;
    // return 0;
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

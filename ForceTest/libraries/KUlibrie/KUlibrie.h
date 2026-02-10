#ifndef KULIBRIE_H
#define KULIBRIE_H

// Import necessary packages
#include <bluefruit.h>              // For bluetooth communication
#include <LSM6DS3.h>                // To read IMU sensor information
#include <BasicLinearAlgebra.h>     // For matrix operations
#include <extendedKalmanFilter.h>   // Includes calculations for the extended Kalman filter
#include <Adafruit_LittleFS.h>      // To save calibration information
#include <InternalFileSystem.h>     // To save calibration information

// Define size of vectors to be send or received via bluetooth (number of bytes)
// Telemetry data 
//      = 12 bytes for accelerometer measurements (4 bytes in each direction)
//        + 12 bytes for filtered accelerometer measurements
//        + 12 bytes for gyroscope measurements
//        + 12 bytes for filtered gyroscope measurements
//        + 12 bytes for the attitude estimation
//        + 4 bytes containing the number of milliseconds passed since the last boot
#define DATANUM_TELEMETRY 64
// Control action data
//      = 4 bytes for thrust
//        + 4 bytes for V0
//        + 4 bytes for dV
//        + 4 bytes for ds
//        + 4 bytes containing the number of milliseconds passed since the last boot
#define DATANUM_CONTROL 20 //((VI0) 4 + (V0) 4 + (dV) 4 + (ds) 4 + (time) 4)
// Actions to control the drone (send by the computer)
#define DATANUM_ACTION 1
// References for the orientation (send by the computer)
#define DATANUM_REFERENCE 2

// Define refreshrates
#define PERIOD_APP 0.27             // The Matlab app refreshes every 0.27s
#define PERIOD_CONTROLLER 0.005     // The microcontroller onboard the KUlibrie refreshes about every 0.005s

// Define how many timesteps are send to the central computer in one vector
#define NB_POINTS_PER_SEND 3

// Define how many measuremenets are used for the calibration of the IMU measurements
#define NB_MEASUREMENTS_CALIBRATIONS 500

// Define the name of the file containing the calibration information
#define FILENAME "/calibration.bin"

// Create union type variables to facilitate data conversion (float to bytes)
// For the telemetry data
union unionDataTelemetry {
    uint8_t bytes[NB_POINTS_PER_SEND * DATANUM_TELEMETRY];
    float value[NB_POINTS_PER_SEND * DATANUM_TELEMETRY / 4];
};

// For the controller data
union unionDataControl {
    uint8_t bytes[NB_POINTS_PER_SEND * DATANUM_CONTROL];
    float value[NB_POINTS_PER_SEND * DATANUM_CONTROL / 4];
};

// For the calibration data
union unionDataCalibration {
    uint8_t bytes[24];
    float value[6];
};

// For the reference orientation (uint8 to uint 16)
union unionDataReference {
    uint8_t bytes[DATANUM_REFERENCE];
    uint16_t value[DATANUM_REFERENCE / 2];
};

using namespace Adafruit_LittleFS_Namespace;

class KUlibrie {
    public:
        // Contructor
        KUlibrie(float *roll, float *pitch, float *yaw_rate, float *VI0, float *V0, float *dV, float *ds, float *ref_roll, float *ref_pitch, float *ref_yaw_rate);

        // Public functions
        void setup_hardware();

        void update_filter(float dt);

        void curve_fit(Matrix<3> x, Matrix<3> y, float axis);
        void set_gyro_biasses(float bias_gx, float bias_gy, float bias_gz);
        void set_calibration();

        float readFloatAccelX();
        float readFloatAccelY();
        float readFloatAccelZ();

        float readFloatGyroX();
        float readFloatGyroY();
        float readFloatGyroZ();
        
        void send_telemetry(unsigned long t);
        void send_control(unsigned long t);

        // Not used anymore
        // {
        void set_ax_filter(float ax);
        void set_ay_filter(float ay);
        void set_az_filter(float az);

        void set_roll(float roll);
        void set_pitch(float pitch);
        // }

        // Public variables
        bool control = false;               // Indicates if the controller is active (KUlibrie is then in flight)
        bool calibrate = false;             // Indicates if the drone is currently executing the calibration procedure

        unsigned long start_control;        // Stores the number of milliseconds since the last oot when the controller started

        bool send_telemetry_data = false;   // Indicates if the KUlibrie should be sending telemetry data to the central computer
        bool send_control_data = false;     // Indicates if the KUlibrie should be sending controller data to the central computer
        
        // Pin layout
        const int SLP_PIN1 = 5;
        const int SLP_PIN2 = 6;
        const int  DIREC_A = 3;
        const int PWM_A = 0;
        const int DIREC_B = 1;
        const int PWM_B = 2;

        const int TRIGGERPIN = 8;
        const int TRIGGER = 0;

        
        
    private:
        // Private functions
        void action(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len);
        static void staticAction(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len);

        void reference(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len);
        static void staticReference(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len);

        void write_calibration();
        
        // Private variables
        static KUlibrie* instance;                                  // Stores an instance of the KUlibrie class

        LSM6DS3 imu;                                                // Stores access to the IMU sensor

        // Bluetooth services and characteristics
        BLEService telemetryService;                                // Bluetooth service for sending telemetry data
        BLECharacteristic telemetryChar;                            // Bluetooth characteristic for sending telemetry data

        BLEService controlService;                                  // Bluetooth service for sending controller data
        BLECharacteristic controlChar;                              // Bluetooth characteristic for sending controller data

        BLEService actionService;                                   // Bluetooth service for receiving commands from the central computer
        BLECharacteristic actionChar;                               // Bluetooth characteristic for receiving commands from the central computer

        BLEService referenceService;                                // Bluetooth service for receiving the reference orientation
        BLECharacteristic referenceChar;                            // Bluetooth characteristic for receiving the reference orientation

        // Dataconversion
        union unionDataTelemetry telemetryData;                     // Conversion of telemetry data from float to bytes (to send to the central computer)
        union unionDataControl controlData;                         // Conversion of controller data from float to bytes (to send to the central computer)
        union unionDataCalibration calibrationData;                 // Conversion of the calibration settings from float to bytes (to send to the central computer)
        union unionDataReference referenceData;                     // Conversion of the reference orientation from bytes to uint16 (to interpret from received data)

        Matrix<2,2> Q = {1e-5, 0, 0, 1e-5};                         // Q-matrix for Kalman filter
        Matrix<3,3> R = {1e-2, 0, 0, 0, 2e-3, 0, 0, 0, 1e-3};       // R-matrix for Kalman filter
        
        float f_acc = 1;                                            // Cut-off frequency for accelerometer measurements
        float f_gyr = 40;                                           // Cut-off frequency for gyroscope measurements

        ExtendedKalman filter;                                      // Stores an instance of the Kalman filter

        // calibration could be a 3x2 matrix instead of 5x2, this size results from a failed attempt
        Matrix<5,2> calibration = {1, 1, 1, 1, 1, 0, 0, 0, 0, 0};   // Stores the calibration settings (slope x, slope y, slope z, ~, ~, bias x, bias y, bias z, ~, ~)
        Matrix<3> gyro_biasses;                                     // Stores the gyroscope biasses in three directions

        float ax, ay, az;                                           // Stores the accelerometer measurements
        float gx, gy, gz;                                           // Stores the gyroscope measurements

        float ax_filt, ay_filt, az_filt;                            // Stores the filtered accelerometer measurements
        float gx_filt, gy_filt, gz_filt;                            // Stores the filtered gyroscope measurements

        float *_roll, *_pitch, *_yaw_rate;                          // Stores (references to) the attitude estimation

        float *_ref_roll, *_ref_pitch, *_ref_yaw_rate;              // Stores (references to) the reference orientation

        float *_VI0, *_V0, *_dV, *_ds;                              // Stores (references to) the control actions

        int count_timesteps_telemetry = 0;                          // Counts how many timesteps have passed
        int count_period_telemetry = 0;                             // Counts how many timesteps are currently stored in the vector that will be send to the central computer

        int count_timesteps_control = 0;                            // Counts how many timesteps have passed
        int count_period_control = 0;                               // Counts how many timesteps are currently stored in the vector that will be send to the central computer

        float d2r = asin(1)/90;                                     // Conversion from degrees to radians
        float r2d = 90/asin(1);                                     // Conversion from radians to degrees

        File file;                                                  // Stores a file instance to write the calibration settings
};

#endif
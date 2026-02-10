#ifndef KULIBRIE_H
#define KULIBRIE_H

// Import necessary packages
#include <bluefruit.h>              // For bluetooth communication
#include <LSM6DS3.h>                // To read IMU sensor information
#include <BasicLinearAlgebra.h>     // For matrix operations
#include <LongitudinalEKF.h>        // REPLACED extendedKalmanFilter.h with your new EKF
#include <Adafruit_LittleFS.h>      // To save calibration information
#include <InternalFileSystem.h>     // To save calibration information

// Define size of vectors to be send or received via bluetooth (number of bytes)
// Telemetry data 
#define DATANUM_TELEMETRY 64
// Control action data
#define DATANUM_CONTROL 20 
// Actions to control the drone (send by the computer)
#define DATANUM_ACTION 1
// References for the orientation (send by the computer)
#define DATANUM_REFERENCE 2

// Define refreshrates
#define PERIOD_APP 0.27             
#define PERIOD_CONTROLLER 0.005     

// Define how many timesteps are send to the central computer in one vector
#define NB_POINTS_PER_SEND 3

// Define how many measuremenets are used for the calibration of the IMU measurements
#define NB_MEASUREMENTS_CALIBRATIONS 500

// Define the name of the file containing the calibration information
#define FILENAME "/calibration.bin"

// Create union type variables to facilitate data conversion (float to bytes)
union unionDataTelemetry {
    uint8_t bytes[NB_POINTS_PER_SEND * DATANUM_TELEMETRY];
    float value[NB_POINTS_PER_SEND * DATANUM_TELEMETRY / 4];
};

union unionDataControl {
    uint8_t bytes[NB_POINTS_PER_SEND * DATANUM_CONTROL];
    float value[NB_POINTS_PER_SEND * DATANUM_CONTROL / 4];
};

union unionDataCalibration {
    uint8_t bytes[24];
    float value[6];
};

union unionDataReference {
    uint8_t bytes[DATANUM_REFERENCE];
    uint16_t value[DATANUM_REFERENCE / 2];
};

using namespace Adafruit_LittleFS_Namespace;
using namespace BLA; // Added this to ensure Matrix<> types are found

class KUlibrie {
    public:
        // Contructor
        // Outputs: u, w, q, theta
        // Controls: VI0 (Voltage), servo_angle (Deg)
        // Reference: ref_pitch (from BLE)
        KUlibrie(float *u, float *w, float *q, float *theta, 
                 float *VI0, float *servo_angle, 
                 float *ref_pitch);

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

        // Legacy / Unused setters (kept as requested)
        void set_ax_filter(float ax);
        void set_ay_filter(float ay);
        void set_az_filter(float az);
        void set_roll(float roll);
        void set_pitch(float pitch);

        // Public variables
        bool control = false;               
        bool calibrate = false;             

        unsigned long start_control;        

        bool send_telemetry_data = false;   
        bool send_control_data = false;     
        
        // Pin layout
        const int SLP_PIN1 = 5;
        const int SLP_PIN2 = 6;
        const int DIREC_A = 3;
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
        static KUlibrie* instance;                                  

        LSM6DS3 imu;                                                

        // Bluetooth services and characteristics
        BLEService telemetryService;                                
        BLECharacteristic telemetryChar;                            

        BLEService controlService;                                  
        BLECharacteristic controlChar;                              

        BLEService actionService;                                   
        BLECharacteristic actionChar;                               

        BLEService referenceService;                                
        BLECharacteristic referenceChar;                            

        // Dataconversion
        union unionDataTelemetry telemetryData;                     
        union unionDataControl controlData;                         
        union unionDataCalibration calibrationData;                 
        union unionDataReference referenceData;                     

        float f_acc = 1;                                            
        float f_gyr = 40;                                           

        // --- FILTER OBJECT ---
        LongitudinalEKF ekf;
        
        // Matrices for the EKF
        Matrix<4,4> Q = {1e-5, 0, 0, 0, 
                         0, 1e-5, 0, 0, 
                         0, 0, 1e-5, 0, 
                         0, 0, 0, 1e-5}; 
                              
        Matrix<3,3> R = {1e-2, 0, 0, 
                         0, 1e-2, 0, 
                         0, 0, 1e-3};

        // Calibration Settings
        Matrix<5,2> calibration = {1, 1, 1, 1, 1, 0, 0, 0, 0, 0};   
        Matrix<3> gyro_biasses;                                     

        // --- STATE POINTERS (New Longitudinal) ---
        float *_u, *_w, *_q, *_theta;

        // --- CONTROL INPUT POINTERS ---
        float *_VI0;          // Voltage Input
        float *_servo_angle;  // Tail Servo Angle (Degrees)
        float *_ref_pitch;    // Reference Pitch

        // --- SENSOR DATA ---
        float ax, ay, az;     // Raw Accel                                      
        float gx, gy, gz;     // Raw Gyro                                      

        // Filtered holders (used for internal storage/legacy)
        float ax_filt, ay_filt, az_filt;                            
        float gx_filt, gy_filt, gz_filt;                            

        // --- LEGACY POINTERS (Kept to minimize changes, initialized to nullptr) ---
        float *_roll, *_pitch_legacy, *_yaw_rate;                          
        float *_ref_roll, *_ref_yaw_rate;              

        int count_timesteps_telemetry = 0;                          
        int count_period_telemetry = 0;                             

        int count_timesteps_control = 0;                            
        int count_period_control = 0;                               

        float d2r = asin(1)/90;                                     
        float r2d = 90/asin(1);                                     

        File file;                                                  
};

#endif
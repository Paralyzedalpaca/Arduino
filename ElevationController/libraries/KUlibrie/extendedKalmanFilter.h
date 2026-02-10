#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

// Import necessary packages
#include <cmath>                // For goniometric calculations
#include <filters.h>            // For low-pass filters
#include <BasicLinearAlgebra.h> // For matrix operations

using namespace BLA;

class ExtendedKalman {
    public:
        // Constructors
        ExtendedKalman();
        ExtendedKalman(Matrix<2,2> Q, Matrix<3,3> R, 
                        float f_acc, float f_gyr, 
                        float *ax_filt, float *ay_filt, float *az_filt, 
                        float *gx_filt, float *gy_filt, float *gz_filt, 
                        float *roll, float *pitch, float *yaw_rate);

        // Public functions
        void set_calibration(Matrix<5,2> calibration, Matrix<3> gyro_biasses);

        void predict(float gx, float gy, float gz, float dt, bool calibrating);
        void update(float ax, float ay, float az, bool calibrating);
        
        // Getters
        float get_roll();
        float get_pitch();

         Matrix<3> get_accel();
        Matrix<3> get_accel_filt();
        Matrix<3> get_gyro();
        Matrix<3> get_gyro_filt();
        Matrix<3> get_attitude();
        
        // Not used anymore
        // {
        void reset_ax();
        void reset_ay();
        void reset_az();

        void reset_ax(float ax);
        void reset_ay(float ay);
        void reset_az(float az);
        // }

       
    private:
        // Private variables
        float *_ax_filt, *_ay_filt, *_az_filt;          // (Reference to) filtered accelerometer data
        float *_gx_filt, *_gy_filt, *_gz_filt;          // (Reference to) filtered gyroscope data

        Filter ax_filter;                               // Instance of low-pass filter for accelerometer (in x-direction)
        Filter ay_filter;                               // Instance of low-pass filter for accelerometer (in y-direction)
        Filter az_filter;                               // Instance of low-pass filter for accelerometer (in z-direction)

        Filter gx_filter;                               // Instance of low-pass filter for gyroscope (in x-direction)
        Filter gy_filter;                               // Instance of low-pass filter for gyroscope (in y-direction)
        Filter gz_filter;                               // Instance of low-pass filter for gyroscope (in z-direction)

        float *_roll = 0;                               // (Reference to) roll estimation
        float *_pitch = 0;                              // (Reference to) pitch estimation
        float *_yaw_rate = 0;                           // (Reference to) yaw rate estimation

        Matrix<2,2> Q;                                  // Q matrix for the Kalman filter
        Matrix<3,3> R;                                  // R matrix for the Kalman filter

        Matrix<2,2> P;                                  // Error covariance matrix
        Matrix<2,2> A;                                  // Jacobian gyroscope
        Matrix<3,2> C;                                  // Jacobian accelerometer
        
        Matrix<2,3> K;                                  // Kalman gain

        Matrix<2,2> I = {1, 0, 0, 1};                   // Identity matrix

        float d2r = asin(1)/90;                         // Conversion from degrees to radians
        float r2d = 90/asin(1);                         // Conversion from radians to degrees

        Matrix<5,2> _calibration = {1, 0, 1, 0, 1, 0};  // Calibration settings {slope ax, bias ax, slope ay, bias ay, slope az, bias az} (Should actually be a 3x2 matrix)
        Matrix<3> _gyro_biasses = {0, 0, 0};            // Gyroscope biasses {bias gx, bias gy, bias gz}
};

#endif
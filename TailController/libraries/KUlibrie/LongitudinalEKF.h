#ifndef LONGITUDINAL_EKF_H
#define LONGITUDINAL_EKF_H

#include <cmath>
#include <Filters.h>            
#include <BasicLinearAlgebra.h> 

using namespace BLA;
// test 123
class LongitudinalEKF {
    public:
        // Empty Constructor
        LongitudinalEKF();

        // Full Constructor
        // Q, R: Noise matrices
        // f_acc, f_gyr: Filter cutoff frequencies
        // u, w, q, theta: Pointers to STATE variables (outputs)
        // ax_filt, az_filt, q_filt: Pointers to FILTERED MEASUREMENT variables (outputs)
        // ax_bias, az_bias, q_bias: CALIBRATION offsets (inputs)
        LongitudinalEKF(Matrix<4,4> Q, Matrix<3,3> R, 
                        float f_acc, float f_gyr, 
                        float *u, float *w, float *q, float *theta,
                        float *ax_filt, float *az_filt, float *q_filt,
                        float ax_bias, float az_bias, float q_bias);
        // Calibration Routine (Blocking)
        void set_calibration(Matrix<5,2> calibration, Matrix<3> gyro_biasses); // made it a 5x2 for some reason?
        // Update step: Takes RAW sensor data, calibrates it, filters it, then fuses with model
        // Added 'calibrating' flag to match logic in .cpp
        void update(float ax, float az, float q, bool calibrating);
        
        // Prediction step: MODEL based
        // Voltage -> Force -> Velocity + tail angle -> pitch rate
        void predict(float V_IO, float servo_angle, float dt);

    private:
        // Pointers to external state variables
        float *_u = 0;      
        float *_w = 0;      
        float *_q = 0;      
        float *_theta = 0;  

        // Pointers to external low pass filtered measurement variables
        // is not made zero in og files
        float *_ax_filt;
        float *_az_filt;
        float *_q_filt_val;

        // Calibration Biases
        float _ax_bias = 0;
        float _az_bias = 0;
        float _q_bias = 0;

        // Filters
        Filter ax_filter;
        Filter az_filter;
        Filter q_filter;

        // Kalman Matrices
        Matrix<4,4> Q;      
        Matrix<3,3> R;      
        Matrix<4,4> P;                                  // Error covariance matrix      
        
        Matrix<4,4> A;                                  // Jacobian gyroscope     
        Matrix<3,4> C;                                  // Jacobian accelerometer
        Matrix<4,3> K;                                  // Kalman gain           
        
        // Identity matrix
        Matrix<4,4> I = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

        // Physical Constants
        const float mass = 0.200; // kg
        const float Iyy = 0.0005; 
        const float g = 9.81;

        // Aerodynamic Derivatives (Model) 
        // NOTE take in account that VIO is in voltage and servo angle d is in degrees!
        float Xu = -0.1, Xw = 0.1, Xq = 0.0, Xv = 1.0, Xd = 0.0;
        float Zu = 0.0, Zw = -0.5, Zq = 0.0, Zv = 0.0, Zd = -0.5;
        float Mu = 0.1, Mw = 0.1, Mq = -0.1, Mv = 0.2, Md = -1.0;

        float d2r = asin(1)/90;                         // Conversion from degrees to radians
        float r2d = 90/asin(1);                         // Conversion from radians to degrees

        Matrix<5,2> _calibration = {1, 0, 1, 0, 1, 0};  // Calibration settings {slope ax, bias ax, slope ay, bias ay, slope az, bias az} (Should actually be a 3x2 matrix)
        Matrix<3> _gyro_biasses = {0, 0, 0};            // Gyroscope biasses {bias gx, bias gy, bias gz}
};

#endif
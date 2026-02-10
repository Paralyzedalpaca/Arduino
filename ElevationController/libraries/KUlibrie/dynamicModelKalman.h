#ifndef DYNAMIC_MODEL_KALMAN_H
#define DYNAMIC_MODEL_KALMAN_H

#include <cmath>
#include <filters.h>
#include <BasicLinearAlgebra.h>

#define CONTROL2STROKE_PITCH 0.61442
#define CONTROL2STROKE_ROLL 0.80208
#define CONTROL2STROKE_YAW 2.16943

using namespace BLA;

class ModelKalman {
    public:
        ModelKalman();
        ModelKalman(Matrix<5,5> Q, Matrix<6,6> R, float f_acc, float f_gyr, float *ax_filt, float *ay_filt, float *az_filt, float *gx_filt, float *gy_filt, float *gz_filt, float *roll, float *pitch, float *yaw);

        void set_calibration(Matrix<2,3> calibration, Matrix<3> gyro_biasses);

        void predict(float V0, float dV, float ds, float dt);
        void update(float ax, float ay, float az, float gx, float gy, float gz);
        // void update(float ax, float ay, float az, float gx, float gy, float gz, float dt);

        // void update_acc_est(float ax, float ay, float az);
        // void update_gyr_est(float gx, float gy, float gz, float dt);

        float get_roll();
        float get_pitch();

        Matrix<3> get_accel();
        Matrix<3> get_accel_filt();
        Matrix<3> get_gyro();
        Matrix<3> get_gyro_filt();
        Matrix<3> get_attitude();

    private:
        float *_ax_filt, *_ay_filt, *_az_filt;
        float *_gx_filt, *_gy_filt, *_gz_filt;

        Filter ax_filter;
        Filter ay_filter;
        Filter az_filter;

        Filter gx_filter;
        Filter gy_filter;
        Filter gz_filter;

        float *_roll = 0;
        float *_pitch = 0;
        float *_yaw = 0;

        Matrix<5,5> Q;
        Matrix<6,6> R;

        Matrix<5,5> P;
        Matrix<6,5> C;      // Jacobian accelerometer
        
        Matrix<5,6> K;

        Matrix<5,5> I = {1, 0, 0, 0, 0, 
                        0, 1, 0, 0, 0, 
                        0, 0, 1, 0, 0,
                        0, 0, 0, 1, 0,
                        0, 0, 0, 0, 1};

        float d2r = asin(1)/90;
        float r2d = 90/asin(1);

        Matrix<2,3> _calibration = {0, 0, 0};
        Matrix<3> _gyro_biasses = {0, 0, 0};

        float Mq = -3.753e-5;
        float Lp = -9.9582e-5;
        float Np = -4.7923e-9;
        float Lr = 5.0146e-9;
        float Nr = -0.0003605;
        float Mpsi_e = 0.00010951;
        float Lpsi_e = 0;
        float Npsi_e = 0;
        float Mpsi_o = 0;
        float Lpsi_o = -1.2387e-9;
        float Npsi_o = -6.1555e-6;
        float Msigma_L = 0;
        float Lsigma_L = 0.00015481;
        float Nsigma_L = 1.825e-10;
        float Msigma_R = 0;
        float Lsigma_R = -0.00015481;
        float Nsigma_R = -1.825e-10;
        
        float Ixx = 79.21e-6;
        float Iyy = 78.11e-6;
        float Izz = 2.247e-6;
        float Ixz = 0.278e-06;

        float I1 = (Ixx*Izz - Ixz*Ixz)/Izz;
        float I2 = (Ixx*Izz - Ixz*Ixz)/Ixz;
        float I3 = (Ixx*Izz - Ixz*Ixz)/Ixx;

        Matrix<5,5> A = {Mq/Iyy,    0, 0,           0,          0,
                         1,         0, 0,           0,          0,
                         0,         0, Lp/I1+Np/I2, Lr/I1+Nr/I2,0,
                         0,         0, Lp/I2+Np/I3, Lr/I2+Nr/I3,0,
                         0,         0, 1,           0,          0};

        Matrix<5,4> B = {Mpsi_e/Iyy,            Mpsi_o/Iyy,             Msigma_L/Iyy,           Msigma_R/Iyy,
                         0,                     0,                      0,                      0,
                         Lpsi_e/I1+Npsi_e/I2,   Lpsi_o/I1+Npsi_o/I2,    Lsigma_L/I1+Nsigma_L/I2,Lsigma_R/I1+Nsigma_R/I2,
                         Lpsi_e/I2+Npsi_e/I3,   Lpsi_o/I2+Npsi_o/I3,    Lsigma_L/I2+Nsigma_L/I3,Lsigma_R/I2+Nsigma_R/I3,
                         0,                     0,                      0,                      0};

        Matrix<5> x = {0, 0, 0, 0, 0};
        Matrix<5> x_dot;
        Matrix<4> u;
        
};

#endif
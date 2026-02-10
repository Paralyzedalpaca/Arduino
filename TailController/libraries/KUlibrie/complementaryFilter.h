#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <cmath>
#include <filters.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

class ComplementaryFilter {
    public:
        ComplementaryFilter();
        ComplementaryFilter(float alpha, float f_acc, float f_gyr, float *ax_filt, float *ay_filt, float *az_filt, float *gx_filt, float *gy_filt, float *gz_filt, float *roll, float *pitch, float *yaw);

        void update(float ax, float ay, float az, float gx, float gy, float gz, float dt);

        void update_acc_est(float ax, float ay, float az);
        void update_gyr_est(float gx, float gy, float gz, float dt);

        // float get_roll();
        // float get_pitch();

        // Matrix<3> get_accel();
        // Matrix<3> get_accel_filt();
        // Matrix<3> get_gyro();
        // Matrix<3> get_gyro_filt();
        // Matrix<3> get_attitude();

    private:
        float alpha;

        float roll_acc;
        float pitch_acc;

        float roll_gyro;
        float pitch_gyro;

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
};

#endif
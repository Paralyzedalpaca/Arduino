#include <complementaryFilter.h>

ComplementaryFilter::ComplementaryFilter()
    : alpha(0) {}

ComplementaryFilter::ComplementaryFilter(float alpha, float f_acc, float f_gyr, float *ax_filt, float *ay_filt, float *az_filt, float *gx_filt, float *gy_filt, float *gz_filt, float *roll, float *pitch, float *yaw)
    :   alpha(alpha), 
        ax_filter(2, f_acc, 4e-3, 0), ay_filter(2, f_acc, 4e-3, 0), az_filter(2, f_acc, 4e-3, 0),
        gx_filter(2, f_gyr, 4e-3, 0), gy_filter(2, f_gyr, 4e-3, 0), gz_filter(2, f_gyr, 4e-3, 0) {
            _ax_filt = ax_filt;
            _ay_filt = ay_filt;
            _az_filt = az_filt;

            _gx_filt = gx_filt;
            _gy_filt = gy_filt;
            _gz_filt = gz_filt;

            _roll = roll;
            _pitch = pitch;
            _yaw = yaw;
    }

float sign(float x) {
    return (x > 0) - (x < 0);
}

void ComplementaryFilter::update_acc_est(float ax, float ay, float az) {
    ax_filter.update(ax);
    ay_filter.update(ay);
    az_filter.update(az);

    *_ax_filt = ax_filter.get_filtered();
    *_ay_filt = ay_filter.get_filtered();
    *_az_filt = az_filter.get_filtered();

    float m = sqrt(*_ax_filt*(*_ax_filt) + *_ay_filt*(*_ay_filt) + *_az_filt*(*_az_filt));
    *_ax_filt /= m;
    // ay_filt /= m;            Only ay/az is used so not necessary to normalize
    // az_filt /= m;

    roll_acc = atan2(*_ay_filt, -*_az_filt)/asin(1)*90;
    pitch_acc = asin(*_ax_filt)/asin(1)*90;
}

void ComplementaryFilter::update_gyr_est(float gx, float gy, float gz, float dt) {
    *_gx_filt = gx_filter.get_filtered();
    *_gy_filt = gy_filter.get_filtered();
    *_gz_filt = gz_filter.get_filtered();
    
    float rollrate = gx + sin(*_roll)*tan(*_pitch)*gy + cos(*_roll)*tan(*_pitch)*gz;
    float pitchrate = cos(*_roll)*gy - sin(*_roll)*gz;

    roll_gyro = *_roll + dt*rollrate;
    pitch_gyro = *_pitch + dt*pitchrate;
}

void ComplementaryFilter::update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    update_acc_est(ax, ay, az);
    update_gyr_est(gx, gy, gz, dt);

    *_roll = alpha*roll_acc + (1-alpha)*roll_gyro;
    *_pitch = alpha*pitch_acc + (1-alpha)*pitch_gyro;
}

// float ComplementaryFilter::get_roll() {
//     return roll;
// }

// float ComplementaryFilter::get_pitch() {
//     return pitch;
// }

// Matrix<3> ComplementaryFilter::get_accel() {
//     Matrix<3> acc = {ax_filter.get_measurement(), ay_filter.get_measurement(), az_filter.get_measurement()};

//     return acc;
// }
// Matrix<3> ComplementaryFilter::get_accel_filt() {
//     Matrix<3> acc_filt = {ax_filt, ay_filt, az_filt};

//     return acc_filt;
// }
// Matrix<3> ComplementaryFilter::get_gyro() {
//     Matrix<3> gyr = {gx_filter.get_measurement(), gy_filter.get_measurement(), gz_filter.get_measurement()};

//     return gyr;
// }
// Matrix<3> ComplementaryFilter::get_gyro_filt() {
//     Matrix<3> gyr_filt = {gx_filt, gy_filt, gz_filt};

//     return gyr_filt;
// }
// Matrix<3> ComplementaryFilter::get_attitude() {
//     Matrix<3> att = {roll, pitch, yaw};

//     return att;
// }
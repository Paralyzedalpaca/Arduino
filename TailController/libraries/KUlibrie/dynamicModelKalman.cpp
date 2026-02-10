#include <dynamicModelKalman.h>

ModelKalman::ModelKalman()
    :   Q({0, 0, 0, 0}), R({0, 0, 0, 0, 0, 0, 0, 0, 0}) {}

ModelKalman::ModelKalman(Matrix<5,5> Q, Matrix<6,6> R, float f_acc, float f_gyr, float *ax_filt, float *ay_filt, float *az_filt, float *gx_filt, float *gy_filt, float *gz_filt, float *roll, float *pitch, float *yaw)
    :   Q(Q), R(R),
        ax_filter(2, f_acc, 4e-3, 0), ay_filter(2, f_acc, 4e-3, 0), az_filter(2, f_acc, 4e-3, 0),
        gx_filter(2, f_gyr, 4e-3, 0), gy_filter(2, f_gyr, 4e-3, 0), gz_filter(2, f_gyr, 4e-3, 0)  {
            _ax_filt = ax_filt;
            _ay_filt = ay_filt;
            _az_filt = az_filt;

            _gx_filt = gx_filt;
            _gy_filt = gy_filt;
            _gz_filt = gz_filt;

            _roll = roll;
            _pitch = pitch;
            _yaw = yaw;

            P = {0.1, 0, 0, 0.1};
        }

void ModelKalman::set_calibration(Matrix<2,3> calibration, Matrix<3> gyro_biasses) {
    _calibration = calibration;
    _gyro_biasses = gyro_biasses;
}

void ModelKalman::predict(float V0, float dV, float ds, float dt) {
    u(0) = CONTROL2STROKE_PITCH*V0;
    u(1) = CONTROL2STROKE_YAW*ds;
    u(2) = max(CONTROL2STROKE_ROLL*dV, 0);
    u(3) = min(CONTROL2STROKE_ROLL*dV, 0);

    x_dot = A*x + B*u;

    x += x_dot*dt;


    P += (A*P + P*~A + Q)*dt;
}

void ModelKalman::update(float ax, float ay, float az, float gx, float gy, float gz) {
    gx_filter.update(gx*d2r);
    gy_filter.update(gy*d2r);
    gz_filter.update(gz*d2r);

    ax_filter.update(_calibration(2, 0) * ax + _calibration(2, 1));
    ay_filter.update(_calibration(1, 0) * ax + _calibration(1, 1));
    az_filter.update(_calibration(0, 0) * ax + _calibration(0, 1));

    *_gx_filt = gx_filter.get_filtered();
    *_gy_filt = gy_filter.get_filtered();
    *_gz_filt = gz_filter.get_filtered();
    
    float sr = sin(x(4));
    float cr = cos(x(4));
    float sp = sin(x(1));
    float cp = cos(x(1));
    
    C = {0, 0, 1, 0, 0,
        1, 0, 0, 0, 0, 
        0, 0, 0, 1, 0,
        0, cp, 0, 0, 0,
        0, sp*sr, 0, 0, -cp*cr,
        0, sp*cr, 0, 0, cp*sr};
    
    K = P*~C*Inverse(C*(P*~C) + R);

    Matrix<6> y = {gx, gy, gz, ax, ay, az};
    Matrix<6> h = {x(2), x(0), x(3), sp, -cp*sr, -cp*cr};

    x += K*(y - h);

    P = (I - K*C)*P;

    *_roll = x(4);
    *_pitch = x(1);
}

float ModelKalman::get_roll() {
    return *_roll*r2d;
}

float ModelKalman::get_pitch() {
    return *_pitch*r2d;
}

Matrix<3> ModelKalman::get_accel() {
    Matrix<3> acc = {ax_filter.get_measurement(), ay_filter.get_measurement(), az_filter.get_measurement()};

    return acc;
}
Matrix<3> ModelKalman::get_accel_filt() {
    Matrix<3> acc_filt = {*_ax_filt, *_ay_filt, *_az_filt};

    return acc_filt;
}
Matrix<3> ModelKalman::get_gyro() {
    Matrix<3> gyr = {gx_filter.get_measurement(), gy_filter.get_measurement(), gz_filter.get_measurement()};

    return gyr;
}
Matrix<3> ModelKalman::get_gyro_filt() {
    Matrix<3> gyr_filt = {*_gx_filt, *_gy_filt, *_gz_filt};

    return gyr_filt;
}
Matrix<3> ModelKalman::get_attitude() {
    Matrix<3> att = {*_roll, *_pitch, *_yaw};

    return att;
}
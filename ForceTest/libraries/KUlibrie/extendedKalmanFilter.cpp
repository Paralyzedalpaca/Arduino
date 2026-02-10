#include <extendedKalmanFilter.h>

// -----------------
// Empty constructor
// -----------------
ExtendedKalman::ExtendedKalman()
    :   Q({0, 0, 0, 0}), R({0, 0, 0, 0, 0, 0, 0, 0, 0}) {
        /*
        This is an empty constructor setting Q and R matrix to zeros.
        */
    }

// -----------
// Constructor
// -----------
ExtendedKalman::ExtendedKalman(Matrix<2,2> Q, Matrix<3,3> R, float f_acc, float f_gyr, float *ax_filt, float *ay_filt, float *az_filt, float *gx_filt, float *gy_filt, float *gz_filt, float *roll, float *pitch, float *yaw_rate)
    :   Q(Q), R(R),
        ax_filter(2, f_acc, 4e-3, 0), ay_filter(2, f_acc, 4e-3, 0), az_filter(2, f_acc, 4e-3, 0, -1),
        gx_filter(2, f_gyr, 4e-3, 0), gy_filter(2, f_gyr, 4e-3, 0), gz_filter(2, 0.8, 4e-3, 0)  {
            /*
            This is the normal constructor.

            Inputs:
                - Q             2x2 matrix          Q-matrix
                - R             3x3 matrix          R-matrix
                - f_acc         float               Cutoff frequency for accelerometer measurements
                - f_gyr         float               Cutoff frequency for gyroscope measurements
                - *ax_filt      float               (Reference to) the filtered accelerometer measurements in the x-direction
                - *ay_filt      float               (Reference to) the filtered accelerometer measurements in the y-direction
                - *az_filt      float               (Reference to) the filtered accelerometer measurements in the z-direction
                - *gx_filt      float               (Reference to) the filtered gyroscope measurements in the x-direction
                - *gy_filt      float               (Reference to) the filtered gyroscope measurements in the y-direction
                - *gz_filt      float               (Reference to) the filtered gyroscope measurements in the z-direction
                - *roll         float               (Reference to) the roll estimation
                - *pitch        float               (Reference to) the pitch estimation
                - *yaw_rate     float               (Reference to) the yaw rate estimation
            
            The function stores the references to the different variables to local variables within the class.
            It also initializes the error covariance matrix.
            */
            _ax_filt = ax_filt;
            _ay_filt = ay_filt;
            _az_filt = az_filt;

            _gx_filt = gx_filt;
            _gy_filt = gy_filt;
            _gz_filt = gz_filt;

            _roll = roll;
            _pitch = pitch;
            _yaw_rate = yaw_rate;

            P = {0.1, 0, 0, 0.1};
        }

// ---------------------------------------------------------------------------
// Function that copies the calibration settings to this instance of the class
// ---------------------------------------------------------------------------
void ExtendedKalman::set_calibration(Matrix<5,2> calibration, Matrix<3> gyro_biasses) {
    /*
    This function sets the calibration settings in this instance of the extendedKalmanFilter class.

    Inputs:
        - calibration           5x2 matrix          Accelerometer settings (should actually be 3x2)
        - gyro_biasses          3d vector           Gyroscope biasses
    */
    _calibration = calibration;
    _gyro_biasses = gyro_biasses;
}

// ------------------------------------------
// Function that executes the prediction step
// ------------------------------------------
void ExtendedKalman::predict(float gx, float gy, float gz, float dt, bool calibrating) {
    /*
    This function executes the prediction step of the Kalman filter.

    Inputs:
        - gx            float           New gyroscope measurement in the x-direction
        - gy            float           New gyroscope measurement in the y-direction
        - gz            float           New gyroscope measurement in the z-direction
        - dt            float           Time since last update step
        - calibrating   bool            Indicates if the drone is flying or being calibrated
    */
    
    // Filter the gyroscope data
    gx_filter.update(gx);
    gy_filter.update(gy);
    gz_filter.update(gz);

    // If the drone is calibrating, the raw filtered data should be used, otherwise, 
    // the bias should be subtracted from the measurements
    if (calibrating) {
        *_gx_filt = gx_filter.get_filtered();
        *_gy_filt = gy_filter.get_filtered();
        *_gz_filt = gz_filter.get_filtered();
    } else{
        *_gx_filt = (gx_filter.get_filtered() - _gyro_biasses(0))*d2r;
        *_gy_filt = (gy_filter.get_filtered() - _gyro_biasses(1))*d2r;
        *_gz_filt = (gz_filter.get_filtered() - _gyro_biasses(2))*d2r;
    }

    // The transformation to the world reference frame divides by cos(pitch) so to obtain accurate results, 
    // the transformation should only be done when the pitch angle is sufficiently far from 90° (or pi/2 radians)
    if (abs(*_pitch - asin(1)) > 0.1) {
        // Calculate goniometric values
        float sr = sin(*_roll);
        float cr = cos(*_roll);
        float tp = tan(*_pitch);
        float cp = cos(*_pitch);

        // Transform to the world reference frame
        float rollrate = *_gx_filt + sr*tp*(*_gy_filt) + cr*tp*(*_gz_filt);
        float pitchrate = cr*(*_gy_filt) - sr*(*_gz_filt);
        float yawrate = sr/cp*(*_gy_filt) + cr/cp*(*_gz_filt);

        // Predict the orientation based on the transformed gyroscope measurements
        *_roll = *_roll + dt*rollrate;
        *_pitch = *_pitch + dt*pitchrate;
        *_yaw_rate = yawrate;

    } else {
        // If pitch is close to 90°, predict the orientation based on the raw gyroscope measurements
        *_roll = *_roll + dt*(*_gx_filt);
        *_pitch = *_pitch + dt*(*_gy_filt);
        *_yaw_rate = *_gz_filt;
    }
    
    // Keep the value of roll and pitch between -90° and +90°
    if (abs(*_pitch) > asin(1)) {
        *_pitch = *_pitch/abs(*_pitch)*(acos(-1) - abs(*_pitch));
    }
    if (abs(*_roll) > acos(-1)) {
        *_roll = *_roll - 2*(*_roll/abs(*_roll)*acos(-1));
    }

    // Recalculate the goniometric values
    float sr = sin(*_roll);
    float cr = cos(*_roll);
    float tp = tan(*_pitch);
    float cp = cos(*_pitch);

    // Predict the gyroscope Jacobian and the error covariance matrix
    A = {cr*tp*(*_gy_filt)-sr*tp*(*_gz_filt), sr/cp/cp*(*_gy_filt) + cr/cp/cp*(*_gz_filt), -sr*(*_gy_filt)-cr*(*_gz_filt), 0};
    P += (A*P + P*~A + Q)*dt;
    
}

// --------------------------------------
// Function that executes the update step
// --------------------------------------
void ExtendedKalman::update(float ax, float ay, float az, bool calibrating) {
    /*
    This function executes the update step of the Kalman filter.

    Inputs:
        - ax            float           New accelerometer measurement in the x-direction
        - ay            float           New accelerometer measurement in the y-direction
        - az            float           New accelerometer measurement in the z-direction
        - calibrating   bool            Indicates if the drone is flying or being calibrated
    */

    // Filter the accelerometer measurements
    ax_filter.update(ax);
    ay_filter.update(ay);
    az_filter.update(az);

    // If the drone is calibrating, the raw filtered data should be used, otherwise, 
    // the measurements should be transformed using the calibration settings
    if (calibrating) {
        *_ax_filt = ax_filter.get_filtered();
        *_ay_filt = ay_filter.get_filtered();
        *_az_filt = az_filter.get_filtered();
    } else {
        *_ax_filt = ax_filter.get_filtered()*_calibration(0, 0) + _calibration(0, 1);
        *_ay_filt = ay_filter.get_filtered()*_calibration(1, 0) + _calibration(1, 1);
        *_az_filt = az_filter.get_filtered()*_calibration(2, 0) + _calibration(2, 1);
    }

    // Normalize the data
    float norm = sqrt(*_ax_filt**_ax_filt + *_ay_filt**_ay_filt + *_az_filt**_az_filt);
    *_ax_filt /= norm;
    *_ay_filt /= norm;
    *_az_filt /= norm;

    // Calculate goniometric values
    float sr = sin(*_roll);
    float cr = cos(*_roll);
    float sp = sin(*_pitch);
    float cp = cos(*_pitch);

    // Calculate the measurement vector
    Matrix<3> h = {sp, -cp*sr, -cp*cr};

    // Calculate the accelerometer Jacobian
    C = {0, cp, -cp*cr, sp*sr, cp*sr, sp*cr};

    // Calculate the Kalman gain
    K = P*~C*Inverse(C*(P*~C) + R);

    // Update the error covariance matrix
    P = (I - K*C)*P;

    // Update the roll and pitch estimation
    *_roll = *_roll + K(0, 0)*(*_ax_filt-h(0)) + K(0, 1)*(*_ay_filt-h(1)) + K(0, 2)*(*_az_filt-h(2));
    *_pitch = *_pitch + K(1, 0)*(*_ax_filt-h(0)) + K(1, 1)*(*_ay_filt-h(1)) + K(1, 2)*(*_az_filt-h(2));

}

// ----------------------------------------------------------------------------------------------
// Getters
// These functions return the roll and pitch estimation and the raw and filtered IMU measurements
// ----------------------------------------------------------------------------------------------
float ExtendedKalman::get_roll() {
    return *_roll*r2d;
}

float ExtendedKalman::get_pitch() {
    return *_pitch*r2d;
}

Matrix<3> ExtendedKalman::get_accel() {
    Matrix<3> acc = {ax_filter.get_measurement(), ay_filter.get_measurement(), az_filter.get_measurement()};

    return acc;
}
Matrix<3> ExtendedKalman::get_accel_filt() {
    Matrix<3> acc_filt = {*_ax_filt, *_ay_filt, *_az_filt};

    return acc_filt;
}
Matrix<3> ExtendedKalman::get_gyro() {
    Matrix<3> gyr = {gx_filter.get_measurement(), gy_filter.get_measurement(), gz_filter.get_measurement()};

    return gyr;
}
Matrix<3> ExtendedKalman::get_gyro_filt() {
    Matrix<3> gyr_filt = {*_gx_filt, *_gy_filt, *_gz_filt};

    return gyr_filt;
}
Matrix<3> ExtendedKalman::get_attitude() {
    Matrix<3> att = {*_roll, *_pitch, *_yaw_rate};

    return att;
}


// These functions are not used anymore
// {
void ExtendedKalman::reset_ax() {
    ax_filter.reset();
}
void ExtendedKalman::reset_ay() {
    ay_filter.reset();
}
void ExtendedKalman::reset_az() {
    az_filter.reset();
}

void ExtendedKalman::reset_ax(float ax) {
    ax_filter.reset(ax);
}
void ExtendedKalman::reset_ay(float ay) {
    ay_filter.reset(ay);
}
void ExtendedKalman::reset_az(float az) {
    az_filter.reset(az);
}
// }
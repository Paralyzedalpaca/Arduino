#include "LongitudinalEKF.h"

// -----------------
// Empty constructor setting Q & R matrices to zeros
// -----------------
LongitudinalEKF::LongitudinalEKF() 
: Q({0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}), R({0,0,0, 0,0,0, 0,0,0}) {}

// -------------------------------------------------------------------------
// Full Constructor with Calibration and Filtered Measurement Pointers
// -------------------------------------------------------------------------
LongitudinalEKF::LongitudinalEKF(Matrix<4,4> Q_in, Matrix<3,3> R_in, 
                                 float f_acc, float f_gyr, 
                                 float *u, float *w, float *q, float *theta,
                                 float *ax_filt, float *az_filt, float *q_filt,
                                 float ax_bias, float az_bias, float q_bias)
    : Q(Q_in), R(R_in),
      ax_filter(2, f_acc, 4e-3, 0), az_filter(2, f_acc, 4e-3, 0, -1),
      q_filter(2, f_gyr, 4e-3, 0) 
      // az_filter init to -1g (assuming flat on ground)
{
    // Link State Pointers
    _u = u; _w = w; _q = q; _theta = theta;

    // Link Filtered Measurement Pointers
    _ax_filt = ax_filt;
    _az_filt = az_filt;
    _q_filt_val = q_filt;

    // Store Calibration
    _ax_bias = ax_bias;
    _az_bias = az_bias;
    _q_bias = q_bias;

    // Initialize Covariance
    P = {0.1, 0, 0, 0,
         0, 0.1, 0, 0,
         0, 0, 0.1, 0,
         0, 0, 0, 0.1};
}

// ---------------------------------------------------------------------------
// Function that copies the calibration settings to this instance of the class
// ---------------------------------------------------------------------------
void LongitudinalEKF::set_calibration(Matrix<5,2> calibration, Matrix<3> gyro_biasses) {
    _calibration = calibration;
    _gyro_biasses = gyro_biasses;
}

// ------------------------------------------
// Prediction Step (Physics Model)
// ------------------------------------------
void LongitudinalEKF::predict(float V_IO, float servo_angle, float dt) {
    // [DEBUG] Check Inputs and Constants
    Serial.print("PREDICT dt: "); Serial.println(dt, 6);
    Serial.print("PREDICT In State (u,w,q,th): ");
    Serial.print(*_u); Serial.print(", ");
    Serial.print(*_w); Serial.print(", ");
    Serial.print(*_q); Serial.print(", ");
    Serial.println(*_theta);
    
    float u = *_u;
    float w = *_w;
    float q = *_q;
    float theta = *_theta;

    // Model Forces
    float Fx = Xu*u + Xw*w + Xq*q + Xv*V_IO + Xd*servo_angle;
    float Fz = Zu*u + Zw*w + Zq*q + Zv*V_IO + Zd*servo_angle;
    float My = Mu*u + Mw*w + Mq*q + Mv*V_IO + Md*servo_angle;

    // Equations of Motion
    float u_dot = Fx/mass - g*sin(theta) - q*w;
    float w_dot = Fz/mass + g*cos(theta) + q*u;
    float q_dot = My/Iyy;
    float theta_dot = q;

    // Integration
    *_u += u_dot * dt;
    *_w += w_dot * dt;
    *_q += q_dot * dt;
    *_theta += theta_dot * dt;

    // Jacobian A Update
    float cTh = cos(theta);
    float sTh = sin(theta);
    
    A(0,0) = Xu/mass;       A(0,1) = Xw/mass - q;   A(0,2) = Xq/mass - w;   A(0,3) = -g*cTh;
    A(1,0) = Zu/mass + q;   A(1,1) = Zw/mass;       A(1,2) = Zq/mass + u;   A(1,3) = -g*sTh;
    A(2,0) = Mu/Iyy;        A(2,1) = Mw/Iyy;        A(2,2) = Mq/Iyy;        A(2,3) = 0;
    A(3,0) = 0;             A(3,1) = 0;             A(3,2) = 1;             A(3,3) = 0;
    
    P += (A*P + P*~A + Q)*dt; 
    // [DEBUG] Check P matrix after prediction
    Serial.print("PREDICT P_diag: ");
    Serial.print(P(0,0)); Serial.print(", ");
    Serial.print(P(1,1)); Serial.print(", ");
    Serial.print(P(2,2)); Serial.print(", ");
    Serial.println(P(3,3));
    
    if (isnan(P(0,0))) Serial.println("ERROR: P became NaN in Predict step!");
}

// ----------------------------------------------------
// Update Step (Calibrate -> Filter -> Correct Model)
// ----------------------------------------------------
void LongitudinalEKF::update(float ax, float az, float q_raw, bool calibrating) {
    // 1. Filter Measurements
    ax_filter.update(ax);
    az_filter.update(az);
    q_filter.update(q_raw); // Use q_filter, not gy_filter

    // 2. Apply Calibration
    if (calibrating) {
        *_ax_filt = ax_filter.get_filtered();
        *_az_filt = az_filter.get_filtered();
        *_q_filt_val = q_filter.get_filtered();
    } else {
        *_ax_filt = ax_filter.get_filtered() * _calibration(0, 0) + _calibration(0, 1);
        *_az_filt = az_filter.get_filtered() * _calibration(2, 0) + _calibration(2, 1);
        
        // !!!! Need to watch out _gyro_biasses if 1x3 => must be _gyro_biasses(2)!!
        *_q_filt_val = (q_filter.get_filtered() - _gyro_biasses(2)) * d2r;
    }

    // 3. EKF Innovation (Measurement - Prediction)
    float u = *_u; 
    float w = *_w; 
    float q = *_q;

    float pred_ax = (Xu*u + Xw*w + Xq*q) / mass; 
    float pred_az = (Zu*u + Zw*w + Zq*q) / mass;
    float pred_q  = q;

    Matrix<3> z_meas = {*_ax_filt, *_az_filt, *_q_filt_val};
    Matrix<3> z_pred = {pred_ax, pred_az, pred_q};
    Matrix<3> y = z_meas - z_pred; 

    // 4. Jacobian C (Measurement Matrix)
    C(0,0) = Xu/mass; C(0,1) = Xw/mass; C(0,2) = Xq/mass; C(0,3) = 0;
    C(1,0) = Zu/mass; C(1,1) = Zw/mass; C(1,2) = Zq/mass; C(1,3) = 0;
    C(2,0) = 0;       C(2,1) = 0;       C(2,2) = 1;       C(2,3) = 0;

    // 5. Correction
    Matrix<3,3> S = C * P * (~C) + R;

    // [DEBUG] Check S before inversion
    Serial.print("UPDATE S_diag: ");
    Serial.print(S(0,0), 6); Serial.print(", ");
    Serial.print(S(1,1), 6); Serial.print(", ");
    Serial.println(S(2,2), 6);

    // [DEBUG] Check R just to be absolutely sure
    Serial.print("UPDATE R_diag: ");
    Serial.print(R(0,0), 6); Serial.print(", ");
    Serial.print(R(1,1), 6); Serial.print(", ");
    Serial.println(R(2,2), 6);
    
    Matrix<3,3> S_inv = Inverse(S);
    
    // [DEBUG] Check Inversion result
    Serial.print("UPDATE S_inv(0,0): "); Serial.println(S_inv(0,0));
    
    K = P * (~C) * S_inv; // Changed to use the pre-calculated S_inv for debugging
    
    Matrix<4> x_corr = K * y;
    *_u += x_corr(0);
    *_w += x_corr(1);
    *_q += x_corr(2);
    *_theta += x_corr(3);

    P = (I - K * C) * P;
}
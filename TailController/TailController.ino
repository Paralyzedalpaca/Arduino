// Import the necessary packages
#include <AutoPID.h>
#include <KUlibrie.h>
#include <Servo.h> // Include the Servo library


//-------------------------------------------------
#include <Filters.h> // Include the Filter library

// Create a low-pass filter object for pitch
Filter control_filter(2, 3, 4e-3, 0); 
float filtered_control;
Filter pitchrate_filter(2, 1, 4e-3, 0); 
float pitchrate_numerical;
float filtered_pitchrate;
//-------------------------------------------------


Servo myServo; // Create a Servo object

const int servoPin = 8; // Pin where the servo is connected
const float deg_to_rad = 3.1415/180; // Conversion factor from degrees to radians
const float rad_to_deg = 180/3.1415; // Conversion factor from radians to degrees

float servo_angle_deg = 0; // Variable to store the servo angle


// Initiate the controller actions
float VI0 = 2.35;//1.75;
float V0 = 0.2;
float dV = 0.0;
float ds = 0.0;
float servo_angle = 0.0;
// Possibly add some biasses (I don't think this is actually necessary, 
// the integral action should take care of this, but I used it in the 
// beginning to facilitate the tuning)
// float V0_bias = 0.15;
float V0_bias = 0;
// float dV_bias = -0.05;
float dV_bias = 0;
float ds_bias = 0;

// Maximum values for control variables
#define DV_MIN -0.4
#define DV_MAX 0.4
#define V0_MIN -1
#define V0_MAX 1
#define DS_MIN -0.25
#define DS_MAX 0.25
#define servo_min -60*deg_to_rad
#define servo_max 60*deg_to_rad
#define servo_bias (90)*deg_to_rad//(95+5)*deg_to_rad


// Conversion from degrees to radians
#define D2R asin(1)/90

// PID variables (need to be tuned)
// Roll
float Kp_r = 0.9;
float Ki_r = 0.2;
float Kd_r = 0.3;

// Pitch
// float Kp_p = 1.5;
// float Ki_p = 0.2;
// float Kd_p = 0.7;
//0.9,0.3 werkt
//1.4 0.9 werkt met oscillaties



//(((((((PD CONSTANTS:
//1.2 0.22 R=10
//1.33 0.32 werkt goed R=5
//1.46 0.5 R=2.5
//1.7 0.8 R=1
//PID CONSTANTS:
//P D I
//1.4 0.3 0.22 R=10
//1.6 0.33 0.44 R=5
//1.3 0.16 0.22 R=20))))))


//PD constants:
//R10 q:0.08 p:1.26:1.5 tested
//R5 q:0.09 p:1.33 ss:1.3 tested
//R1 q:0.11 p:1.75 ss:0.8 tested
//R0.5 q:0.135 p:2.12 ss:0.64

//PID constants:
//alpha1 R100 q:0.086 p:1.24  I:0.1 tested
//alpha1 R10 q:0.09 p:1.4  I:0.32 tested
//alpha1 R1 q:0.12 p:2  I:1    tested very good performance 
//alpha1 R0.5 q:0.15 p:2.4  I:1.4
//alpha1 R0.1 q:0.22 p:4.2  I:3.16


//alpha10 R50 q:0.09 p:1.4  I:0.44
//alpha10 R10 q:0.1 p:1.6  I:1 tested good performance
//alpha10 R1 q:0.15 p:2.5  I:3

float Kp_p =2.0;//1.33;//0.8;//1.4;//0.66; //0.9;//1.4;
float Ki_p =1.0;
float Kd_p =0.12;//0.32;//0.4;//0.9;//0.28; //0.3;//*1000000.0;

float ss_error=1.5;


float start_ramp = 30*1000.0;//ms
float end_pitch = 20*deg_to_rad;
float ramp = 10*deg_to_rad;//rad/s
float start_controller =  18*1000.0;
float end_controller =  42*1000.0;


// Yaw
float Kp_y = -0.2;
float Ki_y = -0.3;
float Kd_y = 0.0;

// Reference attitude
float ref_roll = 0*D2R;
float ref_pitch = 0*D2R;
float ref_yaw_rate = 6*D2R; // To limit the drift
float pitch_reference = 70.0*deg_to_rad;


// Variables to store the attitude estimation
float roll, pitch, yaw_rate;
// Variables for Longitudinal EKF States
float u = 0;      // Forward Velocity (m/s)
float w = 0;      // Vertical Velocity (m/s)
float q_ekf = 0;  // Pitch Rate from EKF (rad/s)
float theta = 0;  // Pitch Angle from EKF (rad)

float previous_pitch=0;
float pitch_error;
float pitch_error_i=0;

bool started = false;
// KUlibrie object to control the hardware settings, bluetooth communication, ...
KUlibrie kulibrie(&roll, &pitch, &yaw_rate,
                &VI0, &V0, &dV, &ds, &ref_roll, 
                &pitch_reference, &ref_yaw_rate,
                &servo_angle_deg);
KUlibrie kulibrie(&roll, &pitch, &yaw_rate, &u, &w, &q_ekf, &theta, // Added new pointers
                  &VI0, &V0, &dV, &ds, 
                  &ref_roll, &pitch_reference, &ref_yaw_rate, 
                  &servo_angle_deg);


// PID controllers
AutoPID roll_control(&roll, &ref_roll, &dV, DV_MIN, DV_MAX, Kp_r, Ki_r, Kd_r);
//AutoPID pitch_control(&pitch, &ref_pitch, &V0, V0_MIN, V0_MAX, Kp_p, Ki_p, Kd_p);
AutoPID yaw_control(&yaw_rate, &ref_yaw_rate, &ds, DS_MIN, DS_MAX, Kp_y, Ki_y, Kd_y);
AutoPID pitch_control(&pitch, &ref_pitch, &servo_angle, servo_min, servo_max, Kp_p, Ki_p, Kd_p);


unsigned long time_old, time_new;               // Variables to calculate the timestep
float dt;                                       // Stores the last timestep
unsigned long cycle_timer;                      // Stores the time in the latest flapping cycle
int cycle_counter = 0;                          // Stores how many flapping cycles have passed (to create a rampup during the first 10 cycles)
float flapping_frequency = 20;                  // Stores the flapping frequency
float cycle_period = 1000/flapping_frequency;   // Stores the time of one flapping cycle

float time_start;

// --------
// Run once
// --------
void setup() {
  //delay(1000*4);
  /*
  This function is run once
  */
  // Start Serial to be able to print things (for debug purposes)
  Serial.begin(115200);
  // while (!Serial);





  // Setup KUlibrie hardware (leds, start IMU, setup bluetooth, ...)
  kulibrie.setup_hardware();
  myServo.attach(servoPin); // Attach the servo to the specified pin
  myServo.write(95); // Set the servo to the neutral position (90 degrees)
  //kulibrie.calibrate = true;
  kulibrie.control = true;//true;
  kulibrie.send_telemetry_data = true;
  time_old = millis();
  //time_start = millis();
}

// ----
// Loop
// ----
void loop() {
  
  /*
  This function loops continuously
  */
  // Serial.println(pitch);
//  Serial.println(kulibrie.readFloatGyroX());
  // Serial.println(kulibrie.readFloatGyroY());
  // Serial.println(kulibrie.readFloatGyroZ());
  // Serial.println(kulibrie.readFloatAccelX());
  // Serial.println(kulibrie.readFloatAccelY());
  // Serial.println(kulibrie.readFloatAccelZ());
  // Serial.println(kulibrie.get_accel_filt());
  // Serial.println(kulibrie.get_gyro_filt());
//  Serial.println(kulibrie.readIMU());
  if (kulibrie.control) {
    if (!started) {
      time_start = millis();
      started = true;
  }
    // Run this loop only when the drone is flying
    //Serial.println(pitch);
    //Serial.println(pitch*rad_to_deg);
    //Serial.println(pitch*rad_to_deg);
    
//Serial.println();
    time_new = millis();
    dt = (time_new - time_old)/1000.0;
    time_old = time_new;

    // if (started == false){
    // time_start = millis();
    // started = true;
    // }
    //update reference
    
    if (time_new-time_start<=start_ramp){
      pitch_reference = 0;
    }
    else if (time_new-time_start>start_ramp&&pitch_reference<end_pitch){
      pitch_reference = (time_new-time_start-start_ramp)*ramp/1000.0;
    }
    else {
      pitch_reference = end_pitch;
    }
    //pitch_reference = 0*deg_to_rad;
    //Serial.println(pitch);
    // Update attitude estimation
    kulibrie.update_filter(dt);

    // Set timesteps for pid controllers
    roll_control.setTimeStep(dt);
    pitch_control.setTimeStep(dt);
    yaw_control.setTimeStep(dt);


    //Serial.println("actual");
    pitch_error = pitch-pitch_reference;
    float safe_dt = max(dt, 1e-6); // Prevents division by near-zero
    pitchrate_numerical = (pitch - previous_pitch) / safe_dt;
    pitchrate_filter.update(pitchrate_numerical);
    filtered_pitchrate = pitchrate_filter.get_filtered();
    previous_pitch = pitch;
    // Serial.print(pitchrate_numerical);
    // Serial.print(",");
    // Serial.println(filtered_pitchrate);
    // Serial.print(pitch_error_i);
    // Serial.print("           ");
    // Serial.println(servo_angle);
    
    // Run controllers
    // roll_control.run();
    pitch_control.run();
    // yaw_control.run();

    //--------------------------------------
    control_filter.update(servo_angle);
    filtered_control = control_filter.get_filtered();

    //Serial.print(filtered_control);
    //-------------------------------------


    Serial.println(servo_angle_deg);
    // Set the desired voltages to the motors of the KUlibrie
    
    if ((time_new<time_start+end_controller)&&(time_new>time_start+start_controller)){
    pitch_error_i = pitch_error_i+pitch_error*dt;
    move_servo();
    set_voltage();// WEG GEDAAN!!!!!!!!!!!!!!!!!!
    
    } else{
      analogWrite(kulibrie.PWM_A, 0);
      analogWrite(kulibrie.PWM_B, 0);
    }
  } else if (kulibrie.calibrate) {
    // Run this loop only when the drone is calibrating
    calibrate();
  } else {
    // If the drone is idle, the motors should not be powered
    analogWrite(kulibrie.PWM_A, 0);
    analogWrite(kulibrie.PWM_B, 0);
  }

  // Send telemetry and controller information when asked
  if (kulibrie.send_telemetry_data) {
    kulibrie.send_telemetry(time_new);
  }
  if (kulibrie.send_control_data) {
    kulibrie.send_control(time_new);
  }
}

// --------------------------------------------------------------
// Function to keep track of the number of passed flapping cycles
// --------------------------------------------------------------
void update_cycle() {
  if (millis() - cycle_timer > cycle_period) {
    cycle_timer = millis();
    cycle_counter += 1;
  }
}

// ---------------------
// Calibration procedure
// ---------------------
void calibrate() {
    // The LED indicates the drone is calibrating
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);

    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("Calibration");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    // Store gyroscope bias
    float gx_bias = 0;
    float gy_bias = 0;
    float gz_bias = 0;

    // Variables for linear regression accelerometer
    String axis_vec[3] = {"z", "y", "x"};
    String cal_directions[3] = {"upward", "downward", "perpendicular to gravity"};
    int cal_indices[3] = {2, 1, 0};
    
    // Variables to store which axis is currently being calibrated
    String ax_qq;
    String direc;

    // Keep track of time
    unsigned long t;

    // Loop over the three axes
    for (int qq=0; qq<3; qq++) {
        ax_qq = axis_vec[qq];

        Matrix<3> ax_offsets;
        Serial.println("----------------------------------------------------------------------------");

        // For every axis, loop over the three directions (up -> down -> neutral)
        for (int direc_ii=0; direc_ii<3; direc_ii++) {
            time_old = millis();

            direc = cal_directions[direc_ii];

            Serial.print("---- Turn the KUlibrie such that the ");
            Serial.print(ax_qq);
            Serial.print("-axis points ");
            Serial.println(direc);
            
            // Store the sum of the accelerometer measurements
            float meas = 0;
            
            // Keep track of the number of passed cycles (To nicely visualize where the drone is in the calibration cycle => for debugging purposes)
            int i = 0;

            // Keep track of the time
            unsigned long start_time = millis();

            // Flap for 5 seconds
            while (i < NB_MEASUREMENTS_CALIBRATIONS) {
              time_new = millis();
              dt = (time_new - time_old)/1000.0;
              time_old = time_new;

              // Update the IMU filters
              kulibrie.update_filter(dt);
              
              if (millis() - start_time > 5000) {
                // Set the correct voltage to the motors
                set_voltage();

                // Update the calculation of the gyroscope biasses
                gx_bias += kulibrie.readFloatGyroX();
                gy_bias += kulibrie.readFloatGyroY();
                gz_bias += kulibrie.readFloatGyroZ();
                
                // Visualize the calibration cycle (for debugging purposes)
                if ((i+1)%100 == 0) {
                    Serial.println("-");
                } else {
                    Serial.print("-");
                }

                // Check which axis is being calibrated
                switch(qq) {
                    case 0:
                        meas += kulibrie.readFloatAccelZ();
                        break;
                    case 1:
                        meas += kulibrie.readFloatAccelY();
                        break;
                    case 2:
                        meas += kulibrie.readFloatAccelX();
                        break;
                }   

                i++;

                // Give 5 seconds to the operator to change the orientation of the drone
                delay(5);
              }
            }

            // Put the drone into idle
            analogWrite(kulibrie.PWM_A, 0);
            analogWrite(kulibrie.PWM_B, 0);

            // Calculate the average of the measurements
            ax_offsets(direc_ii) = meas / NB_MEASUREMENTS_CALIBRATIONS;
            
            // Print average (for debugging purposes)
            Serial.println(ax_offsets(direc_ii));

            // Reset the cycle counter
            cycle_counter = 0;
        }

        // This is what the accelerometer measurements should ideally return for the three orientations {up, down, neutral}
        Matrix<3> y = {1, -1, 0};
        
        // Since the drone can only rotate about one axis, 1 of the accelerometer axes should always return zero
        // If pitch is being tuned, qq == 1 should be zero
        // If roll is being tuned, qq == 2 should  be zero
        if (qq == 1) {
          y = {0, 0, 0};
        }

        // Calculate the linear regression
        kulibrie.curve_fit(ax_offsets, y, cal_indices[qq]);
    }
    // Set the calibration settings
    kulibrie.set_calibration();

    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("Calibration done");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    
    // Turn of the LED
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);

    kulibrie.calibrate = false;
}

// -------------------------------------------------------
// Function that applies the desired voltage to the motors
// -------------------------------------------------------
void set_voltage() {
  update_cycle();

  analogWrite(kulibrie.PWM_A, 0);
  analogWrite(kulibrie.PWM_B, 0);

  int volt_A = 0;
  int volt_B = 0;

  if (millis() - cycle_timer > cycle_period * (0.5 - (ds + ds_bias))) {
    digitalWrite(kulibrie.DIREC_A, LOW);

    if (cycle_counter < 10) {
      volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
    } else {
      volt_A = min((int)((-(V0+V0_bias) + VI0 - (dV+dV_bias))/3.7*255), (int)255);
    }
  } else {
    digitalWrite(kulibrie.DIREC_A, HIGH);
    
    if (cycle_counter < 10) {
      volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
    } else {
      volt_A = min((int)(((V0+V0_bias) + VI0 - (dV+dV_bias))/3.7*255), (int)255);
    }
  }

  if (millis() - cycle_timer > cycle_period * (0.5 + (ds + ds_bias))) {
    digitalWrite(kulibrie.DIREC_B, HIGH);

    if (cycle_counter < 10) {
      volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
    } else {
      volt_B = min((int)((-(V0+V0_bias) + VI0 + (dV+dV_bias))/3.7*255), (int)255);
    }
  } else {
    digitalWrite(kulibrie.DIREC_B, LOW);

    if (cycle_counter < 10) {
      volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
    } else {
      volt_B = min((int)(((V0+V0_bias) + VI0 + (dV+dV_bias))/3.7*255), (int)255);
    }
  }
  
  analogWrite(kulibrie.PWM_A, volt_A);
  analogWrite(kulibrie.PWM_B, volt_B);

  // This is a second possibility (the difference lies in the yaw control)
  // analogWrite(kulibrie.PWM_A, 0);
  // analogWrite(kulibrie.PWM_B, 0);

  // int volt_A = 0;
  // int volt_B = 0;

  // if (millis() - cycle_timer > cycle_period * 0.5) {
  //   digitalWrite(kulibrie.DIREC_A, LOW);
  //   digitalWrite(kulibrie.DIREC_B, HIGH);

  //   if (cycle_counter < 10) {
  //     volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
  //     volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
  //   } else {
  //     volt_A = min((int)((-(V0+V0_bias) - (ds+ds_bias) + VI0 - (dV+dV_bias))/3.7*255), (int)255);
  //     volt_B = min((int)((-(V0+V0_bias) + (ds+ds_bias) + VI0 + (dV+dV_bias))/3.7*255), (int)255);
  //     // volt_B = 0;
  //   }
  // } else {
  //   digitalWrite(kulibrie.DIREC_A, HIGH);
  //   digitalWrite(kulibrie.DIREC_B, LOW);

  //   if (cycle_counter < 10) {
  //     volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
  //     volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
  //   } else {
  //     volt_A = min((int)(((V0+V0_bias) + (ds+ds_bias) + VI0 - (dV+dV_bias))/3.7*255), (int)255);
  //     volt_B = min((int)(((V0+V0_bias) - (ds+ds_bias) + VI0 + (dV+dV_bias))/3.7*255), (int)255);
  //     // volt_B = 0;
  //   }
  // }

  // analogWrite(kulibrie.PWM_A, volt_A);
  // analogWrite(kulibrie.PWM_B, volt_B);
}

void move_servo(){
  //servo_angle_deg = (servo_angle+servo_bias)*rad_to_deg; // Calculate the servo angle based on the pitch value
  //servo_angle_deg = (-Kp_p*pitch+pitch_reference/ss_error+servo_bias-filtered_pitchrate*Kd_p)*rad_to_deg; // Calculate the servo angle based on the pitch value
  servo_angle_deg = (-Kp_p*pitch+servo_bias-filtered_pitchrate*Kd_p-pitch_error_i*Ki_p)*rad_to_deg;
    if (servo_angle_deg > (servo_max+servo_bias)*rad_to_deg) {
      servo_angle_deg = (servo_max+servo_bias)*rad_to_deg;
    } else if (servo_angle_deg < (servo_min+servo_bias)*rad_to_deg) {
      servo_angle_deg = (servo_min+servo_bias)*rad_to_deg;
    }
    // Serial.println((int)servo_angle_deg);
    myServo.write((int)servo_angle_deg); // Set the servo angle based on the pitch value
    // Update timestep
}







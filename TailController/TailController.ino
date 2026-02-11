// Import the necessary packages
#include <AutoPID.h>
#include <KUlibrie.h>
#include <Servo.h> 
#include <filters.h> 

// ---------------------------------------------------
// 1. Define Variables 
// ---------------------------------------------------

// --- EKF State Variables (New) ---
float u = 0;      // Forward velocity (m/s)
float w = 0;      // Vertical velocity (m/s)
float q = 0;      // Pitch rate (rad/s) from EKF
float theta = 0;  // Pitch angle (rad) from EKF

// --- Controls ---
float VI0 = 0;           // Main Voltage
float servo_angle_deg = 90; // Servo angle in degrees (Maintained by KUlibrie)
float servo_angle_rad = 0;  // Helper for math

// --- Legacy Control Variables (Kept for set_voltage mixer) ---
float V0 = 0;
float dV = 0;
float ds = 0;

// Biases
float V0_bias = 0;
float dV_bias = 0;
float ds_bias = 0;

// --- Filters (Auxiliary) ---
// Note: Pitch rate filtering is now largely done by EKF, but keeping control filter
Filter control_filter(2, 3, 4e-3, 0); 
float filtered_control;

// --- Hardware ---
Servo myServo; 
const int servoPin = 8; 

// --- Constants ---
const float deg_to_rad = 3.1415/180.0; 
const float rad_to_deg = 180.0/3.1415; 

// Limits
#define DV_MIN -0.4
#define DV_MAX 0.4
#define V0_MIN -1
#define V0_MAX 1
#define DS_MIN -0.25
#define DS_MAX 0.25
#define servo_min -60*deg_to_rad
#define servo_max 60*deg_to_rad
#define servo_bias (90)*deg_to_rad

// --- Tuning (Longitudinal) ---
// Updated to provided values
float Kp_p = 2.0; 
float Ki_p = 1.0;
float Kd_p = 0.12; 
float ss_error = 1.5;

// Ramp settings
float start_ramp = 30*1000.0; // ms
float end_pitch = 20*deg_to_rad;
float ramp = 10*deg_to_rad; // rad/s
float start_controller = 18*1000.0;
float end_controller = 42*1000.0;

// References
float pitch_reference = 0.0; // Voltage ref via BLE or Ramping

// Errors
float pitch_error;
float pitch_error_i = 0;

// --- Instantiation ---

// KUlibrie Object (Longitudinal Setup)
// Passing pointers to EKF states and relevant controls
KUlibrie kulibrie(&u, &w, &q, &theta, 
                  &VI0, &servo_angle_deg, 
                  &pitch_reference);

// --- Timing ---
unsigned long time_old, time_new;               
float dt;                                       
unsigned long cycle_timer;                      
int cycle_counter = 0;                          
float flapping_frequency = 20;                  
float cycle_period = 1000/flapping_frequency;   
float time_start;
bool started = false;

// ---------------------------------------------------
// Setup
// ---------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Setup Hardware (BLE, IMU, Calibration load)
  kulibrie.setup_hardware();
  
  myServo.attach(servoPin); 
  myServo.write(95); // Neutral position

  // Default flags
  kulibrie.control = true;
  kulibrie.send_telemetry_data = true;
  
  time_old = millis();
}

// ---------------------------------------------------
// Main Loop
// ---------------------------------------------------
void loop() {
  
  // 1. Check Control State
  if (kulibrie.control) {
    if (!started) {
      time_start = millis();
      started = true;
    }

    // 2. Time Management
    time_new = millis();
    dt = (time_new - time_old)/1000.0;
    time_old = time_new;

    // 3. Update Reference (Ramping)
    if (time_new - time_start <= start_ramp){
      // pitch_reference is used as a target ANGLE here, not voltage
      // Note: The BLE reference overwrites 'pitch_reference'. 
      // If using ramp, ensure BLE isn't fighting it.
      // Assuming ramp logic takes precedence:
      // pitch_reference = 0; 
    }
    else if (time_new - time_start > start_ramp && pitch_reference < end_pitch){
      // pitch_reference = (time_new - time_start - start_ramp) * ramp / 1000.0;
    }
    else {
      // pitch_reference = end_pitch;
    }
    // Force zero for test if needed:
    // pitch_reference = 0; 

    // 4. Update Filter (Runs the EKF)
    // This updates u, w, q, and theta based on IMU, VI0, and servo_angle
    kulibrie.update_filter(dt);

    // 5. Calculate Errors
    // Use 'theta' from EKF instead of old 'pitch'
    pitch_error = theta - pitch_reference; 

    // 6. Control Logic
    control_filter.update(servo_angle_rad);
    filtered_control = control_filter.get_filtered();

    // 7. Actuation
    // Only actuate if within controller time window
    if ((time_new < time_start + end_controller) && (time_new > time_start + start_controller)){
      pitch_error_i = pitch_error_i + pitch_error * dt;
      move_servo();
      set_voltage(); 
    } else {
      analogWrite(kulibrie.PWM_A, 0);
      analogWrite(kulibrie.PWM_B, 0);
    }
    
    // Debug
    // Serial.println(servo_angle_deg);
    Serial.print("Pitch angle: "); // Prints the text without a new line
    Serial.println(theta);         // Prints the value of 'theta' and starts a new line
    Serial.print(" u: "); // Prints the text without a new line
    Serial.println(u);         // Prints the value of 'theta' and starts a new line

  } 
  else if (kulibrie.calibrate) {
    calibrate();
  } 
  else {
    // Idle
    analogWrite(kulibrie.PWM_A, 0);
    analogWrite(kulibrie.PWM_B, 0);
  }

  // 8. Telemetry
  if (kulibrie.send_telemetry_data) {
    kulibrie.send_telemetry(time_new);
  }
  if (kulibrie.send_control_data) {
    kulibrie.send_control(time_new);
  }
}

// --------------------------------------------------------------
// Helper: Flapping Cycle Timer
// --------------------------------------------------------------
void update_cycle() {
  if (millis() - cycle_timer > cycle_period) {
    cycle_timer = millis();
    cycle_counter += 1;
  }
}

// --------------------------------------------------------------
// Control Law: Move Servo
// --------------------------------------------------------------
void move_servo(){
  // Calculate Desired Angle
  // Using 'theta' (Angle) and 'q' (Rate) from EKF directly
  // Note: pitch_reference/ss_error term kept from your original code style
  
  float target_deg = (-Kp_p * theta + servo_bias - q * Kd_p - pitch_error_i * Ki_p) * rad_to_deg;

  // Saturation
  if (target_deg > (servo_max + servo_bias) * rad_to_deg) {
    target_deg = (servo_max + servo_bias) * rad_to_deg;
  } else if (target_deg < (servo_min + servo_bias) * rad_to_deg) {
    target_deg = (servo_min + servo_bias) * rad_to_deg;
  }

  servo_angle_deg = target_deg;
  servo_angle_rad = target_deg * deg_to_rad; // Update rad version for filter/math

  myServo.write((int)servo_angle_deg);
}

// --------------------------------------------------------------
// Actuator Mixer: Set Voltage
// --------------------------------------------------------------
void set_voltage() {
  update_cycle();

  analogWrite(kulibrie.PWM_A, 0);
  analogWrite(kulibrie.PWM_B, 0);

  int volt_A = 0;
  int volt_B = 0;

  // Logic using V0, dV, ds globals (kept for compatibility)
  if (millis() - cycle_timer > cycle_period * (0.5 - (ds + ds_bias))) {
    digitalWrite(kulibrie.DIREC_A, LOW);

    if (cycle_counter < 10) {
      volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/10.0*(VI0/3.7*255)));
    } else {
      volt_A = min((int)((-(V0+V0_bias) + VI0 - (dV+dV_bias))/3.7*255), (int)255);
    }
  } else {
    digitalWrite(kulibrie.DIREC_A, HIGH);
    
    if (cycle_counter < 10) {
      volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/10.0*(VI0/3.7*255)));
    } else {
      volt_A = min((int)(((V0+V0_bias) + VI0 - (dV+dV_bias))/3.7*255), (int)255);
    }
  }

  if (millis() - cycle_timer > cycle_period * (0.5 + (ds + ds_bias))) {
    digitalWrite(kulibrie.DIREC_B, HIGH);

    if (cycle_counter < 10) {
      volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/10.0*(VI0/3.7*255)));
    } else {
      volt_B = min((int)((-(V0+V0_bias) + VI0 + (dV+dV_bias))/3.7*255), (int)255);
    }
  } else {
    digitalWrite(kulibrie.DIREC_B, LOW);

    if (cycle_counter < 10) {
      volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/10.0*(VI0/3.7*255)));
    } else {
      volt_B = min((int)(((V0+V0_bias) + VI0 + (dV+dV_bias))/3.7*255), (int)255);
    }
  }
  
  analogWrite(kulibrie.PWM_A, volt_A);
  analogWrite(kulibrie.PWM_B, volt_B);
}

// --------------------------------------------------------------
// Calibration (Legacy, preserved for 3-axis logic)
// --------------------------------------------------------------
void calibrate() {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);

    Serial.println("!!! Calibration Started !!!");

    float gx_bias = 0;
    float gy_bias = 0;
    float gz_bias = 0;

    String axis_vec[3] = {"z", "y", "x"};
    String cal_directions[3] = {"upward", "downward", "perpendicular to gravity"};
    int cal_indices[3] = {2, 1, 0};
    
    String ax_qq;
    String direc;

    for (int qq=0; qq<3; qq++) {
        ax_qq = axis_vec[qq];
        Matrix<3> ax_offsets;
        Serial.println("-------------------------");

        for (int direc_ii=0; direc_ii<3; direc_ii++) {
            time_old = millis();
            direc = cal_directions[direc_ii];

            Serial.print("Turn: "); Serial.print(ax_qq); Serial.print(" -> "); Serial.println(direc);
            
            float meas = 0;
            int i = 0;
            unsigned long start_time = millis();

            while (i < NB_MEASUREMENTS_CALIBRATIONS) {
              time_new = millis();
              dt = (time_new - time_old)/1000.0;
              time_old = time_new;

              kulibrie.update_filter(dt);
              
              if (millis() - start_time > 5000) {
                set_voltage(); // Keep motors running for vibration noise cal

                gx_bias += kulibrie.readFloatGyroX();
                gy_bias += kulibrie.readFloatGyroY();
                gz_bias += kulibrie.readFloatGyroZ();
                
                if ((i+1)%100 == 0) Serial.print(".");

                switch(qq) {
                    case 0: meas += kulibrie.readFloatAccelZ(); break;
                    case 1: meas += kulibrie.readFloatAccelY(); break;
                    case 2: meas += kulibrie.readFloatAccelX(); break;
                }   
                i++;
                delay(5);
              }
            }
            analogWrite(kulibrie.PWM_A, 0);
            analogWrite(kulibrie.PWM_B, 0);

            ax_offsets(direc_ii) = meas / NB_MEASUREMENTS_CALIBRATIONS;
            Serial.println();
            Serial.print("Avg: "); Serial.println(ax_offsets(direc_ii));
            cycle_counter = 0;
        }

        Matrix<3> y = {1, -1, 0};
        if (qq == 1) y = {0, 0, 0}; // Pitch axis logic from original file

        kulibrie.curve_fit(ax_offsets, y, cal_indices[qq]);
    }
    
    // Save to EKF
    kulibrie.set_calibration();

    Serial.println("!!! Calibration Done !!!");
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);

    kulibrie.calibrate = false;
}

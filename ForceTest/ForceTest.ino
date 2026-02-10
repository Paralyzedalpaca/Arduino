// Import the necessary packages

#include <filters.h>
#include <AutoPID.h>
#include <BasicLinearAlgebra.h>     // For matrix operations
#include <KUlibrie.h>



// Initiate the controller actions
float VI0 = 0;//1.75;
float V0 = 0.0;
float dV = 0.0;
float ds = 0.0;

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

// Conversion from degrees to radians
#define D2R asin(1)/90

// PID variables (need to be tuned)
// Roll
float Kp_r = 0.9;
float Ki_r = 0.2;
float Kd_r = 0.3;

// Pitch
float Kp_p = 1.5;
float Ki_p = 0.2;
float Kd_p = 0.7;

// Yaw
float Kp_y = -0.2;
float Ki_y = -0.3;
float Kd_y = 0.0;

// Reference attitude
float ref_roll = 0*D2R;
float ref_pitch = 0*D2R;
float ref_yaw_rate = 6*D2R; // To limit the drift

// Variables to store the attitude estimation
float roll, pitch, yaw_rate;

// KUlibrie object to control the hardware settings, bluetooth communication, ...
KUlibrie kulibrie(&roll, &pitch, &yaw_rate, &VI0, &V0, &dV, &ds, &ref_roll, &ref_pitch, &ref_yaw_rate);

// PID controllers
AutoPID roll_control(&roll, &ref_roll, &dV, DV_MIN, DV_MAX, Kp_r, Ki_r, Kd_r);
AutoPID pitch_control(&pitch, &ref_pitch, &V0, V0_MIN, V0_MAX, Kp_p, Ki_p, Kd_p);
AutoPID yaw_control(&yaw_rate, &ref_yaw_rate, &ds, DS_MIN, DS_MAX, Kp_y, Ki_y, Kd_y);


unsigned long time_old, time_new;               // Variables to calculate the timestep
float dt;                                       // Stores the last timestep
unsigned long cycle_timer;                      // Stores the time in the latest flapping cycle
int cycle_counter = 0;                          // Stores how many flapping cycles have passed (to create a rampup during the first 10 cycles)
float flapping_frequency = 20;                  // Stores the flapping frequency
float cycle_period = 1000/flapping_frequency;   // Stores the time of one flapping cycle

// --------
// Run once
// --------
void setup() {
  /*
  This function is run once
  */
  // Start Serial to be able to print things (for debug purposes)
  Serial.begin(115200);
  // while (!Serial);
  
  // Setup KUlibrie hardware (leds, start IMU, setup bluetooth, ...)
  kulibrie.setup_hardware();
  // kulibrie.calibrate = true;
  //kulibrie.control = true;
  
  time_old = millis();
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
    // Run this loop only when the drone is flying
    Serial.println(pitch);
    // Update timestep
    time_new = millis();
    dt = (time_new - time_old)/1000.0;
    time_old = time_new;

    // Update attitude estimation
    kulibrie.update_filter(dt);

    // Set timesteps for pid controllers
    roll_control.setTimeStep(dt);
    pitch_control.setTimeStep(dt);
    yaw_control.setTimeStep(dt);

    // Run controllers
    // roll_control.run();
    // pitch_control.run();
    // yaw_control.run();
    // print applied voltage
    Serial.println(VI0);
     // Set the desired voltages to the motors of the KUlibrie 
    set_voltage();
  } else if (kulibrie.calibrate) {
    // Run this loop only when the drone is calibrating
    //calibrate();
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


// -------------------------------------------------------
// Function that applies the desired voltage to the motors
// -------------------------------------------------------
void set_voltage() {
  update_cycle();

  // This is a second possibility (the difference lies in the yaw control) (GEEN ds Control!)
  analogWrite(kulibrie.PWM_A, 0);
  analogWrite(kulibrie.PWM_B, 0);

  int volt_A = 0;
  int volt_B = 0;

  if (millis() - cycle_timer > cycle_period * 0.5) {
     digitalWrite(kulibrie.DIREC_A, LOW);
     digitalWrite(kulibrie.DIREC_B, HIGH);

     if (cycle_counter < 10) {
       volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
       volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
     } else {
       volt_A = min((int)((VI0)/3.7*255), (int)255);
       volt_B = min((int)((VI0)/3.7*255), (int)255);
     }
   } else {
     digitalWrite(kulibrie.DIREC_A, HIGH);
     digitalWrite(kulibrie.DIREC_B, LOW);

     if (cycle_counter < 10) {
       volt_A = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
       volt_B = min((int)(VI0/3.7*255), (int)(cycle_counter/10*(VI0/3.7*255)));
     } else {
       volt_A = min((int)((VI0)/3.7*255), (int)255);
       volt_B = min((int)((VI0)/3.7*255), (int)255);
     }
   }

  analogWrite(kulibrie.PWM_A, volt_A);
  analogWrite(kulibrie.PWM_B, volt_B);
}

#include <SoftwareSerial.h>

// Create a software serial port on pins 10 (RX) and 11 (TX)
SoftwareSerial controllerSerial(10, 11);  // RX, TX

// Init buttons:
#define START_STABILIZATION_BUTTON 2
#define REDO_STAGE_INIT_BUTTON     3
#define STEP_STAGE_POS             4
#define STEP_STAGE_NEG             5
#define GREEN_LED                  6
#define YELLOW_LED                 7
#define RED_LED                    8

// ADC port:
#define ANALOG_PIN A0

// global variables
uint16_t target_adc_value = 0;
uint16_t current_adc_value = 0;
float slope = 0.0f;
float adjustment_angle = 0.0f;
float Ki = 0.0f;   // Integral gain
float integral_error = 0.0f;

// global bools
bool stabilization_on = false;
bool stabilization_start = false;
bool program_failed = false;

// CONSTANTS:
#define AVERAGING_TIME_BEGINING_MS     20000
#define WAIT_TIME_CONTROL_LOOP_MS      800
#define AVERAGING_TIME_ACQUISITION_MS  500
#define SLOPE_CALC_ANGLE               1.5f
#define MANUAL_ADJ_ANGLE               1.0f
#define FAIL_LOWER_THRES               30
#define FAIL_HIGHER_THRES              900
const float MIN_SLOPE = 0.01f;
const float MAX_ANGLE_STEP = 0.4f;  // max allowed angle adjustment per step
const float MIN_ANGLE_STEP = 0.005f;
// PI CONTROLLER CONSTANTS:
#define MAX_INTEGRAL                   400.0f // in ADC steps!!
#define KI_FACTOR                      10.0f
#define EPSILON                        0.00001f  // for safe float comparison

#define MOVE_TO_ANGLE_AT_BEGINNING     54
#define ABS_MOVE_DELAY_MS              16000

/*
  =============
      SETUP
  =============
*/
void setup() {
  analogReference(INTERNAL);  // Use 1.1V internal reference
  delay(500);                 // Allow time for reference to stabilize

  controllerSerial.begin(57600);   // Controller communication

  // Initialize button pins
  pinMode(START_STABILIZATION_BUTTON, INPUT_PULLUP);
  pinMode(REDO_STAGE_INIT_BUTTON, INPUT_PULLUP);
  pinMode(STEP_STAGE_POS, INPUT_PULLUP);
  pinMode(STEP_STAGE_NEG, INPUT_PULLUP);

  // Init the LEDs
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  //initialize_controller();
  // move to angle
  //move_to_angle(MOVE_TO_ANGLE_AT_BEGINNING);
  //delay(2000);

  // Set the deadband to a high value to avoid dithering 
  //set_controller_deadband();
}


/*
  =================
      MAIN LOOP
  =================
*/
void loop() {
  if (!program_failed) {
    if (!stabilization_on) {
      if (!stabilization_start) {
        /*
        ======================
            SETTINGS PART
        ======================
        */

        // Yellow
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(YELLOW_LED, HIGH);
        digitalWrite(RED_LED, LOW);

        // Button press to redo the stage init:
        if (digitalRead(REDO_STAGE_INIT_BUTTON) == LOW) {
          initialize_controller();
          // wait for some time while the controller moves
          blinkLed(ABS_MOVE_DELAY_MS, YELLOW_LED);

          move_to_angle(MOVE_TO_ANGLE_AT_BEGINNING);
          // wait for some time while the controller moves
          blinkLed(ABS_MOVE_DELAY_MS, YELLOW_LED);

          set_controller_deadband();
        }

        // Button press to start the stabilization init
        if (digitalRead(START_STABILIZATION_BUTTON) == LOW) {
          // Leave the settings part
          stabilization_start = true;
        }

        // Buttons for manual adjustment:
        if (digitalRead(STEP_STAGE_POS) == LOW) {
          // Move in positive direction:
          move_relative_angle(MANUAL_ADJ_ANGLE);
          delay(1000);
        }
        if (digitalRead(STEP_STAGE_NEG) == LOW) {
          // Move in negative direction:
          move_relative_angle(-MANUAL_ADJ_ANGLE);
          delay(1000);
        }

        delay(200);

      } else {
        /*
        ================================
            STABILIZATION INIT PART
        ================================
        */
        // Yellow & Green
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(YELLOW_LED, HIGH);
        digitalWrite(RED_LED, LOW);

        // Get the average ADC value over 20 seconds:
        target_adc_value = readAveragedADC(AVERAGING_TIME_BEGINING_MS);

        // Find the slope at the current point:
        calculate_slope();

        // Set Ki as a fraction of the slope
        Ki =  (1 / slope) / KI_FACTOR;

        // Now move on to the actual program part:
        stabilization_on = true;

        // Set up for program part:
        // Green
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(YELLOW_LED, LOW);
        digitalWrite(RED_LED, LOW);
      }

    } else {
      /*
      =====================
          PROGRAM PART
      =====================
      */

      // Get the current ADC value:
      current_adc_value = readAveragedADC(AVERAGING_TIME_ACQUISITION_MS);

      // Calculate the adjustment angle
      adjustment_angle = calc_adjustment_angle_PI(current_adc_value, target_adc_value);

      // Move the HWP accordingly
      if (fabs(adjustment_angle) > MIN_ANGLE_STEP) {
        move_relative_angle(adjustment_angle);
      }

      // Wait to overcome the Melles Griot response time and to be easy on the stage:
      delay(WAIT_TIME_CONTROL_LOOP_MS);
    }
  } else {
    /*
      =======================
          PROGRAM FAILED
      =======================
    */
    // Red
    // TODO: Different error states
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    delay(1000);
  }
}

/**
 * Blink a LED every 300ms for a specified total duration.
 *
 * :param duration_ms: The total duration in milliseconds to blink the LED.
 * :param led_pin: The pin number connected to the yellow LED.
 */
void blinkLed(unsigned long duration_ms, int led_pin) {
  unsigned long start_time = millis();
  bool led_state = false;
  unsigned long previous_blink_time = 0;

  while (millis() - start_time < duration_ms) {
    if (millis() - previous_blink_time >= 300) {
      led_state = !led_state;
      digitalWrite(led_pin, led_state ? HIGH : LOW);
      previous_blink_time = millis();
    }
  }

  digitalWrite(led_pin, LOW);  // Ensure LED is off after blinking
}

// Function with a linear controller to calculate the required adjustment angle:
float calc_adjustment_angle_P(int16_t current_adc_value, uint16_t target_adc_value) {
  static float prev_error = 0;
  static float last_adjustment = 0;

  // Calculate the current error
  float error = (float)target_adc_value - (float)current_adc_value;

  // Proportional control
  float adjustment = error / slope;

  // Limit the adjustment step
  if (adjustment > MAX_ANGLE_STEP) adjustment = MAX_ANGLE_STEP;
  if (adjustment < -MAX_ANGLE_STEP) adjustment = -MAX_ANGLE_STEP;

  prev_error = error;
  last_adjustment = adjustment;
  return adjustment;
}

// Function with a PI controller to calculate the required adjustment angle:
float calc_adjustment_angle_PI(int16_t current_adc_value, uint16_t target_adc_value) {
  // Calculate current error
  float error = (float)target_adc_value - (float)current_adc_value;

  // Update integral term
  integral_error += error;

  // Anti-windup clamp
  if (integral_error > MAX_INTEGRAL) integral_error = MAX_INTEGRAL;
  if (integral_error < -MAX_INTEGRAL) integral_error = -MAX_INTEGRAL;

  // PI control: Kp = 1 / slope
  float adjustment = (error / slope) + Ki * integral_error;

  // Clamp adjustment
  if (adjustment > MAX_ANGLE_STEP) adjustment = MAX_ANGLE_STEP;
  if (adjustment < -MAX_ANGLE_STEP) adjustment = -MAX_ANGLE_STEP;

  return adjustment;
}

// Function for value over time averaging
uint16_t readAveragedADC(unsigned long duration_ms) {
  unsigned long start = millis();
  uint32_t sum = 0;
  uint16_t count = 0;

  while (millis() - start < duration_ms) {
    sum += analogRead(ANALOG_PIN);
    ++count;
    delay(1);  // Prevent too frequent reads, adjust as needed
  }

  // Use averaged value instead of count for failsafe check
  uint16_t average = sum / count;
  if (average < FAIL_LOWER_THRES || average > FAIL_HIGHER_THRES) {
    program_failed = true;
    return 1;
  }

  return average;
}

void set_controller_deadband() {
  // Go to disabled state
  controllerSerial.print("1MM0\r\n");
  delay(2000);

  controllerSerial.print("1DB");
  controllerSerial.print(0.05f);
  controllerSerial.print("\r\n");
  delay(1000);

  // Go back to ready state
  controllerSerial.print("1MM1\r\n");
  delay(1000);
}

void calculate_slope() {
  float slope_sum = 0.0f;

  for (int i = 0; i < 3; ++i) {
    uint16_t counts_pos_direction = 0;
    uint16_t counts_neg_direction = 0;

    // Go 1.5 degrees in one direction
    move_relative_angle(SLOPE_CALC_ANGLE);
    delay(1500);
    counts_pos_direction = readAveragedADC(AVERAGING_TIME_ACQUISITION_MS);

    // Go 3 degrees in the other direction
    move_relative_angle(-2.0f * SLOPE_CALC_ANGLE);
    delay(1500);
    counts_neg_direction = readAveragedADC(AVERAGING_TIME_ACQUISITION_MS);

    // Go back to starting position
    move_relative_angle(SLOPE_CALC_ANGLE);
    delay(1500);

    float current_slope = ((float)counts_pos_direction - (float)counts_neg_direction) / (2.0f * SLOPE_CALC_ANGLE);
    slope_sum += current_slope;
  }

  slope = slope_sum / 3.0f;

  if (fabs(slope) < EPSILON) {
    // Set to failed program status
    program_failed = true;
  }
}

// Move the HWP by a relative angle (deg)
void move_relative_angle(float angle_adjustment) {
  controllerSerial.print("1PR");
  controllerSerial.print(angle_adjustment);
  controllerSerial.print("\r\n");
  delay(10);
}

// Move to an absolute angle
void move_to_angle(float angle) {
  controllerSerial.print("1PA");
  controllerSerial.print(angle);
  controllerSerial.print("\r\n");
}

void initialize_controller() {
  controllerSerial.print("1OR\r\n");
}

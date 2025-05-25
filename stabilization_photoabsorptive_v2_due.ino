#define START_STABILIZATION_BUTTON 2
#define REDO_STAGE_INIT_BUTTON     3
#define STEP_STAGE_POS             4
#define STEP_STAGE_NEG             5
#define GREEN_LED                  6
#define YELLOW_LED                 7
#define RED_LED                    8

#define ANALOG_PIN A0  // Channel 7

// Constants
#define AVERAGING_TIME_BEGINING_MS     2000
#define WAIT_TIME_CONTROL_LOOP_MS      100
#define AVERAGING_TIME_ACQUISITION_MS  900
#define SLOPE_CALC_ANGLE               1.5f
#define MANUAL_ADJ_ANGLE               1.0f
#define FAIL_LOWER_THRES               30
#define FAIL_HIGHER_THRES              1200
const float MIN_SLOPE = 0.0001f;
const float MAX_ANGLE_STEP = 0.4f;
const float MIN_ANGLE_STEP = 0.004f;

#define MAX_INTEGRAL                   400.0f
#define KI_FACTOR                      10.0f
#define EPSILON                        0.0001f

#define MOVE_TO_ANGLE_AT_BEGINNING     54
#define ABS_MOVE_DELAY_MS              16000

// Global variables
uint16_t target_adc_value = 0;
uint16_t current_adc_value = 0;
float slope = 0.0f;
float adjustment_angle = 0.0f;
float Ki = 0.0f;
float integral_error = 0.0f;

bool stabilization_on = false;
bool stabilization_start = false;
bool program_failed = false;

void setup() {
  // Serial for debug output (via USB)
  //Serial.begin(115200);
  //while (!Serial);  // Wait for USB serial to be ready

  // Serial1 for controller communication (pins 18/19)
  Serial1.begin(57600);

  pinMode(START_STABILIZATION_BUTTON, INPUT_PULLUP);
  pinMode(REDO_STAGE_INIT_BUTTON, INPUT_PULLUP);
  pinMode(STEP_STAGE_POS, INPUT_PULLUP);
  pinMode(STEP_STAGE_NEG, INPUT_PULLUP);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // Setup ADC for fast read
  ADC->ADC_MR |= ADC_MR_FREERUN_ON | ADC_MR_TRGEN_DIS | ADC_MR_LOWRES_BITS_12;
  ADC->ADC_CHER = ADC_CHER_CH7;
  ADC->ADC_CR = ADC_CR_START;
}

void loop() {
  if (!program_failed) {
    if (!stabilization_on) {
      if (!stabilization_start) {
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(YELLOW_LED, HIGH);
        digitalWrite(RED_LED, LOW);

        if (digitalRead(REDO_STAGE_INIT_BUTTON) == LOW) {
          initialize_controller();
          blinkLed(ABS_MOVE_DELAY_MS, YELLOW_LED);
          move_to_angle(MOVE_TO_ANGLE_AT_BEGINNING);
          blinkLed(ABS_MOVE_DELAY_MS, YELLOW_LED);
          set_controller_deadband();
        }

        if (digitalRead(START_STABILIZATION_BUTTON) == LOW) {
          stabilization_start = true;
        }

        if (digitalRead(STEP_STAGE_POS) == LOW) {
          move_relative_angle(MANUAL_ADJ_ANGLE);
          delay(1000);
        }
        if (digitalRead(STEP_STAGE_NEG) == LOW) {
          move_relative_angle(-MANUAL_ADJ_ANGLE);
          delay(1000);
        }

        delay(200);

      } else {
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(YELLOW_LED, HIGH);
        digitalWrite(RED_LED, LOW);

        target_adc_value = readAveragedADC(AVERAGING_TIME_BEGINING_MS);
        calculate_slope();
        Ki = (1.0f / slope) / KI_FACTOR;
        stabilization_on = true;

        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(YELLOW_LED, LOW);
        digitalWrite(RED_LED, LOW);
        //Serial.println("Stabilization started");
      }

    } else {

      current_adc_value = readAveragedADC(AVERAGING_TIME_ACQUISITION_MS);

      adjustment_angle = calc_adjustment_angle_PI(current_adc_value, target_adc_value);
      if (fabs(adjustment_angle) > MIN_ANGLE_STEP) {
        move_relative_angle(adjustment_angle);
      }
      delay(WAIT_TIME_CONTROL_LOOP_MS);
    }
  } else {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    //Serial.println("Program failed - check sensor range or slope");
    delay(1000);
  }
}

// =========================
// Helper Functions
// =========================

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

  digitalWrite(led_pin, LOW);
}

float calc_adjustment_angle_PI(int16_t current_adc_value, uint16_t target_adc_value) {
  float error = (float)target_adc_value - (float)current_adc_value;
  integral_error += error;
  if (integral_error > MAX_INTEGRAL) integral_error = MAX_INTEGRAL;
  if (integral_error < -MAX_INTEGRAL) integral_error = -MAX_INTEGRAL;
  float adjustment = (error / slope) + Ki * integral_error;
  if (adjustment > MAX_ANGLE_STEP) adjustment = MAX_ANGLE_STEP;
  if (adjustment < -MAX_ANGLE_STEP) adjustment = -MAX_ANGLE_STEP;
  return adjustment;
}

uint16_t readAveragedADC(unsigned long duration_ms) {
  uint32_t start_time = micros();
  uint32_t duration_us = duration_ms * 1000UL;
  uint64_t sum = 0;
  uint32_t count = 0;

  while ((micros() - start_time) < duration_us) {
    while ((ADC->ADC_ISR & ADC_ISR_EOC7) == 0);  // Wait for ADC ready
    uint16_t value = ADC->ADC_CDR[7];
    sum += value;
    count++;
  }

  if (count == 0) {
    program_failed = true;
    return 1;
  }

  uint16_t average = (uint16_t)(sum / count);

  if (average < FAIL_LOWER_THRES || average > FAIL_HIGHER_THRES) {
    program_failed = true;
    return 1;
  }

  return average;
}

void set_controller_deadband() {
  Serial1.print("1MM0\r\n");
  delay(2000);
  Serial1.print("1DB");
  Serial1.print(0.05f);
  Serial1.print("\r\n");
  delay(1000);
  Serial1.print("1MM1\r\n");
  delay(1000);
}

void calculate_slope() {
  float slope_sum = 0.0f;
  for (int i = 0; i < 3; ++i) {
    move_relative_angle(SLOPE_CALC_ANGLE);
    delay(1500);
    uint16_t counts_pos = readAveragedADC(AVERAGING_TIME_ACQUISITION_MS);

    move_relative_angle(-2.0f * SLOPE_CALC_ANGLE);
    delay(1500);
    uint16_t counts_neg = readAveragedADC(AVERAGING_TIME_ACQUISITION_MS);

    move_relative_angle(SLOPE_CALC_ANGLE);
    delay(1500);

    float current_slope = ((float)counts_pos - (float)counts_neg) / (2.0f * SLOPE_CALC_ANGLE);
    slope_sum += current_slope;
  }

  slope = slope_sum / 3.0f;
  if (fabs(slope) < EPSILON) {
    program_failed = true;
  }
}

void move_relative_angle(float angle_adjustment) {
  Serial1.print("1PR");
  Serial1.print(angle_adjustment);
  Serial1.print("\r\n");
  delay(10);
}

void move_to_angle(float angle) {
  Serial1.print("1PA");
  Serial1.print(angle);
  Serial1.print("\r\n");
}

void initialize_controller() {
  Serial1.print("1OR\r\n");
}

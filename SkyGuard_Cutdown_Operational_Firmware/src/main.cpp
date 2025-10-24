#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_Sensor.h"
#include "pins.h"

// Uncomment to arm the timer instantly and not wait for the pressure to drop at launch
#define ARM_TIMER_INSTANTLY

// Firmware Version
const uint8_t FIRMWARE_MAJOR_VERSION = 1;
const uint8_t FIRMWARE_MINOR_VERSION = 1;

// Objects
Adafruit_BMP3XX pressure_sensor;
Servo releaseServo;

// Globals
enum states {S_INITIALIZE, S_ERROR, S_RUNCYCLE,
             S_DOCUTDOWN, S_ARMTIMER, S_FLIGHTCOMPLETE};

uint8_t current_state = S_INITIALIZE;

bool timer_armed = false;
uint16_t starting_pressure_hPa = 2000;
uint16_t pressure_arming_delta_hPa = 20; // Roughly 550 ft above launch
uint8_t SERVO_RELEASE_POSITION = 105;
uint8_t SERVO_CAPTURE_POSITION = 5;
uint8_t SERVO_WIGGLE_POSITION = 45;
uint32_t timer_armed_ms = 0;
uint8_t SAMPLES_BELOW_BEFORE_CUT = 3; // How many samples below the threshold before we cut
uint32_t TURN_OFF_SERVO_AFTER_MS = 60000; // Time after cutdown to turn off servo to save wear/tear
/*
 * HELPERS
 */
uint16_t getCutdownPressurehPa()
{
  /*
   * Return the cutdown pressure in hPa from the DIP switch settings.
   * If the pressure feature is disabled we return a pressure of 0.
   */
  uint8_t set_index = ! digitalRead(PIN_PRESSURE_BIT0);
  set_index = (set_index << 1) | ! digitalRead(PIN_PRESSURE_BIT1);
  set_index = (set_index << 1) | ! digitalRead(PIN_PRESSURE_BIT2);
  set_index = (set_index << 1) | ! digitalRead(PIN_PRESSURE_BIT3);

  uint16_t pressures_hPa[16] = {0, 10, 100, 150, 200, 250, 300,
                                350, 400, 450, 500, 600, 700,
                                800, 900, 1000};
  return pressures_hPa[set_index];
}

uint16_t getCutdownTimeMinutes()
{
  /*
   * Return the cutdown time in minutes from the DIP switch settings.
   * If the time feature is disabled we return a time of 0.
   */
  uint8_t set_index = ! digitalRead(PIN_TIME_BIT0);
  set_index = (set_index << 1) | ! digitalRead(PIN_TIME_BIT1);
  set_index = (set_index << 1) | ! digitalRead(PIN_TIME_BIT2);
  set_index = (set_index << 1) | ! digitalRead(PIN_TIME_BIT3);

  uint16_t times_minutes[16] = {0, 1, 2, 5, 10, 20, 30, 40,
                                50, 60, 70, 80, 90, 100, 110,
                                120};
  return times_minutes[set_index];
}

uint16_t getPressurehPa()
{
  /*
   * Read the pressure from the sensor in hPa. If the sensor malfunctions, we
   * return 2000 hPa so we don't cut too early and hope the user has a time
   * backup. Don't want to error and stall the whole device because of a single
   * bad sensor or bad reading.
   */
  if (! pressure_sensor.performReading())
  {
    return 2000;
  }
  
  return round(pressure_sensor.pressure / 100.0);
}

/*
 * STATES
 */

uint8_t stateInitialize()
{
  /*
   * Sets up sensor, pins, etc for operation of device
   */

  // Configure debug serial
  Serial.begin(9600);

  Wire.begin();

  // Configure the DIP switch pins
  pinMode(PIN_PRESSURE_BIT0, INPUT);
  pinMode(PIN_PRESSURE_BIT1, INPUT);
  pinMode(PIN_PRESSURE_BIT2, INPUT);
  pinMode(PIN_PRESSURE_BIT3, INPUT);
  pinMode(PIN_TIME_BIT0, INPUT);
  pinMode(PIN_TIME_BIT1, INPUT);
  pinMode(PIN_TIME_BIT2, INPUT);
  pinMode(PIN_TIME_BIT3, INPUT);

  // Configure servo
  releaseServo.attach(PIN_SERVO);
  releaseServo.write(SERVO_CAPTURE_POSITION);

  // Configure the pressure sensor
  bool good_bme_start = false;
  for (int i=0; i<5; i++)
  {
    if (pressure_sensor.begin_I2C(0x76))
    {
      good_bme_start = true;
      break;
    }
    else
    {
      Serial.println("ERROR - Pressure Sensor Startup Failed");
      good_bme_start = false;
      delay(50);
    }
  }

  if (!good_bme_start)
  {
    return S_ERROR;
  }

  // Set up oversampling and filter initialization
  pressure_sensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  pressure_sensor.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  pressure_sensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  pressure_sensor.setOutputDataRate(BMP3_ODR_50_HZ);


  // Setup LED pins and blink to confirm that we are booted up and configured
  pinMode(PIN_YELLOW_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);
  digitalWrite(PIN_YELLOW_LED, HIGH);
  digitalWrite(PIN_GREEN_LED, LOW);
  for (uint8_t i=0; i<5; i++)
  {
    digitalWrite(PIN_YELLOW_LED, !digitalRead(PIN_YELLOW_LED));
    digitalWrite(PIN_GREEN_LED, !digitalRead(PIN_GREEN_LED));
    delay(500);
  }
  digitalWrite(PIN_YELLOW_LED, LOW);
  digitalWrite(PIN_GREEN_LED, LOW);

  // Verify that we can get valid pressure readings. If not, error out, if so go on.
  // Also sets the instrument starting pressure for arming of the timer function.
  bool good_pressure_reading = false;
  uint16_t last_good_pressure = 2000;
  for (uint8_t i=0; i<5; i++)
  {
    uint16_t read_pressure = getPressurehPa();
    if (read_pressure != 2000)
    {
      last_good_pressure = read_pressure;
      good_pressure_reading = true;
    }
    delay(50);
  }

  if (!good_pressure_reading)
  {
    return S_ERROR;
  }
  else
  {
    starting_pressure_hPa = last_good_pressure;
  }

  // If the starting pressure is below 500 hPa we must have reset and we'll assume the
  // starting pressure was actually 1000 hPa. This makes sure the timer arms again.
  if (starting_pressure_hPa < 500)
  {
    starting_pressure_hPa = 1000;
  }
 
  // Wiggle the servo to make sure it is working
  releaseServo.write(SERVO_WIGGLE_POSITION);
  delay(2000);
  releaseServo.write(SERVO_CAPTURE_POSITION);
  delay(1000);
  releaseServo.detach();

  // If we are arming instantly, do it
  #ifdef ARM_TIMER_INSTANTLY
  return S_ARMTIMER;
  #endif

  return S_RUNCYCLE;
}

uint8_t stateError()
{
  /*
   * We are in an error, spin forever with all LEDs off. This is not good in flight
   * so we should only get here from a bad startup.
   */
  Serial.println("Error - shutting down");
  digitalWrite(PIN_YELLOW_LED, LOW);
  digitalWrite(PIN_GREEN_LED, LOW);
  delay(100);
  return S_ERROR;
}

uint8_t stateRunCycle()
{
  /*
   * Runs a cycle of checking for cut conditions and operating the device.
   */

  // To verify watchdog works you can uncomment this delay and make sure the system resets itself.
  //delay(5000);

  static uint8_t nreadings_below_threshold = 0;

  // Toggle Ready Green LED for heartbeat indication
  //digitalWrite(PIN_GREEN_LED, !digitalRead(PIN_GREEN_LED));
  digitalWrite(PIN_GREEN_LED, HIGH);
  delay(50);
  digitalWrite(PIN_GREEN_LED, LOW);
  delay(950);

  // Read switches for current settings
  uint16_t pressure_criteria_hPa = getCutdownPressurehPa();
  uint16_t time_criteria_minutes = getCutdownTimeMinutes();

  // Read the current pressure
  uint16_t current_pressure_hPa = getPressurehPa();

  // Show system state
  static uint8_t header_counter = 0 ;
  if(header_counter%10==0){
  Serial.println("StartPress    CurrentPress  SetPress      SetTime       TimerArmed    ET");
  }
  char buffer[150];
  sprintf(buffer, "%-14u%-14u%-14u%-14u%-14d%-14lu", //"%-14u\t%-14u\t%-14u\t%-14u\t%-14d\t%-14lu", 
          starting_pressure_hPa, 
          current_pressure_hPa, 
          pressure_criteria_hPa, 
          time_criteria_minutes, 
          timer_armed, 
          (millis() - timer_armed_ms) / 1000);
  Serial.println(buffer);
  header_counter+=1;

  // Check if we need to arm the timer - this is when the time setting is not 0 and
  // the timer is not already enabled and we have met the launch criteria
  if (time_criteria_minutes == 0)
  {
    // Not enabled! Keep disarmed and move on
    timer_armed = false;
  }
  else
  {
    // Enabled, see if we are not already armed and have met criteria
    if (timer_armed){}
    else
    {
      // Make sure we are going up
      if (starting_pressure_hPa > current_pressure_hPa)
      {
        if ((starting_pressure_hPa - current_pressure_hPa) >= pressure_arming_delta_hPa)
        {
          return S_ARMTIMER;
        }
      }
    }
  }

  // Check if we have reached the pressure cutdown criteria
  if (current_pressure_hPa <= pressure_criteria_hPa)
  {
    nreadings_below_threshold += 1;
  }
  else
  {
    nreadings_below_threshold = 0;
  }

  if (nreadings_below_threshold >= SAMPLES_BELOW_BEFORE_CUT)
  {
    return S_DOCUTDOWN;
  }

  // Check if we have reached the time cutdown criteria
  if (timer_armed)
  {
    uint32_t elapsed_time_seconds = (millis() - timer_armed_ms) / 1000;
    uint32_t time_criteria_seconds = time_criteria_minutes * 60;
    if (elapsed_time_seconds >= time_criteria_seconds)
    {
      return S_DOCUTDOWN;
    }
  }

  // If no other actions have been taken, just come back here
  return S_RUNCYCLE;
}

uint8_t stateDoCutdown()
{
  /*
   * Moves the servo to the release position to drop the payload and goes to flight
   * complete state.
   */

  Serial.println("Activating release mechanism");

  // Move servo to the release position to drop
  releaseServo.attach(PIN_SERVO);
  delay(500);
  releaseServo.write(SERVO_RELEASE_POSITION);
  delay(500);

  return S_FLIGHTCOMPLETE;
}

uint8_t stateArmTimer()
{
  /*
   * We have reached launch conditions and need to arm the timer by setting the
   * armed flag and storing the arming time in milliseconds.
   */

  Serial.println("Arming timer");
  timer_armed = true;
  timer_armed_ms = millis();
  digitalWrite(PIN_YELLOW_LED, HIGH);
  return S_RUNCYCLE;
}

uint8_t stateFlightComplete()
{
  /*
   * We have released the payload and there isn't anything to do but blink the
   * lights in case it helps us find the package and deplete the battery.
   */
  static uint32_t complete_ms = millis();
  bool servo_on = true;
  Serial.println("Flight complete - shutting down");
  digitalWrite(PIN_YELLOW_LED, HIGH);
  digitalWrite(PIN_GREEN_LED, LOW);
  while(1)
  {
    if (((millis() - complete_ms) > TURN_OFF_SERVO_AFTER_MS) && servo_on)
    {
      releaseServo.detach();
      servo_on = false;
    }
    digitalWrite(PIN_YELLOW_LED, !digitalRead(PIN_YELLOW_LED));
    digitalWrite(PIN_GREEN_LED, !digitalRead(PIN_GREEN_LED));
    delay(500);
  }
  return S_FLIGHTCOMPLETE;
}

/*
 * Setup - all handled by initialize state of state machine
 */
void setup(){}

/*
 * Main Loop
 */
void loop()
{
  // Follow wherever the state machine is supposed to go next
  switch (current_state)
  {
    case S_INITIALIZE:
      current_state = stateInitialize();
      break;
    case S_ERROR:
      current_state = stateError();
      break;
    case S_RUNCYCLE:
      current_state = stateRunCycle();
      break;
    case S_DOCUTDOWN:
      current_state = stateDoCutdown();
      break;
    case S_ARMTIMER:
      current_state = stateArmTimer();
      break;
    case S_FLIGHTCOMPLETE:
      current_state = stateFlightComplete();
      break;
    default:
      current_state = S_RUNCYCLE;
  }
}

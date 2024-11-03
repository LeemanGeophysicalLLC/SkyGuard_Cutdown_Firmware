#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#include "Adafruit_BMP3XX.h"
#include "Adafruit_Sensor.h"
#include "pins.h"

// Objects
Adafruit_BMP3XX pressure_sensor;
Servo releaseServo;

// Servo Config
uint8_t SERVO_RELEASE_POSITION = 0;
uint8_t SERVO_CAPTURE_POSITION = 90;

/*
 * HELPERS
 */
uint16_t getCutdownPressurehPa()
{
  /*
   * Return the cutdown pressure in hPa from the DIP switch settings.
   * If the pressure feature is disabled we return a pressure of 0.
   */
  uint8_t set_index = digitalRead(PIN_PRESSURE_BIT0);
  set_index = (set_index << 1) | digitalRead(PIN_PRESSURE_BIT1);
  set_index = (set_index << 1) | digitalRead(PIN_PRESSURE_BIT2);
  set_index = (set_index << 1) | digitalRead(PIN_PRESSURE_BIT3);

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
  uint8_t set_index = digitalRead(PIN_TIME_BIT0);
  set_index = (set_index << 1) | digitalRead(PIN_TIME_BIT1);
  set_index = (set_index << 1) | digitalRead(PIN_TIME_BIT2);
  set_index = (set_index << 1) | digitalRead(PIN_TIME_BIT3);

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

void setup()
{
   // Configure debug serial
  Serial.begin(9600);

  // Configure the DIP switch pins
  pinMode(PIN_PRESSURE_BIT0, INPUT);
  pinMode(PIN_PRESSURE_BIT1, INPUT);
  pinMode(PIN_PRESSURE_BIT2, INPUT);
  pinMode(PIN_PRESSURE_BIT3, INPUT);
  pinMode(PIN_TIME_BIT0, INPUT);
  pinMode(PIN_TIME_BIT1, INPUT);
  pinMode(PIN_TIME_BIT2, INPUT);
  pinMode(PIN_TIME_BIT3, INPUT);

  // Servo
  Serial.println("Moving Servo");
  releaseServo.attach(PIN_SERVO);
  releaseServo.write(SERVO_RELEASE_POSITION);
  delay(5000);
  releaseServo.write(SERVO_CAPTURE_POSITION);

  // Configure the pressure sensor
  if (!pressure_sensor.begin_I2C())
  {
    Serial.println("ERROR - Pressure Sensor Startup Failed");
    while(1){}
  }

  // Set up oversampling and filter initialization
  pressure_sensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  pressure_sensor.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  pressure_sensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  pressure_sensor.setOutputDataRate(BMP3_ODR_50_HZ);

  // Setup LED pins and blink to confirm that we are booted up and configured
  pinMode(PIN_YELLOW_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);

  Serial.println("Red LED");
  digitalWrite(PIN_YELLOW_LED, HIGH);
  digitalWrite(PIN_GREEN_LED, LOW);
  delay(1000);
  digitalWrite(PIN_YELLOW_LED, LOW);
  delay(500);
  Serial.println("Green LED");
  digitalWrite(PIN_GREEN_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_GREEN_LED, LOW);

  // Check Pressure Sensor
  Serial.println("Checking pressure sensor");
  for (uint8_t i=0; i<5; i++)
  {
    Serial.println(getPressurehPa());
  }

  Serial.println("Checking DIP Switches");
  delay(2000);
}

void loop()
{
  // Check dip switch function
  Serial.print(digitalRead(PIN_PRESSURE_BIT0));
  Serial.print("|");
  Serial.print(digitalRead(PIN_PRESSURE_BIT1));
  Serial.print("|");
  Serial.print(digitalRead(PIN_PRESSURE_BIT2));
  Serial.print("|");
  Serial.print(digitalRead(PIN_PRESSURE_BIT3));
  Serial.print("|");
  Serial.print(digitalRead(PIN_TIME_BIT0));
  Serial.print("|");
  Serial.print(digitalRead(PIN_TIME_BIT1));
  Serial.print("|");
  Serial.print(digitalRead(PIN_TIME_BIT2));
  Serial.print("|");
  Serial.print(digitalRead(PIN_TIME_BIT3));
  Serial.print("|");
  Serial.print(getCutdownPressurehPa());
  Serial.print("|");
  Serial.print(getCutdownTimeMinutes());
  Serial.println();
  delay(100);
}

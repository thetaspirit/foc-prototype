#include <Arduino.h>

//    FILE: AS5600L_set_address.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//     URL: https://github.com/RobTillaart/AS5600
//
//  Examples may use AS5600 or AS5600L devices.
//  Check if your sensor matches the one used in the example.
//  Optionally adjust the code.

#include "AS5600.h"

//  Uncomment the line according to your sensor type
// AS5600L ASL;   //  use default Wire
AS5600 ASL; //  use default Wire

#define NUM_MOSFETS 6
#define NUM_STEPS 6

#define LINE_A 0
#define LINE_B 1
#define LINE_C 2

#define POSITIVE_LINE 0
#define NEGATIVE_LINE 1

#define LINE_A_POS 13
#define LINE_A_NEG 18
#define LINE_B_POS 12
#define LINE_B_NEG 19
#define LINE_C_POS 11
#define LINE_C_NEG 20

const int line_pins[3][2] = {
    {LINE_A_POS, LINE_A_NEG},
    {LINE_B_POS, LINE_B_NEG},
    {LINE_C_POS, LINE_C_NEG}};

const int MOSFETS[NUM_MOSFETS] = {LINE_A_POS, LINE_A_NEG, LINE_B_POS, LINE_B_NEG, LINE_C_POS, LINE_C_NEG};
const float STEPS[NUM_STEPS][3] = {
    {1, 0, -1},
    {0, 1, -1},
    {-1, 1, 0},
    {-1, 0, 1},
    {0, -1, 1},
    {1, -1, 0}};

void write_mosfet(int pin, float duty_cycle)
{
  analogWrite(pin, (int)((1.0 - duty_cycle) * 4095.0));
}

void write_line(int line, float duty_cycle)
{
  if (duty_cycle >= 0)
  {
    write_mosfet(line_pins[line][POSITIVE_LINE], duty_cycle);
    write_mosfet(line_pins[line][NEGATIVE_LINE], 0);
  }
  else
  {
    write_mosfet(line_pins[line][POSITIVE_LINE], 0);
    write_mosfet(line_pins[line][NEGATIVE_LINE], -duty_cycle);
  }
}

void write_lines(float a, float b, float c)
{
  write_line(LINE_A, a);
  write_line(LINE_B, b);
  write_line(LINE_C, c);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Set up PWM
  analogWriteResolution(12);

  for (int i = 0; i < NUM_MOSFETS; i++)
  {
    pinMode(MOSFETS[i], OUTPUT);
    write_mosfet(MOSFETS[i], 0);
  }

  Serial.begin(115200);
  Serial.println("Starting...");

  Wire.begin();
  ASL.begin(3); //  set direction pin.
}

void loop()
{
  static unsigned long last_measurement = micros();
  static unsigned long last_update = micros();
  static float duty_cycle = 0.0;
  static int step = 0;
  static float offset = 25.0;
  static float last_angle = ASL.rawAngle() * AS5600_RAW_TO_DEGREES;
  static float measured_angle = last_angle;
  static float angle = last_angle;
  static float deg_per_ms = 0.0;

  if (Serial.available())
  {
    duty_cycle = Serial.parseFloat();
    Serial.println(duty_cycle);

    while (Serial.available())
      Serial.read();
  }

  if (micros() - last_measurement >= 600)
  {
    last_measurement = micros();
    last_angle = measured_angle;
    measured_angle = ASL.rawAngle() * AS5600_RAW_TO_DEGREES;

    deg_per_ms = 0.2 * ((measured_angle - last_angle) / 600.0e-3) + 0.8 * deg_per_ms;
  }

  if (micros() - last_update >= 5)
  {
    last_update = micros();
    angle = 0.8 * fmod(angle + deg_per_ms * 5.0e-3 + 360.0, 360.0) + 0.2 * measured_angle;
    step = ((int)((angle - offset) / (60.0 / 7) + NUM_STEPS)) % NUM_STEPS;

    write_lines(
        STEPS[step][LINE_A] * duty_cycle,
        STEPS[step][LINE_B] * duty_cycle,
        STEPS[step][LINE_C] * duty_cycle);
  }

  // if (millis() % 1000 == 0)
  //   Serial.println(deg_per_ms * (1000.0 * 60 / 360.0));

  //   Serial.println(ASL.rawAngle() * AS5600_RAW_TO_DEGREES);
}

//  -- END OF FILE --
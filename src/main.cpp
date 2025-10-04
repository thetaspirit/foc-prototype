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

#define PHASE_U_SOURCE 13
#define PHASE_U_DRAIN 18
#define PHASE_V_SOURCE 12
#define PHASE_V_DRAIN 19
#define PHASE_W_SOURCE 11
#define PHASE_W_DRAIN 20

// #define PHASE_U_SOURCE 13
// #define PHASE_W_DRAIN 18
// #define PHASE_V_SOURCE 12
// #define PHASE_U_DRAIN 19
// #define PHASE_W_SOURCE 11
// #define PHASE_V_DRAIN 20

#define NUM_MOSFETS 6
#define NUM_STEPS 6

const int MOSFETS[NUM_MOSFETS] = {PHASE_U_SOURCE, PHASE_V_SOURCE, PHASE_W_SOURCE, PHASE_U_DRAIN, PHASE_V_DRAIN, PHASE_W_DRAIN};

void write_mosfet(int pin, float duty_cycle)
{
    analogWrite(pin, (int)((1 - duty_cycle) * 4095.0));
}

// void write_coils(float u, float v, float w)
// {
//   if (u > 0)
//   {
//     analogWrite(PHASE_U_SOURCE, (int)(u * 4095.0));
//     analogWrite(PHASE_V_DRAIN, (int)(u * 4095.0));
//   }
//   else
//   {
//     analogWrite(PHASE_V_SOURCE, (int)(u * 4095.0));
//     analogWrite(PHASE_U_DRAIN, (int)(u * 4095.0));
//   }
// }

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
  // static int step = 0;
  // static unsigned long last_step = millis();
  // static float frequency = 0.1;

  // if (Serial.available())
  // {
  //   frequency = Serial.parseFloat();

  //   while (Serial.available())
  //     Serial.read();
  // }

  // if (millis() - last_step >= 1000.0 / frequency / 6)
  // {
  //   last_step = millis();
  //   step = (step + 1) % NUM_STEPS;
  //   run_coil_state(STEPS[step]);
  // }

  delay(1000);
  Serial.print(millis());
  Serial.print("\t");
  Serial.println(ASL.rawAngle() * AS5600_RAW_TO_DEGREES);
  Serial.flush();
}

//  -- END OF FILE --
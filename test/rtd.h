/* 09:32 15/03/2023 - change triggering comment */
#ifndef RTD_H
#define RTD_H

#include "pindef.h"
#include "../lib/Common/sensors_state.h"


static void RTD_read(void);

#if defined SINGLE_BOARD
#include <Adafruit_MAX31865.h>
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(thermoCS, thermoDI, thermoDO, thermoCLK);
//  use hardware SPI, just pass in the CS pin
// Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

//SPIClass thermoSPI(thermoDI, thermoDO, thermoCLK);
//Adafruit_MAX31855 thermocouple(thermoCS, &thermoSPI);
#else
#include <max6675.h>

#endif

static inline void RTD_Init_Max31865()
{

  thermo.begin(MAX31865_3WIRE); // set to 2WIRE or 4WIRE as necessary
}

#endif

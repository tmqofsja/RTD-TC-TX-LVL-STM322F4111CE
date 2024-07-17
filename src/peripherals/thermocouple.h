/* 09:32 15/03/2023 - change triggering comment */
#ifndef THERMOCOUPLE_H
#define THERMOCOUPLE_H

#include "pindef.h"

#if defined(MAX31855) || defined(max6675) //(TC K TYPE)
  #include "Adafruit_MAX31855.h"
  SPIClass thermoSPI(thermoDI, thermoDO, thermoCLK);
  Adafruit_MAX31855 thermocouple(thermoCS, &thermoSPI);
#elif defined(MAX31865) //(RTD PT100)
  #include <Adafruit_MAX31865.h>
  #define RREF 430.0 // The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
  #define RNOMINAL 100.0 // 100.0 for PT100, 1000.0 for PT1000 // Use software SPI: CS, DI, DO, CLK
  Adafruit_MAX31865 thermo = Adafruit_MAX31865(thermoCS, thermoDI, thermoDO, thermoCLK);
#endif

static inline void thermocoupleInit(void) {
#if defined(MAX31855) || defined(MAX6675 ) //(TC K TYPE)
  thermocouple.begin();
#elif defined(MAX31865) //(RTD PT100)
  thermo.begin(MAX31865_3WIRE); // set to 2WIRE or 4WIRE as necessary
#endif
}

static inline float thermocoupleRead(void) {

  #if defined(MAX31865) //(RTD PT100)
  
     // Check and print any faults
    uint8_t fault = thermo.readFault();
    if (fault)
    {
      Serial.print("Fault 0x");
      Serial.println(fault, HEX);
      if (fault & MAX31865_FAULT_HIGHTHRESH)
      {
        Serial.println("RTD High Threshold");
      }
      if (fault & MAX31865_FAULT_LOWTHRESH)
      {
        Serial.println("RTD Low Threshold");
      }
      if (fault & MAX31865_FAULT_REFINLOW)
      {
        Serial.println("REFIN- > 0.85 x Bias");
      }
      if (fault & MAX31865_FAULT_REFINHIGH)
      { 
        Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
      }
      if (fault & MAX31865_FAULT_RTDINLOW)
      {
        Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
      }
      if (fault & MAX31865_FAULT_OVUV)
      {
        Serial.println("Under/Over voltage");
      }
      thermo.clearFault();
    }
    //Serial.println();
    delay(1000);
    return  thermo.temperature(RNOMINAL, RREF);
    //return thermocouple.readCelsius();
}
  #elif defined(MAX31855) || defined(MAX6675) //(TC K TYPE)
    return thermocouple.readCelsius();
  #endif
#endif

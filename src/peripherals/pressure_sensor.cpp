/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#include "pressure_sensor.h"
#include "pindef.h"
#include "ADS1X15.h"
#include "lcd/lcd.h"
//#include "../log.h"
#include "i2c_bus_reset.h"

#if defined SINGLE_BOARD
ADS1015 ADS(0x48);
#else
ADS1115 ADS(0x48);
#endif

float previousPressure;
float currentPressure;

void adsInit(void) {
  ADS.begin();
  ADS.setGain(0);      // 6.144 volt
  ADS.setDataRate(4);  // fast
  ADS.setMode(0);      // continuous mode
  ADS.readADC(0);      // first read to trigger
}

float getPressure(void) {  //returns sensor pressure data
  // voltageZero = 0.5V --> 25.6 (8 bit) or 102.4 (10 bit) or 2666.7 (ADS 15 bit)
  // voltageMax = 4.5V --> 230.4 (8 bit) or 921.6 (10 bit) or 24000 (ADS 15 bit)
  // range 921.6 - 102.4 = 204.8 or 819.2 or 21333.3
  // pressure gauge range 0-1.2MPa - 0-12 bar
  // 1 bar = 17.1 or 68.27 or 1777.8
  // 166-1500 =0 to 12

  getAdsError();

  previousPressure = currentPressure;
#if defined SINGLE_BOARD
  currentPressure = (ADS.getValue() - 166) / 111.11f; // 12bit
#else
  currentPressure = (ADS.getValue() - 2666) / 1777.8f; // 16bit
#endif
//===========================================
//float ddd = ADS.getValue();
//USART_ESP.print("ADS.getvalue = ");
//USART_ESP.println(ddd);  
//=============================================

Serial.print("press = ");
Serial.println(currentPressure);
//==================================================
//USART_ESP.print("currentPressure = ");
//USART_ESP.println(currentPressure);

//USART_ESP.print("PIN_WIRE_SDA = ");
//USART_ESP.println(digitalRead(PIN_WIRE_SDA));
//USART_ESP.print("PIN_WIRE_SCL = ");
//USART_ESP.println(digitalRead(PIN_WIRE_SCL));
//USART_ESP.print("ADS.isConnected() = ");
//USART_ESP.println(ADS.isConnected());
//==========================================

  return currentPressure;
}

void getAdsError(void) {
  // Reset the hw i2c to try and recover comms
  // on fail to do so throw error
  i2cResetState();

  // Throw error code on ADS malfunction/miswiring
  // Invalid Voltage error code: -100
  // Invalid gain error code: 255
  // Invalid mode error code: 254
  short result = ADS.getError();
//======================================
  if (result != 0) { 
//    USART_ESP.print("geterror no = ");
//    USART_ESP.println(result);  
  }
//=====================================  
  if (result == 0) return;
  char tmp[25];
  unsigned int check = snprintf(tmp, sizeof(tmp), "ADS error code: %i", result);
  if (check > 0 && check <= sizeof(tmp)) {
    lcdShowPopup(tmp);
  }
}

//Serial.print(digitalRead(PIN_SCL));    //should be HIGH
//Serial.println(digitalRead(PIN_SDA));   //should be HIGH, is LOW on stuck I2C bus
// ERROR CODE 1: I2C bus error. Could not clear sclPin clock line held low
// ERROR CODE 2: I2C bus error. Could not clear. sclPin clock line held low by slave clock for > 2sec
// ERROR CODE 3: I2C bus error. Could not clear. sdaPin data line held low
void i2cResetState(void) {
  if (digitalRead(PIN_WIRE_SDA) != HIGH || digitalRead(PIN_WIRE_SCL) != HIGH || !ADS.isConnected()) {
   // LOG_INFO("Reset I2C pins");
    short result = I2C_ClearBus(PIN_WIRE_SDA, PIN_WIRE_SCL);
    char tmp[25];
    unsigned int check = snprintf(tmp, sizeof(tmp), "I2C error code: %i", result);
    if (check > 0 && check <= sizeof(tmp)) {
      result == 0 ? adsInit() : lcdShowPopup(tmp);
    }
    delay(50);
  }
}

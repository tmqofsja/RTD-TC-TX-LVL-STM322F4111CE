#include <Adafruit_MAX31865.h>
#include "pindef.h"
#include <Arduino.h>
#include <Wire.h>
#include "RTD_PR_LVL.h"
#include <VL6180X.h>

SimpleKalmanFilter smoothPressure(0.6f, 0.6f, 0.1f);
SimpleKalmanFilter smoothPumpFlow(0.1f, 0.1f, 0.01f);
SimpleKalmanFilter smoothScalesFlow(0.5f, 0.5f, 0.01f);
SimpleKalmanFilter smoothConsideredFlow(0.1f, 0.1f, 0.1f);

SensorState currentState;

VL6180X sensor; // TOF050C-VL6180x Raser waterlevel

void setup()
{
  SerialInit();
  pinInit();
  lcdInit();
  thermocoupleInit();
//  RTD_Init_Max31865();
  lvl_init_VL6180X();
  adsInit(); // press tx

  // Init the tof sensor
  // tof.init(currentState);
}

void loop()
{
  lcdListen();
  sensorsRead();
  lcdRefresh();
}
//======================================================
//
//======================================================
static void sensorsRead(void)
{

  sensorsReadTemperature();
  readTankWaterLevel();
  sensorsReadPressure();
}
//==================================
//
//===================================
static void sensorsReadTemperature(void)
{
  if (millis() > thermoTimer)
  {
    currentState.temperature = thermocoupleRead() - 0.1f;//runningCfg.offsetTemp;
    thermoTimer = millis() + GET_KTYPE_READ_EVERY;
    Serial.print("temp value = ");
    Serial.println(currentState.temperature);

  }
}
//============================================
//
//============================================
static void sensorsReadPressure(void)
{
  uint32_t elapsedTime = millis() - pressureTimer;

  if (elapsedTime > GET_PRESSURE_READ_EVERY)
  {
    float elapsedTimeSec = elapsedTime / 1000.f;
    currentState.pressure = getPressure();
    previousSmoothedPressure = currentState.smoothedPressure;
    currentState.smoothedPressure =  smoothPressure.updateEstimate(currentState.pressure);
    currentState.pressureChangeSpeed = (currentState.smoothedPressure - previousSmoothedPressure) / elapsedTimeSec;

    pressureTimer = millis();

  }
}

static void lvl_init_VL6180X(void)
{

  Wire.begin();
  sensor.init();
  sensor.configureDefault();

  // Reduce range max convergence time and ALS integration
  // time to 30 ms and 50 ms, respectively, to allow 10 Hz
  // operation (as suggested by table "Interleaved mode
  // limits (10 Hz operation)" in the datasheet).
  sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

  sensor.setTimeout(500);

  // stop continuous mode if already active
  sensor.stopContinuous();
  // in case stopContinuous() triggered a single-shot
  // measurement, wait for it to complete
  delay(300);
  // start interleaved continuous mode with period of 100 ms
  sensor.startInterleavedContinuous(100);
}
//=======================================================
// TOF050C-VL6180x Raser waterlevel
//=======================================================
static void readTankWaterLevel()
{
  if (sensor.timeoutOccurred())
  {
    Serial.println(" TIMEOUT");
  }
  float Span = 148;
  float GageSpan = 100;
  float RatioValue = GageSpan / Span;
  float RawValue = sensor.readRangeContinuousMillimeters();

  float Gauge_Value = RatioValue * RawValue;
  if (RawValue < 0)
    RawValue = 0;
  if (RawValue > Span)
    RawValue = Span;

  currentState.waterLvl = fabs(Gauge_Value - 100);
  //Serial.print("lvl value = ");
  //Serial.println(currentState.waterLvl);

  if (sensor.timeoutOccurred())
  {
    Serial.println(" TIMEOUT");
  }
}

//======================================================
//
//======================================================
/*static void RTD_read()
{
  uint16_t rtd = thermo.readRTD();

  // Serial.print("RTD value: ");
  // Serial.println(rtd);
  // float ratio = rtd;
  // ratio /= 32768;
  // Serial.print("Ratio = ");
  // Serial.println(ratio, 8);
  // Serial.print("Resistance = ");
  // Serial.println(RREF * ratio, 8);
  Serial.print("Temperature = ");
  Serial.println(thermo.temperature(RNOMINAL, RREF));
  currentState.temperature = thermo.temperature(RNOMINAL, RREF);
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
  Serial.println();
  delay(1000);
}*/

// #############################################################################################
// ################################____LCD_REFRESH_CONTROL___###################################
// #############################################################################################

static void lcdRefresh(void)
{
  uint16_t tempDecimal;

  if (millis() > pageRefreshTimer)
  {
/*LCD pressure output, as a measure to beautify the graphs locking the live pressure read for the LCD alone*/
#ifdef BEAUTIFY_GRAPH
    lcdSetPressure(currentState.smoothedPressure * 10.f);
#else
    lcdSetPressure(
        currentState.pressure > 0.f
            ? currentState.pressure * 10.f
            : 0.f);
#endif

    /*LCD temp output*/

    lcdSetTemperature(std::floor((uint16_t)currentState.temperature));
    lcdSettemp1((float)currentState.waterLvl); //==========================Monitor=========================

    /*LCD weight & temp & water lvl output*/
    switch (lcdCurrentPageId)
    {
    case NextionPage::Home:
      // temp decimal handling
      tempDecimal = (currentState.waterTemperature - (uint16_t)currentState.waterTemperature) * 10;
      lcdSetTemperatureDecimal(tempDecimal);
      // water lvl
      lcdSetTankWaterLvl(currentState.waterLvl);
      // weight
      //       if (homeScreenScalesEnabled) lcdSetWeight(currentState.weight);
      break;
    case NextionPage::BrewGraph:
    case NextionPage::BrewManual:
      // temp decimal handling
      tempDecimal = (currentState.waterTemperature - (uint16_t)currentState.waterTemperature) * 10;
      lcdSetTemperatureDecimal(tempDecimal);
      // If the weight output is a negative value lower than -0.8 you might want to tare again before extraction starts.
      if (currentState.shotWeight)
        lcdSetWeight(currentState.shotWeight > -0.8f ? currentState.shotWeight : -0.9f);
      /*LCD flow output*/
      lcdSetFlow(currentState.smoothedPumpFlow * 10.f);
      break;
    default:
      break; // don't push needless data on other pages
    }

#ifdef DEBUG_ENABLED
    lcdShowDebug(readTempSensor(), getAdsError());
#endif

    /*LCD timer and warmup*/
    //    if (brewActive) {
    //      lcdSetBrewTimer((millis() > brewingTimer) ? (int)((millis() - brewingTimer) / 1000) : 0);
    //      lcdBrewTimerStart(); // nextion timer start
    //      lcdWarmupStateStop(); // Flagging warmup notification on Nextion needs to stop (if enabled)
    //    } else {
    //      lcdBrewTimerStop(); // nextion timer stop
    //    }

    pageRefreshTimer = millis() + REFRESH_SCREEN_EVERY;
  }
}

//======================================================
//
//======================================================
static inline void SerialInit()
{

  Serial.begin(115200);
}

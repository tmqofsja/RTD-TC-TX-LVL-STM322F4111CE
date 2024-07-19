#include "RTD_PR_LVL.h"

SimpleKalmanFilter smoothPressure(0.6f, 0.6f, 0.1f);
SimpleKalmanFilter smoothPumpFlow(0.1f, 0.1f, 0.01f);
SimpleKalmanFilter smoothScalesFlow(0.5f, 0.5f, 0.01f);
SimpleKalmanFilter smoothConsideredFlow(0.1f, 0.1f, 0.1f);

SensorState currentState;

TOF tof; // level raser

void setup()
{
  log_init();
  LOG_INFO("RTD PR LVL (fw: %s) booting", AUTO_VERSION);

  SerialInit();
  pinInit(); 
  lcdInit(); //nextion lcd
  thermocoupleInit(); //temp
  adsInit(); // press tx

  // Init the tof sensor
  tof.init(currentState);//water level
}

void loop()
{
  lcdListen();
  sensorsRead();
  lcdRefresh();
}

//======================================================
static void sensorsRead(void)
{
  sensorsReadTemperature();
  readTankWaterLevel();
  sensorsReadPressure();
  Serial.print("temp value = ");
  Serial.println(currentState.temperature);
  Serial.print("pressure = ");
  Serial.println(currentState.pressure);
  Serial.print("water tk lvl = ");
  Serial.println(currentState.waterLvl);
  
}
//==================================
//
//===================================
static void sensorsReadTemperature(void)
{
  if (millis() > thermoTimer)
  {
    currentState.temperature = thermocoupleRead() - 0.1f; // runningCfg.offsetTemp;
    thermoTimer = millis() + GET_KTYPE_READ_EVERY;
  }
}

//============================================
static void sensorsReadPressure(void)
{
  uint32_t elapsedTime = millis() - pressureTimer;

  if (elapsedTime > GET_PRESSURE_READ_EVERY)
  {
    float elapsedTimeSec = elapsedTime / 1000.f;
    currentState.pressure = getPressure();
    previousSmoothedPressure = currentState.smoothedPressure;
    currentState.smoothedPressure = smoothPressure.updateEstimate(currentState.pressure);
    currentState.pressureChangeSpeed = (currentState.smoothedPressure - previousSmoothedPressure) / elapsedTimeSec;

    pressureTimer = millis();
  }
}

// return the reading in mm of the tank water level.
static void readTankWaterLevel(void)
{
  if (lcdCurrentPageId == NextionPage::Home)
  {
    // static uint32_t tof_timeout = millis();
    // if (millis() >= tof_timeout) {
    currentState.waterLvl = tof.readLvl();
    // tof_timeout = millis() + 500;
    // }
  }
}

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
static inline void SerialInit()
{
  Serial.begin(115200);
}

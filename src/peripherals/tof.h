#ifndef TOF_H
#define TOF_H

#include <stdint.h> // for uint8_t
#include <movingAvg.h>
#include "../../lib/Common/sensors_state.h"

#if defined(TOF_VL53L0X)
  #include <Adafruit_VL53L0X.h>
  Adafruit_VL53L0X tof_sensor;
  movingAvg mvAvg(4);
#elif defined(TOF_VL6180X)
  #include <VL6180X.h>
  VL6180X tof_sensor;
#endif


class TOF {
  public:
    TOF();
    void init(SensorState& sensor);
    uint16_t readLvl();
    uint16_t readRangeToPct(uint16_t val);

  private:
    // HardwareTimer* hw_timer;
    // static void TimerHandler10(void);
    uint32_t tofReading;
};

TOF::TOF() {}

// void TOF::TimerHandler10() {
//   if (instance != nullptr && tof_sensor.isRangeComplete()) {
//     TOF::tofReading = tof_sensor.readRangeResult();
//   }
// }

void TOF::init(SensorState& sensor) {
#if defined(TOF_VL53L0X)
  while(!sensor.tofReady) {
    sensor.tofReady = tof_sensor.begin(0x29, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  }
  tof_sensor.startRangeContinuous();
  mvAvg.begin();
  // Configure the hardware timer
  // hw_timer = new HardwareTimer(TIM10);
  // hw_timer->setCount(100000, MICROSEC_FORMAT);
  // hw_timer->setOverflow(100000, MICROSEC_FORMAT);
  // hw_timer->setInterruptPriority(1, 1);
  // hw_timer->attachInterrupt(TOF::TimerHandler10); // Attach the ISR function to the timer

#elif defined(TOF_VL6180X)
    Wire.begin();
    tof_sensor.init();
    tof_sensor.configureDefault();

    // Reduce range max convergence time and ALS integration
    // time to 30 ms and 50 ms, respectively, to allow 10 Hz
    // operation (as suggested by table "Interleaved mode
    // limits (10 Hz operation)" in the datasheet).
    tof_sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    tof_sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

    tof_sensor.setTimeout(500);

    // stop continuous mode if already active
    tof_sensor.stopContinuous();
    // in case stopContinuous() triggered a single-shot
    // measurement, wait for it to complete
    delay(300);
    // start interleaved continuous mode with period of 100 ms
    tof_sensor.startInterleavedContinuous(100);  
#endif
}

uint16_t TOF::readLvl() {
  #if defined(TOF_VL53L0X)
  if(tof_sensor.isRangeComplete()) {
    TOF::tofReading = mvAvg.reading(tof_sensor.readRangeResult());
  }
   return  TOF::tofReading != 0 ? readRangeToPct(TOF::tofReading) : 30u;

  #elif defined(TOF_VL6180X)
   if (tof_sensor.timeoutOccurred())
  {
    Serial.println(" TIMEOUT");
  }
  float Span = 148;
  float GageSpan = 100;
  float RatioValue = GageSpan / Span;
  float RawValue = tof_sensor.readRangeContinuousMillimeters();

  float Gauge_Value = RatioValue * RawValue;
  if (RawValue < 0)
    RawValue = 0;
  if (RawValue > Span)
    RawValue = Span;

  //currentState.waterLvl = fabs(Gauge_Value - 100);
  //Serial.print("lvl value = ");
  //Serial.println(currentState.waterLvl);

  if (tof_sensor.timeoutOccurred())
  {
    Serial.println(" TIMEOUT");
  }
  return fabs(Gauge_Value - 100);
  fabs(Gauge_Value - 100) != 0 ? (Gauge_Value) - 100 : 30u;
  #endif
}

uint16_t TOF::readRangeToPct(uint16_t val) { //Adafruit_VL53L0X
  static const std::array<uint16_t, 10> water_lvl = { 100u, 90u, 80u, 70u, 60u, 50u, 40u, 30u, 20u, 10u };
  static const std::array<uint16_t, 9> ranges = { 15u, 30u, 45u, 60u, 75u, 90u, 105u, 115u, 125u };
  for (size_t i = 0; i < ranges.size(); i++) {
    if (val <= ranges[i]) {
      return water_lvl[i];
    }
  }
  return 9u;
}

#endif

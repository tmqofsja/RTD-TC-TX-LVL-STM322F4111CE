#include "lcd/lcd.h"
#include <Arduino.h>
//#include "peripherals/rtd.h"

static inline void pinInit(void);
static inline void SerialInit(void);
//static inline void RTD_Init_Max31865(void);
static void sensorsRead(void);
//static void RTD_read(void);
static void lvl_init_VL6180X(void);
static void readTankWaterLevel();
static void lcdRefresh();
static void sensorsReadTemperature(void);
//static void readtkwaterlevel()
static void sensorsReadPressure(void);

#include "sensors_state.h"
#include "peripherals/tof.h"
#include <SimpleKalmanFilter.h>
#include "lcd/lcd.h"
#include "peripherals/pressure_sensor.h"
#include "peripherals/thermocouple.h"
#include "peripherals/peripherals.h"




//Timers
unsigned long systemHealthTimer;
unsigned long pageRefreshTimer;
unsigned long pressureTimer;
unsigned long brewingTimer;
unsigned long thermoTimer;
unsigned long scalesTimer;
unsigned long flowTimer;
unsigned long steamTime;

// Define some const values
#if defined SINGLE_BOARD
    #define GET_KTYPE_READ_EVERY    70 // max31855 amp module data read interval not recommended to be changed to lower than 70 (ms)
#else
    #define GET_KTYPE_READ_EVERY    250 // max6675 amp module data read interval not recommended to be changed to lower than 250 (ms)
#endif
#define GET_PRESSURE_READ_EVERY 10 // Pressure refresh interval (ms)
#define GET_SCALES_READ_EVERY   100 // Scales refresh interval (ms)
#define REFRESH_SCREEN_EVERY    150 // Screen refresh interval (ms)
#define REFRESH_FLOW_EVERY      50 // Flow refresh interval (ms)
#define HEALTHCHECK_EVERY       30000 // System checks happen every 30sec
#define BOILER_FILL_START_TIME  3000UL // Boiler fill start time - 3 sec since system init.
#define BOILER_FILL_TIMEOUT     8000UL // Boiler fill timeout - 8sec since system init.
#define BOILER_FILL_SKIP_TEMP   85.f // Boiler fill skip temperature threshold
#define SYS_PRESSURE_IDLE       0.7f // System pressure threshold at idle
#define MIN_WATER_LVL           10u // Min allowable tank water lvl

// Other util vars
float previousSmoothedPressure;

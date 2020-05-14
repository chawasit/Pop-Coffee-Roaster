#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>

uint16_t colorToNumber(uint16_t r, uint16_t g, uint16_t b) {
  return (r << 16) + (g << 8) + (b);
}

#define LONG_PRESSED_DURATION 2000
const uint16_t PINK = 0xfbe4;
const uint16_t ORANGE = 0xfbe4;
const uint16_t PURPLE = colorToNumber(127, 0, 255);
const uint16_t VIOLET = 0xa254;
const uint16_t YELLOW = 0xff80;
const uint16_t BROWN = 0x8e7a6f;
const uint16_t CYAN = 0x51d;
const uint16_t GRAY = 0x7bef;

#include <WiFi.h>

#include <ModbusRtu.h>

#include <M5Stack.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 5
#define TEMPERATURE_PRECISION 10

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
// inbean              exhaust
// 3BE6BD5D06D84CC5    3B4E8E5D06D82C97
//
// air heater 1     air heater 2
// 3B6AB45D06D80CE7    3B0DA35D06D82CAF
DeviceAddress airThermometer1 = {0x3B, 0x6A, 0xB4, 0x5D, 0x06, 0xD8, 0x0C, 0xE7};
DeviceAddress airThermometer2 = {0x3B, 0x0D, 0xA3, 0x5D, 0x06, 0xD8, 0x2C, 0xAF};
DeviceAddress beanThermometer    = {0x3B, 0xE6, 0xBD, 0x5D, 0x06, 0xD8, 0x4C, 0xC5};
DeviceAddress exhaustThermometer = {0x3B, 0x4E, 0x8E, 0x5D, 0x06, 0xD8, 0x2C, 0x97};

/* data array for modbus network sharing
 *  
 *  2 = airTemp
 *  3 = outsideTemp
 *  4 = heater level
 *  5 = fan level
 *  
 *  10 = state
 */
uint16_t au16data[16] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1 };

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus slave(1, Serial, 0); // this is slave @1 and RS-232 or USB-FTDI

// fan Config
const int fanPowerPin = 17;
const int fanPwmChannel = 0;
const int fanPwmFrequency = 4096;
const int fanDutyResolution = 12;

// Heater Config
const int heaterPowerPin = 2;
const int heaterPwmChannel = 3;
const int heaterPwmFrequency = 4096;
const int heaterDutyResolution = 12;

const int fanMaxDuty = (1 << fanDutyResolution) - 1;
const int heaterMaxDuty = (1 << heaterDutyResolution) - 1;

// sensor variable
float airTemperature = 0.0;
float airTemperature1 = 0.0;
float airTemperature2 = 0.0;

float beanTemperature = 0.0;
float exhaustTemperature = 0.0;

float newAirTemperature1 = 0.0;
float newAirTemperature2 = 0.0;
float newBeanTemperature = 0.0;
float newExhaustTemperature = 0.0;
const float acceptableRangeOfTemperatureChange = 100;

// global config
int fanLevel, heaterLevel, fanDuty, heaterDuty;

double rateOfRise = 0;
float temperatureHistory[13];
unsigned int temperatureTimestamp[13];
int temperatureHistoryIndex = 0;
unsigned long lastRecord = millis();
double rateOfRiseHistory[12];
int rateOfRiseHistoryIndex = 0;


float targetTemperature = 0;
unsigned int lastHeaterStateChanged = millis();
int baseHeaterDuty = 0;
int heaterState = 0;
int lastHeaterState = 0;
float airTemperatureRateOfRise = 0;
unsigned int lastHeaterRoRStep = millis();

const int HEATER_RUN = 1;
const int HEATER_STAY = 0;
const int HEATER_INCREASE_DUTY = 2;
const int HEATER_DECREASE_DUTY = 3;

#define STATE_INIT 0
#define STATE_PREHEAT 1
#define STATE_CHARGE 2
#define STATE_ROASTING 3
#define STATE_COOLING 4
#define FIRST_CRACK_MARK 99
int currentState = -1;

int frame = 0;
float fps = 0;
unsigned int lastFrame = millis();

void fpsCheck() 
{
  frame++;
  if (millis() - lastFrame > 1000) {
      fps = frame * 1000 / (millis() - lastFrame);
      frame = 0;
      lastFrame = millis();
  }
}

void rateOfRiseCode(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);

  xLastWakeTime = xTaskGetTickCount();

  for (int i = 0; i < 13; i++) {
    temperatureHistory[i] = beanTemperature;
    temperatureTimestamp[i] = millis() - 999999999;
  }
  
  for(;;) {
    unsigned int currentTimestamp = millis();
    
    temperatureHistoryIndex++; // increase index
    temperatureHistoryIndex = temperatureHistoryIndex % 7;

    temperatureTimestamp[temperatureHistoryIndex] = currentTimestamp;
    temperatureHistory[temperatureHistoryIndex] = beanTemperature;

    int pastTemperatureIndex = (temperatureHistoryIndex + 1) % 7;
    float pastTemperature = temperatureHistory[pastTemperatureIndex];
    unsigned int pastTemperatureTimestamp = temperatureTimestamp[pastTemperatureIndex];
    
    if (currentTimestamp - pastTemperatureTimestamp > 0) {
      double newRateOfRise = (
            (beanTemperature - pastTemperature) * 1000 
            / 
            (currentTimestamp - pastTemperatureTimestamp)
          );
      rateOfRiseHistory[rateOfRiseHistoryIndex++] = newRateOfRise;
      rateOfRiseHistoryIndex %= 6;
    }
    
    double sumRoR = 0;
    for (int i = 0; i < 6; i++) {
      sumRoR += rateOfRiseHistory[i];
    }
    
    rateOfRise = (rateOfRise + (sumRoR * 60 / 6)) / 2;
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void sensorsReadingCode(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(200);

  xLastWakeTime = xTaskGetTickCount();
  
  sensors.requestTemperatures();
  
  airTemperature1 = sensors.getTempC(airThermometer1);
  airTemperature2 = sensors.getTempC(airThermometer2);
  beanTemperature = sensors.getTempC(beanThermometer);
  exhaustTemperature = sensors.getTempC(exhaustThermometer);
  
  for(;;) {
    sensors.requestTemperatures();

    newAirTemperature1 = sensors.getTempC(airThermometer1);
    if (!isnan(newAirTemperature1) and fabs(newAirTemperature1 - airTemperature1) <= acceptableRangeOfTemperatureChange) {
      airTemperature1 = newAirTemperature1;
    }
    
    newAirTemperature2 = sensors.getTempC(airThermometer2);
    if (!isnan(newAirTemperature2) and fabs(newAirTemperature2 - airTemperature2) <= acceptableRangeOfTemperatureChange) {
      airTemperature2 = newAirTemperature2;
    }

    airTemperature = (airTemperature1 + airTemperature2) / 2;
    
    newBeanTemperature = sensors.getTempC(beanThermometer);
    if (!isnan(newBeanTemperature) and fabs(newBeanTemperature - beanTemperature) <= acceptableRangeOfTemperatureChange) {
      beanTemperature = newBeanTemperature;
    }

    newExhaustTemperature = sensors.getTempC(exhaustThermometer);
    if (!isnan(newExhaustTemperature) and fabs(newExhaustTemperature - exhaustTemperature) <= acceptableRangeOfTemperatureChange) {
      exhaustTemperature = newExhaustTemperature;
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// State
void setPWMPower()
{
  if (airTemperature > 300) {
    ledcWrite(heaterPwmChannel, 0);
    ledcWrite(fanPwmChannel, fanMaxDuty); 
  } else {
    ledcWrite(heaterPwmChannel, heaterDuty);
    ledcWrite(fanPwmChannel, fanDuty); 
  }
}

void fanSpeedControl()
{
  if (fanLevel > 0) {
    fanDuty = ((int) ((fanMaxDuty * 7 / 10) + ((max(0, min(fanLevel, 30)) * fanMaxDuty / 30) * 3 / 10)));
  } else {
    fanDuty = ((int) fanMaxDuty * 3 / 10);
  }
}

void resetTemperatureControl() {
  lastHeaterRoRStep = millis();
  lastHeaterStateChanged = millis();
}

void targetTemperatureControl() {
  unsigned int timeSinceLastRoRStep = millis() - lastHeaterRoRStep;

  float requiredTime = (60000.0 / fabs(airTemperatureRateOfRise));
  if (timeSinceLastRoRStep > requiredTime) {
    float temperatureStep = airTemperatureRateOfRise * timeSinceLastRoRStep / (60000.0);
    targetTemperature += temperatureStep;
    
    lastHeaterRoRStep = millis();
  }

  if (fabs(airTemperatureRateOfRise) == 0.0) {
    lastHeaterRoRStep = millis();
  }
}

void heaterTemperatureControl()
{
  if (targetTemperature <= 50 or currentState == STATE_INIT) {
    heaterDuty = 0;
    baseHeaterDuty = 0;
  } else {
    unsigned int timeSinceLastStateChange = millis() - lastHeaterStateChanged;

    int temperatureError = targetTemperature - airTemperature;
    if (airTemperature < targetTemperature) { // need more power
      if (heaterState == HEATER_STAY) { // turn on heater if it turn off
        heaterState = HEATER_RUN;
        lastHeaterStateChanged = millis();
      } else if (heaterState == HEATER_RUN and timeSinceLastStateChange > 3000) { // increase base heater duty if it continue on for 2s.
        lastHeaterStateChanged = millis();
        baseHeaterDuty += max(1, min(temperatureError / 10, 3));
      }

      heaterDuty = baseHeaterDuty + 1;
    } else { // need less power
      if (heaterState == HEATER_RUN) {
        heaterState = HEATER_STAY;
        lastHeaterStateChanged = millis();
      } else if (heaterState == HEATER_STAY and timeSinceLastStateChange > 2000) { // decrease base heater duty if it continue off for 2s.
        lastHeaterStateChanged = millis();
        baseHeaterDuty += -1;
      }
      
      heaterDuty =  baseHeaterDuty;
    }

    baseHeaterDuty = max(-40, min(70, baseHeaterDuty)); 
    heaterDuty += 160;
    heaterDuty = max(120, min(230, heaterDuty)); 
  }
}

/*
 * state 0: init
 * fan spin at minimum speed
 * heater turned off
 */
bool buttonReleased;
unsigned int lastHot = millis();

void stateInit()
{
  if (currentState != STATE_INIT) {
    currentState = STATE_INIT;
    targetTemperature = 100;
    buttonReleased = false;
  }
  
  heaterDuty = 0;
  baseHeaterDuty = 0;

  if (airTemperature > 50) {
    lastHot = millis();
    fanLevel = 20;
  } else {
    if (millis() - lastHot > 2000) {
      fanLevel = 0;
    }
  }
  
  fanSpeedControl();
  
  if (M5.BtnA.wasReleased()) {
    targetTemperature--;
  }

  if (M5.BtnB.wasReleased()) {
    statePreheat();
  }
  
  if (M5.BtnC.wasReleased()) {
    targetTemperature++;
  }
}

void displayInit()
{
  M5.Lcd.setCursor(0, 0);
  
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.println("Ready to Roast!");

  M5.Lcd.println();
  M5.Lcd.setTextSize(2);
  M5.Lcd.println("Charge Temperature. ?");
  M5.Lcd.println();
  M5.Lcd.setTextSize(5);
  M5.Lcd.setTextColor(RED, BLACK);
  M5.Lcd.printf(" %3.0f C\n", targetTemperature);

  M5.Lcd.println();
  M5.Lcd.println();
  M5.Lcd.setTextColor(BLACK, YELLOW);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("   -  P.HEAT  +  ");
}


/*
 * state 1: PRE HEAT
 * fan start spinning at 20
 * heater turned on to desired temperature c
 */
unsigned int startPreheatTime;
bool readyToCharge;
void statePreheat()
{
  if (currentState != STATE_PREHEAT) {
    currentState = STATE_PREHEAT;

    fanLevel = 20;
    fanSpeedControl();

    startPreheatTime = millis();

    readyToCharge = false;

    resetTemperatureControl();
  }

  fanSpeedControl();
  heaterTemperatureControl();

  if (M5.BtnA.wasReleased()) {
    stateInit();
  }

  if ((fabs(targetTemperature - airTemperature) < 4 and fabs(rateOfRise) < 10) or readyToCharge) {
    readyToCharge = true;
    // ready to rasting
    if (M5.BtnB.wasReleased()) {
      stateCharge();
    }
  }
}

void displayPreheat()
{
  M5.Lcd.setCursor(0, 0);
  
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.println("Pre Heating!!");
  M5.Lcd.println();
  M5.Lcd.println();

  M5.Lcd.setTextColor(RED, BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("Air Temperature\n");
  M5.Lcd.setTextSize(4);
  M5.Lcd.printf("%6.2f => %3.0f\n", airTemperature, targetTemperature);

  M5.Lcd.println();
  M5.Lcd.println();
  M5.Lcd.setTextColor(BLACK, YELLOW);
  M5.Lcd.setTextSize(3);
  if (readyToCharge)
    M5.Lcd.printf("BACK  CHARGE     ");
  else
    M5.Lcd.printf("BACK             ");
}


/*
 * state 2: CHARGE
 * wait bean temperature to turn up and
 * go to roasing state
 */
unsigned int startTime;

#define MAX_HISTORY 1200 // 20 min roasted log every 1 sec = 600 points
float airTemperatureProfile[MAX_HISTORY];
float beanTemperatureProfile[MAX_HISTORY];
int8_t rateOfRiseProfile[MAX_HISTORY];
uint8_t fanProfile[MAX_HISTORY];
int profileCounter;
unsigned int logTimeProfile[MAX_HISTORY];
unsigned int lastLogProfile;
int lastFanLevel;
float lastRateOfRise;

void logProfile(bool force = false) {
  if (profileCounter > 2000)
    return;

  unsigned int currentTime = millis();
  unsigned int timeSinceLastLog = currentTime - lastLogProfile;
  
  bool event = (lastFanLevel != fanLevel) or (lastRateOfRise != airTemperatureRateOfRise);
  
  if (force or timeSinceLastLog > 1000) {
    airTemperatureProfile[profileCounter] = airTemperature;
    beanTemperatureProfile[profileCounter] = beanTemperature;
    rateOfRiseProfile[profileCounter] = airTemperatureRateOfRise;
    fanProfile[profileCounter] = fanLevel;
    logTimeProfile[profileCounter] = currentTime - startTime;
    
    lastFanLevel = fanLevel;
    lastRateOfRise = airTemperatureRateOfRise;

    profileCounter++;

    
    if (timeSinceLastLog > 1000) {
      lastLogProfile = currentTime;
    }
  } 

  if (force) {
    lastLogProfile = currentTime;
  }
}

void stateCharge()
{
  if (currentState != STATE_CHARGE) {
    currentState = STATE_CHARGE;
    startTime = millis();
    logProfile(true);
    resetTemperatureControl();
  }

  heaterTemperatureControl();
  logProfile();
  
  // wait until bean temperature rise up
  if (rateOfRise > 0 and millis() - startTime > 3000) {
    stateRoasting();
  }
}

void displayCharge()
{
  M5.Lcd.setCursor(40, 40);
  M5.Lcd.fillScreen(ORANGE);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setTextColor(WHITE, ORANGE);
  M5.Lcd.println("CHARGE!");
}


/*
 * state 3: ROASTING
 * heater rise temperature follow air temperature's ror
 */
int selector;
const int HEATER_SELECTED = 100;
const int FAN_SELECTED = 222;
unsigned int firstCrackTime, endTime;
int firstCrackMinutes, firstCrackSeconds, endMinutes, endSeconds;
float firstCrackTemperature, endTemperature;
bool hadFirstCrack;

void firstCrackMark() {
  hadFirstCrack = true;
  firstCrackTime = millis();
  firstCrackMinutes = (firstCrackTime - startTime) / 60000;
  firstCrackSeconds = ((firstCrackTime - startTime) / 1000) % 60;
  firstCrackTemperature = beanTemperature;
}

void endTimeMark() {
  endTime = millis();
  endMinutes = (endTime - startTime) / 60000;
  endSeconds = ((endTime - startTime) / 1000) % 60;
  endTemperature = beanTemperature;
}

void stateRoasting()
{
  if (currentState != STATE_ROASTING) {
    currentState = STATE_ROASTING;
    airTemperatureRateOfRise = 25.0;
    hadFirstCrack = false;
    buttonReleased = true;

    resetTemperatureControl();
  }
  
  fanSpeedControl();
  targetTemperatureControl();
  heaterTemperatureControl();
  logProfile();

  if (M5.BtnB.wasReleased()) {
    if (selector == HEATER_SELECTED) {
      selector = FAN_SELECTED;
    } else {
      selector = HEATER_SELECTED;
    }
  }

  if (M5.BtnA.wasReleased()) {
    if (selector == HEATER_SELECTED) {
      airTemperatureRateOfRise -= 1.0;
    } else {
      fanLevel -= 1;
    }
  }

  if (M5.BtnC.wasReleased()) {
    if (selector == HEATER_SELECTED) {
      airTemperatureRateOfRise += 1.0;
    } else {
      fanLevel += 1;
    }
  }

  fanLevel = min(30, max(1, fanLevel));

  if (M5.BtnB.wasReleased()) {
    buttonReleased = true;
  }
  
  if (M5.BtnB.pressedFor(2000) and buttonReleased) {
    buttonReleased = false;
    
    if (!hadFirstCrack) {
      firstCrackMark();
    } else {
      stateCooling();
    }
  } 
}

void displayRoasting()
{
  unsigned int roastTime = millis() - startTime;
  int minutes = (roastTime / 60000);
  int seconds = (roastTime / 1000) % 60;
  
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(RED, BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("ROASTING! "); // 10
  M5.Lcd.setTextColor(ORANGE, BLACK);
  M5.Lcd.printf(" %2d:%02d\n", minutes, seconds);
  
  // 17 characters
  M5.Lcd.setTextColor(ORANGE, BLACK);
  if (hadFirstCrack) {
    float devPercentage = (millis() - firstCrackTime) * 100.0 / (firstCrackTime - startTime);
    M5.Lcd.printf("FC%2d:%02d DEV %4.1f%%\n", firstCrackMinutes, firstCrackSeconds, devPercentage);
  } else {
    M5.Lcd.println();
  }
  
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setTextSize(3);
  if (fanLevel == 30) {
    M5.Lcd.printf("FAN    MAX       \n");
  } else {
    M5.Lcd.printf("FAN     %2d/30    \n", fanLevel);
  }

  if (heaterDuty == 0) {
    M5.Lcd.printf("HEATER OFF %3.0f\n", airTemperatureRateOfRise);
  } else {
    M5.Lcd.printf("HEATER %3.0f c/m\n", airTemperatureRateOfRise);
  }
  
  M5.Lcd.setTextColor(RED, BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("Air Temperature\n");
  M5.Lcd.setTextSize(4);
  M5.Lcd.printf("%6.2fc > %3.0f\n", airTemperature, targetTemperature);
  
  M5.Lcd.setTextColor(BLUE, BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("Bean Temperature\n");
  M5.Lcd.setTextSize(4);
  M5.Lcd.printf("%6.2fc R%4.0f\n", beanTemperature, rateOfRise);
  
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("fps:%5.0f, hd:%3d, %3d, %s\n", fps, heaterDuty, baseHeaterDuty, (heaterState == HEATER_STAY? "ST":"RN"));

  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.setTextSize(3);
  if (selector == HEATER_SELECTED)
    M5.Lcd.printf("   -  HEATER  +");
  else
    M5.Lcd.printf("   -   FAN    +");
}


/*
 * state 4: COOLING
 * heater turn off
 * fan run at speed 16
 */
unsigned int startCoolingTime;
bool readyForReset;
bool showProfile;
void stateCooling()
{
  if (currentState != STATE_COOLING) {
    currentState = STATE_COOLING;
    fanLevel = 20;
    startCoolingTime = millis();
    readyForReset = false;

    endTimeMark();
    logProfile(true);

    showProfile = false;
  }

  heaterLevel = 0;
  heaterDuty = 0;
  
  fanSpeedControl();

  if (M5.BtnB.wasReleased())
    buttonReleased = true;

  if (fabs(rateOfRise) < 7 and millis() - startCoolingTime > 20000) {
    // ready for next roast
    readyForReset = true;
    
    if (M5.BtnB.wasReleased()) {
      ESP.restart();
    }
  }

  if (M5.BtnA.wasReleased()) 
    fanLevel--;

  if (M5.BtnC.wasReleased())
    fanLevel++;

  if (M5.BtnC.pressedFor(2000)) {
    showProfile = showProfile ? false : true;
    while(M5.BtnC.isPressed()) {
      vTaskDelay(100);
      M5.update();
    }
  }

  fanLevel = min(30, max(1, fanLevel));
}

bool profileRendered;
void displayCooling()
{
  if (showProfile) {
    if (!profileRendered) {
      M5.Lcd.fillScreen(BLACK);
      profileRendered = true;
    
      M5.Lcd.setCursor(1, 1);
      M5.Lcd.setTextSize(3);
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.println("Roast Profile");

      // draw dev area
      unsigned int fcDuration = firstCrackTime - startTime; 
      unsigned int endDuration = endTime - startTime; 
      int fcX = 30 + ((260 * fcDuration / endDuration));
      M5.Lcd.fillRect(fcX, 30, 290 - fcX, 180, BROWN);
      
      // draw graph legend
      M5.Lcd.drawLine(30, 30, 290, 30, GRAY); // 300
      M5.Lcd.drawLine(30, 60, 290, 60, GRAY); // 250
      M5.Lcd.drawLine(30, 90, 290, 90, GRAY); // 200
      M5.Lcd.drawLine(30, 120, 290, 120, GRAY); // 150
      M5.Lcd.drawLine(30, 150, 290, 150, GRAY); // 100

      // Right Axis line
      M5.Lcd.drawLine(30, 210 - 60, 290, 210 - 60, GRAY); // 30
      M5.Lcd.drawLine(30, 210 - 40, 290, 210 - 40, GRAY); // 20
      M5.Lcd.drawLine(30, 210 - 20, 290, 210 - 20, GRAY); // 10

      // Graph side
      M5.Lcd.drawLine(30, 30, 30, 210, WHITE);
      M5.Lcd.drawLine(30, 210, 290, 210, WHITE);
      M5.Lcd.drawLine(290, 210, 290, 30, WHITE);

      // Left Axis
      M5.Lcd.setTextColor(WHITE, BLACK);
      
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(5, 30 -3);
      M5.Lcd.print("300");
      M5.Lcd.setCursor(5, 60 - 3);
      M5.Lcd.print("250");
      M5.Lcd.setCursor(5, 90 - 3);
      M5.Lcd.print("200");
      M5.Lcd.setCursor(5, 120 - 3);
      M5.Lcd.print("150");
      M5.Lcd.setCursor(5, 150 - 3);
      M5.Lcd.print("100");
      M5.Lcd.setCursor(5, 180 - 3);
      M5.Lcd.print(" 50");

      // Right Axis
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(290 + 5, 210 - 60 - 3);
      M5.Lcd.print("30");
      M5.Lcd.setCursor(290 + 5, 210 - 40 - 3);
      M5.Lcd.print("20");
      M5.Lcd.setCursor(290 + 5, 210 - 20 - 3);
      M5.Lcd.print("10");
//      M5.Lcd.setCursor(290 + 5, 220 - 3);
//      M5.Lcd.print(" 0");


      // Bottom
      M5.Lcd.setCursor(30 - 10, 210 + 3);
      M5.Lcd.print("0:00");

      unsigned int timeStep = fcDuration / 3;
      for (int i = 1; i <= 3; i++) {
        int tX = 30 + ((260 * timeStep * i / endDuration));
        M5.Lcd.setCursor(tX - 7, 210 + 3);
        int minutes = (timeStep * i) / 60000;
        int seconds = ((timeStep * i) / 1000 ) % 60;
        M5.Lcd.printf("%d:%02d", minutes, seconds);
      }

      // Draw End Time
      M5.Lcd.setCursor(290 - 7, 210 + 3);
      M5.Lcd.setTextColor(PURPLE, BLACK);
      M5.Lcd.printf("%d:%02d", endMinutes, endSeconds);

      // Draw First Crack Time
      M5.Lcd.setCursor(fcX - 7, 210 + 3);
      M5.Lcd.setTextSize(1);
      M5.Lcd.setTextColor(ORANGE, BLACK);
      M5.Lcd.printf("%d:%02d", firstCrackMinutes, firstCrackSeconds);

      M5.Lcd.setTextSize(1);
      M5.Lcd.setTextColor(RED, BLACK);
      int eTY = 210 - ((180 * airTemperatureProfile[profileCounter - 1] / 300));
      M5.Lcd.setCursor(290 + 5, eTY - 3);
      M5.Lcd.printf("%.0f", airTemperatureProfile[profileCounter - 1]);

      M5.Lcd.setTextColor(BLUE, BLACK);
      eTY = 210 - ((180 * beanTemperatureProfile[profileCounter - 1] / 300));
      M5.Lcd.setCursor(290 + 5, eTY - 3);
      M5.Lcd.printf("%.0f", beanTemperatureProfile[profileCounter - 1]);
      
      // draw profile
      for (int i = 0; i < profileCounter - 1; i++) {
        int sX = 30 + ((260 * logTimeProfile[i] / endDuration));
        int sY = 210 - ((180 * airTemperatureProfile[i] / 300));
        int eX = 30 + ((260 * logTimeProfile[i + 1] / endDuration));
        int eY = 210 - ((180 * airTemperatureProfile[i + 1] / 300));
        M5.Lcd.drawLine(sX, sY, eX, eY, RED);
        M5.Lcd.drawLine(sX, sY+1, eX, eY+1, RED);

        sY = 210 - ((180 * beanTemperatureProfile[i] / 300));
        eY = 210 - ((180 * beanTemperatureProfile[i + 1] / 300));
        M5.Lcd.drawLine(sX, sY, eX, eY, BLUE);
        M5.Lcd.drawLine(sX, sY+1, eX, eY+1, BLUE);

        sY = 210 - ((60 * fanProfile[i] / 30));
        eY = 210 - ((60 * fanProfile[i + 1] / 30));
        M5.Lcd.drawLine(sX, sY, eX, eY, CYAN);

        sY = 210 - ((60 * rateOfRiseProfile[i] / 30));
        eY = 210 - ((60 * rateOfRiseProfile[i + 1] / 30));
        M5.Lcd.drawLine(sX, sY, eX, eY, ORANGE);
      }

      // Legend
      M5.Lcd.setCursor(5, 223);
      M5.Lcd.fillRect(0, 220, 320, 240, GRAY);
      M5.Lcd.setTextColor(RED, GRAY);
      M5.Lcd.setTextSize(2);
      M5.Lcd.print("AIR T.");

      M5.Lcd.setTextColor(BLUE, WHITE);
      M5.Lcd.print(" BEAN T.");

      M5.Lcd.setTextColor(CYAN, WHITE);
      M5.Lcd.print(" FAN");

      M5.Lcd.setTextColor(ORANGE, WHITE);
      M5.Lcd.print(" AirRoR.");
    }
  } else {
    if (profileRendered) {
      M5.Lcd.fillScreen(BLACK);
    }
    
    profileRendered = false;
    M5.Lcd.setCursor(0, 0);
    
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.println("COOLING!!");
    M5.Lcd.println();
  
    M5.Lcd.setTextColor(BLUE, BLACK);
    M5.Lcd.setTextSize(3);
    M5.Lcd.printf("Bean Temperature\n");
    M5.Lcd.setTextSize(4);
    M5.Lcd.printf("%6.2fC\n", beanTemperature);
    
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.printf("FC %2d:%02d @ %5.1fc\n", firstCrackMinutes, firstCrackSeconds, firstCrackTemperature);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.printf("END%2d:%02d @ %5.1fc\n", endMinutes, endSeconds, endTemperature);
    float devPercentage = (endTime - firstCrackTime) * 100.0 / (firstCrackTime - startTime);
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.printf("DEV %6.2f%%\n", devPercentage);
    
    M5.Lcd.setTextSize(3);
    M5.Lcd.println();
    if (readyForReset) {
      M5.Lcd.setTextColor(ORANGE, BLACK);
      M5.Lcd.printf("      RESET      ");
    } else {
      M5.Lcd.setTextColor(BLACK, YELLOW);
      M5.Lcd.printf("  -    FAN   +[P]");
    }   
  }
}

void modbusPolling()
{ 
  // write data to buffer
  au16data[2] = ((uint16_t) (airTemperature * 100));
  au16data[3] = ((uint16_t) (beanTemperature * 100));
  au16data[4] = ((uint16_t) targetTemperature);
  au16data[5] = ((uint16_t) fanLevel);
  au16data[6] = ((uint16_t) airTemperatureRateOfRise + 100.0); // offset 100
  au16data[7] = ((uint16_t) (exhaustTemperature * 100));
  // ..
//  au16data[9] = ((uint16_t) 0);
  au16data[10] = ((uint16_t) currentState);
  // for debug
  au16data[12] = ((uint16_t) (airTemperature1 * 100));
  au16data[13] = ((uint16_t) (airTemperature2 * 100));
  
  // server read buffer or write to buffer
  slave.poll(au16data, 16);

  targetTemperature = ((float) au16data[4]);
  fanLevel = ((int) au16data[5]);
//  airTemperatureRateOfRise = ((float) au16data[6]) - 100.0; // offset 100
  // ..
  if (au16data[9] > 0) {
    float requestAirRateOfRise = ((float) au16data[9]) - 100.0;
    airTemperatureRateOfRise = requestAirRateOfRise;
    
    au16data[9] = 0;
  }
  
  
  int requestState = ((int) au16data[10]);
  if (requestState != currentState) {
    switch (requestState) {
      case STATE_INIT:
        stateInit();
        break;
      case STATE_PREHEAT:
        statePreheat();
        break;
      case STATE_CHARGE:
        stateCharge();
        break;
      case STATE_ROASTING:
        stateRoasting();
        break;
      case STATE_COOLING:
        stateCooling();
        break;
      case FIRST_CRACK_MARK:
        firstCrackMark();
        break;
    }
  }
}

int lastState = -99;
void runState()
{ 
  switch (currentState) {
    case STATE_INIT:
      stateInit();
      break;
    case STATE_PREHEAT:
      statePreheat();
      break;
    case STATE_CHARGE:
      stateCharge();
      break;
    case STATE_ROASTING:
      stateRoasting();
      break;
    case STATE_COOLING:
      stateCooling();
      break;
  }
}

void displayCode(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(200);

  xLastWakeTime = xTaskGetTickCount();
  
  M5.Lcd.fillScreen(BLACK);
  
  for(;;) {
    if (lastState != currentState) {
      lastState = currentState;
      M5.Lcd.fillScreen(BLACK);
//      M5.Speaker.tone(2700, 200);
    }
    
    switch (currentState) {
      case STATE_INIT:
        displayInit();
        break;
      case STATE_PREHEAT:
        displayPreheat();
        break;
      case STATE_CHARGE:
        displayCharge();
        break;
      case STATE_ROASTING:
        displayRoasting();
        break;
      case STATE_COOLING:
        displayCooling();
        break;
      default:
        M5.Lcd.fillScreen(PURPLE);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  // Disable WIFI and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();

  // (bool LCDEnable, bool SDEnable, bool SerialEnable, bool I2CEnable)
  M5.begin(true, false, false, true);
  M5.Speaker.mute();
  M5.Lcd.fillScreen(WHITE);
  
  Serial.begin( 115200, SERIAL_8E1); // 115200 baud, 8-bits, even, 1-bit stop
  slave.setTimeOut( 1000 );
  Serial.flush();
  slave.start();  

  // fan control
  ledcSetup(fanPwmChannel, fanPwmFrequency, fanDutyResolution);
  ledcAttachPin(fanPowerPin, fanPwmChannel);
  ledcWrite(fanPwmChannel, 0); 

  // setup heater control
  ledcSetup(heaterPwmChannel, heaterPwmFrequency, heaterDutyResolution);
  ledcAttachPin(heaterPowerPin, heaterPwmChannel);
  ledcWrite(heaterPwmChannel, 0);

  // Error flag
  bool hasError = false;
  
  // Start up the temperature sensor library
  sensors.begin();

  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  if (sensors.getDeviceCount() != 4) {
    hasError = true;
    Serial.println("Some thermometer not working!!!");
  }

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  if (!sensors.isConnected(airThermometer1)) {
    hasError = true;
    Serial.println("airThermometer1 is not connected!");  
  }

  if (!sensors.isConnected(airThermometer2)) {
    hasError = true;
    Serial.println("airThermometer2 is not connected!");  
  }

  if (!sensors.isConnected(beanThermometer)) {
    hasError = true;
    Serial.println("beanThermometer is not connected!");  
  }

  if (!sensors.isConnected(exhaustThermometer)) {
    hasError = true;
    Serial.println("exhaustThermometer is not connected!");  
  }
  
  // stall if error found
  if (hasError) {
    Serial.println("ERROR");
    M5.Lcd.fillScreen(RED);
    for (;;);
  }

//  sensors.setResolution(airThermometer1, TEMPERATURE_PRECISION);
//  sensors.setResolution(beanThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(TEMPERATURE_PRECISION);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(airThermometer1), DEC); 
  Serial.println();

  Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(beanThermometer), DEC); 
  Serial.println();
  
  // start sensors reading task
  xTaskCreatePinnedToCore(
    sensorsReadingCode, /* Function to implement the task */
    "SensTask", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    1,  /* Priority of the task */
    NULL,  /* Task handle. */
    tskNO_AFFINITY); /* Core where the task should run */
    
  xTaskCreatePinnedToCore(
    displayCode, /* Function to implement the task */
    "DispTask", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    1,  /* Priority of the task */
    NULL,  /* Task handle. */
    tskNO_AFFINITY); /* Core where the task should run */
    
  xTaskCreatePinnedToCore(
    rateOfRiseCode, /* Function to implement the task */
    "RorTask", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    1,  /* Priority of the task */
    NULL,  /* Task handle. */
    tskNO_AFFINITY); /* Core where the task should run */
  
  stateInit();
}

void loop()
{
  fpsCheck();
  
  modbusPolling();
  
  runState();
  
  setPWMPower();

  M5.update();
}

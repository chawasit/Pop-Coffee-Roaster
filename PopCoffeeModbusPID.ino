#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>


int colourToNumber(int r, int g, int b) {
  return (r << 16) + (g << 8) + (b);
}

#define LONG_PRESSED_DURATION 2000
const int ORANGE = colourToNumber(255, 128, 0);
const int PURPLE = colourToNumber(127, 0, 255);
const int YELLOW = colourToNumber(255, 255, 0);
const int CYAN = colourToNumber(0, 255, 255);

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
DeviceAddress insideThermometer, outsideThermometer;


/* data array for modbus network sharing
 *  
 *  2 = insideTemp
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
const int heaterPwmChannel = 1;
const int heaterPwmFrequency = 4096;
const int heaterDutyResolution = 12;

const int fanMaxDuty = (1 << fanDutyResolution) - 1;
const int heaterMaxDuty = (1 << heaterDutyResolution) - 1;

// sensor variable
float insideTemperature = 0.0;
float outsideTemperature = 0.0;
float newInsideTemperature = 0.0;
float newOutsideTemperature = 0.0;
const float acceptableRangeOfTemperatureChange = 80;

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
    temperatureHistory[i] = outsideTemperature;
    temperatureTimestamp[i] = millis() - 999999999;
  }
  
  for(;;) {
    unsigned int currentTimestamp = millis();
    
    temperatureHistoryIndex++; // increase index
    temperatureHistoryIndex = temperatureHistoryIndex % 7;

    temperatureTimestamp[temperatureHistoryIndex] = currentTimestamp;
    temperatureHistory[temperatureHistoryIndex] = outsideTemperature;

    int pastTemperatureIndex = (temperatureHistoryIndex + 1) % 7;
    float pastTemperature = temperatureHistory[pastTemperatureIndex];
    unsigned int pastTemperatureTimestamp = temperatureTimestamp[pastTemperatureIndex];
    
    if (currentTimestamp - pastTemperatureTimestamp > 0) {
      double newRateOfRise = (
            (outsideTemperature - pastTemperature) * 1000 
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
  const TickType_t xFrequency = pdMS_TO_TICKS(250);

  xLastWakeTime = xTaskGetTickCount();
  
  insideTemperature = sensors.getTempC(insideThermometer);
  outsideTemperature = sensors.getTempC(outsideThermometer);
  
  for(;;) {
    sensors.requestTemperatures();
    
    newInsideTemperature = sensors.getTempC(insideThermometer);
    if (!isnan(newInsideTemperature) and fabs(newInsideTemperature - insideTemperature) <= acceptableRangeOfTemperatureChange) {
//      insideTemperature = (insideTemperature + newInsideTemperature) / 2;
      insideTemperature = newInsideTemperature;
    }
    
    newOutsideTemperature = sensors.getTempC(outsideThermometer);
    if (!isnan(newOutsideTemperature) and fabs(newOutsideTemperature - outsideTemperature) <= acceptableRangeOfTemperatureChange) {
//      outsideTemperature = (outsideTemperature + newOutsideTemperature) / 2;
      outsideTemperature = newOutsideTemperature;
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// State
void setPWMPower()
{
  if (insideTemperature > 300) {
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
}

void heaterTemperatureControl()
{
  if (targetTemperature <= 50 or currentState == STATE_INIT) {
    heaterDuty = 0;
    baseHeaterDuty = 0;
  } else {
    unsigned int timeSinceLastStateChange = millis() - lastHeaterStateChanged;

    int temperatureError = targetTemperature - insideTemperature;
    if (insideTemperature < targetTemperature) { // need more power
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
      } else if (heaterState == HEATER_STAY and timeSinceLastStateChange > 3000) { // decrease base heater duty if it continue off for 2s.
        lastHeaterStateChanged = millis();
        baseHeaterDuty += -1;
      }
      
      heaterDuty =  baseHeaterDuty;
    }

    baseHeaterDuty = max(0, min(50, baseHeaterDuty)); 
    heaterDuty += 160;
    heaterDuty = max(160, min(210, heaterDuty)); 
  }
}

/*
 * state 0: init
 * fan spin at minimum speed
 * heater turned off
 */
bool buttonReleased;
void stateInit()
{
  if (currentState != STATE_INIT) {
    currentState = STATE_INIT;
    targetTemperature = 100;
    buttonReleased = false;
  }
  
  heaterDuty = 0;
  baseHeaterDuty = 0;

  if (insideTemperature > 50) {
    fanLevel = 20;
  } else {
    fanLevel = 0;
  }
  
  fanSpeedControl();
  
  if (M5.BtnA.wasPressed()) {
    targetTemperature--;
  }

  if (M5.BtnB.wasReleased())
    buttonReleased = true;

  if (buttonReleased and M5.BtnB.pressedFor(LONG_PRESSED_DURATION)) {
    statePreheat();
  }
  
  if (M5.BtnC.wasPressed()) {
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

  heaterTemperatureControl();

  if (M5.BtnA.wasPressed()) {
    stateInit();
  }

  if (fabs(targetTemperature - insideTemperature) < 4 and fabs(rateOfRise) < 10) {
    readyToCharge = true;
    // ready to rasting
    if (M5.BtnB.wasPressed()) {
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
  M5.Lcd.printf("%6.2f => %3.0f\n", insideTemperature, targetTemperature);

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
void stateCharge()
{
  if (currentState != STATE_CHARGE) {
    currentState = STATE_CHARGE;
    startTime = millis();

    resetTemperatureControl();
  }

  heaterTemperatureControl();

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
void stateRoasting()
{
  if (currentState != STATE_ROASTING) {
    currentState = STATE_ROASTING;
    airTemperatureRateOfRise = 20.0;
    resetTemperatureControl();
  }
  
  fanSpeedControl();
  targetTemperatureControl();
  heaterTemperatureControl();

  if (M5.BtnB.wasPressed()) {
    if (selector == HEATER_SELECTED) {
      selector = FAN_SELECTED;
    } else {
      selector = HEATER_SELECTED;
    }
  }

  if (M5.BtnA.wasPressed()) {
    if (selector == HEATER_SELECTED) {
      airTemperatureRateOfRise -= 1.0;
    } else {
      fanLevel -= 1;
    }
  }

  if (M5.BtnC.wasPressed()) {
    if (selector == HEATER_SELECTED) {
      airTemperatureRateOfRise += 1.0;
    } else {
      fanLevel += 1;
    }
  }

  if (M5.BtnB.pressedFor(2000)) {
    stateCooling();
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
  M5.Lcd.printf("ROASTING!  ");
  M5.Lcd.setTextColor(ORANGE, BLACK);
  M5.Lcd.printf("%2d:%02d\n", minutes, seconds);
  
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setTextSize(3);
  if (fanLevel == 30) {
    M5.Lcd.printf("FAN    MAX\n");
  } else {
    M5.Lcd.printf("FAN     %2d\n", fanLevel);
  }

  if (heaterDuty == 0) {
    M5.Lcd.printf("HEATER OFF %3.0f\n", airTemperatureRateOfRise);
  } else {
    M5.Lcd.printf("HEATER %3.0f C/m\n", airTemperatureRateOfRise);
  }
  
  M5.Lcd.setTextColor(RED, BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("Air Temperature\n");
  M5.Lcd.setTextSize(4);
  M5.Lcd.printf("%6.2f => %3.0f\n", insideTemperature, targetTemperature);
  
  M5.Lcd.setTextColor(BLUE, BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("Bean Temperature\n");
  M5.Lcd.setTextSize(4);
  M5.Lcd.printf("%6.2fc [%3.0f]\n", outsideTemperature, rateOfRise);
  
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
void stateCooling()
{
  if (currentState != STATE_COOLING) {
    currentState = STATE_COOLING;
    fanLevel = 20;
    startCoolingTime = millis();
    readyForReset = false;
  }

  heaterLevel = 0;
  heaterDuty = 0;
  
  fanSpeedControl();

  if (fabs(rateOfRise) < 7 and millis() - startCoolingTime > 20000) {
    // ready for next roast
    readyForReset = true;
    
    if (M5.BtnB.pressedFor(LONG_PRESSED_DURATION)) {
      stateInit();
    }
  }

  if (M5.BtnA.wasPressed()) 
    fanLevel--;

  if (M5.BtnC.wasPressed())
    fanLevel++;
}

void displayCooling()
{
  M5.Lcd.setCursor(0, 0);
  
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.println("COOLING!!");
  M5.Lcd.println();
  M5.Lcd.println();

  M5.Lcd.setTextColor(BLUE, BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.printf("Bean Temperature\n");
  M5.Lcd.setTextSize(5);
  M5.Lcd.printf("%6.2fC\n", outsideTemperature);

  M5.Lcd.println();
  M5.Lcd.println();
  M5.Lcd.setTextColor(BLACK, YELLOW);
  M5.Lcd.setTextSize(3);
  if (readyForReset)
    M5.Lcd.printf("  -   RESET  +   ");
  else
    M5.Lcd.printf("  -    FAN   +   ");
}


void modbusPolling()
{
  // write data to buffer
  au16data[2] = ((uint16_t) insideTemperature * 100);
  au16data[3] = ((uint16_t) outsideTemperature * 100);
  au16data[4] = ((uint16_t) targetTemperature);
  au16data[5] = ((uint16_t) fanLevel);
  au16data[6] = ((uint16_t) airTemperatureRateOfRise + 100.0); // offset 100
  // ..
//  au16data[9] = ((uint16_t) 0);
  au16data[10] = ((uint16_t) currentState);

  // server read buffer or write to buffer
  slave.poll(au16data, 16);

  targetTemperature = ((float) au16data[4]);
  fanLevel = ((int) au16data[5]);
  airTemperatureRateOfRise = ((float) au16data[6]) - 100.0; // offset 100
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
  M5.begin(true, false, true, true);
  M5.Speaker.mute();
  
  M5.Lcd.fillScreen(WHITE);

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

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  if (!sensors.getAddress(insideThermometer, 0)) { 
    hasError = true;
    Serial.println("Unable to find address for Device 0"); 
    M5.Lcd.printf("T inlet ERROR!");
  }
  if (!sensors.getAddress(outsideThermometer, 1)) {
    hasError = true;
    Serial.println("Unable to find address for Device 1"); 
    M5.Lcd.printf("T outlet ERROR!");
  }


  // stall if error found
  if (hasError) {
    Serial.println("ERROR");
    M5.Lcd.fillScreen(RED);
    for (;;);
  }

//  sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
//  sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(TEMPERATURE_PRECISION);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

  Serial.print("Device 1 Resolution: ");
  Serial.print(sensors.getResolution(outsideThermometer), DEC); 
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

  // start modbus
  slave.begin(19200); // 19200 baud, 8-bits, even, 1-bit stop

  stateInit();
}

void loop()
{
  modbusPolling();
  
  runState();
  
  setPWMPower();

  M5.update();
}

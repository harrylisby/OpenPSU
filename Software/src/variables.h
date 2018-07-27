#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ADS1115.h>
#include "PID_v1.h"
#include "LiquidCrystal_I2C.h"
#include "EnableInterrupt.h"

//LCD start
LiquidCrystal_I2C lcd(0x27,16,4);

//DAC/ADC Start
Adafruit_MCP4725 dac;
ADS1115 adc(0x48);

//PID_v1
double Setpoint, Input, Output;
double kpV=0.2245, kiV=0.2500,kdV=0.0000;
double kpI=5.7500, kiI=1.5000,kdI=0.0000; //20.7500 4.5000
PID psuPID(&Input,&Output,&Setpoint,kpV,kiV,kdV,DIRECT);

bool firstTime =true;

//Encoder configuration
#define pinA 2
#define pinB 3
#define pinP 5
volatile int16_t encoderPos,incrementAmount,decreaseAmount;
bool newRead,lastRead,increaseFlag,decreaseFlag,paramenterSelect;
byte oldButtonState = HIGH;  // assume switch open because of pull-up resistor
const unsigned long debounceTime = 10;  // milliseconds
unsigned long buttonPressTime;  // when the switch last changed state
bool buttonPressed = 0; // a flag variable
bool debounceFlag=true;
double debounceLastTime=0;
int prevEncoderPos=0;

//Menu System
byte currentMenuQuantity=3;
int8_t currentMenu=0;
bool pendingAction=false;
bool inMod=false,buttonPress=false;
String mode="CV";
int menu0ParamTracker=0;
int incrementing=0;
int subMenuIndex;

//Analog-Digital system
#define alertReadyPin 4
float targetVoltage = 0, targetCurrent = 0;
float currentVoltage = 0, currentCurrent = 0;
double voltsDigital = 0;
bool runOnce=true, canChange=true;
int32_t lastTime;

//Temperature System
int fanSpeed=0;
float detectedTemp=0;
#define fanOut 11

//Calibration System
float calVoltage, calCurrent, voltsFactor=5.7583, currentFactor=1;//2.4093; //This values should be written and read to EEPROM later
bool oneTime=true, calibrationConfirmed=false;

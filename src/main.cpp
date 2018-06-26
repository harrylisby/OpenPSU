#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ADS1115.h>
#include "PID_v1.h"
#include "LiquidCrystal_I2C.h"
#include "EnableInterrupt.h"

/*
********************************************************************************
OpenPSU is a digitally controlled linear power supply developed by Harry Lisby
********************************************************************************

This open source project makes possible a hackable power supply which allows you
to have a high precision psu or a high power one depending on the needs.

Uses very common signal processing modules for voltage referencing and feedback,
based on an Arduino nano with an MCP4725 10 bit DAC, an ADS1115 16 bit ADC for
cheap and high precision control. Aditionally uses an i2c 20x4 lcd screen and an
incremental encoder with pushbutton for control and configuration.

Also uses de PID_v1 library which allows fast transition between set voltages,
working modes and changing loads [output stability].

********************************************************************************
*/

//LCD start
LiquidCrystal_I2C lcd(0x27,16,4);

//DAC/ADC Start
Adafruit_MCP4725 dac;
ADS1115 adc(0x48);

//PID_v1
double Setpoint, Input, Output;
double kpV=0.2250, kiV=0.1000,kdV=0.0000;
double kpI=20.7500, kiI=4.5000,kdI=0.0000;
PID psuPID(&Input,&Output,&Setpoint,kpV,kiV,kdV,DIRECT);

//Encoder configuration
#define pinA 2
#define pinB 3
#define pinP 5
volatile int32_t encoderPos;
bool newRead,lastRead,increaseFlag,decreaseFlag,paramenterSelect;
byte oldButtonState = HIGH;  // assume switch open because of pull-up resistor
const unsigned long debounceTime = 10;  // milliseconds
unsigned long buttonPressTime;  // when the switch last changed state
bool buttonPressed = 0; // a flag variable

#define alertReadyPin 4
float targetVoltage = 0, targetCurrent = 0;
float currentVoltage = 0, currentCurrent = 0;
double voltsDigital = 0;
bool runOnce = true;
int32_t lastTime;

String mode = "CV";

void reader(){
  newRead = digitalRead(pinA);
  if((lastRead == LOW)&&(newRead == HIGH)){
    if(digitalRead(pinB)==LOW){
      encoderPos++;
      increaseFlag=true;
    }else{
      encoderPos--;
      decreaseFlag=true;
    }
  }
  lastRead = digitalRead(pinA);
}

void setup(void) {
  Wire.begin();
  Serial.begin(115200);
  Serial.println();Serial.println();
  Serial.println("OpenPSU Version: b0.2");
  Serial.print("Made by Harry Lisby - ");
  Serial.println("github.com/harrylisby/OpenPSU");
  Serial.println();Serial.println();

  dac.begin(0x64);
  Serial.println(adc.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");
  adc.initialize();
  adc.setMode(ADS1115_MODE_SINGLESHOT);
  adc.setRate(ADS1115_RATE_64);
  adc.setGain(ADS1115_PGA_6P144);
  pinMode(alertReadyPin,INPUT_PULLUP);
  adc.setConversionReadyPinMode();

  lcd.begin();                      // initialize the lcd
  lcd.backlight();

  targetVoltage = 0;
  targetCurrent = 0;

  psuPID.SetOutputLimits(-511,511);
  psuPID.SetSampleTime(1);
  psuPID.SetMode(AUTOMATIC);

  dac.setVoltage(0,false); //Start with output low

  pinMode(pinA,INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinP,INPUT_PULLUP);
  enableInterrupt(pinA, reader, CHANGE);
}

void pollAlertReadyPin() {
  for (uint32_t i = 0; i<100000; i++)
    if (!digitalRead(alertReadyPin)) return;
    Serial.println("Failed to wait for AlertReadyPin, it's stuck high!");
}

//parameter should be (pinP,oldButtonState)
int buttonWatcher(int buttonPin, bool oldState){
  byte buttonState = digitalRead(buttonPin);
  if (buttonState != oldState){
    if (millis() - buttonPressTime >= debounceTime){ // debounce adjusted at 10ms
      buttonPressTime = millis();  // when we closed the switch
      oldState = buttonState;  // remember for next time
      if (buttonState == LOW){
        Serial.println("Button pressed"); // DEBUGGING: print that button has been closed
        buttonPressed = 1;
      }
      else {
        Serial.println("Button released"); // DEBUGGING: print that button has been opened
        buttonPressed = 0;
      }
    }
  }
  return buttonPressed;
}

void loop(void) {
  // The below method sets the mux for channel one [volts]
  adc.setMultiplexer(ADS1115_MUX_P0_NG);
  adc.triggerConversion();
  pollAlertReadyPin();
  currentVoltage = 6.035*(adc.getMilliVolts(false));
  // The below method sets the mux for channel two [current]
  adc.setMultiplexer(ADS1115_MUX_P1_NG);
  adc.triggerConversion();
  pollAlertReadyPin();
  currentCurrent = (adc.getMilliVolts(false));

  if(currentCurrent>targetCurrent){     //If current is above the setpoint
    psuPID.SetTunings(kpI, kiI, kdI);
    Input = currentCurrent;
    Setpoint = targetCurrent;
    mode = "CI";
  }else{                                //If current is below the setpoint
    psuPID.SetTunings(kpV, kiV, kdV);
    Input = currentVoltage;
    Setpoint = targetVoltage;
    mode = "CV";
  }

  if(digitalRead(pinP)==LOW){
    paramenterSelect=!paramenterSelect;
    Serial.println(paramenterSelect);
    delay(500);  //Find a way to antibounce and prevent spam detection
  }
  if((increaseFlag)&&!(paramenterSelect)){
    targetVoltage+=100;
    increaseFlag=false;
    Serial.println(targetVoltage);
  }else if((increaseFlag)&&(paramenterSelect)){
    targetCurrent+=10;
    increaseFlag=false;
    Serial.println(targetCurrent);
  }
  if((decreaseFlag)&&!(paramenterSelect)){
    targetVoltage-=100;
    decreaseFlag=false;
    Serial.println(targetVoltage);
  }else if((decreaseFlag)&&(paramenterSelect)){
    targetCurrent-=10;
    decreaseFlag=false;
    Serial.println(targetCurrent);
  }

  psuPID.Compute();
  voltsDigital+=Output;                 //Sums PID output to current output
  voltsDigital=constrain(voltsDigital, 0, 4095);
  dac.setVoltage(voltsDigital, false);

  if((millis()-lastTime)>=500){
    lastTime=millis();
    //Uncomment next section for serial debugging

    Serial.print(voltsDigital);Serial.println(": ");
    //Serial.print(currentVoltage); Serial.print(": ");
    //Serial.println(Output); //Serial.print(": ");
    //Serial.print(currentCurrent); Serial.println();
    //Serial.println(encoderPos);
    //Serial.println(digitalRead(pinP));

    //lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OpenPSU - HarryLisby");
    lcd.setCursor(0, 1);
    lcd.print("Vt: ");
    lcd.print(targetVoltage/1000);
    lcd.print("V  ");
    lcd.setCursor(10, 1);
    lcd.print("It: ");
    lcd.print(targetCurrent/1000);
    lcd.print("A ");
    lcd.setCursor(0, 2);
    lcd.print("V: ");
    lcd.print(currentVoltage/1000);
    lcd.print("V  ");
    lcd.setCursor(11, 2);
    lcd.print("I: ");
    lcd.print(currentCurrent/1000);
    lcd.print("A ");
    lcd.setCursor(0, 3);
    lcd.print("P: ");
    lcd.print(((currentVoltage/1000)*(currentCurrent/1000)));
    lcd.print("W  ");
    lcd.setCursor(11, 3);
    lcd.print("M: ");
    lcd.print(mode);

  }
}

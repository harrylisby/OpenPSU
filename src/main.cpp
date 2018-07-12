/*******************************************************************************
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

*******************************************************************************/

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
byte currentMenuQuantity=2;
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

//Calibration System
float calVoltage, calCurrent, voltsFactor=6.035, currentFactor=2; //This values should be written and read to EEPROM later

void reader(){
  newRead = digitalRead(pinA);
  if((lastRead == LOW)&&(newRead == HIGH)){
    if(digitalRead(pinB)==LOW){
      encoderPos--;
    }else{
      encoderPos++;
    }
  }
  lastRead = digitalRead(pinA);
}

bool buttonRead(){
  bool pendingPress=false;
  bool pinPStatus=digitalRead(pinP);
  if((pinPStatus==LOW)&&debounceFlag&&canChange){
    pendingPress=true; //Sets the pendingPress to true if not debouncing and pressed
    debounceFlag=false;
    canChange=false;
  }else if((pinPStatus==HIGH)&&!canChange){
    pendingPress=false; //if debouncing is done and button released sets flag to false
    canChange=true;
  }
  if(!debounceFlag){
    debounceLastTime=millis()-debounceLastTime;
    if(debounceLastTime>100){
      debounceFlag=true;
    }
  }
  return pendingPress;
}

int pendingIncrements(){
  int newDifference = 0;
  if(prevEncoderPos!=encoderPos){
    newDifference=prevEncoderPos-encoderPos;
  }
  prevEncoderPos = encoderPos;
  return newDifference;
}

void pollAlertReadyPin() {
  for (uint32_t i = 0; i<100000; i++)
    if (!digitalRead(alertReadyPin)) return;
    Serial.println("Failed to wait for AlertReadyPin, it's stuck high!");
}

int adcRead(byte channel,double scaler = 1){
  int read = 0;
  switch (channel) {
    case 0:
      adc.setMultiplexer(ADS1115_MUX_P0_NG);
      adc.triggerConversion();
      pollAlertReadyPin();
      read = scaler*(adc.getMilliVolts(false));
      break;
    case 1:
      adc.setMultiplexer(ADS1115_MUX_P1_NG);
      adc.triggerConversion();
      pollAlertReadyPin();
      read = scaler*(adc.getMilliVolts(false));
      break;
    case 2:
      adc.setMultiplexer(ADS1115_MUX_P1_NG);
      adc.triggerConversion();
      pollAlertReadyPin();
      read = scaler*(adc.getMilliVolts(false));
      break;
    case 3:
      adc.setMultiplexer(ADS1115_MUX_P1_NG);
      adc.triggerConversion();
      pollAlertReadyPin();
      read = scaler*(adc.getMilliVolts(false));
      break;
  }
  return read;
}

//CALIBRATIONROUTINES///////////////////////////////////////////////////////////

void calValues(void){
  //ADD EEPROM FUNCTIONALITY
  voltsFactor = calVoltage/currentVoltage;
  currentFactor = calCurrent/currentCurrent;

}

//MENUSYSTEM////////////////////////////////////////////////////////////////////

void menu0(){
  if(pendingAction||buttonPress){
    if(buttonPress){
      subMenuIndex++;
      if(subMenuIndex>2)subMenuIndex=0;
      Serial.println("SubMenuIndex: "+String(subMenuIndex));
      if(subMenuIndex==1||subMenuIndex==2){
        inMod=true;
      }else{
        inMod=false;
      }
    }
    buttonPress=false;
    if(pendingAction){
      if((incrementing!=0)&&subMenuIndex==1){
        targetVoltage+=100*incrementing;
        incrementing=0;
        Serial.println("Volts incremented");

      }else if((incrementing!=0)&&subMenuIndex==2){
        targetCurrent+=10*incrementing;
        incrementing=0;
        Serial.println("Amps incremented");

      }else if(subMenuIndex==0){
        subMenuIndex=0;
        inMod=false;
        Serial.println("Exiting submenu");
        lcd.noCursor();
      }
    }
  }
  //lcd.setCursor(0, 0);
  //lcd.print("OpenPSU - HarryLisby");
  lcd.setCursor(0, 0);
  lcd.print("V: ");
  lcd.print(currentVoltage/1000);
  lcd.print("V  ");
  lcd.setCursor(11, 0);
  lcd.print("I: ");
  lcd.print(currentCurrent/1000);
  lcd.print("A ");
  lcd.setCursor(0, 1);
  lcd.print("P: ");
  lcd.print(((currentVoltage/1000)*(currentCurrent/1000)));
  lcd.print("W  ");
  lcd.setCursor(11, 1);
  lcd.print("M: ");
  lcd.print(mode);
  lcd.setCursor(0, 3);
  lcd.print("Vt:");
  lcd.print(targetVoltage/1000);
  lcd.print("V ");
  lcd.setCursor(11, 3);
  lcd.print("It:");
  lcd.print(targetCurrent/1000);
  lcd.print("A ");

  if(subMenuIndex==1){
    lcd.setCursor(0,3);
    lcd.blink();
  }else if(subMenuIndex==2){
    lcd.setCursor(11,3);
    lcd.blink();
  }else{
    lcd.noBlink();
  }

}

void menu1(){
  if(pendingAction||buttonPress){
    if(buttonPress){
      subMenuIndex++;
      if(subMenuIndex>2)subMenuIndex=0;
      Serial.println("SubMenuIndex: "+String(subMenuIndex));
      if(subMenuIndex==1||subMenuIndex==2){
        inMod=true;
      }else{
        inMod=false;
      }
    }
    buttonPress=false;
    if(pendingAction){
      if((incrementing!=0)&&subMenuIndex==1){
        targetVoltage=10000;
        calVoltage+=incrementing;
        incrementing=0;
        Serial.println("CalVolts incremented");
      }else if((incrementing!=0)&&subMenuIndex==2){
        targetCurrent=1000;
        calCurrent+=incrementing;
        incrementing=0;
        Serial.println("CalAmps incremented");
      }else if(subMenuIndex==0){
        subMenuIndex=0;
        inMod=false;
        Serial.println("VF: "+String(voltsFactor));
        Serial.println("IF: "+String(currentFactor));
        Serial.println("Exiting submenu");
        lcd.noCursor();
        calValues();
      }
    }
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibration");
  lcd.setCursor(0,2);
  lcd.print("Mea.volt: ");
  lcd.print(calVoltage);
  lcd.print("mV");
  lcd.setCursor(0,3);
  lcd.print("Mea.curr: ");
  lcd.print(calCurrent);
  lcd.print("mA");

  if(subMenuIndex==1){
    lcd.setCursor(0,2);
    lcd.blink();
  }else if(subMenuIndex==2){
    lcd.setCursor(0,3);
    lcd.blink();
  }else{
    lcd.noBlink();
  }
}

void menu2(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("TestMenu");
}

void writeLCD(int menuIndex){
  switch(menuIndex){
    case 0:
      menu0();
      break;
    case 1:
      menu1();
      break;
    case 2:
      menu2();
      break;
  }
}

//EXTRAS////////////////////////////////////////////////////////////////////////

void serialDebugging(void){
  //Uncomment next section for serial debugging
  //depending on the data you want to get
  //Serial.print(voltsDigital);Serial.print(": ");
  //Serial.print(currentVoltage); Serial.print(": ");
  //Serial.println(Output); //Serial.println(": ");
  //Serial.print(currentCurrent); Serial.println();
  //Serial.println(encoderPos);
  //Serial.println(digitalRead(pinP));

}

//SETUP/////////////////////////////////////////////////////////////////////////

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

  lcd.setCursor(7, 1);
  lcd.print("OpenPSU");
  lcd.setCursor(5, 2);
  lcd.print("Harry Lisby");
  delay(2000);
  lcd.clear();
  writeLCD(0);
}

//PSUMAINROUTINE////////////////////////////////////////////////////////////////

void psuHandle(){
  currentVoltage = adcRead(0,voltsFactor); // This method sets the mux for channel one [volts]
  currentCurrent = adcRead(1,currentFactor);  // This method sets the mux for channel two [current]

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

  psuPID.Compute();
  voltsDigital+=Output;                 //Sums PID output to current output
  voltsDigital=constrain(voltsDigital, 0, 4095);
  dac.setVoltage(voltsDigital, false);
}

//MENUHANDLER///////////////////////////////////////////////////////////////////

void menuHandle(){
  incrementing=pendingIncrements();
  if((incrementing!=0)&&!inMod){
    if(incrementing>0)currentMenu++;
    if(incrementing<0)currentMenu--;
    if(currentMenu>currentMenuQuantity)currentMenu=0;
    if(currentMenu<=(-1))currentMenu=currentMenuQuantity;
    writeLCD(currentMenu);
    Serial.println("Menu changed "+String(incrementing)+" "+String(currentMenu));
  }else if((incrementing!=0)&&inMod){
    pendingAction=true;
    writeLCD(currentMenu);
    Serial.println("Modifying submenu " + String(subMenuIndex));
  }

  if(buttonRead()){
    buttonPress=true;
    writeLCD(currentMenu);
    Serial.println("Button read");
  }
  if((millis()-lastTime)>=1000){
    writeLCD(currentMenu);
    lastTime=millis();
    serialDebugging();
  }
}

//LOOP//////////////////////////////////////////////////////////////////////////

void loop(void) {
  psuHandle();
  menuHandle();
}

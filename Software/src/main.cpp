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

#include <variables.h>

//INPUTHANDLER//////////////////////////////////////////////////////////////////

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

//CALIBRATIONROUTINE////////////////////////////////////////////////////////////

void calValues(void){
  //ADD EEPROM FUNCTIONALITY
  currentVoltage = adcRead(0,1);
  currentCurrent = adcRead(1,1);
  voltsFactor = calVoltage/currentVoltage;
  currentFactor = calCurrent/currentCurrent;

}

//BATTERYCHARGERMODULE//////////////////////////////////////////////////////////

void batteryChargerModule(float battLowVolts, float battHighVolts, float battMaxCurrent){
  //todo: a way out of here
  targetVoltage=battHighVolts;
  targetCurrent=battMaxCurrent;
  currentMenu=5;
  lcd.clear();
  Serial.println(String(battLowVolts) + "-" + String(battHighVolts));
}

//MENUSYSTEM////////////////////////////////////////////////////////////////////

void menu0(){ //MAIN MENU
  if(pendingAction||buttonPress){
    if(buttonPress){
      subMenuIndex++;
      if(subMenuIndex>2)subMenuIndex=0;
      //Serial.println("SubMenuIndex: "+String(subMenuIndex));
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
        //Serial.println("Volts incremented");
      }else if((incrementing!=0)&&subMenuIndex==2){
        targetCurrent+=10*incrementing;
        incrementing=0;
        //Serial.println("Amps incremented");
      }else if(subMenuIndex==0){
        subMenuIndex=0;
        inMod=false;
        //Serial.println("Exiting submenu");
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

void menu1(){ //CAL MENU
  if(pendingAction||buttonPress){
    if(buttonPress){
      subMenuIndex++;
      if(subMenuIndex>2)subMenuIndex=0;
      //Serial.println("SubMenuIndex: "+String(subMenuIndex));
      if(subMenuIndex==1||subMenuIndex==2){
        inMod=true;
      }else{
        inMod=false;
      }
    }
    buttonPress=false;
    if(pendingAction){
      if((incrementing!=0)&&subMenuIndex==1){
        //targetVoltage=1000;
        //targetCurrent=1000;
        calVoltage+=10*incrementing;
        incrementing=0;
        //Serial.println("CalVolts incremented");
      }else if((incrementing!=0)&&subMenuIndex==2){
        calCurrent+=incrementing;
        incrementing=0;
        calibrationConfirmed=true;
        //Serial.println("CalAmps incremented");
      }else if(subMenuIndex==0){
        subMenuIndex=0;
        inMod=false;
        if(oneTime&&calibrationConfirmed){
          oneTime=false;
          calibrationConfirmed=false;
          calValues();
          Serial.println("VF: "+String(voltsFactor));
          Serial.println("IF: "+String(currentFactor));
          Serial.println("Exiting submenu");
          lcd.noCursor();
        }
      }
    }
  }

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

void menu2(){ //PID MENU
  if(pendingAction||buttonPress){
    if(buttonPress){
      subMenuIndex++;
      const byte menu2Mods=6;
      if(subMenuIndex>menu2Mods)subMenuIndex=0;
      //Serial.println("SubMenuIndex: "+String(subMenuIndex));
      if(subMenuIndex>=1 && subMenuIndex<=menu2Mods){
        inMod=true;
      }else{
        inMod=false;
      }
    }
    buttonPress=false;
    if(pendingAction){
      if((incrementing!=0)&&subMenuIndex==1){
        kpV+=0.01*incrementing;
        incrementing=0;
      }else if((incrementing!=0)&&subMenuIndex==2){
        kiV+=0.01*incrementing;
        incrementing=0;
      }else if((incrementing!=0)&&subMenuIndex==3){
        kdV+=0.01*incrementing;
        incrementing=0;
      }else if((incrementing!=0)&&subMenuIndex==4){
        kpI+=0.01*incrementing;
        incrementing=0;
      }else if((incrementing!=0)&&subMenuIndex==5){
        kiI+=0.01*incrementing;
        incrementing=0;
      }else if((incrementing!=0)&&subMenuIndex==6){
        kdI+=0.01*incrementing;
        incrementing=0;
      }else if(subMenuIndex==0){
        subMenuIndex=0;
        inMod=false;
        if(oneTime){
          oneTime=false;
          Serial.println("Changed PID tunnings");
          lcd.noCursor();
        }
      }
    }
  }

  lcd.setCursor(0,0);
  lcd.print("PID tunning:");
  lcd.setCursor(0, 1);
  lcd.print("KpV: ");
  lcd.print(kpV);
  lcd.setCursor(0, 2);
  lcd.print("KiV: ");
  lcd.print(kiV);
  lcd.setCursor(0, 3);
  lcd.print("KdV: ");
  lcd.print(kdV);
  lcd.setCursor(10, 1);
  lcd.print("KpI: ");
  lcd.print(kpI);
  lcd.setCursor(10, 2);
  lcd.print("KiI: ");
  lcd.print(kiI);
  lcd.setCursor(10, 3);
  lcd.print("KdI: ");
  lcd.print(kdI);

  if(subMenuIndex!=0 && subMenuIndex<=3){
    lcd.setCursor(0,subMenuIndex);
    lcd.blink();
  }else if(subMenuIndex>=4 && subMenuIndex<=6){
    lcd.setCursor(10,(subMenuIndex-3));
    lcd.blink();
  }else{
    lcd.noBlink();
  }
}

void menu3(){ //TEST MENU
  if(pendingAction||buttonPress){
    if(buttonPress){
      subMenuIndex++;
      if(subMenuIndex>2)subMenuIndex=0;
      //Serial.println("SubMenuIndex: "+String(subMenuIndex));
      if(subMenuIndex==1||subMenuIndex==2){
        inMod=true;
      }else{
        inMod=false;
      }
    }
    buttonPress=false;
    if(pendingAction){
      if((incrementing!=0)&&subMenuIndex==1){
        fanSpeed+=5*incrementing;
        incrementing=0;
        //Serial.println("Volts incremented");
      }else if((incrementing!=0)&&subMenuIndex==2){
        detectedTemp+=0.10*incrementing;
        //Serial.println("Amps incremented");
      }else if(subMenuIndex==0){
        subMenuIndex=0;
        inMod=false;
        //Serial.println("Exiting submenu");
        lcd.noCursor();
      }
    }
  }
  //lcd.setCursor(0, 0);
  //lcd.print("OpenPSU - HarryLisby");
  lcd.setCursor(0, 0);
  lcd.print("Testing Menu");
  lcd.setCursor(0, 1);
  lcd.print("Fan Speed: ");
  lcd.print(fanSpeed);
  lcd.print("  ");
  lcd.setCursor(0, 2);
  lcd.print("Mea. Temp: ");
  lcd.print(detectedTemp);
  lcd.print("  ");

  if(subMenuIndex==1){
    lcd.setCursor(0,1);
    lcd.blink();
  }else if(subMenuIndex==2){
    lcd.setCursor(0,2);
    lcd.blink();
  }else{
    lcd.noBlink();
  }
}

void menu4(){ //BATTERY CHARGER
  if(pendingAction||buttonPress){
    if(buttonPress){
      subMenuIndex++;
      const byte menu4Mods=5;
      if(subMenuIndex>menu4Mods)subMenuIndex=0;
      if(subMenuIndex>=1 && subMenuIndex<=menu4Mods){
        inMod=true;
      }else{
        inMod=false;
      }
    }
    buttonPress=false;
    if(pendingAction){
      if((incrementing!=0)&&subMenuIndex==1){
        currentBatteryType++;
        if(currentBatteryType>2)currentBatteryType=0;
        incrementing=0;
        //Serial.println("Changed battery type to: " + batteryType[currentBatteryType] );
      }else if((incrementing!=0)&&subMenuIndex==2){
        maximumChargeCurrent+=10*incrementing;
        if(maximumChargeCurrent<0)maximumChargeCurrent=0;
        incrementing=0;
      }else if((incrementing!=0)&&subMenuIndex==3){
        maximumChargeCurrent+=100*incrementing;
        if(maximumChargeCurrent<0)maximumChargeCurrent=0;
        incrementing=0;
      }else if((incrementing!=0)&&subMenuIndex==4){
        numberOfCells+=incrementing;
        if(numberOfCells>9)numberOfCells=1;
        incrementing=0;
      }else if((incrementing!=0)&&subMenuIndex==5){
        if(!chargeConfirm){
          chargeConfirm=true;
        }else{
          chargeConfirm=false;
        }
        incrementing=0;
      }else if(subMenuIndex==0){
        subMenuIndex=0;
        inMod=false;
        //targetCurrent=maximumChargeCurrent;
        //Serial.println("Exiting submenu");
        lcd.noCursor();
        if(chargeConfirm){
          currentMenu=5;
          chargeConfirm=false;
          lcd.clear();
          batteryChargerModule(minimumVoltage[currentBatteryType],maximumVoltage[currentBatteryType],maximumChargeCurrent);
        }
      }
    }
  }

  lcd.setCursor(0, 0);
  lcd.print("Battery Charger");
  lcd.setCursor(0, 1);
  lcd.print("Type: ");
  lcd.print(batteryType[currentBatteryType]);
  lcd.setCursor(0, 2);
  lcd.print("Max I: ");
  lcd.print(maximumChargeCurrent/1000);
  lcd.setCursor(0, 3);
  lcd.print("Cells: ");
  lcd.print(numberOfCells);

  /*

  FUNCTIONS NEEDED TO BE ADDED IN HERE:
    - Show current values and battery %
    - extra: voltage is still lost in menus switching

  */

  if(subMenuIndex==1){
    lcd.setCursor(0,1);
    lcd.blink();
  }else if(subMenuIndex==2){
    lcd.setCursor(0,2);
    lcd.blink();
    lcd.setCursor(10,2);
    lcd.cursor();
  }else if(subMenuIndex==3){
    lcd.setCursor(0,2);
    lcd.blink();
    lcd.setCursor(9,2);
    lcd.cursor();
  }else if(subMenuIndex==4){
    lcd.setCursor(0,3);
    lcd.blink();
  }else if(subMenuIndex==5){
    lcd.setCursor(0, 3);
    lcd.print("Confirm? ");
    lcd.print(" Yes  No");
    if(chargeConfirm){
      lcd.setCursor(9, 3);
      lcd.blink();
    }else if(!chargeConfirm){
      lcd.setCursor(14, 3);
      lcd.blink();
    }
  }else{
    lcd.noBlink();
    lcd.noCursor();
  }
}

void chargingMenu(){
  lcd.setCursor(0, 0);
  lcd.print("OpenPSU Charger");
  lcd.setCursor(0, 2);
  lcd.print("Charge: ");
  lcd.print((currentVoltage/targetVoltage)*100);
  lcd.print("%   ");
  lcd.setCursor(0, 3);
  lcd.print("Current: ");
  lcd.print(currentCurrent/1000);
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
    case 3:
      menu3();
      break;
    case 4:
      menu4();
      break;
    case 5:
      chargingMenu();
      break;
  }
}

//EXTRAS////////////////////////////////////////////////////////////////////////

void serialDebugging(void){
  //Uncomment next section for serial debugging
  //depending on the data you want to get
  //Serial.print("Digital Volts: ");Serial.println(voltsDigital);
  //Serial.print("mV: ");Serial.println(currentVoltage);
  //Serial.print("mA: "); Serial.println(currentCurrent);
  //Serial.print("PID output: ");Serial.println(Output);
  //Serial.print("Encoder: ");Serial.println(encoderPos);
  Serial.print("Menu: ");Serial.println(currentMenu);

}

//SETUP/////////////////////////////////////////////////////////////////////////

void setup(void) {
  Wire.begin();
  Serial.begin(115200);
  Serial.println();Serial.println();
  Serial.println("OpenPSU Version: b0.6");
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

  psuPID.SetOutputLimits(PIDMinimum,PIDMaximum);
  psuPID.SetSampleTime(1);
  psuPID.SetMode(AUTOMATIC);

  dac.setVoltage(0,false); //Start with output low

  pinMode(pinA,INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinP,INPUT_PULLUP);
  enableInterrupt(pinA, reader, CHANGE);
  pinMode(fanOut,OUTPUT);

  analogWrite(fanOut, 255);

  lcd.setCursor(7, 1);
  lcd.print("OpenPSU");
  lcd.setCursor(5, 2);
  lcd.print("Harry Lisby");
  delay(1000);
  lcd.clear();
  writeLCD(0);
}

//PSUMAINROUTINE////////////////////////////////////////////////////////////////

void psuHandle(){
  currentVoltage = adcRead(0,voltsFactor); // This method sets the mux for channel one [volts]
  currentCurrent = adcRead(1,currentFactor);  // This method sets the mux for channel two [current]

  if(currentCurrent>overCurrentLimit){
    dac.setVoltage(0,false);
    targetVoltage=0;
    targetCurrent=0;
    lcd.setCursor(0, 2);
    lcd.print("ERROR: OVER CURRENT!");
  }

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

  if(previousVoltage!=targetVoltage||previousCurrent!=targetCurrent){
    previousVoltage=targetVoltage;
    previousCurrent=targetCurrent;
    psuPID.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    psuPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
    psuPID.SetOutputLimits(PIDMinimum, PIDMaximum);  // Set the limits back to normal
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
    lcd.clear();
    writeLCD(currentMenu);
    //Serial.println("Menu changed "+String(incrementing)+" "+String(currentMenu));
  }else if((incrementing!=0)&&inMod){
    pendingAction=true;
    writeLCD(currentMenu);
    //Serial.println("Modifying submenu " + String(subMenuIndex));
  }

  if(buttonRead()){
    buttonPress=true;
    writeLCD(currentMenu);
    Serial.println("Button pressed");
  }
}

//TEMPSYSTEM////////////////////////////////////////////////////////////////////

void tempSystem(){
  fanSpeed=map(currentCurrent,0,3000,50,255);
  fanSpeed=constrain(fanSpeed, 30, 255);
  analogWrite(fanOut,fanSpeed);
}

//LOOP//////////////////////////////////////////////////////////////////////////

void loop(void) {
  psuHandle();
  menuHandle();

  //The next lines only happen every one second
  if((millis()-lastTime)>=1000){
    writeLCD(currentMenu);
    lastTime=millis();
    serialDebugging();
    tempSystem();
  }
}

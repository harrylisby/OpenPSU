
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ADS1115.h>
#include <PID_v1.h>

//DAC/ADC Start
Adafruit_MCP4725 dac;
ADS1115 adc(0x48);

//PID_v1
double Setpoint, Input, Output;
double kpV=0.2250, kiV=0.1000,kdV=0.0000;
double kpI=20.7500, kiI=4.5000,kdI=0.0000;
PID psuPID(&Input,&Output,&Setpoint,kpV,kiV,kdV,DIRECT);

const int alertReadyPin = 2;
float targetVoltage = 0, targetCurrent = 0;
float currentVoltage = 0, currentCurrent = 0;
double voltsDigital = 0;
bool runOnce = true;
int32_t lastTime;

void setup(void) {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("OpenPSU Version: 0.0");
  dac.begin(0x64);
  Serial.println(adc.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");
  adc.initialize();
  adc.setMode(ADS1115_MODE_SINGLESHOT);
  adc.setRate(ADS1115_RATE_64);
  adc.setGain(ADS1115_PGA_6P144);
  pinMode(alertReadyPin,INPUT_PULLUP);
  adc.setConversionReadyPinMode();

  targetVoltage = 25000;
  targetCurrent = 100;

  psuPID.SetOutputLimits(-127,127);
  psuPID.SetSampleTime(1);
  psuPID.SetMode(AUTOMATIC);

  dac.setVoltage(0,false); //Start with output low
}

void pollAlertReadyPin() {
  for (uint32_t i = 0; i<100000; i++)
    if (!digitalRead(alertReadyPin)) return;
    Serial.println("Failed to wait for AlertReadyPin, it's stuck high!");
}

void loop(void) {
  // The below method sets the mux and gets a reading.
  adc.setMultiplexer(ADS1115_MUX_P0_NG);
  adc.triggerConversion();
  pollAlertReadyPin();
  currentVoltage = 6.035*(adc.getMilliVolts(false));

  adc.setMultiplexer(ADS1115_MUX_P1_NG);
  adc.triggerConversion();
  pollAlertReadyPin();
  currentCurrent = (adc.getMilliVolts(false));

  if(currentCurrent>targetCurrent){
    psuPID.SetTunings(kpI, kiI, kdI);
    Input = currentCurrent;
    Setpoint = targetCurrent;
  }else{
    psuPID.SetTunings(kpV, kiV, kdV);
    Input = currentVoltage;
    Setpoint = targetVoltage;
  }

  psuPID.Compute();
  voltsDigital+=Output;
  voltsDigital=constrain(voltsDigital, 0, 4095);
  dac.setVoltage(voltsDigital, false);

  if((millis()-lastTime)>=1000){
    lastTime=millis();
    Serial.print(voltsDigital);Serial.print(": ");
    Serial.print(currentVoltage); Serial.print(": ");
    Serial.print(Output); Serial.print(": ");
    Serial.print(currentCurrent); Serial.println();
  }
}

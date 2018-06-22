
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
double kp=0.2300, ki=0.1000,kd=0.0000;
PID psuPID(&Input,&Output,&Setpoint,kp,ki,kd,DIRECT);

const int alertReadyPin = 2;
float targetVoltage = 0;
float currentVoltage = 0;
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

  Setpoint = 5000;
  psuPID.SetOutputLimits(-127,127);
  psuPID.SetSampleTime(1);
  psuPID.SetMode(AUTOMATIC);

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
  if((millis()-lastTime)>=1000){
    lastTime=millis();
    Serial.print(voltsDigital);Serial.print(": ");
    Serial.print(currentVoltage); Serial.print(": "); Serial.println(Output);
  }

  Input = currentVoltage;
  psuPID.Compute();

  if(runOnce){
    dac.setVoltage(0,false);
    runOnce=false;
  }

  voltsDigital+=Output;
  if(voltsDigital>4095)voltsDigital=4095;
  if(voltsDigital<0)voltsDigital=0;
  dac.setVoltage(voltsDigital, false);
}

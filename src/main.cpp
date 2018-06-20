
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ADS1115.h>

Adafruit_MCP4725 dac;
ADS1115 adc(0x48);

const int alertReadyPin = 2;
float targetVoltage = 0;
float currentVoltage = 0;
int voltsDigital = 0;
bool runOnce = true;

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
  Serial.print(voltsDigital);Serial.print(": ");
  Serial.println(currentVoltage);
  targetVoltage = 15000.00;

  if(voltsDigital>4095)voltsDigital=4095;
  if(voltsDigital<0)voltsDigital=0;

  if(runOnce){
    dac.setVoltage(0,false);
    runOnce=false;
  }

  if(targetVoltage<currentVoltage){
    voltsDigital--;
    dac.setVoltage(voltsDigital, false);
  }else if(targetVoltage>currentVoltage){
    voltsDigital++;
    dac.setVoltage(voltsDigital, false);
  }
}


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ADS1115.h>

Adafruit_MCP4725 dac;
ADS1115 adc(0x48);

const int alertReadyPin = 2;

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
    dac.setVoltage(2048, false);

    // The below method sets the mux and gets a reading.
    adc.setMultiplexer(ADS1115_MUX_P0_NG);
    adc.triggerConversion();
    pollAlertReadyPin();
    Serial.print("A0: "); Serial.print(adc.getMilliVolts(false)); Serial.print("mV\t");

    adc.setMultiplexer(ADS1115_MUX_P1_NG);
    adc.triggerConversion();
    pollAlertReadyPin();
    Serial.print("A1: "); Serial.print(adc.getMilliVolts(false)); Serial.print("mV\t");

    adc.setMultiplexer(ADS1115_MUX_P2_NG);
    adc.triggerConversion();
    pollAlertReadyPin();
    Serial.print("A2: "); Serial.print(adc.getMilliVolts(false)); Serial.print("mV\t");

    adc.setMultiplexer(ADS1115_MUX_P3_NG);
    // Do conversion polling via I2C on this last reading:
    Serial.print("A3: "); Serial.print(adc.getMilliVolts(true)); Serial.print("mV");

    Serial.println(digitalRead(alertReadyPin));

    delay(1);

}

/*

void setup()
{
Wire.begin();

Serial.begin(115200);
while (!Serial);             // Leonardo: wait for serial monitor
Serial.println("\nI2C Scanner");
}


void loop()
{
byte error, address;
int nDevices;

Serial.println("Scanning...");

nDevices = 0;
for(address = 1; address < 127; address++ )
{
// The i2c_scanner uses the return value of
// the Write.endTransmisstion to see if
// a device did acknowledge to the address.
Wire.beginTransmission(address);
error = Wire.endTransmission();

if (error == 0)
{
Serial.print("I2C device found at address 0x");
if (address<16)
Serial.print("0");
Serial.print(address,HEX);
Serial.println("  !");

nDevices++;
}
else if (error==4)
{
Serial.print("Unknown error at address 0x");
if (address<16)
Serial.print("0");
Serial.println(address,HEX);
}
}
if (nDevices == 0)
Serial.println("No I2C devices found\n");
else
Serial.println("done\n");

delay(5000);           // wait 5 seconds for next scan
}
*/

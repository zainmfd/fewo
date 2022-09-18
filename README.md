# FightFire-_in_edge

I tester the new Grove - VOC and eCO2 Gas Sensor(SGP30) with using code written below and this code was source from seeedstudio official documentation.
```#include <Arduino.h>

#include "sensirion_common.h"
#include "sgp30.h"


void setup() {
    s16 err;
    u16 scaled_ethanol_signal, scaled_h2_signal;
    Serial.begin(115200);
    Serial.println("serial start!!");

   
    /*  Init module,Reset all baseline,The initialization takes up to around 15 seconds, during which
        all APIs measuring IAQ(Indoor air quality ) output will not change.Default value is 400(ppm) for co2,0(ppb) for tvoc*/
    while (sgp_probe() != STATUS_OK) {
        Serial.println("SGP failed");
        while (1);
    }
    /*Read H2 and Ethanol signal in the way of blocking*/
    err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,
                                            &scaled_h2_signal);
    if (err == STATUS_OK) {
        Serial.println("get ram signal!");
    } else {
        Serial.println("error reading signals");
    }
    err = sgp_iaq_init();
    //
}

void loop() {
    s16 err = 0;
    u16 tvoc_ppb, co2_eq_ppm;
    err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
    if (err == STATUS_OK) {
        Serial.print("tVOC  Concentration:");
        Serial.print(tvoc_ppb);
        Serial.println("ppb");

        Serial.print("CO2eq Concentration:");
        Serial.print(co2_eq_ppm);
        Serial.println("ppm");
    } else {
        Serial.println("error reading IAQ values\n");
    }
    delay(1000);
}
```
![image](https://user-images.githubusercontent.com/30889567/188283975-b6774e83-1bf0-4c3a-b7e7-9929ee7dfddc.png)


Code for Grove Temp&Hum using sht40 and this is also copied from seeedstudio official doc

```#include <Arduino.h>
#include <SensirionI2CSht4x.h>
#include <Wire.h>

SensirionI2CSht4x sht4x;

void setup() {

    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    Wire.begin();

    uint16_t error;
    char errorMessage[256];

    sht4x.begin(Wire);

    uint32_t serialNumber;
    error = sht4x.serialNumber(serialNumber);
    if (error) {
        Serial.print("Error trying to execute serialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Serial Number: ");
        Serial.println(serialNumber);
    }
}

void loop() {
    uint16_t error;
    char errorMessage[256];

    delay(1000);

    float temperature;
    float humidity;
    error = sht4x.measureHighPrecision(temperature, humidity);
    if (error) {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Temperature:");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity);
    }
}

```

![image](https://user-images.githubusercontent.com/30889567/188283664-5a6a692f-45f6-43f6-9b10-9bac514c7fb4.png)


Code for soil moisture sensor

```
int sensorPin = A0;
int sensorValue = 0;
 
void setup() {
    Serial.begin(9600);
}
void loop() {
    // read the value from the sensor:
    sensorValue = analogRead(sensorPin);
    Serial.print("Moisture = " );
    Serial.println(sensorValue);
    delay(1000);
}
```
![image](https://user-images.githubusercontent.com/30889567/188284115-cbceb47a-3ad1-488d-8a67-9e9ec11bd983.png)


Code for data collection in edge impulse
``` #include <Wire.h>

#include <SensirionI2CSht4x.h>
#include "sensirion_common.h"
#include "sgp30.h"

// Settings
#define BTN_START           0                         // 1: press button to start, 0: loop
#define BTN_PIN             WIO_5S_PRESS              // Pin that button is connected to
#define SAMPLING_FREQ_HZ    4                         // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
#define NUM_SAMPLES         8                         // 8 samples at 4 Hz is 2 seconds

int sensorPin = A0;
int sensorValue = 0;

SensirionI2CSht4x sht4x;

void setup() {
  // Initialize button
  pinMode(BTN_PIN, INPUT_PULLUP);
  
  // Start serial
  Serial.begin(115200);

  // Initialize environmental sensor
 while (!Serial) {
        delay(100);
    }
    Wire.begin();

    uint16_t error;
    char errorMessage[256];
    sht4x.begin(Wire);
    uint32_t serialNumber;
    error = sht4x.serialNumber(serialNumber);
    if (error) {
        Serial.print("Error trying to execute serialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } 

  // Initialize VOC and eCO2 sensor
  while (sgp_probe() != STATUS_OK) {
    Serial.println("Trying to initialize SGP30...");
    delay(1000);
  }

  
}

void loop() {


  uint16_t error;
  int16_t sgp_err;
  uint16_t sgp_tvoc;
  uint16_t sgp_co2;
  unsigned long timestamp;
  char errorMessage[256];

    float temperature;
    float humidity;
    error = sht4x.measureHighPrecision(temperature, humidity);

sensorValue = analogRead(sensorPin);

  // Wait for button press
#if BTN_START
  while (digitalRead(BTN_PIN) == 1);
#endif

  // Print header
  Serial.println("timestamp,temp,humd,co2,voc1,soilmst");

  // Transmit samples over serial port
  for (int i = 0; i < NUM_SAMPLES; i++) {

    // Take timestamp so we can hit our target frequency
    timestamp = millis();

  
    // Read BME680 environmental sensor
   if (error) {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
  
    // Read SGP30 sensor
    sgp_err = sgp_measure_iaq_blocking_read(&sgp_tvoc, &sgp_co2);
    if (sgp_err != STATUS_OK) {
      Serial.println("Error: Could not read from SGP30");
      return;
    }

    // Print CSV data with timestamp
    Serial.print(timestamp);
    Serial.print(",");
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(humidity);
    Serial.print(",");
    Serial.print(sgp_co2);
    Serial.print(",");
    Serial.print(sgp_tvoc);
    Serial.print(",");
    Serial.print(sensorValue);
    Serial.println();

    // Wait just long enough for our sampling period
    while (millis() < timestamp + SAMPLING_PERIOD_MS);
  }

  // Print empty line to transmit termination of recording
    Serial.println();

  // Make sure the button has been released for a few milliseconds
#if BTN_START
  while (digitalRead(BTN_PIN) == 0);
  delay(100);
#endif
}
```


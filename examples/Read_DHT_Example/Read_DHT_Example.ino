/*
  DHT11_Lite - Basic Example

  Reads temperature and humidity from a DHT11 sensor and prints
  the values to the Serial Monitor.

  The library uses a non-blocking state machine internally,
  allowing loop() to continue running freely while the sensor
  communication is in progress.

  Circuit:
    - DHT11 data pin connected to digital pin 2
    - 5K pull-up resistor between DATA and VCC
    - VCC connected to 5V
    - GND connected to GND

  created 2026
  by Sirio Guy
*/

#include <DHT11_Lite.h>


// DHT11 connected to digital pin 2.
// Minimum delay between measurements: 2 seconds.
DHT11_Lite sensor(2, 2);

// Stores the latest sensor reading.
DHT11Data data;


void setup() {

  Serial.begin(9600);

  while (!Serial) {
    ;
  }

  Serial.println(F("DHT11_Lite example"));
}


void loop() {

  // read() must be called continuously.
  // It returns true only when a complete reading is available.
  if (sensor.read(data)) {

    // Always verify whether the reading completed successfully.
    if (data.error) {

      Serial.println(F("Sensor read failed"));

    } else {

      Serial.print(F("Temperature: "));
      Serial.print(data.temperature);
      Serial.print(F(" °C"));

      Serial.print(F("  Humidity: "));
      Serial.print(data.humidity);
      Serial.println(F(" %"));
    }
  }

  // Other non-blocking code can run here.
}
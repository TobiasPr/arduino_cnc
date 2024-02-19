#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Start up the library
  sensors.begin();
}

void loop() {
  // Request temperature conversion from all devices on the bus
  sensors.requestTemperatures();

  // Get the temperature from the PT1000 sensor
  float temperatureC = sensors.getTempCByIndex(0);

  // Check if the temperature is valid
  if (temperatureC != DEVICE_DISCONNECTED_C) {
    // Print the temperature to serial monitor
    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.println(" °C");
  } else {
    // Print an error message if temperature reading failed
    Serial.println("Error: Unable to read temperature");
  }

  // Delay before taking next temperature reading
  delay(1000);
}

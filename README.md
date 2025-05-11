# Heater-Control-System

This project simulates a basic heater control system using an ESP32 and a DHT22 temperature sensor. The heater is controlled based on a preset temperature threshold, and the system uses an LED to simulate the heater turning on and off. The system monitors the temperature continuously and adjusts the heater state based on defined temperature thresholds. 

## Features:
- Simulated temperature control using the DHT22 sensor.
- Heater is turned on/off based on temperature thresholds.
- LED (on pin 3) is used to simulate the heater's state (ON/OFF).
- Serial monitor prints out the current temperature and heater state.
- Basic state machine for managing heater behavior: `IDLE`, `HEATING`, `STABILIZING`, `TARGET_REACHED`, and `OVERHEAT`.

## Components:
- **ESP32**: The microcontroller used for the project.
- **DHT22**: Temperature and humidity sensor.
- **LED (Pin 3)**: Simulates the heater.
- **Serial Monitor**: Used for displaying simulated BLE notifications and the heater state.

## Circuit Diagram:
- **DHT22 Sensor**: 
  - VCC → 3.3V (ESP32)
  - GND → GND (ESP32)
  - DATA → GPIO2 (ESP32)
  
- **LED (Heater Simulation)**: 
  - Anode → Pin 3 (ESP32)
  - Cathode → GND (with a current-limiting resistor, 220Ω)

## Software:
- **Arduino IDE**: The code is written in C++ using the Arduino framework.
- **DHT Sensor Library**: Used for reading temperature and humidity data from the DHT22 sensor.

## Code :
```
#include <DHT.h>

#define DHTPIN 2          
#define DHTTYPE DHT22     // Define the DHT sensor type
#define HEATER_PIN 3      // Pin where the LED is connected to simulate heater

DHT dht(DHTPIN, DHTTYPE);  // Initialize the DHT sensor

// Temperature thresholds
const float lowerThreshold = 22.0;  // Temperature threshold to turn heater ON
const float upperThreshold = 30.0;  // Temperature threshold to avoid overheating
const float targetTemperature = 25.0;  // Desired temperature

// State variables
float temperature = 18.0;  // Start with a low initial temperature (18°C)
int heaterState = LOW;  // Initial heater state (OFF)

// Define heater states
enum HeaterState { IDLE, HEATING, STABILIZING, TARGET_REACHED, OVERHEAT };
HeaterState currentState = IDLE;

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);

  // Initialize the DHT sensor
  dht.begin();

  // Set the heater pin (LED) as output
  pinMode(HEATER_PIN, OUTPUT);

  // Initial state print
  Serial.println("Heater Control System Initialized.");
}

void loop() {
  // Simulate gradual temperature change
  // Gradually increase the temperature
  temperature += 0.25;  // Gradually increase temperature by 0.1°C per loop

  // Print the temperature to the Serial Monitor
  Serial.print("Simulated Temperature: ");
  Serial.println(temperature);

  // State machine for heater control
  switch (currentState) {
    case IDLE:
      if (temperature < lowerThreshold) {
        currentState = HEATING;  // Start heating if the temperature is too low
        digitalWrite(HEATER_PIN, HIGH);  // Turn LED (heater) ON
        heaterState = HIGH;
        Serial.println("Heater is ON (Heating)...");
      }
      break;

    case HEATING:
      if (temperature >= targetTemperature) {
        currentState = STABILIZING;  // Move to stabilizing state
        Serial.println("Target temperature reached. Stabilizing...");
      }
      break;

    case STABILIZING:
      if (abs(temperature - targetTemperature) < 0.5) {
        currentState = TARGET_REACHED;  // If stabilized, reach the target
        digitalWrite(HEATER_PIN, LOW);  // Turn LED (heater) OFF
        heaterState = LOW;
        Serial.println("Target Reached. Heater is OFF.");
      }
      break;

    case TARGET_REACHED:
      // Monitor the temperature and decide if we need to turn the heater on again
      if (temperature < lowerThreshold) {
        currentState = HEATING;  // If temperature falls below lower threshold, heat again
        digitalWrite(HEATER_PIN, HIGH);  // Turn LED (heater) ON
        heaterState = HIGH;
        Serial.println("Temperature dropped. Heater is ON (Heating)...");
      } else if (temperature > upperThreshold) {
        currentState = OVERHEAT;  // If overheat, turn off the heater
        digitalWrite(HEATER_PIN, LOW);  // Turn LED (heater) OFF
        heaterState = LOW;
        Serial.println("Overheating! Heater is OFF.");
      }
      break;

    case OVERHEAT:
      // If overheated, the system will turn off the heater and enter the overheat state
      Serial.println("System in Overheat. Heater OFF!");
      break;
  }

  // Delay between temperature readings (1 second)
  delay(1000);  // Slow down the loop to observe the state changes
}
```

## Code Explanation:
### States:
1. **IDLE**: Heater is off, and the system waits for the temperature to drop below the lower threshold.
2. **HEATING**: The heater is turned on, and the temperature increases towards the target.
3. **STABILIZING**: The system tries to stabilize the temperature around the target temperature.
4. **TARGET_REACHED**: The target temperature has been reached, and the heater is turned off.
5. **OVERHEAT**: If the temperature exceeds the upper threshold, the system turns off the heater.

### Temperature Control:
- **Lower Threshold**: 22°C (Heater is turned on when the temperature drops below this value).
- **Target Temperature**: 25°C (Heater will stop when the temperature reaches this value).
- **Upper Threshold**: 30°C (If the temperature exceeds this value, the heater is turned off to prevent overheating).

## How to Use:
1. **Hardware Setup**:
   - Connect the DHT22 sensor to the ESP32 according to the circuit diagram.
   - Connect an LED to Pin 3 to simulate the heater.
   
2. **Software Setup**:
   - Open the Arduino IDE.
   - Install the DHT sensor library if you haven't already: Go to `Sketch -> Include Library -> Manage Libraries`, search for `DHT sensor library` by Adafruit, and install it.
   - Upload the code to your ESP32.
   
3. **Monitor the Output**:
   - Open the Serial Monitor in the Arduino IDE to view the simulated temperature readings and heater status.
   
4. **Observe the Heater Control**:
   - The heater (LED) will turn on when the temperature drops below the lower threshold and will turn off once the target temperature is reached.

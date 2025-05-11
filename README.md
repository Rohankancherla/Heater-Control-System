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

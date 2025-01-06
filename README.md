# Community Health and Safety Kiosk
This project integrates air quality monitoring with an intelligent mask dispenser system. It detects air pollution levels and dispenses masks when necessary, using a Maker Feather AIoT S3 microcontroller, SCD30 sensor, IR sensor, LEDs and a servo motor. The project also includes an MQTT client for publishing telemetry data to the V-ONE cloud.

## Features
- **Air Quality Monitoring:** Measures CO2 concentration, temperature, and humidity using the SCD30 sensor.
  
- **LED Indicators:**
  - Green: Low pollution (<1000 ppm).
  - Yellow: Moderate pollution (1000–4999 ppm).
  - Red: High pollution (>= 5000 ppm).
    
- **Intelligent Mask Dispensing:**
  - Detects objects using an IR sensor.
  - Opens the mask container lid when CO2 levels exceed moderate air pollution level and an object is detected.

- **Cloud Integration:** Publishes data to V-ONE cloud platform.

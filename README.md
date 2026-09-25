# LineFollowerRobot
A Line follower robot using the maker pi pico

This line follower robot was a project that is part of a university course subject called "Ενσωματωμένα Συστήματα" in Dimocritus University of Thrace.

Authors: 
+ Savvapetrakis Angelos
+ Providas Konstantinos
+ Zarkadas Fotis

## Project structure
- `/main.py`: Main MicroPython control loop (PID control, calibration, motor and buzzer handling).
- `/mcp3008.py`: MCP3008 ADC driver used to read line sensor channels over SPI.
- `/Instruction manual`: Notes and walkthrough for the code and behavior.

## Development notes
- This project targets **MicroPython on Maker Pi Pico** hardware.
- Running `main.py` directly on a desktop Python interpreter is not supported because it depends on MicroPython-only modules (`machine`, `utime`, `ujson`).
- `calibration_data.json` is generated on-device during calibration and is intentionally ignored in git.

# Parts: 
+ Maker pi pico
+ Adjustable DC buck converter
+ N20 micro metal gear motors(500rpm@6V)
+2 18650 battery
+ MCP3008 (10-bit 8-cannel ADC SPI)
+ Ball Caster 0.4"
+ Waveshare Tracker Sensor
+ 3D printed chasis and wheels

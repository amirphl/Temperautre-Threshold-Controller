# Temperautre-Threshold-Controller
A circuit for monitoring environment temperature and alarming.
## Getting Started
In this project, we have implemented a circuit which measures environment temperature by LM35 sensor then converts it to celcius.
If the computed temperature is more than a selected threshold (determined by user with clicking keyboard keys), timer counter of ATMEGA16
will be activated in CTC mode.If after 6 seconds the temperature holds and it is more than threshold, the buzzer will be activated and alarms.
## Modules
```
ATMEGA16
LCD
7 segment
LM35 sensor
keypad
regulator
buzzer
LED
```
## circuit
![circuit](https://github.com/amirphl/Temperautre-Threshold-Controller/new/master/circuit.png)
## schematic
![schematic](https://github.com/amirphl/Temperautre-Threshold-Controller/blob/master/schematic.PNG)
## pcb
![pcb](https://github.com/amirphl/Temperautre-Threshold-Controller/blob/master/pcb.PNG)

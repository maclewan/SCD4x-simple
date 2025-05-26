# SCD4x
Simplified and derussified for of the SCD4x MicroPython library.
Aim is to create as small and simple one file dependency for micropython as possible,
creating out of the box, easu to use solution for SCD4x sensor.


MicroPython module for work with SCD4x temperature&humidity&CO2 sensor from Sensirion.

# Self heating
During periodic measurement (data reading period of 5 seconds), I detected a suspicious 
increase in temperature read from the sensor.
When switching to single measurement mode with a period of 15 seconds, I saw a drop in temperature.
It looks like the sensor is self-heating in the periodic measurement mode!
I recommend switching the sensor to single measurement mode with a period of at least 15 seconds!

# Autocalibration problem

The person expressed a useful idea in my opinion. You can read it at the [link](https://www.reddit.com/r/esp32/comments/12y0x5k/warning_about_the_sensirion_scd4041_co2_sensors/).

tl;dr disable autocalibration by default, after calibrating the device, so measurements are more consistent.

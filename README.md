# MidgetRaceCar-Telemetry
Since testing has been moved there had to be redesigns in the current model.  Instead of using Lora board and make it modular it is now all on board.

This set up now uses the MPU6500 along with a GY-271 to get six axis and measure the acceleration, along with roll pitch and yaw.

GPS Module, which uses a GT-U7 gps module clocked at 10Hz for a more accurate reading of speed

Throttle position sensor using angle position attached at the throttle linkage

Integrating he tach pin out from the EFI Box

SD Card reader so that anyone can pul the sd card out and run my app.exe to see the data that has been acquired. 

Lap timer which uses a Lidar sensor that since in the middle of the straightaway.  
The lidar detech the change in distance from the wall being constant, when a car passes by distance changes and a car is detected it then start/stops the timer.
It is hooked up to an ESP 32 to allow for someone to connect to the local wifi on the phoe and see the lap times
It will also have an sd card reader on it so that you can save all data to a .csv


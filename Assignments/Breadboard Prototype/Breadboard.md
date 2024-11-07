# Breadboard Prototype Update

Our breadboard prototype is coming along nicely. All features that we currently have in hand are functional and controllable, including:

- Controlling the ESP32-C6
- Configuring, writing to, and reading from the LSM6D032 Accelerometer and Gyroscope sensor
- Enabling and controlling interrupts from the LSM6D032 sensor
- Controlling a fixed number of NeoPixel or WS2811 LEDs with fixed timing


Below is a close-up image of our current breadboard prototype
![Breadboard Prototype close-up](/Assignments/Breadboard%20Prototype/close-up.JPG)


We also have a couple images representing our data transfers between the ESP32-C6 and the LSM6D032 sensor over a 4-wire SPI interface:

Example SPI transfer using a logic analyzer:
![Logic analyzer example](/Assignments/Breadboard%20Prototype/WaveForm_example.png)

In this image, you can clearly see the interaction and data that was transferred between the two devices. In this case, the ESP32-C6 was reading from four registers starting at register 0xA2 and receiving four data packets from the LSM6D032.

Next, an image showing our data after some computation into a human-readable format:  
![Example data in human-readable format](/Assignments/Breadboard%20Prototype/Screenshot%202024-11-06%20at%208.00.42 PM.png)  
In this image, the hex data from the LSM6D032 is converted to a physical number we can use to determine information such as when the die stops moving and what orientation it is in when it stops moving.

Finally, the lightshow!
[Click here for lightshow video](https://youtu.be/P9tXDaBaYNc), or alternatively see the "animated-lights.mov" file.


With all of our code functioning individually, we are very close to having a fully functional project on the code side.


### Note
One of our project requirements is to make sound when a certain roll is detected. Currently the Piezo to do such sounds is not installed on the ESP32-C6, but creating a PWM signal from the ESP32-C6 to drive a passive buzzer is beyond trivial. The main focus on the code to this point has been to develop the more complex and intricate code and components to get them to function in unison.

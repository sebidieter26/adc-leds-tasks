Operating Systems in Automation Project
=
This university project was meant to help us learn about tasks and get a better grasp of ADC (Analog-to-Digital Converter).                      

In this project I have used freeRTOS to create and control 4 tasks that are getting values from 2 potentiometers. 

One potentiometer is responsible for an AUTOMATIC MODE and the other is responsible for a MANUAL MODE.

In both modes the ADC aquires values from one of the potentiometers, based on the mode chose and then turns on 1,2 or 3 leds if the values aquired are positioned between given values.

The switch between modes is realised using an external interrupt actioned by a push button.


---
- 1st task suspends task 4 that is responsible or the MANUAL MODE and aquires values using an ADC from the first potentiometer. The values are placed in 2 queues that are sent to second and third tasks
- 2nd task gets adc values using one of the queues created in the first task and then compares it with some given values. Based on these values it turns 1,2 or 3 leds
- 3rd task aquires ADC values using the second queue and prints them on serial port
- 4th task is basically the whole MANUAL MODE. Here the first 3 tasks are suspended and then task 4 aquires ADC values and based on these values turns on 1,2 or 3 leds just like the second task. When switched in MANUAL MODE, the 4th task prints "mod manual" on the serial port.
---
- I have created a function for the external interrupt that checks if the button is pressed and if so it switches between modes. Optionally, when the application is on the AUTOMATIC MODE, the buildin led is on.
---
What did I use ?
=
HARDWARE PART
-
- 1x NUCLEO F401-RE DEVELOPMENT BOARD
- 1x PUSH BUTTON
- 1x 10k RESISTOR
- 2x 10k POTENTIOMETERS
- 3x LEDS(RED,GREEN,YELLOW)
- 3x 220ohm RESISTORS
- 14x JUMPER WIRES
- 1x USB-A TO MINI USB

![WhatsApp Image 2024-01-14 at 16 14 13](https://github.com/sebidieter26/adc-leds-tasks/assets/107187446/14b64335-5a8f-43b8-8375-a292d93c6fd6)

---
SOFTWARE PART
-
- STM32CUBEIDE
- FREERTOS
---
DEMONSTRATION VIDEO
-



https://github.com/sebidieter26/adc-leds-tasks/assets/107187446/2e834207-53b3-4597-9cbb-4079d833d68c



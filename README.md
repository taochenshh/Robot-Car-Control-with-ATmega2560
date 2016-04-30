# Robot-Car-Control-with-ATmega2560

##MCU: Arduino Mega 2560 (ATmega 2560)
IDE: WinAVR (not Arduino IDE)

```
Functions:
1. Serial communication at 4800 baud rate
2. Timer 4 outputs two pins' pwm waves
3. Robot can move forward and backward, turn left while moving forward,
   turn right while moving forward, turn left while standing at the same
   position,turn right while standing at the same position
4. The robot's moving speed can be adjusted via pwm waves
5. Measure a motor's speed via INT 1,2 and Timer 1
   The speed value is stored in variable "speed"
6. Detect a motor's direction via INT 1,2
  if the motor rotates forward, then the serial port will output: +
  if the motor rotates backward,then the serial port will output: -
7. Real-time measurement of the battery voltage whose value is
   stored in variable "batteryvoltage"
8. Robot will be stopped if the position switch is triggered.
9. Motors are controlled wirelessly using radio controller with channel 1,2,4
   Channel 1: turn left or turn right while moving forward
   Channel 2: move forward or backward
   Channel 4: turn left or turn right while standing at the same position
```

```
Pin Connection Description:
1. PA0,PA1,PA3,PA4 connect to left motor's IN1,IN2 and right motor's IN1 IN2
2. PH3(OC4A),PH4(OC4B) connect to left motor's EN and right motor' EN and
   these two pins can output pwm waves
3. PD0(INT0),PD1(INT1),PD2(INT2) connect to limit switch, encoder's A phase
   encoder's B phase
4. PD3(INT3),PE4(INT4),PE5(INT5) connect to the radio controller's receiver's
   channel 1,channel 2 and channel 3. These three channels control motion of the robot
5. The AVCC, AREF and ADC0 should connect to the supply power in order to
   measure its voltage
PA0 is digital pin 22 on arduino mega 2560
PA1 is digital pin 23 on arduino mega 2560
PA2 is digital pin 24 on arduino mega 2560
PA3 is digital pin 25 on arduino mega 2560
PD0 is digital pin 21 on arduino mega 2560
PD1 is digital pin 20 on arduino mega 2560
PD2 is digital pin 19 on arduino mega 2560
PD3 is digital pin 18 on arduino mega 2560
PE4 is digital pin 2 on arduino mega 2560
PE5 is digital pin 3 on arduino mega 2560
PH3 is digital pin 6 on arduino mega 2560
PH4 is digital pin 7 on arduino mega 2560
ADC0 is analog pin 0 on arduino mega 2560
```
The code here does only the basic control, not with PID control.

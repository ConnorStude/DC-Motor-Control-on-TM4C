# DC-Motor-Control-on-TM4C

```
Project 3: DC Motor Control
Introduction to Robotics: CPEN 443



**Design Process**


Figure 1: Flow Chart of workflow

The process above shown in Figure 1 is a high level flow chart of the order in which each
deliverable was worked on. All deliverables in green are working, while the controller in red is
the only delivery that is not functioning properly. This is further explained in the Complications
section of the report.

**Equipment Used**

For this project, a variety of equipment was utilized to
realize the motor control system. This equipment
includes a voltmeter and oscilloscope, which are
shown in Figure 2, to measure voltages at different
points in the system and the output at a PWM. An
external breadboard was also utilized to make room
for external connections. The Analog to Digital
Converter (ADC) used for this project was the
ADSS7806. As with previous projects, the Texas
Instruments TM4C123GH6PM microcontroller was
used.

**Design Approach**

Threads

2 threads were utilized in this project, 1 controlled input from the keypad then outputted the
values onto the LCD, and the other controlled the output of the LCD for the current RPM of the
motor. All actions that involved either the LCD or the keypad were contained within these 2
threads. In thread 1 an array was used to store each value entered into keypad, then each value
was converted from an ASCII value to decimal and finally was combined into a final value. This
value is then displayed on the LCD and stored so that the controller can correctly tune the
voltage, so the motor spins to the desired RPM.

Interrupt Handler

Timer0A was used extensively in this project to produce an interrupt every 10ms. During this
interrupt an ADC sampled the voltage of the motor every 100 μs and once 100 samples were
gathered an average was taken. Once the voltage was finalized it was converted to an RPM and

Figure 2: Voltmeter and Oscilliscope

then reported to thread 2 so that it could be displayed. The PWM was then modified to bring the
RPM to the desired speed from the keypad input.

Controller Parameters

The team was able to implement almost everything in this project except for getting the
controller 100% right. We planned to use an incremental controller due to its simplicity
compared to that of the PID and fuzzy logic controller, but with trouble initializing the interrupt
handler, we were unable to test Had we been able to, it would have more effectively been able to
control the voltage output from the PWM so that the desired RPM of the motor could be reached.

## LCD

The LCD, as shown in Figure 3, is used to
display the inputs from a user on the keypad,
the target speed, and the current speed of the
motor. For the user input, the array that stores
the user’s keypresses is put through a series
of if statements, each reading an index of the
array. If the corresponding index does not
have a null character, then the char value of
the ASCII code is stored in a char variable,
which is then put the Display_Char function
from LCD.s do display the character. The
same process is done for the target speed and
current speed arrays.

Keypad

The Keypad was used to accept a target
speed from a user. It is implemented in its
own thread, which scan’s the Keypad for a
keypress. When a number on the keypad is
pressed, it is stored in an array, unless the
array already has four numbers in which case
that array is then translated to another array
which holds the target value. If the pound
sign is pressed, then a series of if statements
null the remaining spots in the keypress array
and translate that array to the target value.

Figure 3: LCD Showing Input and Target Speed

Figure 4: Motor Control System

The function ASCII2Hex from Keypad.s translates the array for the target speed into an RPM
value, then a series of if else statements make sure the target speed is in between 400 and 2400 or

0. Finally, the function Hex2ASCII is used to turn that target speed back into an array of ASCII
characters to be used in the 3rd thread for the LCD.

**Allocation of work**

Work was separated so that both team members worked in their area of specialty, Zach being an
electrical major built the ADC used for sampling the voltage and Connor worked on making the
drivers for the ADC. Zach built the entirety of the LCD display thread and spent time updating
and debugging the basic keypad input thread. Connor made the initialization functions for all
ports used including that of the ADC, and PWM. He also originally made a third thread that
eventually became the handler for reading the voltage applied to the motor through the PWM
then adjusting that voltage so the motor was able to output the desired RPM.

**Complications throughout the project**
The team ran into more complications during the project than expected. Initial complications in
volved the keypad. This was solved by storing the output of the ScanKeypad() function in an
unsigned 32 bit integer variable as opposed to an unsigned 8 bit integer variable. There were also
initial complications with the implementation of the PWM. This was solved by setting M1 and
M2 motion direction, as well as putting the negative terminal of the motor directly into M
instead of to ground on the breadboard.

Further complications were introduced with a misunderstanding of the Timer0A_Handler.
Initially, the plan for the project was to utilize three threads: one for the keypad input, one for the
LCD output, and one to capture data from the ADC. This caused problems, as the ADC would
voltage data in a manner that was to fast for our purposes. This caused a pivot to two threads, for
the keypad and LCD, and the use of the Timer0A_Handler to handle the ADC voltage samples
as well as the controller implementation. This takes a sample of the motor speed every 100
milliseconds. Then after 100 samples are taken, the controller will adjust the speed to match the
target speed.

We ran into further complications with the Timer0A_Handler when the ADC code was moved
into the function. This was due to the Handler initialization being above the LCD initialization.
Because the LCD initialization is unfriendly, it needs to be called first. Moving this fixed the
ADC problem.

As of now, everything in the program works up to the sampling collection in the Handler. There
is a loop in the handler as an array of voltage samples is filled, which means that the controller is
never reached.

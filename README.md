# Robotic arm Arduino

Some time ago I've bough one of those DIY Robotic arm kits from Aliexpress. The
kit comes with all acrylic pieces to assembly the arm structure, one arduino UNO
board and one generic "mearm.WDT" joystick control shield (Fig. 1). This is a
sample code to control the arm through the joysticks on the shield. 

![Robotic ARM with arduino shield](/imgs/arm.jpeg)

Fig. 1: meARM DIY kit. I've added one capacitor between Vdd and GND for filtering
along with one heatsink on top of the voltage regulator to avoid spurious resets
when the servos are overloaded (leading to high current drain).

## Calibration

* Set *CALIBRATION* to 1 and run the code. Look at serial console and annotate
  the offsets when the joysticks are at center position (without touching them).
  Replace defines *JOY_[LEFT,RIGHT]_OFFSET_[X,Y]* with annotated values.

* Move the arm to the maximum position on each axis and annotate the
  maximum angle values for your assembling. Replace the corresponding defines
  with the right values.

* Set *CALIBRATION* to 0 and flash your board again. You are ready!

## Record arm position

1. Move the arm at the position you want to save
2. Press and hold right knob for 2 seconds
3. LED will blink twice indicating that current position is saved
4. Move the arm to another position
5. Press right knob for 1 second to move the arm to saved position

PS: Position is saved to RAM, so it get losted on power off or reset.

## Bug reports

Please, report them to [rene@renesp.com.br](mailto:rene@renesp.com.br)


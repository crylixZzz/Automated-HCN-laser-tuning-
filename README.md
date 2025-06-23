Code is used to automate tuning for the HCN laser machine used to detect plasma activity through electron density measurement. 

components:
1. Arduino Uno
2. LTC 1968 RMS to DC
3. TB6600 Motor driver
4. 12v 5a power supply 
5. RS Pro hybrid stepper motor 0.9 deg/step

Commands:
- A = autotune 2000 deg
- S = selftune, then type degrees (not in same line)
- R = reset to "home" (starting point once code is initialized, position: 0 deg)
- Y = disable motor


Assumptions:
- assuming that 1 micrometer = 280.61 deg
- assuming that deg wanted to rotate back is 2000 deg once limit is hit 
- assuming that resistor is a step up resistor, when recieved reading of 1 on either left or right limit the limit function will run


Real code:
- coarse scan: rotates set amount of degrees cutting out any readings below 0.5V, keep track of degrees at which readings meet the required threshhold.
- pre fine scan: rotate back to degree with highest voltage output and start initializing fine tuning
- fine tune: 
  - Capture 1 wavelength and look for left and right trough (V decrease by around 50%) for both sides
  - scan in finer increments (from 10 deg -> 2.5, zoomed 8 times and concludes after 3 passes with no gains)



simulated code (how it differs from real):
- simulates sine waves instead of input signal from RMS to DC via I2C (from 0-5V avg)
- random sine waves every simulation
- limit pins untested, implemented L1 L2 to test (rotates back 2000 deg opposite direction)





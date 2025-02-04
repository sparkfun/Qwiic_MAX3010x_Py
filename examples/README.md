# Sparkfun MAX3010X Examples Reference
Below is a brief summary of each of the example programs included in this repository. To report a bug in any of these examples or to request a new feature or example [submit an issue in our GitHub issues.](https://github.com/sparkfun/qwiic_max3010x_py/issues). 

NOTE: Any numbering of examples is to retain consistency with the Arduino library from which this was ported. 

## Ex1 Basic Readings
Simple example for the qwiic MAX3010x device
 Outputs all Red/IR/Green values.

## Ex2 Presence Sensing
Simple example for the qwiic MAX3010x device
 This takes an average reading at power up and if the reading changes more than 100,
 then print 'Something is there!'.

## Ex3 Temperature Sense
Simple example for the qwiic MAX3010x device
 This demo outputs the onboard temperature sensor. 
 The temp sensor is accurate to +/-1 C but
 has an astonishing precision of 0.0625 C.

## Ex4 Heartbeat Plotter
Simple example for the qwiic MAX3010x device
 Shows the user's heart beat on a graphical plotter
 Using Matplotlib
 To learn more about plotting data in python check out this tutorial:
 https://learn.sparkfun.com/tutorials/graph-sensor-data-with-python-and-matplotlib/
 The example code below was built using the code from the previously mentioned tutorial.
 Thanks Shawn Hymel!

## Ex5 Heartrate
Simple example for the qwiic MAX3010x device
 This is a demo to show the reading of heart rate or beats per minute (BPM) using
 a Penpheral Beat Amplitude (PBA) algorithm.

## Ex6 Fifo Readings
Simple example for the qwiic MAX3010x device
 Outputs all Red/IR/Green values at  about 5Hz by polling the FIFO
 Note, the Hz is slowed down by printing to the python terminal,
 It can be drmatically speed up if printing is omitted or intermittent



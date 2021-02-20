# GG_EyeTracking
Webcam eye tracking scripts for animatronic and VR puppetry
Origonal Author: Ryan H. (GunGryphon)
Initally written for a college embedded system project

This program is designed to run light weight eye tracking using a head-mounted webcam and a Raspberry Pi 3 or better device.
This does not currently utilize any AI algorithms, opting to use a series of filters and shape qualifications to isolate
the pupil from the rest of the background. The tracking works best under evenly lit conditions, however several filtering techniques were added to accomodate for items such as changing light sources, pupil reflections, and eyelid shadows.

Ensure that the libraries listed in the script are installed prior to use.

Reference the example folder for hints on how to setup the camera view and boundary variables.

V1_1 only has a serial output for it's eye tracking data, for use in driving an animatronic puppet. 

Use and adaptation for other projects (ex. VR eye tracking) is encouraged, simply abide by the terms
as set by the accompanying license file. (Generally speaking though, just give credit where it's due and I'll probably be happy)


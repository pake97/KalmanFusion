# KalmanFusion

Kalman filter based Android application for sensor-fusioning used as anti-spoofing method.

## Application structure

The application has two activities and two services, also other java classes to manage the filtering operations.
STARTACTIVITY: this activity is used to calibrate GPS receiver, sometimes in fact the altitude coordinate isn't set so you have to wait some seconds. This activity also set the first coordinates used for the ECEF-NED conversion.
MAINACTIVITY: this Activity handle messages from and to the other components and converts the coordinates from LLA to ECEF to NED and vice versa.
SYNCSERVICE: This service is used to synchronize sensor datas arriving from the Mainactivity. 
KALMANSERVICE: This service apply the Kalman filter on datas that come from the Mainactivity and return the estimation.
STATE:  java class for kalman filter's state
MATRICE: java class/library for matrix managment: trasposta = trasposition of a matrix; mmp = matrix matric product; mvp= matrix vector product; vmp = vector matrix product;
QUATERNION: java class with quaternion operation.

### Prerequisites

This application uses the LINEAR_ACCELERATION sensor, if it's not allowed you need tu use the ACCELEROMETER sensors subtracting the gravity given by GRAVITY sensor.

### DEVICES:

This application needs lots of computing power, if device has few RAM memory (e.g. 4GB) you could delete KalmanService service and move the kalman filter in the main activity.

### Version

Version 1.0

## PROBLEMS

if you have problem with app installation such as " cannot find sdk.." or "rename application.." try to:
build -> clean project
build -> rebuild project
on the Android studio menu

## Authors

Amedeo Pachera 
contacts:
amedeopachera@gmail.com

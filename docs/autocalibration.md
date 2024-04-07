# Autocalibration

When a light sensor is mounted, the system can automatically calibrate itself. Therefore the system check every 1 h if enough sunlight is available since it will not work on cloudy days due to too much ambient light and no directed light to determine a direction.

When enough light is available, the mirror and sensor is directed into the sun and tries to determine the exact direction of the sunlight. This is done by moving in a spiral around the supposed direction and measure the sunlight in the tube with the light sensor at the end. This process ends when:

* The ambient sunlight falls under a certain level (no direct sunlight anymore)
* The desired direction cannot be reached by the stepper motors
* The maximum search range is analyzed and no value higher than 80% of the ambient light was detected

The successful end is as follows:
* The recorded values contain a value higher than 80% of ambient level. 
* The sun was shining the whole duration with the same power
* The movement was finished without a problem
The maximum value is then stored as sun direction. Therefore it is stored as steps of the motors. When the reading is already the third one, the system attempts to calculate an adapted configuration. This can be repeated for the following measurements of the same day. No measurements over multiple days are stored or used.



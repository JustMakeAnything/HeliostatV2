# Autocalibration

When a light sensor is mounted, the system can help with the calibration. Therefore the system checks if enough sunlight is available since it will not work on cloudy days due to too much ambient light and no directed light to determine a direction. A measurement is done every hour configurable in **calculations.h sunsearchdelay**

When enough light is available, the mirror and sensor is moved into the sun and tries to determine the exact direction of the sunlight. This is done by moving in a spiral around the supposed direction and measure the sunlight in the tube with the light sensor at the end. The center of the spiral also drifts in the direction where most light was measured. This process ends when the ambient sunlight falls under a certain level (no direct sunlight anymore) or the maximum search range is analyzed and no value higher than 94% of the ambient light was detected. The duration of the process is limited to 500 steps. with approx. 10 rotations. when no new measurement with a higher value is found afer 150 steps, the measurement is ended early successfully.

The direction of the maximum value is then stored as sun direction. It is stored as steps of the motors. When the three values are stored, the system attempts to calculate an adapted configuration. This will be repeated with the following measurements of the same day increasing the precision. No measurements over multiple days are stored or used. The values are shown in the log or webui.

Currently the system is not adjusting the values automatically since it is still alpha.


[12:00:56][I][measure:440]: amb: 3.069250 dir: 2.959000 saz: 0 sel: 1      driftaz -232 driftel: 20 maaz: 532 taaz: 764    az 149°
[13:38:45][I][measure:440]: amb: 2.953625 dir: 1.730625 saz: 62 sel: -85   driftaz -221 driftel: 34 maaz: 1317 taaz: 1538  az 185°
[15:19:50][I][measure:440]: amb: 2.989750 dir: 1.623750 saz: 0 sel: 1      driftaz 293 driftel:   2 maaz: 2301 taaz: 2301  az 221°
[16:59:47][I][measure:440]: amb: 2.943500 dir: 2.208625 saz: -75 sel: -103 driftaz 380 driftel: -25 maaz: 3260 taaz: 2867  az 247°

36° = 774
72° = 1537 / 2 = 769
26° = 566   * 36 / 26 = 783
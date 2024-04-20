# Heliostat V2
Files for my Heliostat project. The hardware consists of an ESP32 which is connected to two stepper motors to control the azimuth and elevation of a mirror and 4 switches to detect the endpoints for the motors. Further hardware is currently not supported.

## Instructable
For details see the Instructable at:

[Instructables](https://www.instructables.com/Heliostat-Reflects-Sunlight-Inside/)

## Design

The software uses [ESPHOME](https://esphome.io/) to create the firmware. It is autonomous after it was configured especially to a specific target location (angles) where the mirror should reflect to. There is a web interface to see some values and trigger a motor calibration as well as buttons to set the target. The values are stored in the eeprom of the ESP32.

## ESPHome

These main folder is for the esphome files. Compile and upload them from the command line with:

**esphome run .\code\heliostat.yaml**

To your ESP32. See [Esphome](https://esphome.io/) for details. I haven't tested integration with home assistant. It is not needed to run the heliostat.

### Configuration

To connect to your wifi, you need to adapt the secrects.yaml. Wifi is needed to get the exact time. You can also access the web interface at heliostat.local of the given network to set the target during runtime. Otherwise use the constants.yaml to set an initial target.
There is an experimental [autocalibration](./docs/autocalibration.md) available when an additional light sensor is mounted.

## 3D Print

The STL files for 3D Print are in folder 3DPrint

There is a subfolder STL for the STL files
The folder SolidEdgeParts contains the design files for Solid Edge. Since the parts are usually exposed to the sun it makes sense to print them in a material which is more heat resistant than PLA. Especially the parts with mechanical stress like the cogweels tend to deform when in the sun. I have used transparent PETG.

### 3D Print settings

The parts can be printed with standard settings. You should rotate the parts to the flat side when slicing. Some parts like the axle and the SideNoMotor need supports.

## Electronics

Although the electronics is not complicated, you should have some experience in soldering and ideally done an ESP project before.

# low_level_vehicle_interface

Source codes reuse from Covid-19 Delivery Robot project (ROS1)

## Todos
- Configure vesc/vesc_driver_node.launch
```
  <arg name="right_port" default="/dev/ttyRightESC" />
  <arg name="left_port" default="/dev/ttyLeftESC" />

```
Change /dev/ttyLeft(Right)ESC to the VESC USBPort e.g: ttyACM0 / ttyACM1 

---
Or using UDEV rules to match USB Port with the settings: [HOW TO USE UDEV RULES](https://opensource.com/article/18/11/udev#:~:text=A%20udev%20rule%20must%20contain,this%20is%20a%20removable%20device.)



## Getting started

```
$ roslaunch differential_drive low_level_vehicle_interface.launch
```


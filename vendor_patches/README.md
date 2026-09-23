# Vendor patches

These patches are narrow, reviewed corrections for the separately maintained
Hiwonder workspace on the Jetson. They are not applied by a launch file and do
not open hardware devices.

## Bus-servo state query names

`ros_robot_controller_get_bus_servo_state.patch` corrects two method-name
mismatches in the deployed `ros_robot_controller` service wrapper:

- voltage: `bus_servo_read_voltage` -> `bus_servo_read_vin`
- torque: `bus_servo_read_torque` -> `bus_servo_read_torque_state`

Both replacement methods already exist in the same deployed SDK. The old names
do not exist and caused an `AttributeError` that terminated the controller when
those request flags were used. Apply from the vendor package root:

```bash
cd /home/ubuntu/ros2_ws/src/driver/ros_robot_controller
patch --dry-run -p1 < \
  /home/ubuntu/hiwonder-jetson-robot/vendor_patches/ros_robot_controller_get_bus_servo_state.patch
patch -p1 < \
  /home/ubuntu/hiwonder-jetson-robot/vendor_patches/ros_robot_controller_get_bus_servo_state.patch
```

Then rebuild only `ros_robot_controller`. Building does not start the node or
send a hardware command. Runtime verification of torque feedback remains a
separately authorized hardware operation.

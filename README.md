# ds_dbw_joystick_demo
ROS2 interface to Dataspeed drive-by-wire platforms

Launch the drive-by-wire and the demo
```
ros2 launch ds_dbw_joystick_demo joystick_demo.launch.xml sys:=true
```

Launch the drive-by-wire and the demo and only send brake commands
```
ros2 launch ds_dbw_joystick_demo joystick_demo.launch.xml sys:=true steer:=false brake:=true thrtl:=false shift:=false misc:=false
```

Launch the drive-by-wire and the demo and specify all steer options
```
ros2 launch ds_dbw_joystick_demo joystick_demo.launch.xml sys:=true steer_cmd_type:=angle steer_max:=500.0 steer_rate:=0.0 steer_accel:=0.0
```

Launch the drive-by-wire and the demo and specify all brake options
```
ros2 launch ds_dbw_joystick_demo joystick_demo.launch.xml sys:=true brake_cmd_type:=pressure brake_min:=0.0 brake_max:=80.0 brake_inc:=0.0 brake_dec:=0.0
ros2 launch ds_dbw_joystick_demo joystick_demo.launch.xml sys:=true brake_cmd_type:=accel brake_min:=0.1 brake_max:=-8.0 brake_inc:=0.0 brake_dec:=0.0
```

Launch the drive-by-wire and the demo and specify all throttle options
```
ros2 launch ds_dbw_joystick_demo joystick_demo.launch.xml sys:=true thrtl_cmd_type:=percent thrtl_min:=0.0 thrtl_max:=100.0 thrtl_inc:=0.0 thrtl_dec:=0.0
```

# Parameters

* `steer` Send steer commands. Default `true`
* `steer_cmd_type` Steer command type. Default `percent`. Options: `torque` `angle` `curvature` `yaw_rate` `percent`
* `steer_max` Steer command scale factor applied to joystick axis. Default `100.0`% (changes with the value of `steer_cmd_type`)
* `steer_rate` Steer command rate limit in deg/s. Default `0.0` (firmware selects default value)
* `steer_accel` Steer command acceleration limit in deg/s^2. Default `0.0` (firmware selects default value)
* `brake` Send brake commands. Default `true`
* `brake_cmd_type` Brake command type. Default `percent`. Options: `pressure` `torque` `accel` `accel_acc` `accel_aeb` `pedal_raw` `percent`
* `brake_max` Brake command scale factor applied to joystick axis (with `brake_min`). Default `80.0`% (changes with the value of `brake_cmd_type`)
* `brake_min` Brake command scale factor applied to joystick axis (with `brake_max`). Default `0.0`% (changes with the value of `brake_cmd_type`)
* `brake_inc` Brake command rate limit for increase. Default `0.0` (firmware selects default value) (units change with the value of `brake_cmd_type`)
* `brake_dec` Brake command rate limit for decrease. Default `0.0` (firmware selects default value) (units change with the value of `brake_cmd_type`)
* `thrtl` Send throttle commands. Default `true`
* `thrtl_cmd_type` Throttle command type. Default `percent`. Options: `pedal_raw` `percent`
* `thrtl_max` Throttle command scale factor applied to joystick axis (with `thrtl_min`). Default `100.0`% (changes with the value of `thrtl_cmd_type`)
* `thrtl_min` Throttle command scale factor applied to joystick axis (with `thrtl_max`). Default `0.0`% (changes with the value of `thrtl_cmd_type`)
* `thrtl_inc` Throttle command rate limit for increase. Default `0.0`%/s (firmware selects default value)
* `thrtl_dec` Throttle command rate limit for decrease. Default `0.0`%/s (firmware selects default value)
* `shift` Send gear shift commands. Default `true`
* `misc` Send misc commands (turn-signal and others). Default `true`

# Controls

Logitech F310 Gamepad controls:

* Disable
    * Left Bumper
* Enable
    * Right Bumper
* Brakes
    * Left Trigger
* Throttle
    * Right Trigger
* Steering
    * Left/right of either joystick axis
    * Hold back or start to get full steering range, otherwise half
* Turn Signals
    * Left/Right D-Pad toggle on/off

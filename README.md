# ROS package for radio based follow me function
This package was created as part of a [master's thesis](http://hdl.handle.net/10467/114657). It enables a robot to autonomously follow a human
using Ultra-wideband Two-way Ranging and Bluetooth Angle of Arrival measurements. This package contains nodes for localizing the human,
drivers for the sensors are in separate packages ([UWB](https://github.com/brozb/dwm1001_ros), [Bluetooth](https://github.com/brozb/xplraoa_ros)).

## Usage
Several config files have to be edited based on the used sensor configuration:

1.  In [config/twr.yaml](config/twr.yaml) enter the fixed frame (has to already exist and be connected to the tf tree), the name of the frame that marks
    the position of the human, whether the localisation should operate in 3D or in 2D and the ID of the UWB module that the human carries.
    The next parameters are lists, each element describes one of the UWB modules mounted on the robot. `id` has to match the ID set in the firmware
    of the UWB module. `positions` contains three numbers per module, these are the x, y and z coordinates of the module in the fixed coordinate
    frame defined earlier. Parameter `calibration` allows to set a calibration polynomial for each module separately, this polynomial can have
    arbitrary degree (has to be the same for all modules) and is used to correct the range measurements that the corresponding module publishes.

2.  [config/aoa.yaml](config/aoa.yaml) again contains the fixed frame as a parameter. The next parameter is `mode`, it defines how will be the most relevant
    AoA measurement selected (if multiple are available). It can be set to `RSSI` or `external`, the default one is `RSSI`. `RSSI` uses just the signal
    strength, which might not be a reliable indicator of the measurement quality. `external` uses an estimate from other localisation system (i.e. UWB TWR)
    to select the antenna array that has the human in range. This estimate has the form of a tf frame defined in parameter `estimate_frame`.
    Parameter `positions` again defines where are the AoA antennas mounted with respect to the fixed frame, but since they are directional,
    orientation has to be provided too (as quaternion). A tf frame for each antenna is created, parameter `frames` defines how these frames should be named.

4.  The last step is to prepare the launch files. In [launch/aoa.launch](launch/aoa.launch) and [launch/twr.launch](launch/twr.launch) configure
    the driver for each sensor by specifying the port to which it is connected. In the main launch file [launch/radio.launch](launch/radio.launch)
    set the desired gap between the robot and human and set the available sensor systems (`tdoa_available`, `twr_available` and `bt_aoa_available`)

Once all of the preceding steps are complete, the system is ready to be used. It can be started with the [launch/radio.launch](launch/radio.launch)
launch file.

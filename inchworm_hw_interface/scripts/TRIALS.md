Jan 9 2022
- Republish serial data as measured JointState in serial_interface
- Create a PlotJuggler config to live plot relevant data
- Update RW speed over serial for higher resolution data to process
- Create a common command interface packet with the robot that then gets mapped to ROS messages
- Experiment with creating a custom robot interface for ROS tooling
    - Gazebo modeling of robot dynamics might be enough. If this isn't needed, don't bother.
- Quantify Gazebo model accuracy to real robot performance

- Measure robot precision, extrapolate to placement error
    - Encoder resolution + imperfect joints + allowable error
        - Idea: Allowable error should be much lower when placing a tile. Temporary `i` in PID controllers to minimize error?

- See if we have NFC hardware, get specs if we do

Jan 15 2022
Needed improvements for serial_interface
- Choose joint to plot
- Fix crashes at startup (not reading a correct byte?)
    - Clear out serial buffer, repeat until we get a valid response from the robot
- More robust to user error
- Better interface
- Plot all joints at one
- Live plotting of joints (-> PlotJuggler)

Jan 17 2022
- Implemented better messaging interface with the robot. Not yet tested with embedded software
- 
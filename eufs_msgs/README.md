# eufs_msgs
A collection of all ROS messages and services used by the team

## Messages:
- canState.msg - used to communicate with the ADS-DV board computer via CAN. Note that this message is meant to work with the `ros_can` package
- chassisState.msg - from the Autorally repo. Used to signal what the car is doing at some point in time.
- driveCommand.msg - from the Autorally repo. Used to send torque commands to the vehicle. Only 1 torque value
- driveCommand2.msg - same as above but with 2 torque commands. This is intended for the ADS-DV as it has a front and a rear motor.
- driveCommand4.msg - same as above but with 4 torque commands. Intended for 4WD fantasy car
- FilterOutput.msg - from the Autorally repo. Output of their EKF for state estimation
- fullState.msg - used for data collection during dynamics learning. Used in MPPI training
- lapStats.msg - sends useful statistics from lap timings
- pathIntegralParams.msg - from the Autorally repo. Paramters for MPPI
- pathIntegralStats.msg - from the Autorally repo. Combines lapStats and pathIntegralParams. Used for evaluating performance of MPPI
- stateEstimatorStatus.msg - from the Autorally repo.
- systemState.msg - overall mission status of the car. Used in the `ros_can` package
- wheelSpeeds.msg - output of the wheel odometry. Used in the `ros_can` package

## Services:


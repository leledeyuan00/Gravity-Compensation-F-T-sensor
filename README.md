 # Matlab & Python demonstration on Simplified Bias and Gravity Compensation for Wrist-Mounted F/T sensor

## ATI sensor
We are using the ATI axis-80 EnterCAT f/t sensor. By using a Docker package, we can utilize it with ROS2, thereby avoiding Linux sudo privileges and ROS command conflicts.
Following [ATI-EtherCAT ROS2](https://github.com/smartrobotsdesignlab/ATI_ROS2)

The orientation is recorded as [x,y,z,w] format.

## Python

Requirements: Scipy

`gravity_compute.py`: using this file to any environments (Windows or Linux) to calculate the gravity compensation.
Results will be on the `gravity_result.txt` file.

## Matlab codes

There are several MATLAB codes for prototyping validation and analysis.

Requirements: robotics toolbox

1. `gravity_estimated.m`: Prototype code for proposed LSM-short method.
2. `gravity_estimated_six_methods.m`: Monte Carlo cross-validation(MCCV) on 100 sample datas with 6 different methods.
   > The Grobner methods is refereing to this repo: [Grobner Solver](https://github.com/Chastj/non-contact-force-compensation-with-wrist-mounted-f-t-sensor)
3. `drifting_show.m`: Showing the sensor temperature drifting.
4. `gravity_pose_drifting_show.m`: Showing the sensor response and drifting when changed a new poses and static standing several seconds.

## Data

There are several datas for validation.

1. `datas/recorded_messages_pose` and `datas/recorded_messages_wrench`: 7 poses and wrenches for practice estimation.

    Input:

    - recorded_message_pose: the position and pose of the sensor_link with respect to the robot base frame.
    - recorded_message_wrench: the sensor data transformed on the robot base frame.

    Result:
    - x: The estimation result [gravity, bias_x, bias_y, bias_z]
    - xt: The estimation result [centroid of the gripper, bias_torque_x, bias_torque_y. bias_torque,z]
    - e: The estiamtion error on force with each recorded data.
    - et: The estimation error on torque with each recorded data.



2. `datas/recorded_messages_pose_100` and `datas/recorded_messages_wrench_100` for 100 samples k-fold cross-valid.
3. `datas/static_pose_recording`: we left sensor static standing for 35 hours for showing the temperature drifting.
4. `datas/GP2025-07-22-22-06-02`: changed a new poses and static standing several seconds.



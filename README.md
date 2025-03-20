# Matlab demonstration on Simplified Bias and Gravity Compensation for Wrist-Mounted F/T sensor

## Requirements
Matlab: robotics toolbox

## Running

`gravity_estimation.m` is a Matlab file that realizing the paper code.

The other two files are the author recorded data in the practical robots through an ATI F/T sensor.

Result:
- x: The estimation result [gravity, bias_x, bias_y, bias_z]
- xt: The estimation result [centroid of the gripper, bias_torque_x, bias_torque_y. bias_torque,z]
- e: The estiamtion error on force with each recorded data.
- et: The estimation error on torque with each recorded data.


TODO: cite info

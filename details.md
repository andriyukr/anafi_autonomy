# Package details

## Subscribed topics

| Topic Name 	             | Message Type 				                                                                                                                 | Topic Description                                                  		                                                                                                             |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `camera/pose` | `geometry_msgs/PoseStamped` | Camera's pose. |
| `drone/action` | `std_msgs/UInt8` | Drone actions {`0`: no action, `1`: arm, `2`: take-off, `3`: halt, `4`: land, `5`: disarm, `6`: reset pose, `7`: return to home, `11`: start mission, `12`: pause mission, `13`: stop mission, `21`: initialize VIO, `101`: manual control (using SkyController), `102`: offboard (autonomous) control, `110`: reboot, `111`: calibrate magnetometer}. |
| `drone/altitude` | `std_msgs/Float32` | Drone's altitude. |
| `drone/attitude` | `geometry_msgs/QuaternionStamped` | Drone's altitude. |
| `drone/derivative_command` | [`anafi_autonomy/VelocityCommand`](details.md#velocitycommand) | Drone pose derivative command. |
| `drone/gps/location` | `sensor_msgs/NavSatFix` | Drone's GPS location. |
| `drone/reference/attitude` | [`anafi_autonomy/AttitudeCommand`](details.md#attitudecommand) | Drone attitude command. |
| `drone/reference/command` | [`anafi_autonomy/ReferenceCommand`](details.md#referencecommand) | Drone combined command. |
| `drone/reference/pose` | [`anafi_autonomy/PoseCommand`](details.md#posecommand) | Drone pose commnad. |
| `drone/reference/velocity` | [`anafi_autonomy/VelocityCommand`](details.md#velocitycommand) | Drone velocity command. |
| `drone/pose` | `geometry_msgs/PoseStamped` | Drone's pose. |
| `drone/speed` | `geometry_msgs/Vector3Stamped` | Drone's speed. |
| `drone/state` | `std_msgs/String` | Drone's state {`LANDED`, `MOTOR_RAMPING`,	`USER_TAKEOFF`,	`TAKINGOFF`, `HOVERING`, `FLYING`, `LANDING`, `EMERGENCY`, `INVALID`}. |
| `gimbal/attitude` | `geometry_msgs/QuaternionStamped` | Gimbal's attitude. |
| `gimbal/reference` | `geometry_msgs/Vector3` | Gimbal attitude command. |
| `keyboard/command` | [`anafi_autonomy/KeyboardCommand`](details.md#keyboardcommand) | Commands from keyboard. |
| `skycontroller/command` | [`anafi_ros_interfaces/SkycontrollerCommand`](https://github.com/andriyukr/anafi_ros/blob/ros2/details.md#skycontrollercommand) | Commands from SkyController. |
| `zoom/reference` | `std_msgs/Float32` | Zoom velocity command. |

## Published topics

| Topic Name			         | Message Type 					                                                                                  | Frequency | Topic Description 	 | Units |
| ------------------------ | -------------------------------------------------------------------------------------------------------- | --------- | -------------------- | ----- |
| `camera/command` | [`anafi_ros_interfaces/CameraCommand`](https://github.com/andriyukr/anafi_ros/blob/ros2/details.md#cameracommand) | 100 Hz | Camera zoom command. |  |
| `camera/imu` | `sensor_msgs/Imu` | 30 Hz 	| Camera's simulated IMU data. |  |
| `camera/imu/interpolated` | `sensor_msgs/Imu` | 100 Hz | Drone's simulated fast IMU data. |  | 
| `drone/angular_velocity` | `geometry_msgs/Vector3Stamped` | 30 Hz | Drone's angular velocity. |	º/s |
| `drone/command` | [`anafi_ros_interfaces/PilotingCommand`](https://github.com/andriyukr/anafi_ros/blob/ros2/details.md#pilotingcommand) | 100 Hz | Drone piloting command. |  | 
| `drone/debug/mode` | `geometry_msgs/Vector3Stamped` | 100Hz | Debug topic. |  |
| `drone/imu` | `sensor_msgs/Imu` | 30 Hz | Drone's simulated IMU data. |  | 
| `drone/linear_acceleration` | `geometry_msgs/Vector3Stamped` | 30 Hz | Drone's linear acceleration. | m/s^2 | 
| `drone/odometry` | `nav_msgs/Odometry` | (depends on the localisation feedback) | Drone's odometry. |  |
| `drone/position/vision` | `geometry_msgs/PointStamped` | (depends on the localisation feedback) | Drone's position based on visual(-inertial) odometry. |  | 
| `gimbal/command` | [`anafi_ros_interfaces/GimbalCommand`](https://github.com/andriyukr/anafi_ros/blob/ros2/details.md#gimbalcommand) | 100 Hz | Gimbal attitude commands. |  |

## Services

| Service name | Service type | Service description |
| ------------ | ------------ | ------------------- |
|              |              |                     |

## Parameters

| Parameter name     | Type 		| Default value 	         | Values set/range | Parameter description 		                                                                                                                                                                       | Units |
| ------------------ | -------- | ------------------------ | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----- |
| `armed`	| `bool` | `false` | {`true`: armed, `false`: disarmed} | The drone is armed.	| 		|
| `bounds/x/min` | `float` | `-10.0` | [`-4000.0`, `4000.0`] | Min x position bound. | m |
| `bounds/x/max` | `float` | `10.0` | [`-4000.0`, `4000.0`] | Max x position bound. | m |
| `bounds/y/min` | `float` | `-10.0` | [`-4000.0`, `4000.0`] | Min y position bound. | m |
| `bounds/y/max` | `float` | `10.0` | [`-4000.0`, `4000.0`] | Max y position bound. | m |
| `bounds/z/min` | `float` | `0.0` | [`0.0`, `4000.0`] | Min z position bound. | m |
| `bounds/z/max` | `float` | `2.0` | [`0.0`, `4000.0`] | Max z position bound. | m |
| `fixed_frame` | `bool` | `false` |  | Fixed reference frame for velocity commands. |  |
| `flight_plan/file` | `string` | `/missions/test.mavlink` |  | Absolute path to the FlightPlan file. |  |
| `follow_me/mode` | `int` | `2` | `1`: look at the target without moving automatically, `2`: follow the target keeping the same vector, `3`: follow the target keeping the same orientation to its direction, `4`: follow the target as it was held by a leash | FollowMe mode. |  |
| `gains/position/p` | `float` | `2.0` | [`0.0`, `10.0`] | Position PID controller's proportional gain. |  |
| `gains/position/i` | `float` | `1.0` | [`0.0`, `10.0`] | Position PID controller's integral gain. |  |
| `gains/position/d` | `float` | `0.5` |  [`0.0`, `10.0`] | Position PID controller's derivative gain. |  |
| `gains/position/max_i` | `float` | `0.1` | [`0.0`, `1.0`] | Position PID controller's max integral component. |  |
| `gains/velocity/p` | `float` | `9.1` | [`0.0`, `10.0`] | Velocity PD controller's proportional gain. |  |
| `gains/velocity/d` | `float` | `1.3` | [`0.0`, `10.0`] | Velocity PD controller's derivative gain. |  |
| `gains/yaw/p` | `float` | `70.0` | [`0.0`, `100.0`] | Yaw P controller's proportional gain. |  |
| `hand_launch` | `bool` | `true` | {`true`: enabled, `false`: disabled} | Enable hand launched takeoff. |  |
| `landing_control` | `bool` |  `false` | {`true`: enabled, `false`: disabled} | Enable control during landing. |  |
| `mission_type` | `int` | `0` | {`0`: flight plan, `1`: follow me} | Mission type. |  |
| `takingoff_control` | `bool` | `false` | {`true`: enabled, `false`: disabled} | Enable control during takeoff. |  |
| `world_frame` | `bool` | `false` |  | Yaw aligned with world North. |  |

## Custom messages

#### AttitudeCommand
- *std_msgs/Header* **header**: header of the message
- *float32* **roll**: roll angle command (º)
- *float32* **pitch**: pitch command (º)
- *float32* **yaw**: yaw command (º)
- *float32* **throttle**: vertical speed command (m/s)
#### KeyboardCommand
- *std_msgs/Header* **header**: header of the message
- *uint8* **drone_action**: drone action {`0`: no action, `1`: arm, `2`: take-off, `3`: halt, `4`: land, `5`: disarm, `6`: reset pose, `7`: return to home, `11`: start mission, `12`: pause mission, `13`: stop mission, `21`: initialize VIO, `101`: manual control (using SkyController), `102`: offboard (autonomous) control, `110`: reboot, `111`: calibrate magnetometer}
- *int8* **drone_x**: drone x-axis movement {`-1`: backwards, `0`: no movement, `1`: forward} (moves at the maximum horizontal speed!)
- *int8* **drone_y**: drone y-axis movement {`-1`: right, `0`: no movement, `1`: left} (moves at the maximum horizontal speed!)
- *int8* **drone_z**: drone z-axis movement {`-1`: down, `0`: no movement, `1`: up} (moves at the maximum vertical speed!)
- *int8* **drone_yaw**: drone yaw-axis movement {`-1`: clockwise, `0`: no movement, `1`: contrclockwise} (moves at the maximum yaw rate!)
- *int8* **gimbal_roll**: gimbal roll movement {`-1`: roll left, `0`: no movement, `1`: roll right}
- *int8* **gimbal_pitch**: gimbal pitch movement {`-1`: pitch up, `0`: no movement, `1`: pitch down}
- *int8* **gimbal_yaw**: gimbal yaw movement {`-1`: yaw right, `0`: no movement, `1`: yaw left} (not supported yet)
- *uint8* **camera_action**: camera action {`0`: no action, `1`: take picture, `2`: start recording, `3`: stop recording, `4`: download media, `11`: reset gimbal, `111`: calibrate gimbal}
- *int8* **zoom**: zoom change {`-1`: zoom out, `0`: no change, `1`: zoom in}
#### PoseCommand
- *std_msgs/Header* **header**: header of the message
- *float32* **x**: x position command (m)
- *float32* **y**: y position command (m)
- *float32* **z**: z position command (m)
- *float32* **yaw**: yaw orientation command (º)
#### ReferenceCommand
- *uint8* **horizontal_mode**: horizontal control mode {`0`: no control (ignored), `1`: position, `2`: velocity, `3`: attitude}
- *float64* **x**: `horizontal_mode == 1` -> x position command (m), `horizontal_mode == 2` -> x velocity command (m/s), `horizontal_mode == 3` -> roll angle command (º)
- *float64* **y**: `horizontal_mode == 1` -> y position command (m), `horizontal_mode == 2` -> y velocity command (m/s), `horizontal_mode == 3` -> pitch angle command [º]
- *uint8* **vertical_mode**: vertical control mode {`0`: no control (ignored), `1`: position, `2`: velocity}
- *float64* **z**: `vertical_mode == 1` -> z position command (m), `vertical_mode` == 2: z velocity command (m/s)
- *uint8* **heading_mode**: heading control mode {`0`: no control (ignored), `3`: angle, `4`: rate}
- *float32* **yaw**: `heading_mode == 3` -> yaw angle command [º], `heading_mode == 4` -> yaw rate command [º/s]
#### VelocityCommand
- *float32* **vx**: x velocity command (m/s)
- *float32* **vy**: y velocity command (m/s)
- *float32* **vz**: z velocity command (m/s)
- *float32* **yaw_rate**: yaw rate command (º/s)

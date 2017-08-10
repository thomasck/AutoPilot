# AutoPilot

[GNC](https://en.wikipedia.org/wiki/Guidance,_navigation,_and_control) software for quadrotors that I developed for my masters degree. Runs on Linux systems (I2C bus and a USB 3.0 port required), and interfaces with the following hardware components:
* [3DR PX4flow](https://pixhawk.org/modules/px4flow)
* [Invensense MPU-9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/)
* [Intel Realsense R200](https://software.intel.com/en-us/realsense/previous?language=en)
* [NXP PCA-9685](http://www.nxp.com/products/interfaces/ic-bus-portfolio/ic-led-controllers/16-channel-12-bit-pwm-fm-plus-ic-bus-led-controller:PCA9685)

## Dependencies 
* [librealsense](https://github.com/IntelRealSense/librealsense)
* [upm](https://github.com/intel-iot-devkit/upm)
* [mraa](https://github.com/intel-iot-devkit/mraa)

## Features
1. Collision avoidance and waypoint seeking
    * Uses a body fixed potential field method
2. Velocity command following
    * Virtual control [feedback linearization](https://en.wikipedia.org/wiki/Feedback_linearization)  
3. Attitude stability 4 days
    * Integral [backstepping](https://en.wikipedia.org/wiki/Backstepping) design
    * Provides Exponential [stability](https://en.wikipedia.org/wiki/Lyapunov_stability)
4. [Extended Kalman filters](https://en.wikipedia.org/wiki/Extended_Kalman_filter)
    * filter for acceleration data (MPU-9250)
    * filter for velocity and altitude data (PX4flow)

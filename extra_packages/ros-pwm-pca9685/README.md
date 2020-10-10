# ROS driver for NXP Semiconductor PCA9685 I2C PWM chip

This is a ROS node for the PCA9685. The chip is notably used in the following products:

* [Adafruit 16-Channel 12-bit PWM/Servo Driver](https://www.adafruit.com/product/815)
* [Adafruit DC and Stepper Motor HAT for Raspberry Pi](https://www.adafruit.com/product/2348)
* [Waveshare Motor Driver HAT for Raspberry Pi](https://www.waveshare.com/motor-driver-hat.htm)
* [Waveshare Servo Driver HAT for Raspberry Pi](https://www.waveshare.com/servo-driver-hat.htm)

There should be no dependencies besides libi2c-dev.

## Parameters:

* **device** (string) -- the path to the i2c device. Default is /dev/i2c-1. Use i2cdetect in the i2c-tools package to find out which bus your device is on.
* **address** (int) -- the i2c address of the PCA9685. Default is 0x40.
* **frequency** (int) -- PWM frequency in Hz. Default is 1600.
* **timeout** (list of ints) -- List of 16 integers corresponding to timeouts in milliseconds for each channel. If any particular channel is not updated in this time, that channel will be set to the corresponding value in **timeout_value** until another update is received. Defaults to `[ 5000, 5000, ... ]`, i.e. each channel timeouts after 5 seconds if no updates are published.
* **timeout_value** (list of ints) -- The value each channel will be set to upon timeout. Defaults to `[ 0, 0, ... ]`, i.e. each channel is set to 0 upon timeout.
* **pwm_min** (list of ints) -- The minimum PWM value for each channel. Defaults to `[ 0, 0, ... ]`. If a command lower than the minimum PWM value is issued, the PWM value will be set to param_min, with the exception of a -1 command, which designates no update (see below).
* **pwm_max** (list of ints) -- The maximum PWM value for each channel. Defaults to `[ 65535, 65535, ... ]`. If a command larger than param_max is issued, the PWM value will be set to param_max.

## Subscribers
* **command** -- a Int32MultiArray containing exactly 16 values corresponding to the 16 PWM channels. For each value, specify -1 to make no update to a channel. Specify a value between 0 and 65535 inclusive to update the channel's PWM value. For example, ```{data: [32767, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]}``` will update channel 0 to a PWM of 50%, channel 1 to 0%, and make no updates to other channels.

## Publishers
None.

## Services
None.

# Usage notes

## With the Adafruit 16-channel servo breakout (or any other servo breakout)

The default I2C address is 0x40. A full servo range tyically corresponds to a PWM duty cycle of about 2800/65535 to 7600/65535 (NOT 0/65535 to 65535/65535), so make sure you publish updates according to the servo PWM range. You will also want to set the **frequency** parameter to 50 Hz for servos.

## With the Adafruit Motor Driver HAT

The default I2C address is 0x60.

The PCA9685 generates PWM signals that are connected to two TB6612 dual motor driver chips, for a total of 4 motors. Three PCA9685 channels are connected to each motor, as described below, for a total of 4*3=12 channels wired to the motors. The 4 "extra" PWM pins (0, 1, 14, 15) are not connected to any motors are broken out on a separate header on the board.

Refer to the [Toshiba TB6612FNG documentation](https://www.sparkfun.com/datasheets/Robotics/TB6612FNG.pdf) for more info on the TB6612.

For each motor, there are 3 pins: PWM, IN1, and IN2.
* PWM should be a PWM signal (0 to 65535) indicating the amount of power.
* IN1 should be binary, i.e. set to 0 OR 65535, nothing in-between.
* IN2 should be binary, i.e. set to 0 OR 65535, nothing in-between.
* IN1 = 65535, IN2 = 65535: Brake
* IN1 = 65535, IN2 = 0: Forward
* IN1 = 0, IN2 = 65535: Reverse
* IN1 = 0, IN2 = 0: High impedance (coast)

### Motor channels

* Motor1: PWM = channel 8, IN1 = channel 9, IN2 = channel 10
* Motor2: PWM = channel 13, IN1 = channel 11, IN2 = channel 12
* Motor3: PWM = channel 2, IN1 = channel 3, IN2 = channel 4
* Motor4: PWM = channel 7, IN1 = channel 5, IN2 = channel 6

For example, to set motor 1 to forward at 50%, send ```{data: [-1, -1, -1, -1, -1, -1, -1, -1, 32767, 65535, 0, -1, -1, -1, -1, -1]}```

# One Wheeled Skateboard

#### Developed by Kristian Lauszus, 2018

The code is released under the GNU General Public License.
_________
[![Build Status](https://travis-ci.com/Lauszus/OneWheeledSkateboard.svg?token=ppc6rHRAs23cjxNFyjc1&branch=master)](https://travis-ci.com/Lauszus/OneWheeledSkateboard)

This directory contains the motor and app configuration for the [VESC 6](https://vesc-project.com/) used on the skateboard.

The [VESC firmware](VESC_HW60_v338_custom_BLDC_4_ChibiOS.bin) is slighly modified to send out the battery voltage, current in and the temperatures via CAN-Bus. The source code for the modified firmware can be found at the following repository: <https://github.com/Lauszus/bldc>.

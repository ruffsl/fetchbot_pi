# fetchbot_pi
Ball fetching robot

## Installation

Install dependencies:

Setup udev for lidar via USB:
https://github.com/uos/sick_tim/tree/indigo/udev

## Up and Running

### Connecting to ball IMU via bluetooth

```
sudo rfcomm connect rfcomm0 98:D3:31:FB:27:B2
```
Source: [embeddedprogrammer.blogspot.com](http://embeddedprogrammer.blogspot.com/2012/06/ubuntu-hacking-hc-06-bluetooth-module.html)

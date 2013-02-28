## Boost ASIO + ROS Sandbox
use at your own risk...

## Notes

Boost::posix_time based timer is much higher resolution than ros::WallTimer. Jitter on the STM32 line is about 50-75uS when running the posix_time version at 1khz. With the ros::WallTimer version, we often lost entire packets.

![1khz_good](https://raw.github.com/mikeferguson/sandbox/master/asio_ros/doc/1khz_good.png)
![jitter](https://raw.github.com/mikeferguson/sandbox/master/asio_ros/doc/jitter.png)

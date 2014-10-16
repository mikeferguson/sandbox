## Boost ASIO + ROS Sandbox
This is a simple prototype of using boost::asio + ROS for ethernet connected devices. Use at your own risk...

## Notes

Boost::posix_time based timer is much higher resolution than ros::WallTimer. Jitter on the STM32 line is about 50-75uS when running the posix_time version at 1khz. With the ros::WallTimer version, we often lost entire packets.

![1khz_good](https://raw.githubusercontent.com/mikeferguson/sandbox/hydro-devel/asio_ros/doc/1khz_good.png)
![jitter](https://raw.githubusercontent.com/mikeferguson/sandbox/hydro-devel/asio_ros/doc/jitter.png)

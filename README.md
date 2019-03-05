# aruco_detector_osv

The aruco_detector_osv (aruco_detector_opencv) uses the #include <opencv2/aruco.hpp> library instead of using #include <aruco/aruco.h> (ros-kinetic-aruco). This package was tested on ubuntu 16.04, ROS Kinetic with a Logitech C920 camera. 

With this package you are able to:

* detect position and orientation of an [aruco marker](http://chev.me/arucogen/) relatively to the camera. The corresponding **tf** is published.
* a certainty parameter of how robust the arcuco marker is detected. 
* a result image with the detected markers highlighted is published.

Please calibrate your camera first using: [camera_calibration](http://wiki.ros.org/camera_calibration).

## License BSD
If you want to use this package please contact: [me](https://simact.de/about_me).

## Dependencies:
cv_bridge image_geometry geometry_msgs roscpp rospy std_msgs tf2 tf2_ros image_transport std_msgs

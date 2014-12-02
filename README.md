ofxKinect2
==========

Kinect2 wrapper for openFrameworks

This is for Kinect2 preview version sdk
http://www.microsoft.com/en-us/kinectforwindows/develop/default.aspx

It requires Kinect2 device and KinectSDK-v2.0-PublicPreview1409-Setup installed.

Kinect v2 SDK only works on Windows 8.

Usage: You should add Kinect2 include and library.

add to C++ -> General -> Additional Include Directories: $(KINECTSDK20_DIR)\inc;  
add to Linker -> General -> Additional Library Directories: $(KINECTSDK20_DIR)\lib\x64;
ofxKinect2
==========

Kinect2 wrapper for openFrameworks

This is for Kinect2 preview version sdk

It requires Kinect2 device and KinectSDK-v2.0-DevPreview1311 installed.

Kinect SDK only works on 64bit application on Windows 8.

You can find 64bit OF blanch below.  
https://github.com/sadmb/openFrameworks/tree/Win64

[Note] You should run KinectService while running Kinect2 app.

Usage: You should add Kinect2 include and library.

add to C++ -> General -> Additional Include Directories: $(KINECTSDK20_DIR)\inc;  
add to Linker -> General -> Additional Library Directories: $(KINECTSDK20_DIR)\lib\x64;
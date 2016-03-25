ofxKinect2
==========

Kinect SDK 2.0 wrapper for openFrameworks

Download and install Kinect SDK 2.0 from here.  
https://www.microsoft.com/en-us/download/details.aspx

Usage: You should add Kinect2 include and library manually.

add to C++ -> General -> Additional Include Directories: $(KINECTSDK20_DIR)inc;  
add to Linker -> Input -> Additional Dependencies: Kinect20.lib;  

(for Win32)  
add to Linker -> General -> Additional Library Directories: $(KINECTSDK20_DIR)Lib\x86;  
(for x64)  
add to Linker -> General -> Additional Library Directories: $(KINECTSDK20_DIR)Lib\x64;

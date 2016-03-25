#pragma once

#include "ofMain.h"
#include "ofxKinect2.h"

class ofApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	void exit();

	ofxKinect2::Device* device_;

	ofxKinect2::IrStream ir_;
	ofxKinect2::ColorStream color_;
	ofxKinect2::DepthStream depth_;
	ofxKinect2::BodyStream body_stream_;
    ofxKinect2::Mapper mapper_;

	ofEasyCam cam_;
	ofPixels mapped_pix_;
	float scale_;
	ofVec3f target_;
};

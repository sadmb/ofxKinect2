#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	device_ = new ofxKinect2::Device();
	device_->setup();

	if(depth_.setup(*device_))
	{
		depth_.open();
	}

	if (color_.setup(*device_))
	{
		color_.open();
	}

	if (ir_.setup(*device_))
	{
		ir_.open();
	}
	
	if(body_stream_.setup(*device_))
	{
		body_stream_.open();
	}
}

//--------------------------------------------------------------
void ofApp::update(){
	device_->update();
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
	color_.draw();
	depth_.draw(1920 - 512, 1080 - 424);
	ir_.draw(1920 - 512, 1080 - 848);
	body_stream_.draw(1920 - 512, 1080 - 424, 512, 424);

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
void ofApp::exit()
{
	color_.close();
	depth_.close();
	ir_.close();
	body_stream_.close();
	device_->exit();
	delete device_;
	device_ = NULL;
}
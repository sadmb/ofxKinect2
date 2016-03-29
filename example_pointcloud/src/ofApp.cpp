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
    
    if(mapper_.setup(*device_))
    {
		mapper_.setColor(color_);
		mapper_.setDepth(depth_);
    }

	scale_ = 20;
	mapped_pix_.allocate(512, 424, OF_PIXELS_RGBA);

	ofVec3f scale = cam_.getScale();
	cam_.setScale(scale.x, scale.y, -scale.z);
	cam_.setFarClip(10000000);
}

//--------------------------------------------------------------
void ofApp::update(){
	device_->update();
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
    
	if (!mapper_.isReady())
	{
		return;
	}
    // point cloud
	vector<ofVec3f>& pointcloud_pos = mapper_.mapDepthToCameraSpace();
	vector<ofFloatColor>& pointcloud_colors = mapper_.getFloatColorsCoordinatesToDepthFrame();

	cam_.begin();
	ofPushMatrix();
	ofEnableDepthTest();
	ofScale(100 * scale_, 100 * scale_, 100 * scale_);
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
	mesh.addVertices(pointcloud_pos);
	mesh.addColors(pointcloud_colors);

	int width = depth_.getWidth();
	int height = depth_.getHeight();
	for (int y = 0; y < height - 1; y++)
	{
		for (int x = 0; x < width - 1; x++)
		{
			mesh.addIndex(x + y*width);               // 0
			mesh.addIndex((x + 1) + y*width);           // 1
			mesh.addIndex(x + (y + 1)*width);           // 10

			mesh.addIndex((x + 1) + y*width);           // 1
			mesh.addIndex((x + 1) + (y + 1)*width);       // 11
			mesh.addIndex(x + (y + 1)*width);           // 10
		}
	}
	mesh.draw();
	ofPopMatrix();
	cam_.end();

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key)
	{
	case OF_KEY_UP:
	case OF_KEY_RIGHT:
		scale_++;
		if (scale_ > 100)
		{
			scale_ = 100;
		}
		break;
	case OF_KEY_DOWN:
	case OF_KEY_LEFT:
		scale_--;
		if (scale_ < 1)
		{
			scale_ = 1;
		}
		break;
	}
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
	target_ = ofVec3f(0, 0, 5.0);
	cam_.setTarget(target_);
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
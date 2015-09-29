// @author sadmb
// @date 3,Jan,2014
// modified from ofxNI2.cpp of ofxNI2 by @satoruhiga
#ifndef OFX_KINECT2_H
#define OFX_KINECT2_H

#include "ofMain.h"
#include "ofxKinect2Types.h"
#include "utils/DoubleBuffer.h"

#include <assert.h>

namespace ofxKinect2
{
	void init();
	class Device;
	class Stream;
	
	class IrStream;
	class ColorStream;
	class DepthStream;

	class Body;
	class BodyStream;
	
	class Recorder;

	template<class Interface>
	inline void safe_release(Interface *& p_release)
	{
		if(p_release){
			p_release->Release();
			p_release = NULL;
		}
	}
} // namespace ofxKinect2

// device
class ofxKinect2::Device
{
	friend class ofxKinect2::Stream;

public:
	ofEvent<int> onUpdateStream;

	Device();
	~Device();

	bool setup();
	bool setup(string kinect2_file_path);

	void exit();

	void update();
/*
	bool startRecording(string filename = "");
	void stopRecording();
	bool isRecording() const { return recorder != NULL; }
	*/

	bool isOpen() const
	{
		if (device.kinect2 == NULL) return false;
		bool b = false;
		device.kinect2->get_IsOpen((BOOLEAN*)&b);
		return b;
	}

	void setDepthColorSyncEnabled(bool b = true);
	bool isDepthColorSyncEnabled() const { return enable_depth_color_sync; }

	DeviceHandle& get() { return device; }
	const DeviceHandle& get() const { return device; }

	ICoordinateMapper* getMapper() { 
		if(!mapper)
		{
			HRESULT hr = device.kinect2->get_CoordinateMapper(&mapper);
		}
		return mapper;
	}

protected:

	DeviceHandle device;
	ICoordinateMapper* mapper;
	vector<ofxKinect2::Stream*> streams;
	bool enable_depth_color_sync;

	Recorder* recorder;
};

class ofxKinect2::Recorder
{
};

// stream
class ofxKinect2::Stream : public ofThread
{
	friend class ofxKinect2::Device;

public:

	virtual ~Stream();

	virtual void exit();

	virtual bool open();
	virtual void close();

	virtual void update();
	virtual bool updateMode();

	bool isOpen() const
	{
		bool b = (stream.p_audio_beam_frame_reader != NULL) || (stream.p_body_frame_reader != NULL) || (stream.p_body_index_frame_reader != NULL) || (stream.p_color_frame_reader != NULL) || (stream.p_depth_frame_reader != NULL)
			|| (stream.p_infrared_frame_reader != NULL) || (stream.p_long_exposure_infrared_frame_reader != NULL);
		return b;
	}

	int getWidth() const;
	virtual bool setWidth(int v);
	int getHeight() const;
	virtual bool setHeight(int v);
	virtual bool setSize(int width, int height);

	ofTexture& getTextureReference() {return tex;}

	int getFps();
	bool setFps(int v);

	void setMirror(bool v = true);
	bool isMirror();

	float getHorizontalFieldOfView();
	float getVerticalFieldOfView();
	float getDiagonalFieldOfView();

	inline bool isFrameNew() const { return is_frame_new; }

	void draw(float x = 0, float y = 0);
	virtual void draw(float x, float y, float w, float h);

	operator StreamHandle&() { return stream; }
	operator const StreamHandle&() const { return stream; }

	StreamHandle& get() { return stream; }
	const StreamHandle& get() const { return stream; }

	CameraSettingsHandle& getCameraSettings() { return camera_settings; }
	const CameraSettingsHandle& getCameraSettings() const { return camera_settings; }

protected:
	Frame frame;
	StreamHandle stream;
	CameraSettingsHandle camera_settings;
	uint64_t kinect2_timestamp, opengl_timestamp;

	bool is_frame_new, texture_needs_update;

	bool is_mirror;

	ofTexture tex;
	Device* device;

	Stream();

	void threadedFunction();

	bool setup(Device& device, SensorType sensor_type);
	virtual bool readFrame(IMultiSourceFrame* p_multi_frame = NULL);
	virtual void setPixels(Frame frame);
};

class ofxKinect2::ColorStream : public ofxKinect2::Stream
{
public:
	bool setup(ofxKinect2::Device& device)
	{
		buffer = NULL;
		return Stream::setup(device, SENSOR_COLOR);
	}
	void exit()
	{
		Stream::exit();
		if (buffer)
		{
			delete[] buffer;
			buffer = NULL;
		}
	}
	bool open();
	void close();

	void update();
	bool updateMode();

	bool setWidth(int v);
	bool setHeight(int v);
	bool setSize(int width, int height);

	ofPixels& getPixelsRef() { return pix.getFrontBuffer(); }

	int getExposureTime();
	int getFrameInterval();
	float getGain();
	float getGamma();

	/*
	void setAutoExposureEnabled(bool yn = true) {  }
	bool getAutoExposureEnabled() {  }

	void setAutoWhiteBalanceEnabled(bool yn = true) {  }
	bool getAutoWhiteBalanceEnabled() {  }
	*/

protected:
	DoubleBuffer<ofPixels> pix;
	unsigned char* buffer;

	bool readFrame(IMultiSourceFrame* p_multi_frame = NULL);
	void setPixels(Frame frame);
};

class ofxKinect2::DepthStream : public ofxKinect2::Stream
{
public:
	bool setup(ofxKinect2::Device& device)
	{
		near_value = 50;
		far_value = 10000;
		return Stream::setup(device, SENSOR_DEPTH);
	}

	bool open();
	void close();

	void update();
	bool updateMode();

	ofShortPixels& getPixelsRef() { return pix.getFrontBuffer(); }
	ofShortPixels getPixelsRef(int _near, int _far, bool invert = false);

	inline void setNear(float _near) { near_value = _near; }
	inline float getNear() const { return near_value; }

	inline void setFar(float _far) { far_value = _far; }
	inline float getFar() const { return far_value; }

	inline void setInvert(float invert) { is_invert = invert; }
	inline bool getInvert() const { return is_invert; }

//	ofVec3f getWorldCoordinateAt(int x, int y);

protected:
	DoubleBuffer<ofShortPixels> pix;

	float near_value;
	float far_value;
	bool is_invert;

	bool readFrame(IMultiSourceFrame* p_multi_frame = NULL);
	void setPixels(Frame frame);
};

class ofxKinect2::IrStream : public ofxKinect2::Stream
{
public:

	bool setup(ofxKinect2::Device& device)
	{
		return Stream::setup(device, SENSOR_IR);
	}
	bool open();
	void close();

	void update();
	bool updateMode();


	ofShortPixels& getPixelsRef() { return pix.getFrontBuffer(); }

protected:
	DoubleBuffer<ofShortPixels> pix;

	bool readFrame(IMultiSourceFrame* p_multi_frame = NULL);
	void setPixels(Frame frame);


};

class ofxKinect2::Body
{
	friend class BodyStream;
public:
	typedef ofPtr<Body> Ref;

	Body():left_hand_state(HandState_Unknown), right_hand_state(HandState_Unknown), is_tracked(false), is_update_scale(false) {
		joints.resize(JointType_Count);
		joint_points.resize(JointType_Count);
	}

	void setup(ofxKinect2::Device& device)
	{
		this->device = &device;
	}

	void close();

	void update(IBody* body);
	void drawBody(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight());
	void drawBone(JointType joint0, JointType joint1, int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight());
	void drawHands(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight());
	void drawHandLeft(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight());
	void drawHandRight(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight());
	void drawLean(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight());

	void setId(UINT64 _id) { id = _id; }
	inline int getId() const { return id;}

	void setTracked(bool _is_tracked) { is_tracked = _is_tracked; }
	inline bool isTracked() const { return is_tracked;}

	inline HandState getLeftHandState() const { return left_hand_state; }
	inline HandState getRightHandState() const { return left_hand_state; }

	inline size_t getNumJoints() { return JointType_Count; }

	const Joint& getJoint(size_t idx) { return joints[idx]; }

	const ofPoint& getJointPoint(size_t idx) { return joint_points[idx]; }
	const vector<ofPoint> getJointPoints() { return joint_points; }
	 
	const ofVec2f& getLean() { return body_lean; }
private:
	Device* device;
	vector<Joint> joints;
	vector<ofPoint> joint_points;
	bool is_tracked;
	UINT64 id;
	TrackingState lean_state;
	ofVec2f body_lean;
	bool is_update_scale;

	HandState left_hand_state;
	HandState right_hand_state;

	ofPoint jointToScreen(const JointType jointType, int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight());
	ofPoint bodyPointToScreen(const CameraSpacePoint& bodyPoint, int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight());
};

class ofxKinect2::BodyStream : public Stream
{
public:
	bool setup(ofxKinect2::Device& device)
	{
		for(int i = 0; i < BODY_COUNT; i++)
		{
			Body body;
			body.setup(device);
			bodies.push_back(body);
		}
		return Stream::setup(device, SENSOR_BODY);
	}
	bool open();
	void close();

	void update();
	bool updateMode();

	void draw(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight(), size_t idx = BODY_COUNT);
	void drawBody(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight(), size_t idx = BODY_COUNT);
	void drawBone(JointType joint0 = JointType_SpineBase, JointType joint1 = JointType_SpineMid, int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight(), size_t idx = BODY_COUNT);
	void drawHands(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight(), size_t idx = BODY_COUNT);
	void drawHandLeft(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight(), size_t idx = BODY_COUNT);
	void drawHandRight(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight(), size_t idx = BODY_COUNT);
	void drawLean(int x = 0, int y = 0, int w = ofGetWidth(), int h = ofGetHeight(), size_t idx = BODY_COUNT);


	inline size_t getNumBodies() { return bodies.size(); }
	const vector<Body> getBodies() { return bodies; }
	const Body getBody(size_t idx)
	{
		for(int i = 0; i < bodies.size(); i++)
		{
			if(bodies[i].getId() == idx)
			{
				return bodies[idx];
			}
		}
		return bodies[0];
	}

	ofShortPixels& getPixelsRef() { return pix.getFrontBuffer(); }
	ofShortPixels getPixelsRef(int _near, int _far, bool invert = false);

	ofPoint righthand_pos_;

protected:
	DoubleBuffer<ofShortPixels> pix;
	vector<Body> bodies;

	bool readFrame(IMultiSourceFrame* p_multi_frame = NULL);
	void setPixels(Frame frame);

};

/*
class ofxKinect2::ColorMappingStream : public ofxKinect2::Stream
{
public:
	bool setup(ofxKinect2::Device& device)
	{
//		c_buffer = new unsigned char[c_width * c_height];
//		color_map = new ColorSpacePoint[d_width * d_height];
		return Stream::setup(device, SENSOR_DEPTH);
	}

	void exit()
	{
		Stream::exit();
		if (c_buffer)
		{
			delete[] c_buffer;
			c_buffer = NULL;
		}

		if (color_map)
		{
			delete[] color_map;
			color_map = NULL;
		}
	}
	bool open();
	void close();

	void update();
	bool updateMode();

	ofPixels& getPixelsRef() { return pix.getFrontBuffer(); }

protected:
	DoubleBuffer<ofPixels> pix;
	BYTE* c_buffer;
	ColorSpacePoint* color_map;

	bool readFrame(IMultiSourceFrame* p_multi_frame = NULL);
	void setPixels(Frame frame);
};
/**/

#endif // OFX_KINECT2_H
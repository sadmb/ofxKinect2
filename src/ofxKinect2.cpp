// @author sadmb
// @date 3,Jan,2014
// modified from ofxNI2.cpp of ofxNI2 by @satoruhiga

#include "ofxKinect2.h"
#include "utils\DepthRemapToRange.h"

namespace ofxKinect2
{
	//----------------------------------------------------------
	void init()
	{
		static bool inited = false;
		if(inited) return;
		inited = true;
		
	}
} // namespace ofxKinect2

using namespace ofxKinect2;

//----------------------------------------------------------
#pragma mark - Device
//----------------------------------------------------------

//----------------------------------------------------------
Device::Device() : recorder(NULL), enable_depth_color_sync(false), mapper(NULL)
{
	device.kinect2 = NULL;
}

//----------------------------------------------------------
Device::~Device()
{
	exit();
}

//----------------------------------------------------------
bool Device::setup()
{
	ofxKinect2::init();

	HRESULT hr;

	hr = GetDefaultKinectSensor(&device.kinect2);

	if (SUCCEEDED(hr))
	{
		device.kinect2->Open();
		return true;
	}

	return false;
}

//----------------------------------------------------------
bool Device::setup(string kinect2_file_path)
{
	ofxKinect2::init();

	kinect2_file_path = ofToDataPath(kinect2_file_path);

	ofLogWarning("ofxKinect2::Device") << " Open from path is not supported yet.";
	return setup();
}

//----------------------------------------------------------
void Device::exit()
{
//	stopRecording();
	if(device.kinect2){
		device.kinect2->Close();
	}
	safe_release(device.kinect2);

	vector<Stream*>::iterator it;
	int counter = 0;
	while (!streams.empty())
	{
		it = streams.begin();
		(*it)->exit();
		counter++;

		// just in case streams array mismatch .
		if (counter > 1000)
		{
			ofLogNotice("ofxKinect2::Device") << "streams array is incorrect.";
			break;
		}
	}

	streams.clear();

	if (mapper)
	{
		safe_release(mapper);
	}

}

//----------------------------------------------------------
void Device::update()
{
	if (!isOpen()) return;

	for (int i = 0; i < streams.size(); i++)
	{
		Stream* s = streams[i];
		s->is_frame_new = s->kinect2_timestamp != s->opengl_timestamp;
		s->opengl_timestamp = s->kinect2_timestamp;
	}
	int val = 0;
	ofNotifyEvent(onUpdateStream, val);

	if(mapper)
	{
		safe_release(mapper);
	}
}

//----------------------------------------------------------
void Device::setDepthColorSyncEnabled(bool b)
{
	enable_depth_color_sync = b;
	if (b)
	{
		HRESULT hr;
		hr = device.kinect2->get_CoordinateMapper(&mapper);
		if (SUCCEEDED(hr))
		{
		}
		else
		{
			ofLogWarning("ofxKinect2::Device") << "Cannot start depth color sync";
		}
	}
	else
	{
		if (mapper)
		{
			safe_release(mapper);
		}
	}
}

//----------------------------------------------------------
#pragma mark - Stream
//----------------------------------------------------------

//----------------------------------------------------------
Stream::Stream() {}
//----------------------------------------------------------
Stream::~Stream() {}

//----------------------------------------------------------
bool Stream::setup(Device& device, SensorType sensor_type)
{
	if (!device.isOpen())
	{
		return false;
	}
	kinect2_timestamp = 0;
	opengl_timestamp = 0;
	frame.sensor_type = sensor_type;
	frame.mode.fps = 0;
	frame.frame_index = 0;
	frame.stride = 0;
	frame.data = NULL;
	frame.data_size = 0;
	is_frame_new = false;
	texture_needs_update = false;

	device.streams.push_back(this);
	this->device = &device;

	setMirror(false);

	return true;
}

//----------------------------------------------------------
void Stream::exit()
{
	for (vector<Stream*>::iterator it = device->streams.begin(); it != device->streams.end(); )
	{
		Stream* s = *it;
		if(s == this)
		{
			it = device->streams.erase(it);
			close();
		}
		else
		{
			++it;
		}
	}
}

//----------------------------------------------------------
bool Stream::open()
{
	startThread();
	return true;
}

//----------------------------------------------------------
void Stream::close()
{
	while(!lock())
	{
	}
	{
		frame.frame_index = 0;
		frame.stride = 0;
		frame.data = NULL;
		frame.data_size = 0;
		stopThread();
		unlock();
	}
}

//----------------------------------------------------------
void Stream::threadedFunction()
{
	while(isThreadRunning() != 0)
	{
		if (lock())
		{
			if (readFrame())
			{
				kinect2_timestamp = frame.timestamp;
				texture_needs_update = true;
			}
			unlock();
		}
		if (frame.mode.fps != 0)
		{
			ofSleepMillis(1000.f / frame.mode.fps);
		}
	}
}

//----------------------------------------------------------
bool Stream::readFrame(IMultiSourceFrame* p_multi_frame)
{
	return false;
}

//----------------------------------------------------------
void Stream::setPixels(Frame frame)
{
	kinect2_timestamp = frame.timestamp;
}

//----------------------------------------------------------
void Stream::update()
{
	texture_needs_update = false;
}

//----------------------------------------------------------
bool Stream::updateMode()
{
	return true;
}

//----------------------------------------------------------
bool Stream::setSize(int width, int height)
{
	frame.mode.resolution_x = width;
	frame.mode.resolution_y = height;

	return updateMode();
}

//----------------------------------------------------------
int Stream::getWidth() const
{
	return frame.width;
}

//----------------------------------------------------------
int Stream::getHeight() const
{
	return frame.height;
}

//----------------------------------------------------------
bool Stream::setWidth(int v)
{
	return setSize(v, getHeight());
}

//----------------------------------------------------------
bool Stream::setHeight(int v)
{
	return setSize(getWidth(), v);
}

//----------------------------------------------------------
int Stream::getFps()
{
	return frame.mode.fps;
}

//----------------------------------------------------------
bool Stream::setFps(int v)
{
	frame.mode.fps = v;
	return updateMode();
}

//----------------------------------------------------------
float Stream::getHorizontalFieldOfView()
{
	return frame.horizontal_field_of_view;
}

//----------------------------------------------------------
float Stream::getVerticalFieldOfView()
{
	return frame.vertical_field_of_view;
}

//----------------------------------------------------------
float Stream::getDiagonalFieldOfView()
{
	return frame.diagonal_field_of_view;
}

//----------------------------------------------------------
void Stream::setMirror(bool v)
{
	is_mirror = false;
}

//----------------------------------------------------------
bool Stream::isMirror()
{
	return is_mirror;
}

//----------------------------------------------------------
void Stream::draw(float x, float y)
{
	draw(x, y, getWidth(), getHeight());
}

//----------------------------------------------------------
void Stream::draw(float x, float y, float w, float h)
{
	if (texture_needs_update)
	{
		update();
	}

	if (tex.isAllocated())
	{
		ofSetColor(ofColor::white);
		tex.draw(x, y, w, h);
	}
}

//----------------------------------------------------------
#pragma mark - ColorStream
//----------------------------------------------------------

//----------------------------------------------------------
bool ColorStream::readFrame(IMultiSourceFrame* p_multi_frame)
{
	bool readed = false;
	if (!stream.p_color_frame_reader)
	{
		ofLogWarning("ofxKinect2::ColorStream") << "Stream is not open.";
		return readed;
	}
	Stream::readFrame(p_multi_frame);

	IColorFrame* p_frame = NULL;
	
	HRESULT hr;
	if (!p_multi_frame)
	{
		hr = stream.p_color_frame_reader->AcquireLatestFrame(&p_frame);
	}
	else
	{
		IColorFrameReference* p_frame_reference = NULL;
		hr = p_multi_frame->get_ColorFrameReference(&p_frame_reference);

		if (SUCCEEDED(hr))
		{
			hr = p_frame_reference->AcquireFrame(&p_frame);
		}

		safe_release(p_frame_reference);
	}

	if (SUCCEEDED(hr))
	{
		IFrameDescription* p_frame_description = NULL;
		ColorImageFormat image_format = ColorImageFormat_None;

		hr = p_frame->get_RelativeTime((INT64*)&frame.timestamp);

		if (SUCCEEDED(hr))
		{
			hr = p_frame->get_FrameDescription(&p_frame_description);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_Width(&frame.width);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_Height(&frame.height);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_HorizontalFieldOfView(&frame.horizontal_field_of_view);
		}
		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_VerticalFieldOfView(&frame.vertical_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_DiagonalFieldOfView(&frame.diagonal_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame->get_RawColorImageFormat(&image_format);
		}

		if (SUCCEEDED(hr))
		{
			if (buffer == NULL)
			{
				buffer = new unsigned char[frame.width * frame.height * 4];
			}
			if (image_format == ColorImageFormat_Rgba)
			{
				hr = p_frame->AccessRawUnderlyingBuffer((UINT*)&frame.data_size, reinterpret_cast<BYTE**>(&frame.data));
			}
			else
			{
				frame.data = buffer;
				frame.data_size = frame.width * frame.height * 4 * sizeof(unsigned char);
				hr = p_frame->CopyConvertedFrameDataToArray((UINT)frame.data_size, reinterpret_cast<BYTE*>(frame.data),  ColorImageFormat_Rgba);
			}
		}

		if (SUCCEEDED(hr))
		{
			readed = true;
			setPixels(frame);
		}
		safe_release(p_frame_description);
	}

	safe_release(p_frame);

	return readed;
}

//----------------------------------------------------------
void ColorStream::setPixels(Frame frame)
{
	Stream::setPixels(frame);

	int w = frame.width;
	int h = frame.height;
	int num_pixels = w * h;

	const unsigned char * src = (const unsigned char*)frame.data;
	unsigned char *dst = pix.getBackBuffer().getPixels();

	pix.getBackBuffer().setFromPixels(src, w, h, OF_IMAGE_COLOR_ALPHA);
	pix.swap();
}

//----------------------------------------------------------
bool ColorStream::setWidth(int width)
{
	bool ret = Stream::setWidth(width);
	pix.deallocate();
	pix.allocate(frame.width, frame.height, 4);
	return ret;
}

//----------------------------------------------------------
bool ColorStream::setHeight(int height)
{
	bool ret = Stream::setHeight(height);
	pix.allocate(frame.width, frame.height, 4);
	return ret;
}

//----------------------------------------------------------
bool ColorStream::setSize(int width, int height)
{
	bool ret = Stream::setSize(width, height);
	pix.allocate(frame.width, frame.height, 4);
	return ret;
}

//----------------------------------------------------------
void ColorStream::update()
{
	if(!tex.isAllocated()
		|| tex.getWidth() != getWidth()
		|| tex.getHeight() != getHeight())
	{
		tex.allocate(getWidth(), getHeight(), GL_RGB);
	}

	if (lock())
	{
		tex.loadData(pix.getFrontBuffer());
		Stream::update();
		unlock();
	}
}

//----------------------------------------------------------
bool ColorStream::open()
{
	if (!device->isOpen())
	{
		ofLogWarning("ofxKinect2::ColorStream") << "No ready Kinect2 found.";
		return false;
	}
	IColorFrameSource* p_source = NULL;
	HRESULT hr;

	hr = device->get().kinect2->get_ColorFrameSource(&p_source);

	if (SUCCEEDED(hr))
	{
		hr = p_source->OpenReader(&stream.p_color_frame_reader);
	}
	IFrameDescription* p_frame_description = NULL;
	p_source->get_FrameDescription(&p_frame_description);
	if (SUCCEEDED(hr))
	{
		int res_x, res_y = 0;
		hr = p_frame_description->get_Width(&res_x);
		hr = p_frame_description->get_Width(&res_y);
		frame.mode.resolution_x = res_x;
		frame.mode.resolution_y = res_y;
		frame.width = res_x;
		frame.height = res_y;
		pix.allocate(res_x, res_y, 4);

	}
	safe_release(p_frame_description);
	safe_release(p_source);
	if (FAILED(hr))
	{
		ofLogWarning("ofxKinect2::ColorStream") << "Can't open stream.";
		return false;
	}

	return Stream::open();
}

//----------------------------------------------------------
void ColorStream::close()
{
	Stream::close();
	safe_release(stream.p_color_frame_reader);
}

//----------------------------------------------------------
bool ColorStream::updateMode()
{
	ofLogWarning("ofxKinect2::ColorStream") << "Not supported yet.";
	return false;
}

//----------------------------------------------------------
int ColorStream::getExposureTime()
{
	TIMESPAN exposure_time;
	camera_settings.p_color_camera_settings->get_ExposureTime(&exposure_time);
	return (int)exposure_time;
}

//----------------------------------------------------------
int ColorStream::getFrameInterval()
{
	TIMESPAN frame_interval;
	camera_settings.p_color_camera_settings->get_FrameInterval(&frame_interval);
	return (int)frame_interval;
}

//----------------------------------------------------------
float ColorStream::getGain()
{
	float gain;
	camera_settings.p_color_camera_settings->get_Gain(&gain);
	return gain;
}

//----------------------------------------------------------
float ColorStream::getGamma()
{
	float gamma;
	camera_settings.p_color_camera_settings->get_Gamma(&gamma);
	return gamma;
}

//----------------------------------------------------------
#pragma mark - DepthStream
//----------------------------------------------------------

//----------------------------------------------------------
bool DepthStream::readFrame(IMultiSourceFrame* p_multi_frame)
{
	bool readed = false;
	if(!stream.p_depth_frame_reader)
	{
		ofLogWarning("ofxKinect2::DepthStream") << "Stream is not open.";
		return readed;
	}
	Stream::readFrame(p_multi_frame);

	IDepthFrame* p_frame = NULL;

	HRESULT hr;
	if (!p_multi_frame)
	{
		hr = stream.p_depth_frame_reader->AcquireLatestFrame(&p_frame);
	}
	else
	{
		IDepthFrameReference* p_frame_reference = NULL;
		hr = p_multi_frame->get_DepthFrameReference(&p_frame_reference);

		if (SUCCEEDED(hr))
		{
			hr = p_frame_reference->AcquireFrame(&p_frame);
		}

		safe_release(p_frame_reference);
	}

	if (SUCCEEDED(hr))
	{
		IFrameDescription* p_frame_description = NULL;

		hr = p_frame->get_RelativeTime((INT64*)&frame.timestamp);

		if (SUCCEEDED(hr))
		{
			hr = p_frame->get_FrameDescription(&p_frame_description);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_Width(&frame.width);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_Height(&frame.height);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_HorizontalFieldOfView(&frame.horizontal_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_VerticalFieldOfView(&frame.vertical_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_DiagonalFieldOfView(&frame.diagonal_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame->get_DepthMinReliableDistance((USHORT*)&near_value);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame->get_DepthMaxReliableDistance((USHORT*)&far_value);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame->get_DepthMinReliableDistance((USHORT*)&near_value);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame->AccessUnderlyingBuffer((UINT*)&frame.data_size, reinterpret_cast<UINT16**>(&frame.data));
		}

		if (SUCCEEDED(hr))
		{
			readed = true;
			setPixels(frame);
		}
		safe_release(p_frame_description);
	}

	safe_release(p_frame);

	return readed;
}

//----------------------------------------------------------
void DepthStream::setPixels(Frame frame)
{
	Stream::setPixels(frame);
	
	const unsigned short *pixels = (const unsigned short*)frame.data;
	int w = frame.width;
	int h = frame.height;
	
	pix.allocate(w, h, 1);
	pix.getBackBuffer().setFromPixels(pixels, w, h, OF_IMAGE_GRAYSCALE);
	pix.swap();
}

//----------------------------------------------------------
void DepthStream::update()
{
	if (!tex.isAllocated()
		|| tex.getWidth() != getWidth()
		|| tex.getHeight() != getHeight())
	{
#if OF_VERSION_MINOR <= 7
		static ofTextureData data;
		
		data.pixelType = GL_UNSIGNED_SHORT;
		data.glTypeInternal = GL_LUMINANCE16;
		data.width = getWidth();
		data.height = getHeight();
		
		tex.allocate(data);
#elif OF_VERSION_MINOR > 7
		tex.allocate(getWidth(), getHeight(), GL_RGBA, true, GL_LUMINANCE, GL_UNSIGNED_SHORT);
#endif
	}

	if (lock())
	{
		ofShortPixels _pix;
		depthRemapToRange(pix.getFrontBuffer(), _pix, near_value, far_value, is_invert);
		tex.loadData(_pix);
		Stream::update();
		unlock();
	}
}

//----------------------------------------------------------
ofShortPixels DepthStream::getPixelsRef(int _near, int _far, bool invert)
{
	ofShortPixels _pix;
	depthRemapToRange(getPixelsRef(), _pix, _near, _far, invert);
	return _pix;
}

//----------------------------------------------------------
bool DepthStream::open()
{
	if (!device->isOpen())
	{
		ofLogWarning("ofxKinect2::DepthStream") << "No ready Kinect2 found.";
		return false;
	}
	is_invert = true;
	near_value = 0;
	far_value = 10000;
	IDepthFrameSource* p_source = NULL;
	HRESULT hr;

	hr = device->get().kinect2->get_DepthFrameSource(&p_source);

	if (SUCCEEDED(hr))
	{
		hr = p_source->OpenReader(&stream.p_depth_frame_reader);
	}

	safe_release(p_source);
	if (FAILED(hr))
	{
		ofLogWarning("ofxKinect2::DepthStream") << "Can't open stream.";
		return false;
	}

	return Stream::open();
}

//----------------------------------------------------------
void DepthStream::close()
{
	Stream::close();
	safe_release(stream.p_depth_frame_reader);
}

//----------------------------------------------------------
bool DepthStream::updateMode()
{
	ofLogWarning("ofxKinect2::DepthStream") << "Not supported yet.";
	return false;
}

//----------------------------------------------------------
#pragma mark - IrStream
//----------------------------------------------------------

//----------------------------------------------------------
bool IrStream::readFrame(IMultiSourceFrame* p_multi_frame)
{
	bool readed = false;
	if(!stream.p_infrared_frame_reader)
	{
		ofLogWarning("ofxKinect2::IrStream") << "Stream is not open.";
		return readed;
	}
	Stream::readFrame();

	IInfraredFrame* p_frame = NULL;

	HRESULT hr;
	if (!p_multi_frame)
	{
		hr = stream.p_infrared_frame_reader->AcquireLatestFrame(&p_frame);
	}
	else
	{
		IInfraredFrameReference* p_frame_reference = NULL;
		hr = p_multi_frame->get_InfraredFrameReference(&p_frame_reference);

		if (SUCCEEDED(hr))
		{
			hr = p_frame_reference->AcquireFrame(&p_frame);
		}

		safe_release(p_frame_reference);
	}

	if (SUCCEEDED(hr))
	{
		IFrameDescription* p_frame_description = NULL;

		hr = p_frame->get_RelativeTime((INT64*)&frame.timestamp);

		if (SUCCEEDED(hr))
		{
			hr = p_frame->get_FrameDescription(&p_frame_description);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_Width(&frame.width);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_Height(&frame.height);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_HorizontalFieldOfView(&frame.horizontal_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_VerticalFieldOfView(&frame.vertical_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame_description->get_DiagonalFieldOfView(&frame.diagonal_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_frame->AccessUnderlyingBuffer((UINT*)&frame.data_size, reinterpret_cast<UINT16**>(&frame.data));
		}

		if (SUCCEEDED(hr))
		{
			readed = true;
			setPixels(frame);
		}
		safe_release(p_frame_description);
	}

	safe_release(p_frame);

	return readed;
}

//----------------------------------------------------------
void IrStream::setPixels(Frame frame)
{
	Stream::setPixels(frame);

	const unsigned short *pixels = (const unsigned short*)frame.data;
	int w = frame.width;
	int h = frame.height;
	
	pix.allocate(w, h, 1);
	pix.getBackBuffer().setFromPixels(pixels, w, h, OF_IMAGE_GRAYSCALE);
	pix.swap();
}

//----------------------------------------------------------
void IrStream::update()
{
	if(!tex.isAllocated()
		|| tex.getWidth() != getWidth()
		|| tex.getHeight() != getHeight())
	{
		tex.allocate(getWidth(), getHeight(), GL_LUMINANCE);
	}

	if (lock())
	{
		tex.loadData(pix.getFrontBuffer());
		Stream::update();
		unlock();
	}
}

//----------------------------------------------------------
bool IrStream::open()
{
	if (!device->isOpen())
	{
		ofLogWarning("ofxKinect2::IrStream") << "No ready Kinect2 found.";
		return false;
	}

	IInfraredFrameSource* p_source = NULL;
	HRESULT hr;

	hr = device->get().kinect2->get_InfraredFrameSource(&p_source);

	if (SUCCEEDED(hr))
	{
		hr = p_source->OpenReader(&stream.p_infrared_frame_reader);
	}

	safe_release(p_source);
	if (FAILED(hr))
	{
		ofLogWarning("ofxKinect2::IrStream") << "Can't open stream.";
		return false;
	}

	return Stream::open();
}

//----------------------------------------------------------
void IrStream::close()
{
	Stream::close();
	safe_release(stream.p_infrared_frame_reader);
}

//----------------------------------------------------------
bool IrStream::updateMode()
{
	ofLogWarning("ofxKinect2::IrStream") << "Not supported yet.";
	return false;
}

//----------------------------------------------------------
#pragma mark - Body
//----------------------------------------------------------

//----------------------------------------------------------
void Body::close()
{

}

//----------------------------------------------------------
void Body::update(IBody* body)
{
	joints.resize(JointType_Count);
	joint_points.resize(JointType_Count);

	HRESULT hr = body->get_HandLeftState(&left_hand_state);
	hr = body->get_HandRightState(&right_hand_state);

	body->get_LeanTrackingState(&lean_state);
	PointF pnt;
	body->get_Lean(&pnt);
	CameraSpacePoint cpnt;
	cpnt.X = pnt.X;
	cpnt.Y = pnt.Y;
	cpnt.Z = 0;
	body_lean = bodyPointToScreen(cpnt, ofGetWidth(), ofGetHeight());

	hr = body->GetJoints(JointType_Count, &joints.front());

	if(SUCCEEDED(hr))
	{
		is_update_scale = false;
	}
}

//----------------------------------------------------------
ofPoint Body::jointToScreen(const JointType jointType, int x, int y, int w, int h)
{
	CameraSpacePoint bodyPoint = joints[jointType].Position;
	return bodyPointToScreen(bodyPoint, x, y, w, h);
}

//----------------------------------------------------------
ofPoint Body::bodyPointToScreen(const CameraSpacePoint& bodyPoint, int x, int y, int w, int h)
{
	// Calculate the body's position on the screen
    DepthSpacePoint depthPoint = {0};
	device->getMapper()->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

	// TODO: width/ height
    float screenPointX = static_cast<float>(depthPoint.X * w) / 512 + x;
    float screenPointY = static_cast<float>(depthPoint.Y * h) / 424 + y;

    return ofPoint(screenPointX, screenPointY);
}

//----------------------------------------------------------
void Body::drawBody(int x, int y, int w, int h)
{
	if(is_tracked)
	{
		drawBone(JointType_Head, JointType_Neck, x, y, w , h);
		drawBone(JointType_Neck, JointType_SpineShoulder, x, y, w , h);
		drawBone(JointType_SpineShoulder, JointType_SpineMid, x, y, w , h);
		drawBone(JointType_SpineMid, JointType_SpineBase, x, y, w , h);
		drawBone(JointType_SpineShoulder, JointType_ShoulderLeft, x, y, w , h);
		drawBone(JointType_SpineShoulder, JointType_ShoulderRight, x, y, w , h);
		drawBone(JointType_SpineBase, JointType_HipLeft, x, y, w , h);
		drawBone(JointType_SpineBase, JointType_HipRight, x, y, w , h);

		drawBone(JointType_ShoulderLeft, JointType_ElbowLeft, x, y, w , h);
		drawBone(JointType_ElbowLeft, JointType_WristLeft, x, y, w , h);
		drawBone(JointType_WristLeft, JointType_HandLeft, x, y, w , h);
		drawBone(JointType_HandLeft, JointType_HandTipLeft, x, y, w , h);
		drawBone(JointType_WristLeft, JointType_ThumbLeft, x, y, w , h);

		drawBone(JointType_ShoulderRight, JointType_ElbowRight, x, y, w , h);
		drawBone(JointType_ElbowRight, JointType_WristRight, x, y, w , h);
		drawBone(JointType_WristRight, JointType_HandRight, x, y, w , h);
		drawBone(JointType_HandRight, JointType_HandTipRight, x, y, w , h);
		drawBone(JointType_WristRight, JointType_ThumbRight, x, y, w , h);

		drawBone(JointType_HipLeft, JointType_KneeLeft, x, y, w , h);
		drawBone(JointType_KneeLeft, JointType_AnkleLeft, x, y, w , h);
		drawBone(JointType_AnkleLeft, JointType_FootLeft, x, y, w , h);

		drawBone(JointType_HipRight, JointType_KneeRight, x, y, w , h);
		drawBone(JointType_KneeRight, JointType_AnkleRight, x, y, w , h);
		drawBone(JointType_AnkleRight, JointType_FootRight, x, y, w , h);

		ofPushStyle();
		for(int i = 0; i < JointType_Count; ++i)
		{
			if(joints[i].TrackingState == TrackingState_Tracked)
			{
				ofSetColor(50, 200, 50);
				ofEllipse(jointToScreen((JointType)i, x, y, w, h), 3, 3);
			}
		}
		ofSetColor(ofColor::red);
		ofDrawBitmapString(ofToString(id), jointToScreen(JointType_Head, x, y, w, h));
		ofPopStyle();
	}
}

//----------------------------------------------------------
void Body::drawBone(JointType joint0, JointType joint1, int x, int y, int w, int h)
{
	if(is_tracked)
	{
		ofPushStyle();
		TrackingState state0 = joints[joint0].TrackingState;
		TrackingState state1 = joints[joint1].TrackingState;

		if((state0 == TrackingState_NotTracked) || (state1 == TrackingState_NotTracked))
		{
			return;
		}

		if((state0 == TrackingState_Inferred) && (state1 == TrackingState_Inferred))
		{
			return;
		}

		if((state0 == TrackingState_Tracked) && (state1 == TrackingState_Tracked))
		{
			ofSetColor(ofColor::green);
		}
		else
		{
			ofSetColor(ofColor::gray);
		}
		ofLine(jointToScreen(joint0, x, y, w, h), jointToScreen(joint1, x, y, w, h));
		ofPopStyle();
	}
}

//----------------------------------------------------------
void Body::drawHands(int x, int y, int w, int h)
{
	drawHandLeft();
	drawHandRight();
}

//----------------------------------------------------------
void Body::drawHandLeft(int x, int y, int w, int h)
{
	if(is_tracked)
	{
		ofPushStyle();
		switch(left_hand_state)
		{
		case HandState_Closed:
			ofSetColor(ofColor::red);
			ofEllipse(jointToScreen(JointType_HandLeft, x, y, w, h), 30, 30);
			break;
		case HandState_Open:
			ofSetColor(ofColor::green);
			ofEllipse(jointToScreen(JointType_HandLeft, x, y, w, h), 30, 30);
			break;
		case HandState_Lasso:
			ofSetColor(ofColor::blue);
			ofEllipse(jointToScreen(JointType_HandLeft, x, y, w, h), 30, 30);
			break;
		}
		ofPopStyle();
	}
}

//----------------------------------------------------------
void Body::drawHandRight(int x, int y, int w, int h)
{
	if(is_tracked)
	{
		ofPushStyle();
		switch(right_hand_state)
		{
		case HandState_Closed:
			ofSetColor(ofColor::red);
			ofEllipse(jointToScreen(JointType_HandRight, x, y, w, h), 30, 30);
			break;
		case HandState_Open:
			ofSetColor(ofColor::green);
			ofEllipse(jointToScreen(JointType_HandRight, x, y, w, h), 30, 30);
			break;
		case HandState_Lasso:
			ofSetColor(ofColor::blue);
			ofEllipse(jointToScreen(JointType_HandRight, x, y, w, h), 30, 30);
			break;
		}
		ofPopStyle();
	}
}

//----------------------------------------------------------
void Body::drawLean(int x, int y, int w, int h)
{
	if(is_tracked)
	{
		if(lean_state == TrackingState_Tracked)
		{
			ofPushStyle();
			ofSetColor(ofColor::magenta);
			ofLine(jointToScreen(JointType_SpineBase, x, y, w, h), jointToScreen(JointType_SpineBase, x, y, w, h) + body_lean);
			ofPopStyle();
		}
	}
}

//----------------------------------------------------------
#pragma mark - BodyStream
//----------------------------------------------------------

//----------------------------------------------------------
bool BodyStream::readFrame(IMultiSourceFrame* p_multi_frame)
{
	bool readed = false;
	if(!stream.p_body_frame_reader)
	{
		ofLogWarning("ofxKinect2::BodyStream") << "Stream is not open.";
		return readed;
	}

	IBodyFrame* p_frame = NULL;

	HRESULT hr;
	if (!p_multi_frame)
	{
		hr = stream.p_body_frame_reader->AcquireLatestFrame(&p_frame);
	}
	else
	{
		IBodyFrameReference* p_frame_reference = NULL;
		hr = p_multi_frame->get_BodyFrameReference(&p_frame_reference);

		if (SUCCEEDED(hr))
		{
			hr = p_frame_reference->AcquireFrame(&p_frame);
		}

		safe_release(p_frame_reference);
	}

	if (SUCCEEDED(hr))
	{
		hr = p_frame->get_RelativeTime((INT64*)&frame.timestamp);

		IBody* ppBodies[BODY_COUNT] = {0};

		if (SUCCEEDED(hr))
		{
			hr = p_frame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		}

		if (SUCCEEDED(hr))
		{
			/*
			readed = true;
			righthand_pos_ =  ofPoint(0, 0);
			float nearest = 10000000;
			for(int i = 0; i < _countof(ppBodies); ++i)
			{
				if(ppBodies[i])
				{
					Joint joints[JointType_Count];
					HandState right_hand_state = HandState_Unknown;

					ppBodies[i]->get_HandRightState(&right_hand_state);

					HRESULT hr = ppBodies[i]->GetJoints(_countof(joints), joints);

					if(SUCCEEDED(hr))
					{
						if(joints[JointType_HandRight].TrackingState == TrackingState_Tracked)
						{
							float z_pos = joints[JointType_HandRight].Position.Z;
							if(z_pos < nearest)
							{
								nearest = z_pos;
								// Calculate the body's position on the screen
								DepthSpacePoint depthPoint = {0};
								ICoordinateMapper* mapper = NULL;
								hr = device->get().kinect2->get_CoordinateMapper(&mapper);
								if(SUCCEEDED(hr))
								{
									mapper->MapCameraPointToDepthSpace(joints[JointType_HandRight].Position, &depthPoint);

									// TODO: width/ height
									float screenPointX = static_cast<float>(depthPoint.X * 1920) / 512;
									float screenPointY = static_cast<float>(depthPoint.Y * 1080) / 424;

									righthand_pos_ =  ofPoint(screenPointX, screenPointY);
								}

								safe_release(mapper);
							}
						}
					}
				}
			}

			/**/
			for(int i = 0; i < _countof(ppBodies); ++i)
			{
				BOOLEAN tracked = false;
				if(ppBodies[i])
				{
					ppBodies[i]->get_IsTracked(&tracked);
					UINT64 id = -1;
					ppBodies[i]->get_TrackingId(&id);
					bool isExist = false;
					bodies[i].setTracked((bool)tracked);
					bodies[i].setId(id);
					if(tracked)
					{
						bodies[i].update(ppBodies[i]);
					}
				}
			}
			/**/
			for(int i = 0; i < _countof(ppBodies); ++i)
			{
				safe_release(ppBodies[i]);
			}
			readed = true;
			setPixels(frame);
		}

	}

	safe_release(p_frame);

	return readed;
}

//----------------------------------------------------------
void BodyStream::draw(int x, int y, int w, int h, size_t idx)
{
	for(int i = 0; i < bodies.size(); i++)
	{
		if(idx == BODY_COUNT || idx == i)
		{
			bodies[i].drawBody(x, y, w, h);
			bodies[i].drawHands(x, y, w, h);
			bodies[i].drawLean(x, y, w, h);
		}
	}
}

//----------------------------------------------------------
void BodyStream::drawBody(int x, int y, int w, int h, size_t idx)
{
	for(int i = 0; i < bodies.size(); i++)
	{
		if(idx == BODY_COUNT || idx == i)
		{
			bodies[i].drawBody(x, y, w, h);
		}
	}
}

//----------------------------------------------------------
void BodyStream::drawBone(JointType joint0, JointType joint1, int x, int y, int w, int h, size_t idx)
{
	for(int i = 0; i < bodies.size(); i++)
	{
		if(idx == BODY_COUNT || idx == i)
		{
			bodies[i].drawBone(joint0, joint1, x, y, w, h);
		}
	}
}

//----------------------------------------------------------
void BodyStream::drawHands(int x, int y, int w, int h, size_t idx)
{
	for(int i = 0; i < bodies.size(); i++)
	{
		if(idx == BODY_COUNT || idx == i)
		{
			bodies[i].drawHands(x, y, w, h);
		}
	}
}

//----------------------------------------------------------
void BodyStream::drawHandLeft(int x, int y, int w, int h, size_t idx)
{
	for(int i = 0; i < bodies.size(); i++)
	{
		if(idx == BODY_COUNT || idx == i)
		{
			bodies[i].drawHandLeft(x, y, w, h);
		}
	}
}

//----------------------------------------------------------
void BodyStream::drawHandRight(int x, int y, int w, int h, size_t idx)
{
	for(int i = 0; i < bodies.size(); i++)
	{
		if(idx == BODY_COUNT || idx == i)
		{
			bodies[i].drawHandRight(x, y, w, h);
		}
	}
}

//----------------------------------------------------------
void BodyStream::drawLean(int x, int y, int w, int h, size_t idx)
{
	for(int i = 0; i < bodies.size(); i++)
	{
		if(idx == BODY_COUNT || idx == i)
		{
			bodies[i].drawLean(x, y, w, h);
		}
	}
}

//----------------------------------------------------------
void BodyStream::setPixels(Frame frame)
{
	Stream::setPixels(frame);
}

//----------------------------------------------------------
void BodyStream::update()
{
	if (lock())
	{
		Stream::update();
		unlock();
	}
}

//----------------------------------------------------------
bool BodyStream::open()
{
	if (!device->isOpen())
	{
		ofLogWarning("ofxKinect2::BodyStream") << "No ready Kinect2 found.";
		return false;
	}

	IBodyFrameSource* p_source = NULL;
	HRESULT hr;

	hr = device->get().kinect2->get_BodyFrameSource(&p_source);

	if (SUCCEEDED(hr))
	{
		hr = p_source->OpenReader(&stream.p_body_frame_reader);
	}

	safe_release(p_source);
	if (FAILED(hr))
	{
		ofLogWarning("ofxKinect2::BodyStream") << "Can't open stream.";
		return false;
	}

	return Stream::open();
}

//----------------------------------------------------------
void BodyStream::close()
{
	Stream::close();
	safe_release(stream.p_body_frame_reader);
	for(int i = 0; i < bodies.size(); i++)
	{
		bodies[i].close();
	}

}

//----------------------------------------------------------
bool BodyStream::updateMode()
{
	ofLogWarning("ofxKinect2::BodyStream") << "Not supported yet.";
	return false;
}


/**/


/*
//----------------------------------------------------------
#pragma mark - ColorMappingStream
//----------------------------------------------------------

//----------------------------------------------------------
bool ColorMappingStream::readFrame(IMultiSourceFrame* p_multi_frame)
{
	bool readed = false;
	if (!stream.p_multi_source_frame_reader)
	{
		ofLogWarning("ofxKinect2::ColorMappingStream") << "Stream is not open.";
		return readed;
	}
	if (!device->isDepthColorSyncEnabled())
	{
		ofLogWarning("ofxKinect2::ColorMappingStream") << "You should enable color depth sync.";
		return readed;
	}

	Stream::readFrame(p_multi_frame);

	IMultiSourceFrame* p_frame = NULL;
	HRESULT hr = stream.p_multi_source_frame_reader->AcquireLatestFrame(&p_frame);

    if (SUCCEEDED(hr))
    {
        IDepthFrameReference* p_depth_frame_reference = NULL;

        hr = p_frame->get_DepthFrameReference(&p_depth_frame_reference);
        if (SUCCEEDED(hr))
        {
            hr = p_depth_frame_reference->AcquireFrame(&p_depth_frame);
        }

        safe_release(p_depth_frame_reference);
    }

    if (SUCCEEDED(hr))
    {
        IColorFrameReference* p_color_frame_reference = NULL;

        hr = p_frame->get_ColorFrameReference(&p_color_frame_reference);
        if (SUCCEEDED(hr))
        {
            hr = p_color_frame_reference->AcquireFrame(&p_color_frame);
        }

        safe_release(p_color_frame_reference);
    }

    if (SUCCEEDED(hr))
    {
        IBodyIndexFrameReference* p_body_index_frame_reference = NULL;

        hr = p_frame->get_BodyIndexFrameReference(&p_body_index_frame_reference);
        if (SUCCEEDED(hr))
        {
            hr = p_body_index_frame_reference->AcquireFrame(&p_body_index_frame);
        }

        safe_release(p_body_index_frame_reference);
    }


	if (SUCCEEDED(hr))
	{
		IFrameDescription* p_depth_frame_description = NULL;
		IFrameDescription* p_color_frame_description = NULL;
		IFrameDescription* p_body_index_frame_description = NULL;
        ColorImageFormat image_format = ColorImageFormat_None;

		hr = p_depth_frame->get_RelativeTime((INT64*)&frame.timestamp);

		if (SUCCEEDED(hr))
		{
			hr = p_depth_frame->get_FrameDescription(&p_depth_frame_description);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_depth_frame_description->get_Width(&frame.width);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_depth_frame_description->get_Height(&frame.height);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_depth_frame_description->get_HorizontalFieldOfView(&frame.horizontal_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_depth_frame_description->get_VerticalFieldOfView(&frame.vertical_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_depth_frame_description->get_DiagonalFieldOfView(&frame.diagonal_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_depth_frame->AccessUnderlyingBuffer((UINT*)&frame.data_size, reinterpret_cast<UINT16**>(&frame.data));
		}


		if (SUCCEEDED(hr))
		{
			hr = p_color_frame->get_FrameDescription(&p_color_frame_description);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_color_frame_description->get_Width(&color_frame.width);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_color_frame_description->get_Height(&color_frame.height);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_color_frame_description->get_HorizontalFieldOfView(&color_frame.horizontal_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_color_frame_description->get_VerticalFieldOfView(&color_frame.vertical_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_color_frame_description->get_DiagonalFieldOfView(&color_frame.diagonal_field_of_view);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_color_frame->get_RawColorImageFormat(&image_format);
		}

		if (SUCCEEDED(hr))
		{
			if (c_buffer == NULL)
			{
				c_buffer = new unsigned char[color_frame.width * color_frame.height];
			}
			if (image_format == ColorImageFormat_Bgra)
			{
				hr = p_color_frame->AccessRawUnderlyingBuffer((UINT*)&color_frame.data_size, reinterpret_cast<BYTE**>(&color_frame.data));
			}
			else if(c_buffer)
			{
				color_frame.data = c_buffer;
				color_frame.data_size = color_frame.width * color_frame.height * sizeof(BYTE);
				hr = p_color_frame->CopyConvertedFrameDataToArray((UINT)frame.data_size, reinterpret_cast<BYTE*>(color_frame.data),  ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
				return false;
			}
		}

        // get body index frame data

        if (SUCCEEDED(hr))
        {
            hr = p_body_index_frame->get_FrameDescription(&p_body_index_frame_description);
        }

        if (SUCCEEDED(hr))
        {
			hr = p_body_index_frame_description->get_Width(&body_index_frame.width);
        }

        if (SUCCEEDED(hr))
        {
            hr = p_body_index_frame_description->get_Height(&body_index_frame.height);
        }

        if (SUCCEEDED(hr))
        {
			hr = p_body_index_frame->AccessUnderlyingBuffer((UINT*)&body_index_frame.data_size, reinterpret_cast<BYTE**>(&body_index_frame.data));            
        }

		safe_release(p_depth_frame_description);
		safe_release(p_color_frame_description);
		safe_release(p_body_index_frame_description);
	}

	safe_release(p_frame);
	safe_release(p_depth_frame);
	safe_release(p_color_frame);
	safe_release(p_body_index_frame);

	return true;
}

//----------------------------------------------------------
void ColorMappingStream::setPixels(Frame frame)
{
	Stream::setPixels(frame);
	
	const unsigned short *depth_pixels = (const unsigned short*)frame.data;
	int depth_w = frame.width;
	int depth_h = frame.height;
	int depth_num_pixels = depth_w * depth_h;
	
	const unsigned char *color_pixels = (const unsigned char*)color_frame.data;
	m = color_frame.mode;
	int color_w = m.resolution_x;
	int color_h = m.resolution_y;
	int color_num_pixels = color_w * color_h;
	
	pix.allocate(depth_w, depth_h, 3);

	const unsigned char*body_index_pixels = (const unsigned char*)body_index_frame.data;
	m = body_index_frame.mode;
	int body_index_w = m.resolution_x;
	int body_index_h = m.resolution_y;
	int body_index_num_pixels = body_index_w * body_index_h;

	HRESULT hr;
	ICoordinateMapper* mapper = device->getMapper();
	device->getMapper()->MapDepthFrameToColorSpace(depth_w * depth_h, (UINT16*)depth_pixels, depth_w * depth_h, color_map);

	unsigned char *dst = pix.getBackBuffer().getPixels();
	for (int i = 0; i < depth_num_pixels; i++)
	{
		ColorSpacePoint cp = color_map[i];
		int cx = (int)(floor(cp.X + 0.5));
		int cy = (int)(floor(cp.Y + 0.5));
		if ((cx >= 0) && (cx < color_w) && (cy >= 0) && (cy < color_h))
		{
			int color_index = cy * color_w + cx;
			dst[i + 0] = color_pixels[color_index + 0];
			dst[i + 0] = color_pixels[color_index + 1];
			dst[i + 0] = color_pixels[color_index + 2];
		}
		else
		{
			dst[i + 0] = 0;
			dst[i + 1] = 0;
			dst[i + 2] = 0;
		}
	}
	pix.swap();
}

//----------------------------------------------------------
void ColorMappingStream::update()
{
	if(!tex.isAllocated()
		|| tex.getWidth() != getWidth()
		|| tex.getHeight() != getHeight())
	{
		tex.allocate(getWidth(), getHeight(), GL_RGB);
	}

	if (lock())
	{
		tex.loadData(pix.getFrontBuffer());
		Stream::update();
		unlock();
	}
}

//----------------------------------------------------------
bool ColorMappingStream::open()
{
	if (!device->isOpen())
	{
		ofLogWarning("ofxKinect2::ColorMappingStream") << "No ready Kinect2 found.";
		return false;
	}
	HRESULT hr;

	hr = device->get().kinect2->OpenMultiSourceFrameReader(
                FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_BodyIndex,
				&stream.p_multi_source_frame_reader);

	if (FAILED(hr))
	{
		ofLogWarning("ofxKinect2::ColorMappingStream") << "Can't open stream.";
		return false;
	}

	return Stream::open();
}

//----------------------------------------------------------
void ColorMappingStream::close()
{
	Stream::close();
	safe_release(stream.p_multi_source_frame_reader);
}

//----------------------------------------------------------
bool ColorMappingStream::updateMode()
{
	ofLogWarning("ofxKinect2::ColorMappingStream") << "Not supported yet.";
	return false;
}
/**/

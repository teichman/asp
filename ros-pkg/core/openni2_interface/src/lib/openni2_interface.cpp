#include <openni2_interface/openni2_interface.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <signal.h>

using namespace std;
using namespace openni;

#define SAMPLE_READ_WAIT_TIMEOUT 2000  // ms

bool g_int = false;

void sigint(int none) {
  cout << "OpenNI2Interface caught user signal.  Shutting down..." << endl;
  g_int = true;
}

OpenNI2Interface::OpenNI2Interface(Resolution color_res, Resolution depth_res) :
  color_res_(color_res),
  depth_res_(depth_res),
  sync_(0.018),
  stopping_(false),
  frame_id_(0)
{
  signal(SIGINT, sigint);
}

OpenNI2Interface::~OpenNI2Interface()
{
#if JARVIS_DEBUG
  std::cout << __PRETTY_FUNCTION__ << std::endl;
#endif

  cout << "Destroying OpenNI2Interface." << endl;
  depth_stream_.stop();
  depth_stream_.destroy();
  color_stream_.stop();
  color_stream_.destroy();
  device_.close();
  OpenNI::shutdown();

#if JARVIS_DEBUG
  std::cout << __PRETTY_FUNCTION__ << std::endl;
#endif
}

void OpenNI2Interface::run()
{
  ROS_ASSERT(handler_);
  int rv = connect();
  ROS_ASSERT(rv == 0);

  VideoStream* streams[] = { &color_stream_, &depth_stream_ };
  
  while(!stopping_ && !g_int) {
    int idx = -1;
    OpenNI::waitForAnyStream(streams, 2, &idx, SAMPLE_READ_WAIT_TIMEOUT);
    if(idx == 0)
      processColor();
    else if(idx == 1)
      processDepth();
    else
      cout << "Did not get stream data..." << endl;
  }
}

void OpenNI2Interface::processColor()
{
  openni::VideoFrameRef color_frame;
  Status rc = color_stream_.readFrame(&color_frame);
  ROS_ASSERT(rc == STATUS_OK);
  sync_.addT0(color_frame, color_frame.getTimestamp() * 1e-6);
  processSynchronized();
}

void OpenNI2Interface::processDepth()
{
  openni::VideoFrameRef depth_frame;
  Status rc = depth_stream_.readFrame(&depth_frame);
  ROS_ASSERT(rc == STATUS_OK);
  sync_.addT1(depth_frame, depth_frame.getTimestamp() * 1e-6);
  processSynchronized();
}

void OpenNI2Interface::processSynchronized()
{
  if(sync_.updated_) {
    timespec clk;
    clock_gettime(CLOCK_REALTIME, &clk);
    double timestamp = clk.tv_sec + clk.tv_nsec * 1e-9;
    handler_->rgbdCallback(sync_.current0_, sync_.current1_, frame_id_, timestamp);
    ++frame_id_;
    sync_.updated_ = false;
  }
}

int OpenNI2Interface::connect()
{
  Status rc = OpenNI::initialize();
  if(rc != STATUS_OK) {
    printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
    return 1;
  }

  rc = device_.open(ANY_DEVICE);
  if (rc != STATUS_OK)
  {
    printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
    return 2;
  }

  rc = device_.setDepthColorSyncEnabled(true);
  if(rc != STATUS_OK) {
    cout << "setDepthColorSyncEnabled failed.  " << OpenNI::getExtendedError() << endl;
    return 3;
  }
  

  if (device_.getSensorInfo(SENSOR_DEPTH) != NULL)
  {
    rc = depth_stream_.create(device_, SENSOR_DEPTH);
    depth_stream_.setMirroringEnabled(false);
    if (rc != STATUS_OK)
    {
      printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
      return 3;
    }

    const Array<VideoMode>& dmodes = device_.getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes();
    for(int i = 0; i < dmodes.getSize(); ++i) {
      if(depth_res_ == VGA &&
         dmodes[i].getResolutionX() == 640 && dmodes[i].getResolutionY() == 480 &&
         dmodes[i].getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM && dmodes[i].getFps() == 30)
      {
        rc = depth_stream_.setVideoMode(dmodes[i]);
        ROS_ASSERT(rc == STATUS_OK);
      }
      else if(depth_res_ == QVGA &&
         dmodes[i].getResolutionX() == 320 && dmodes[i].getResolutionY() == 240 &&
         dmodes[i].getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM && dmodes[i].getFps() == 30)
      {
        rc = depth_stream_.setVideoMode(dmodes[i]);
        ROS_ASSERT(rc == STATUS_OK);
      }
    }

  }

  rc = color_stream_.create(device_, openni::SENSOR_COLOR);
  if (rc == openni::STATUS_OK)
  {
    color_stream_.setMirroringEnabled(false);
    const Array<VideoMode>& cmodes = device_.getSensorInfo(SENSOR_COLOR)->getSupportedVideoModes();
    for(int i = 0; i < cmodes.getSize(); ++i) {
      if(color_res_ == VGA &&
         cmodes[i].getResolutionX() == 640 && cmodes[i].getResolutionY() == 480 &&
         cmodes[i].getPixelFormat() == openni::PIXEL_FORMAT_RGB888 && cmodes[i].getFps() == 30)
      {
        rc = color_stream_.setVideoMode(cmodes[i]);
        ROS_ASSERT(rc == STATUS_OK);
      }
      else if(color_res_ == QVGA &&
              cmodes[i].getResolutionX() == 320 && cmodes[i].getResolutionY() == 240 &&
              cmodes[i].getPixelFormat() == openni::PIXEL_FORMAT_RGB888 && cmodes[i].getFps() == 30)
      {
        rc = color_stream_.setVideoMode(cmodes[i]);
        ROS_ASSERT(rc == STATUS_OK);
      }
    }

    rc = color_stream_.start();
    if (rc != openni::STATUS_OK)
    {
      printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
      color_stream_.destroy();
    }
  }
  else
  {
    printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
  }

  rc = depth_stream_.start();
  if (rc != STATUS_OK)
  {
    printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
    return 4;
  }
  
  if (!depth_stream_.isValid() || !color_stream_.isValid())
  {
    printf("SimpleViewer: No valid streams. Exiting\n");
    openni::OpenNI::shutdown();
    return 2;
  }
 
  rc = device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
  if(rc != STATUS_OK) {
    cout << "setImageRegistrationMode failed.  " << OpenNI::getExtendedError() << endl;
    return 4;
  }
  
  VideoMode depth_video_mode = depth_stream_.getVideoMode();
  VideoMode color_video_mode = color_stream_.getVideoMode();
  
  int depth_width = depth_video_mode.getResolutionX();
  int depth_height = depth_video_mode.getResolutionY();
  int color_width = color_video_mode.getResolutionX();
  int color_height = color_video_mode.getResolutionY();

  // ROS_ASSERT(depth_width == color_width);
  // ROS_ASSERT(depth_height == color_height);

  ROS_DEBUG_STREAM("Connected to OpenNI RGBD sensor with "
                   << depth_width << " x " << depth_height << " depth resolution and "
                   << color_width << " x " << color_height << " color resolution");

  ROS_DEBUG_STREAM("Depth pixel format: " << depth_video_mode.getPixelFormat());
  ROS_DEBUG_STREAM("Color pixel format: " << color_video_mode.getPixelFormat());
  ROS_DEBUG_STREAM("Device registration mode: " << device_.getImageRegistrationMode());

  ROS_DEBUG_STREAM("URI: " << device_.getDeviceInfo().getUri());
  ROS_DEBUG_STREAM("Vendor: " << device_.getDeviceInfo().getVendor());
  ROS_DEBUG_STREAM("Name: " << device_.getDeviceInfo().getName());
  ROS_DEBUG_STREAM("Vendor ID: " << device_.getDeviceInfo().getUsbVendorId());
  ROS_DEBUG_STREAM("USB Product ID: " << device_.getDeviceInfo().getUsbProductId());
  
  
// typedef enum
// {
// 	// Depth
// 	PIXEL_FORMAT_DEPTH_1_MM = 100,
// 	PIXEL_FORMAT_DEPTH_100_UM = 101,
// 	PIXEL_FORMAT_SHIFT_9_2 = 102,
// 	PIXEL_FORMAT_SHIFT_9_3 = 103,

// 	// Color
// 	PIXEL_FORMAT_RGB888 = 200,
// 	PIXEL_FORMAT_YUV422 = 201,
// 	PIXEL_FORMAT_GRAY8 = 202,
// 	PIXEL_FORMAT_GRAY16 = 203,
// 	PIXEL_FORMAT_JPEG = 204,
// 	PIXEL_FORMAT_YUYV = 205,
// } PixelFormat;

  return 0;
}


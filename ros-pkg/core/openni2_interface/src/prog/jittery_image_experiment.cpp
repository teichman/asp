#include <iostream>
#include <OpenNI.h>
#include <signal.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <openni2_interface/openni_helpers.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace openni;

bool g_int = false;

void sigint(int none) {
  cout << "caught user signal.  Shutting down..." << endl;
  g_int = true;
}


int main(int argc, char** argv)
{
  signal(SIGINT, sigint);
    
  openni::Device device;
  openni::VideoStream color_stream;
  
  Status rc = OpenNI::initialize();
  if(rc != STATUS_OK) {
    printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
    return 1;
  }
  
  rc = device.open(ANY_DEVICE);
  if (rc != STATUS_OK)
  {
    printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
    return 2;
  }

  rc = color_stream.create(device, openni::SENSOR_COLOR);
  if (rc == openni::STATUS_OK)
  {
    
    const Array<VideoMode>& cmodes = device.getSensorInfo(SENSOR_COLOR)->getSupportedVideoModes();
    for(int i = 0; i < cmodes.getSize(); ++i) {
      if(cmodes[i].getResolutionX() == 320 && cmodes[i].getResolutionY() == 240 &&
         cmodes[i].getPixelFormat() == openni::PIXEL_FORMAT_RGB888 && cmodes[i].getFps() == 30)
      {
        rc = color_stream.setVideoMode(cmodes[i]);
        ROS_ASSERT(rc == STATUS_OK);
      }
    }

    rc = color_stream.start();
    if (rc != openni::STATUS_OK)
    {
      printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
      color_stream.destroy();
    }
  }
  else
  {
    printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
  }

  if(!color_stream.isValid())
  {
    printf("SimpleViewer: No valid streams. Exiting\n");
    openni::OpenNI::shutdown();
    return 2;
  }
 
  VideoMode color_video_mode = color_stream.getVideoMode();
  int color_width = color_video_mode.getResolutionX();
  int color_height = color_video_mode.getResolutionY();
  ROS_DEBUG_STREAM("Connected to OpenNI RGBD sensor with "
                   << color_width << " x " << color_height << " color resolution");
  ROS_DEBUG_STREAM("Color pixel format: " << color_video_mode.getPixelFormat());
  ROS_DEBUG_STREAM("URI: " << device.getDeviceInfo().getUri());
  ROS_DEBUG_STREAM("Vendor: " << device.getDeviceInfo().getVendor());
  ROS_DEBUG_STREAM("Name: " << device.getDeviceInfo().getName());
  ROS_DEBUG_STREAM("Vendor ID: " << device.getDeviceInfo().getUsbVendorId());
  ROS_DEBUG_STREAM("USB Product ID: " << device.getDeviceInfo().getUsbProductId());


  VideoStream* streams[] = { &color_stream };

  int id = 0;
  while(!g_int) {
    int idx = -1;
    OpenNI::waitForAnyStream(streams, 1, &idx, 2000);
    if(idx == 0) {
      openni::VideoFrameRef color_frame;
      Status rc = color_stream.readFrame(&color_frame);
      ROS_ASSERT(rc == STATUS_OK);
      cv::Mat3b img = oniToCV(color_frame);
      ostringstream oss;
      oss << setw(5) << setfill('0') << id << ".jpg";
      cv::imwrite(oss.str(), img);
      cout << "Wrote image to " << oss.str() << endl;
      ++id;
    }
    else
      cout << "Did not get stream data..." << endl;
  }
  
  return 0;
}

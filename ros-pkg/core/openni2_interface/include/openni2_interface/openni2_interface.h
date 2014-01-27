#ifndef OPENNI2_INTERFACE_H
#define OPENNI2_INTERFACE_H

#include <iostream>
#include <OpenNI.h>
#include <openni2_interface/synchronizer.h>
#include <agent/lockable.h>

class OpenNI2Handler
{
public:
  virtual ~OpenNI2Handler()
  {
#if JARVIS_DEBUG
    std::cout << __PRETTY_FUNCTION__ << std::endl;
#endif
  }

  // color and depth contain sensor timestamps.
  // timestamp contains the wall time, in seconds.
  virtual void rgbdCallback(openni::VideoFrameRef color,
                            openni::VideoFrameRef depth,
                            size_t frame_id, double timestamp) = 0;
};

class OpenNI2Interface
{
public:
  enum Resolution { VGA = 0, QVGA = 1 };
  
  OpenNI2Interface(Resolution color_res, Resolution depth_res);
  //! Take care to call the destructor on shut down.
  //! Starting OpenNI the next time can be annoying if you
  //! don't shut it down properly.
  ~OpenNI2Interface();
  void setHandler(OpenNI2Handler* handler) { handler_ = handler; }
  void run();
  void stop() { stopping_ = true; }
  OpenNI2Interface::Resolution colorRes() const { return color_res_; }
  OpenNI2Interface::Resolution depthRes() const { return depth_res_; }
  
private:
  Resolution color_res_;
  Resolution depth_res_;
  OpenNI2Handler* handler_;
  openni::Device device_;
  openni::VideoStream color_stream_;
  openni::VideoStream depth_stream_;
  //! color, depth.
  Synchronizer<openni::VideoFrameRef, openni::VideoFrameRef> sync_;
  bool stopping_;
  size_t frame_id_;
  
  int connect();
  void processColor();
  void processDepth();
  void processSynchronized();
};

#endif // OPENNI2_INTERFACE_H

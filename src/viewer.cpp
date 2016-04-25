
/**
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 * 
 *
 * Modified on 01/31/2016 by nicole cranon (nicole.cranon@ucdenver.edu)
 */

#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include "../include/viewer.h"
#include <iostream>

typedef boost::chrono::high_resolution_clock HRClock;

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
  do \
{ \
  static unsigned count = 0;\
  static double last = pcl::getTime ();\
  double now = pcl::getTime (); \
  ++count; \
  if (now - last >= 1.0) \
{ \
  std::cout << "Average framerate ("<< _WHAT_ << "): " << double (count)/double (now - last) << " Hz" <<  std::endl; \
  count = 0; \
  last = now; \
} \
}while (false)
#else
#define FPS_CALC (_WHAT_) \
  do \
{ \
}while (false)
#endif
int capturedFrameArray[500] = { };

void
printHelp (int, char **argv)
{
  using pcl::console::print_error;
  using pcl::console::print_info;

  print_error ("Syntax is: %s [((<device_id> | <path-to-oni-file>) [-depthmode <mode>] [-imagemode <mode>] [-xyz] | -l [<device_id>]| -h | --help)]\n", argv [0]);
  print_info ("%s -h | --help : shows this help\n", argv [0]);
  print_info ("%s -xyz : use only XYZ values and ignore RGB components (this flag is required for use with ASUS Xtion Pro) \n", argv [0]);
  print_info ("%s -l : list all available devices\n", argv [0]);
  print_info ("%s -l <device-id> :list all available modes for specified device\n", argv [0]);
  print_info ("\t\t<device_id> may be \"#1\", \"#2\", ... for the first, second etc device in the list\n");
#ifndef _WIN32
  print_info ("\t\t                   bus@address for the device connected to a specific usb-bus / address combination\n");
  print_info ("\t\t                   <serial-number>\n");
#endif
  print_info ("\n\nexamples:\n");
  print_info ("%s \"#1\"\n", argv [0]);
  print_info ("\t\t uses the first device.\n");
  print_info ("%s  \"./temp/test.oni\"\n", argv [0]);
  print_info ("\t\t uses the oni-player device to play back oni file given by path.\n");
  print_info ("%s -l\n", argv [0]);
  print_info ("\t\t list all available devices.\n");
  print_info ("%s -l \"#2\"\n", argv [0]);
  print_info ("\t\t list all available modes for the second device.\n");
#ifndef _WIN32
  print_info ("%s A00361800903049A\n", argv [0]);
  print_info ("\t\t uses the device with the serial number \'A00361800903049A\'.\n");
  print_info ("%s 1@16\n", argv [0]);
  print_info ("\t\t uses the device on address 16 at USB bus 1.\n");
#endif
}

template <typename PointType>
void
OpenNI2Viewer<PointType>::cloud_callback (const CloudConstPtr& cloud)
{
  FPS_CALC ("cloud callback");
  boost::mutex::scoped_lock lock (cloud_mutex_);
  cloud_ = cloud;

  /**
   * @ author - nicole cranon
   * only move frame forward by one if it is playing 
   */
  if (is_playing()){
    this->currentFrame += 1;  // iterate current frame
  } else if (is_rewinding()) {
    // handle frame count appropriately for a rewind
  } else if (is_fastforwarding()) {
    // handle frame count appropriately for a fastforward
  }
  std::cout << "\nCurrent frame" << this->currentFrame;  // uncomment to print current frame
}

template<typename PointType>
void
OpenNI2Viewer<PointType>::image_callback (const boost::shared_ptr<pcl::io::openni2::Image>& image)
{
  FPS_CALC ("image callback");
  boost::mutex::scoped_lock lock (image_mutex_);
  image_ = image;

  if (image->getEncoding () != pcl::io::openni2::Image::RGB)
  {
    if (rgb_data_size_ < image->getWidth () * image->getHeight ())
    {
      if (rgb_data_)
        delete [] rgb_data_;
      rgb_data_size_ = image->getWidth () * image->getHeight ();
      rgb_data_ = new unsigned char [rgb_data_size_ * 3];
    }
    image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
  }
}

template<typename PointType>
void
OpenNI2Viewer<PointType>::keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.getKeyCode ()) {

    /**
     * capture keypress
     * @author - nicole cranon
     */
    this->keypressed = event.getKeyCode ();
    cout << "the key \'" << event.getKeyCode () << "\' (" << event.getKeyCode () << ") was";
  }
  else
    cout << "the special key \'" << event.getKeySym () << "\' was";
  if (event.keyDown ())
    cout << " pressed" << endl;
  else
    cout << " released" << endl;
}

template<typename PointType>
void
OpenNI2Viewer<PointType>::mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*)
{
  if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton)
  {
    cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
  }
}

/**
 * @brief starts the main loop
 */
template<typename PointType>
void
OpenNI2Viewer<PointType>::run ()
{
  cloud_viewer_->registerMouseCallback (&OpenNI2Viewer::mouse_callback, *this);
  cloud_viewer_->registerKeyboardCallback (&OpenNI2Viewer::keyboard_callback, *this);
  cloud_viewer_->setCameraFieldOfView (1.02259994f);
  boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&OpenNI2Viewer::cloud_callback, this, _1);
  boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);
  this->keypressed = PAUSE;

  boost::signals2::connection image_connection;
  if (grabber_.providesCallback<void (const boost::shared_ptr<pcl::io::openni2::Image>&)>())
  {
    image_viewer_.reset (new pcl::visualization::ImageViewer ("PCL OpenNI image"));
    image_viewer_->registerMouseCallback (&OpenNI2Viewer::mouse_callback, *this);
    image_viewer_->registerKeyboardCallback (&OpenNI2Viewer::keyboard_callback, *this);
    boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb = boost::bind (&OpenNI2Viewer::image_callback, this, _1);
    image_connection = grabber_.registerCallback (image_cb);
  }

  bool image_init = false, cloud_init = false;

  grabber_.start ();

  // while (currentFrame <= totalFrames && !cloud_viewer_->wasStopped () && (image_viewer_ && !image_viewer_->wasStopped ()))
  // {
  for (unsigned i = 0; i < totalFrames;)
  {
    boost::shared_ptr<pcl::io::openni2::Image> image;
    CloudConstPtr cloud;

    cloud_viewer_->spinOnce ();

    // See if we can get a cloud
    if (cloud_mutex_.try_lock ())
    {
      cloud_.swap (cloud);
      cloud_mutex_.unlock ();
    }

    if (cloud && !is_paused())
    {
      FPS_CALC("drawing cloud");

      if (!cloud_init)
      {
        cloud_viewer_->setPosition (0, 0);
        cloud_viewer_->setSize (cloud->width, cloud->height);
        cloud_init = !cloud_init;
      }

      if (!cloud_viewer_->updatePointCloud (cloud, "OpenNICloud"))
      {
        cloud_viewer_->addPointCloud (cloud, "OpenNICloud");
        cloud_viewer_->resetCameraViewpoint ("OpenNICloud");
        cloud_viewer_->setCameraPosition (
          0,0,0,		// Position
          0,0,1,		// Viewpoint
          0,-1,0);	// Up
      }
    }

    // See if we can get an image
    if (image_mutex_.try_lock ())
    {
      image_.swap (image);
      image_mutex_.unlock ();
    }


    if (image && !is_paused())
    {
      if (!image_init && cloud && cloud->width != 0)
      {
        image_viewer_->setPosition (cloud->width, 0);
        image_viewer_->setSize (cloud->width, cloud->height);
        image_init = !image_init;
      }

      if (image->getEncoding () == pcl::io::openni2::Image::RGB)
        image_viewer_->addRGBImage ( (const unsigned char*)image->getData (), image->getWidth (), image->getHeight ());
      else
        image_viewer_->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
      image_viewer_->spinOnce ();

    }

    /**
     * check for frame capture
     * @author - david andrews
     */
    int capturedFrameCount = 0;
    if (this->keypressed == 'z') {
      //using global array for now.
        capturedFrameArray[capturedFrameCount] = i;
        std::cout<< "Captured Frame Number is: " << capturedFrameArray[capturedFrameCount] << '\n';
        capturedFrameCount++;
    }
      
    /**
     * capture the current playback key pressed
     * @author - nicole cranon
     */
     switch (this->keypressed) {
      case PAUSE:
      case UPPER_PAUSE:
        this->pause();
        break;
      default:
        this->play();
        break;
     }

    /**
     * iterate through frames and check for user exit/stop
     * @author - nicole cranon
     */
    if (!is_paused()) {
      i = currentFrame;
    }

    if (cloud_viewer_->wasStopped () || (!image_viewer_ || image_viewer_->wasStopped ()))
    {
      break;
    }
  }

  std::cout << "\nended loop\n";
  grabber_.stop ();

  cloud_connection.disconnect ();
  image_connection.disconnect ();
  if (rgb_data_)
    delete[] rgb_data_;
}

/**
 * @description - is used for external classes and functions to check if the player is paused
 * @author - nicole cranon
 */
template<typename PointType>
bool
OpenNI2Viewer<PointType>::is_paused () {
  return this->paused;
}

/**
 * @description - is used for external classes and functions to check if the player is playing
 * @author - nicole cranon
 */
template<typename PointType>
bool
OpenNI2Viewer<PointType>::is_playing () {
  return this->playing;
}

/**
 * @description - is used for external classes and functions to check if the player is stopped
 * @author - nicole cranon
 */
template<typename PointType>
bool
OpenNI2Viewer<PointType>::is_stopped () {
  return this->stopped;
}

/**
 * @description - is used for external classes and functions to check if the player is rewinding
 * @author - nicole cranon
 */
template<typename PointType>
bool
OpenNI2Viewer<PointType>::is_rewinding () {
  return this->rewinding;
}

/**
 * @description - is used for external classes and functions to check if the player is fastforwarding
 * @author - nicole cranon
 */
template<typename PointType>
bool
OpenNI2Viewer<PointType>::is_fastforwarding () {
  return this->fastforwarding;
}

/**
 * @function pause
 * @description - uses --- to pause playback
 * @author - nicole cranon
 */
template<typename PointType>
void 
OpenNI2Viewer<PointType>::pause () {
  this->paused = true;
  this->playing = false;
  this->rewinding = false;
  this->fastforwarding = false;
  this->stopped = false;
}

/**
 * @function play
 * @description - breaks the state of a 1) pause, 2) rewind, 3) fastforward, 4) stop?
 * @author - nicole cranon
 */
template<typename PointType>
void 
OpenNI2Viewer<PointType>::play () {
  this->paused = false;
  this->playing = true;
  this->rewinding = false;
  this->fastforwarding = false;
  this->stopped = false;
}


/**
 * @function rewind
 * @description - rewinds the player until 1) the beginning, 2) play is pressed,
 * 3) the stop is pressed, 4) fastforward is pressed
 * @author - nicole cranon,  
 */
template<typename PointType>
void 
OpenNI2Viewer<PointType>::rewind () {
  this->paused = false;
  this->playing = false;
  this->rewinding = true;
  this->fastforwarding = false;
  this->stopped = false;
}

/**
 * @function fastforward
 * @description - fastforwards the player until 1) the end, 2) play is pressed,
 * 3) the stop is pressed, 4) rewind is pressed
 * @author - nicole cranon, 
 */
template<typename PointType>
void 
OpenNI2Viewer<PointType>::fastforward () {
  this->paused = false;
  this->playing = false;
  this->rewinding = false;
  this->fastforwarding = true;
  this->stopped = false;
}

// Create the PCLVisualizer object
boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
boost::shared_ptr<pcl::visualization::ImageViewer> img;

// here for viewer testing purposes should be removed upon integration with project
/* ---[ */
// int
// main (int argc, char** argv)
// {
//   std::string device_id ("");
//   pcl::io::OpenNI2Grabber::Mode depth_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
//   pcl::io::OpenNI2Grabber::Mode image_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
//   bool xyz = false;
//   /**
//    * frame tracking 
//    * @author - nicole cranon
//    */
//   unsigned totalFrames = 0;

//   if (argc >= 2)
//   {
//     device_id = argv[1];
//     if (device_id == "--help" || device_id == "-h")
//     {
//       printHelp (argc, argv);
//       return 0;
//     }
//     else if (device_id == "-l")
//     {
//       if (argc >= 3)
//       {
//         pcl::io::OpenNI2Grabber grabber (argv[2]);
//         /**
//          * frame tracking 
//          * @author - nicole cranon
//          */
//         totalFrames = grabber.getDevice()->getDepthFrameCount();
//         boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = grabber.getDevice ();
//         cout << *device;		// Prints out all sensor data, including supported video modes
//       }
//       else
//       {
//         boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance ();
//         if (deviceManager->getNumOfConnectedDevices () > 0)
//         {
//           for (unsigned deviceIdx = 0; deviceIdx < deviceManager->getNumOfConnectedDevices (); ++deviceIdx)
//           {
//             boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = deviceManager->getDeviceByIndex (deviceIdx);
//             cout << "Device " << device->getStringID () << "connected." << endl;
//           }

//         }
//         else
//           cout << "No devices connected." << endl;

//         cout <<"Virtual Devices available: ONI player" << endl;
//       }
//       return 0;
//     }
//   }
//   else
//   {
//     boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance ();
//     if (deviceManager->getNumOfConnectedDevices () > 0)
//     {
//       boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = deviceManager->getAnyDevice ();
//       cout << "Device ID not set, using default device: " << device->getStringID () << endl;
//     }
//   }

//   unsigned mode;
//   if (pcl::console::parse (argc, argv, "-depthmode", mode) != -1)
//     depth_mode = pcl::io::OpenNI2Grabber::Mode (mode);

//   if (pcl::console::parse (argc, argv, "-imagemode", mode) != -1)
//     image_mode = pcl::io::OpenNI2Grabber::Mode (mode);

//   if (pcl::console::find_argument (argc, argv, "-xyz") != -1)
//     xyz = true;

//   pcl::io::OpenNI2Grabber grabber (device_id, depth_mode, image_mode);
//   /**
//    * frame tracking 
//    * @author - nicole cranon
//    */
//   totalFrames = grabber.getDevice()->getDepthFrameCount();


//   if (xyz || !grabber.providesCallback<pcl::io::OpenNI2Grabber::sig_cb_openni_point_cloud_rgb> ())
//   {
//     OpenNI2Viewer<pcl::PointXYZ> openni_viewer (grabber, totalFrames);
//     openni_viewer.run ();
//   }
//   else
//   {
//     OpenNI2Viewer<pcl::PointXYZRGBA> openni_viewer (grabber, totalFrames);
//     openni_viewer.run ();
//   }

//   return (0);
// }
/* ]---*/ 
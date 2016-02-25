#include <cctype>

#ifndef _OPENNI2VIEWER
#define _OPENNI2VIEWER

template <typename PointType>
class OpenNI2Viewer
{
public:
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;


  OpenNI2Viewer (pcl::io::OpenNI2Grabber& grabber, unsigned totalFrames)
    : cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL OpenNI2 cloud"))
    , image_viewer_ ()
    , grabber_ (grabber)
    , rgb_data_ (0), rgb_data_size_ (0)
    , currentFrame (0)
    , totalFrames (totalFrames)
  {
    //this->pause();  // start the viewer in paused state
  }

  void
  cloud_callback (const CloudConstPtr& cloud);

  void
  image_callback (const boost::shared_ptr<pcl::io::openni2::Image>& image);

  void
  keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*);

  void
  mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*);

  /**
   * @description - is used for external classes and functions to check if the player is paused
   * @author - nicole cranon
   */
  bool is_paused ();

  /**
   * @description - is used for external classes and functions to check if the player is playing
   * @author - nicole cranon
   */
  bool is_playing ();

  /**
   * @description - is used for external classes and functions to check if the player is stopped
   * @author - nicole cranon
   */
  bool is_stopped ();
  
  /**
   * @description - is used for external classes and functions to check if the player is rewinding
   * @author - nicole cranon
   */
  bool is_rewinding ();

  /**
   * @description - is used for external classes and functions to check if the player is fastforwarding
   * @author - nicole cranon
   */
  bool is_fastforwarding ();


  /**
  * @brief starts the main loop
  */
  void
  run ();

  boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
  boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;

  pcl::io::OpenNI2Grabber& grabber_;
  boost::mutex cloud_mutex_;
  boost::mutex image_mutex_;

  CloudConstPtr cloud_;
  boost::shared_ptr<pcl::io::openni2::Image> image_;
  unsigned char* rgb_data_;
  unsigned rgb_data_size_;

private:
  /**
   * @description - used for frame tracking during viewing
   * @author - nicole cranon
   */
  unsigned currentFrame;
  unsigned totalFrames;
  /**
   * @description - used for keypress tracking during viewing
   * @author - nicole cranon
   */
  char keypressed;

  /**
   * Playback control keys
   * @author - nicole cranon
   */ 
  static const char PLAY = ' ';
  static const char PAUSE = 'p';
  static const char UPPER_PAUSE = 'P';
  static const char STOP = 's';
  static const char UPPER_STOP = 'S';
  static const char REWIND = 'r';
  static const char UPPER_REWIND = 'R';
  static const char FORWARD = 'f';
  static const char UPPER_FORWARD = 'F';
  // static const char BEG_BLURRY = 'b';  // beginning blurry frame(s)
  // static const char UPPER_BEG_BLURRY = 'B';  // beginning blurry frame(s)
  // static const char END_BLURRY = '';  // end of blurry frame(s)
  // static const char UPPER_END_BLURRY = 'E';  // end of blurry frame(s)

  /**
   * @description - state holding vars
   * @author - nicole cranon
   */
  bool paused;
  bool stopped;
  bool playing;
  bool rewinding;
  bool fastforwarding;

  /**
   * @function pause
   * @description - uses --- to pause playback
   * @author - nicole cranon
   */
  void
  pause ();

  /**
   * @function play
   * @description - breaks the state of a 1) pause, 2) rewind, 3) fastforward, 4) stop?
   * @author - nicole cranon
   */
  void 
  play ();

  /**
   * @function rewind
   * @description - rewinds the player until 1) the beginning, 2) play is pressed,
   * 3) the stop is pressed, 4) fastforward is pressed
   * @author - nicole cranon, 
   */
  void 
  rewind ();

  /**
   * @function fastforward
   * @description - fastforwards the player until 1) the end, 2) play is pressed,
   * 3) the stop is pressed, 4) rewind is pressed
   * @author - nicole cranon, 
   */
  void 
  fastforward ();

   /**
   * @function stop
   * @description - stops the player until 1) play is pressed
   * @author - nicole cranon, 
   */
  void 
  stop ();

  /**
   * @function launchOniViewer
   * @description - initializes and launches the viewer
   * @author - nicole cranon
   */
  void
  launchOniViewer (std::string& oniFilename);
};

#include "../src/viewer.tem"
#endif


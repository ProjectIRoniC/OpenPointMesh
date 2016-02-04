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

  /**
   * Playback control keys
   * @author - nicole cranon
   */ 
  static const char PLAY = ' ';
  static const char PAUSE = 'p';
  static const char STOP = 's';
  static const char REWIND = 'r';
  static const char FORWARD = 'f';
  static const char BEG_BLURRY = 'b';  // beginning blurry frame(s)
  static const char END_BLURRY = 'e';  // end of blurry frame(s)

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
   * @function pause
   * @description - uses busy waiting to pause playback
   * @author - nicole cranon
   */
   void 
   pause ();
};
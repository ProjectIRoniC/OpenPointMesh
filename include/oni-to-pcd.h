///////////////////////////////////////////////////////////////////////////////////////
/// @file	oni-to-pcd.h
/// @brief	Reads an oni file  and outputs point clouds (pcd files)
/// @bug	No known bugs
///////////////////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <OpenNI.h>
#include <pcl/point_cloud.h>
#include <boost/lockfree/spsc_queue.hpp>
#include <set>

#ifndef _ONI_TO_PCD
#define _ONI_TO_PCD

namespace vba
{
	/// @brief	A type defined structure for a RGBA color value
	typedef union
		{
			struct
			{
				unsigned char Blue;		//!< Intensity of Blue Light, 0 Low - 255 High
				unsigned char Green;	//!< Intensity of Green Light, 0 Low - 255 High
				unsigned char Red;		//!< Intensity of Red Light, 0 Low - 255 High
				unsigned char Alpha;	//!< Opacity of the Combined Colors, 0 Low - 255 High
			};
			float float_value;			//!< RGBA color value in float format
			uint32_t long_value;		//!< RGBA color value in long format
		} RGBValue;
		
	static const int DEFAULT_FRAME_SKIP = 10;	//!< Lowest number of frames to skip between reading a frame
	static const int MAX_FRAME_SKIP = 100;		//!< Highest number of frames to skip between reading a frame
	
	/// @brief A class to handle the reading of an oni file and output the data to csv and point cloud files
	class OniToPcd
	{
		public:
			OniToPcd();

			OniToPcd( std::string outputDirectoryPath, unsigned frameSkipModulus, boost::lockfree::spsc_queue<std::string>* _outputBuffer );

			OniToPcd( std::string outputDirectoryPath, unsigned frameSkipModulus, boost::lockfree::spsc_queue<std::string>* _outputBuffer, const std::set<int>& omitFrames );
			
			virtual ~OniToPcd();

			void setOutputBuffer( boost::lockfree::spsc_queue<std::string>* _outputBuffer );
			
			void setFrameSkip( const int framesToSkip );
			
			void setDebugMode( bool debugBool);

			/*Public facing function that gets the number of frames to skip between reading a frame.
			*
			*/
			int getFrameSkip() { return frameSkip; }

			/*Public facing function that gets the debug mode bool.
			*
			*/
			int getDebugMode() { return debugMode; }
			
			int outputOniData( const std::string inputFile );
			
			static bool minimumSamplingRate (int sampleRate);
			
			void setOmmittedFrames( const std::set<int>& of );
			
		private:
			void init();
			
			void sendOutput( const std::string& output, const bool error );
			
			void outputFrameToCsv( std::ofstream& outFileStream, const openni::VideoFrameRef frameReference );

			template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
			outputFrameToPcd( const std::string outputFrameDirectory, const openni::Device* device, const openni::VideoFrameRef depthFrameReference, const openni::VideoFrameRef colorFrameReference, const float fieldOfView_X, const float fieldOfView_Y );
			
			void fillBufferRGB( const unsigned newWidth, const unsigned newHeight, unsigned char* rgb_buffer, const unsigned oldWidth, const unsigned oldHeight, const openni::VideoFrameRef colorFrameReference, unsigned rgb_line_step = 0 );
			
			void fillBufferDepth( const unsigned newWidth, const unsigned newHeight, unsigned short* depth_buffer, const unsigned oldWidth, const unsigned oldHeight, const openni::VideoFrameRef depthFrameReference, unsigned line_step = 0 );
			
			// Private Data Members
			unsigned frameSkip;										//!< Frames to skip between reading a frame
			std::string outputDirPath;								//!< Directory to put all output files
			boost::lockfree::spsc_queue<std::string>* outputBuffer;	//!< Function Pointer to message output buffer
			bool redirectOutputFlag;								//!< True to redirct display messages to the output buffer, false otherwise
			std::set<int> ommittedFrames;
			bool debugMode;
	};
};

#endif

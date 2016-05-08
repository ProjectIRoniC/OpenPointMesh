/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.h
Description:	Reads an oni file recorded using the Openni2 library and outputs point clouds (pcd files)
*/

#include <fstream>
#include <pcl/io/openni2/openni.h>
#include <pcl/point_cloud.h>
#include <boost/lockfree/spsc_queue.hpp>
#include <set>

#ifndef _ONI_TO_PCD
#define _ONI_TO_PCD

namespace vba
{
	typedef union
		{
			struct
			{
				unsigned char Blue;
				unsigned char Green;
				unsigned char Red;
				unsigned char Alpha;
			};
			float float_value;
			uint32_t long_value;
		} RGBValue;
		
    static const int DEFAULT_FRAME_SKIP = 10;
    static const int MAX_FRAME_SKIP = 100;
	
	class OniToPcd
	{
		public:
			/*Default constructor for this class, which initializes the members to default values.
			 *
			 */
			OniToPcd();

			/*Overloaded constructor
			 *
			 * @param: outputDirectoryPath - path to where output files will be written
			 * @param: frameSkipModulus - how many frames to skip between reading a frame
			 */
			OniToPcd( std::string outputDirectoryPath, unsigned frameSkipModulus, boost::lockfree::spsc_queue<std::string>* _outputBuffer );

			/*Overloaded constructor
			 *
			 * @param: outputDirectoryPath - path to where output files will be written
			 * @param: frameSkipModulus - how many frames to skip between reading a frame
			 * @param: omitFrames - frames to be excluded from sampling
			 */
			OniToPcd( std::string outputDirectoryPath, unsigned frameSkipModulus, boost::lockfree::spsc_queue<std::string>* _outputBuffer, const std::set<int>& omitFrames );

			/*Default destructor that deallocates the dynamically allocated members of the class.
			 *
			 */
			virtual ~OniToPcd();
		
			/*Public facing function that accepts a function pointer. This function pointer will be passed all
			 * the output from this function. If no function pointer is ever given, it will default to sending
			 * all the output to standard out and standard error.
			 *
			 * @param: Function pointer following the signature   void functionName( std::string )
			 */	
			void setOutputBuffer( boost::lockfree::spsc_queue<std::string>* _outputBuffer );

			/*Public facing function that sets the number of frames to skip between reading a frame, a
			 * minimum value of 10 is required
			 *
			 * @param: number of frames to skip, if less than 10, 10 will be set by default
			 */
                        void setFrameSkip( const int framesToSkip );
                        
                        /*Public facing function that sets debug mode to true for testing purposes.
                         *
                         */
                        void setDebugMode( bool debugBool);
                        
                        /*Public facing function that gets the number of frames to skip between reading a frame.
                         *
                         */
                        int getFrameSkip() { return frameSkip; }

                        /*Public facing function that gets the debug mode bool.
                         *
                         */
                        int getDebugMode() { return debugMode; }
			
			/*Public facing function that reads an oni file and exports data as excel docs and point clouds
			 *
			 * @param: The string containing the absolute path pointing towards the directory containing
			 * 			the target input oni file.
			 * @return: 0 if the operation was successful, -1 if not.
			 */
			int outputOniData( const std::string inputFile );

			/*Public facing function that checks if a value meets the minimum frame sampling rate
			 *
			 * @param: The positive integer frame sampling rate to check
			 *
			 * @return: returns true if the value provided meets or is greater than the minimum value
			 *			returns false if the value provided is less than the minimum value
			 */
            static bool minimumSamplingRate (int sampleRate);

			void setOmittedFrames( const std::set<int>& of );
			
		private:
			/*Performs class setup actions
			 * 
			 */
			void init();
			
			/*Sends output messages to the output buffer
			 *
			 * @param: output - contains information to be displayed to the user
			 * @param: error - true if the message is an error message, false otherwise
			 */
			void sendOutput( const std::string& output, const bool error );
			
			/*Exports a depth frame to comma separated value (.csv) file
			 *
			 * @param: outFileStream - file data stream for csv output
			 * @param: frameReference - a depth frame to output
			 */
			void outputFrameToCsv( std::ofstream& outFileStream, const openni::VideoFrameRef frameReference );

			/*Template Function Exports a depth and RGB frame to point cloud data (.pcd) files
			 *
			 * @param: outputFrameDirectory - directory to output the .pcd files
			 * @param: device - pointer to the device with the loaded oni file
			 * @param: depthFrameReference - depth frame to output
			 * @param: colorFrameReference - color frame to output
			 */
			template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
			outputFrameToPcd( const std::string outputFrameDirectory, const openni::Device* device, const openni::VideoFrameRef depthFrameReference, const openni::VideoFrameRef colorFrameReference, const float fieldOfView_X, const float fieldOfView_Y );
			
			/*Fills a RGB24 image buffer with downsampling
			 *
			 * @param: newWidth - width of the new buffer image
			 * @param: newHeight - height of the new buffer image
			 * @param: rgb_buffer - buffer to fill with the new image
			 * @param: oldWidth - width of the input image
			 * @param: oldHeight - height of the input image
			 * @param: colorFrameReference - input image to copy
			 * @param: rgb_line_step - the number of lines to step when downsampling
			 */
			void fillBufferRGB( const unsigned newWidth, const unsigned newHeight, unsigned char* rgb_buffer, const unsigned oldWidth, const unsigned oldHeight, const openni::VideoFrameRef colorFrameReference, unsigned rgb_line_step = 0 );
			
			/*Fills a depth image buffer with downsampling
			 *
			 * @param: newWidth - width of the new buffer image
			 * @param: newHeight - height of the new buffer image
			 * @param: depth_buffer - buffer to fill with the new image
			 * @param: oldWidth - width of the input image
			 * @param: oldHeight - height of the input image
			 * @param: depthFrameReference - input image to copy
			 * @param: line_step - the number of lines to step when downsampling
			 */
			void fillBufferDepth( const unsigned newWidth, const unsigned newHeight, unsigned short* depth_buffer, const unsigned oldWidth, const unsigned oldHeight, const openni::VideoFrameRef depthFrameReference, unsigned line_step = 0 );
			
			// Private Data Members
                        unsigned frameSkip;
			std::string outputDirPath;
			boost::lockfree::spsc_queue<std::string>* outputBuffer;
			bool redirectOutputFlag;
			std::set<int> omittedFrames;
                        bool debugMode;
	};
};

#endif

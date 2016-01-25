/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.h
Description:	Reads an oni file recorded using the Openni2 library and outputs point clouds (pcd files)
*/

#include <pcl/io/openni2_grabber.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <boost/lockfree/spsc_queue.hpp>

#ifndef _ONI_TO_PCD
#define _ONI_TO_PCD


typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;

namespace vba {
	namespace oni2pcd {
		const int DEFAULT_FRAME_SKIP = 25;

		

		/*
		read single oni and write pcds
		*/
		void readOni (const char* oniFile, 
			char* writeToDirPath = NULL, 
			const int framesToSkip = DEFAULT_FRAME_SKIP);

		/*
		return directory path to write pcd files to
		*/
		char* getWriteDirPath (char* writeToDir = NULL);

		/*
		set number of frames to skip in writing pcd files
		*/
		void setFrameSkip (const int framesToSkip);

		/*
		assigns totalFrames, framestoRead, and currentFrame based on framesInOni
		*/
		void setFrameInfo (const int framesInOni);

		/*
		sets timeout based on processing
		*/
		void setTimeout (const int to);

		/*
		callback for our readOniFile, actually writes the pointcloud
		*/
        void writeCloudCb (const CloudConstPtr& cloud);

        /*
         * driver that controls the oni to pcd conversion
         */
        int driver(int argc, char* argv[]);

        /*
         * Pre:  _outputBuffer points to a buffer that will be maintained by external user.
         *       _outputBuffer is not nullptr
         * Post: global outputBuffer points to _outputBuffer
         */
        void setOutputBuffer(boost::lockfree::spsc_queue<std::string>* _outputBuffer);

        /*
         * Pre:  outPut contains information to be displayed to the user
         * Post: if outputBuffer has not been initialized then output will be sent to std::cout or std::cerr
         */
        void sendOutput(const std::string& output, bool error);
	};
};

#endif

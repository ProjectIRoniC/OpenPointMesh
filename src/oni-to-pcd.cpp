/*	
Team:			VolcanoBot A
Project:		TBD
File:			oni-to-pcd.cpp
Description:	Reads an oni file recorded using the Openni2 or Openni library and outputs point clouds (pcd files)
*/

#include "../include/oni-to-pcd.h"
#include "../include/errorMsgHandler.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <iomanip>
#include <string>
#include <cstring>


namespace vba {
	namespace oni2pcd {
		int totalFrames = 0;
		int framesToRead = 0,
			currentFrame = 0,
			currentReadFrame = 0,
			frameSkip = 0,
			timeout = 0;
            char* pcdWriteDirPath = NULL;
            boost::lockfree::spsc_queue<std::string>* outputBuffer = NULL;
            bool redirectOutputFlag = false;

	}
}
//#include "../include/mainwindow.h"
//#include <QApplication>

/***************************************************************
here for execution of code in standalone, will be removed once 
integrated in project
***************************************************************/
/*
int main (int argc, char* argv[]) {
QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
*/

int vba::oni2pcd::driver(int argc, char* argv[]){
	/*
	expect 2 additional arguments, ie. <executable> <oniFile> <pcdWriteDir>
	*/

	const int REQ_ARGS = 3;
	if (argc < REQ_ARGS) {
        sendOutput("\nNot enough arguments provided.\n", true);
 		return EXIT_FAILURE;
	}

	// // int threadTimout = 

	// vba::oni2pcd::readOni (argv[1], argv[2]);
	boost::thread oni2pcd(vba::oni2pcd::readOni, argv[1], argv[2], 0);

	if (oni2pcd.try_join_for (boost::chrono::minutes(2))) {
        sendOutput("\nDone\n", false);
	} else {
        sendOutput("\nBADDDDD!!!!!\n", true);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

/***************************************************************
vba::oni2pcd namespace functions
***************************************************************/
/*
read the given oni file, write the output pcd files to the given directory,
if no directory is given will write to pcdTemp in the executable directory
*/

void vba::oni2pcd::readOni (const char* oniFile, 
		char* writeToDirPath,
		const int framesToSkip) {
	if (vba::isNull(oniFile)) {
		vba::handleEmptyFilePtr (oniFile);
	}

	/*
	create a pointer to an Openni2 grabber instance, register a callback for the grabber, bind the callback writeCloudCb from our vba::oni2pcd namespace
	*/
	pcl::io::OpenNI2Grabber *grabber = new pcl::io::OpenNI2Grabber (oniFile);
	boost::function<void (const CloudConstPtr&) > f = boost::bind (&vba::oni2pcd::writeCloudCb, _1);
	boost::signals2::connection c = grabber->registerCallback (f);

	/*
	setup frame info
	*/
	vba::oni2pcd::setFrameSkip (framesToSkip);
	vba::oni2pcd::setFrameInfo(grabber->getDevice()->getDepthFrameCount());
    sendOutput("\nFrames to read " + vba::oni2pcd::framesToRead + '\n', false);

	/*
	set write pcd directory path
	*/
	vba::oni2pcd::pcdWriteDirPath = vba::oni2pcd::getWriteDirPath(writeToDirPath);

	grabber->start();
	while (currentReadFrame < framesToRead) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds( 1 ) );
	}
	grabber->stop();
}

/*
write pcd files to pcdTemp under given directory path if it exists,
write pcd files to pcdTemp under executable directoy path alternatively
*/
char* vba::oni2pcd::getWriteDirPath (char* writeToDir) {
	if (writeToDir != NULL) {
		boost::filesystem::path writeToDirPath (writeToDir);

		if (boost::filesystem::exists(writeToDirPath) 
			&& boost::filesystem::is_directory (writeToDirPath)) {
			boost::filesystem::path pcdWriteToDirPath(writeToDirPath);
			pcdWriteToDirPath += "/pcdTemp";
			boost::filesystem::create_directory(pcdWriteToDirPath);

			writeToDir = new char [pcdWriteToDirPath.string().length()];
			return strcpy (writeToDir, pcdWriteToDirPath.string().c_str());
		}
	}
	writeToDir = new char [boost::filesystem::current_path().string().length()];
	boost::filesystem::path pcdWriteToDirPath (boost::filesystem::current_path());
	pcdWriteToDirPath += "/pcdTemp";

	boost::filesystem::create_directory(pcdWriteToDirPath);

	return strcpy (writeToDir, boost::filesystem::current_path().string().c_str());
}

/*
sets the number of frames to skip in the oni file, if a number less than 
default, which is minimum, then the frame skip is set to default
*/
void vba::oni2pcd::setFrameSkip (const int framesToSkip) {
	if (framesToSkip < vba::oni2pcd::DEFAULT_FRAME_SKIP) {
		vba::oni2pcd::frameSkip = vba::oni2pcd::DEFAULT_FRAME_SKIP;
		return;
	}

	vba::oni2pcd::frameSkip = framesToSkip;
}

/*
total frames := the number of frames in the oni file,
frames to read := the number of frames to actually create pcd files from
	derived from total frames / frame skip,
current frame := current frame in total frames,
current read frame := current frame to write pcd file for
*/
void vba::oni2pcd::setFrameInfo (const int framesInOni) {
	vba::oni2pcd::totalFrames = framesInOni;
	vba::oni2pcd::framesToRead = vba::oni2pcd::totalFrames / vba::oni2pcd::frameSkip;
	vba::oni2pcd::currentFrame = 0;
	vba::oni2pcd::currentReadFrame = 0;
}

void vba::oni2pcd::setTimeout (const int to) {
	vba::oni2pcd::timeout = to;
}

/*
callback for our instance of the Openni2 grabber, writes the 
point clouds corresponding to currentReadFrame
*/
void vba::oni2pcd::writeCloudCb (const CloudConstPtr& cloud) {
	std::string buffer;
	std::stringstream ss;
	pcl::PCDWriter w;

	if (vba::oni2pcd::currentFrame % vba::oni2pcd::frameSkip == 0) {

		ss << vba::oni2pcd::pcdWriteDirPath << "/frame_" << std::setfill ('0') << std::setw(6) << vba::oni2pcd::currentReadFrame << ".pcd";

        sendOutput("Wrote a cloud to " + ss.str() + '\n', false);
		w.writeBinaryCompressed (ss.str(), *cloud);
		++vba::oni2pcd::currentReadFrame;
	}

	++vba::oni2pcd::currentFrame;
}

void vba::oni2pcd::setOutputBuffer(boost::lockfree::spsc_queue<std::string>* _outputBuffer){
    outputBuffer = _outputBuffer;
    redirectOutputFlag = true;
}

void vba::oni2pcd::sendOutput(const std::string& output, bool error){
    if( redirectOutputFlag == true )
    {
        if(!outputBuffer->push(output)) {
            std::cout << "[" << output << "] did not make it too buffer\n";
        }
    }

    else
    {
        if( error == true )
        {
            std::cerr << output;
        }
        else
        {
            std::cout << output;
        }
    }
}



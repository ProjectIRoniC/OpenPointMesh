///////////////////////////////////////////////////////////////////////////////////////
/// @file	oni-to-pcd.cpp
/// @brief	Implementation of oni-to-pcd.h
/// @bug	No known bugs
/// @todo	Add missing \@details descriptions
/// @todo	Add simple \@example
///////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include "oni-to-pcd.h"
#include "errorMsgHandler.h"
#include "filesystemHelper.h"

/// @brief Default constructor for this class, which initializes the members to default values
vba::OniToPcd::OniToPcd()
	: frameSkip( 0 )
	, outputDirPath( "output" )
	, outputBuffer( NULL )
	, redirectOutputFlag( false )
	, omittedFrames()
	, debugMode( false )
{
	init();
}

///////////////////////////////////////////////////////////////////////////////////////
/// @brief	Overloaded constructor
///
/// @param[in]	outputDirectoryPath		Path to where output files will be written
/// @param[in]	frameSkipModulus		How many frames to skip between reading a frame
/// @param[out]	_outputBuffer			Function Pointer to message output buffer
///////////////////////////////////////////////////////////////////////////////////////
vba::OniToPcd::OniToPcd( std::string outputDirectoryPath, unsigned frameSkipModulus, boost::lockfree::spsc_queue<std::string>* _outputBuffer )
	: frameSkip( frameSkipModulus )
	, outputDirPath( outputDirectoryPath )
	, outputBuffer( _outputBuffer )
	, redirectOutputFlag( true )
	, debugMode( false )
{
	init();
}

/*Overloaded constructor
*
* @param: outputDirectoryPath - path to where output files will be written
* @param: frameSkipModulus - how many frames to skip between reading a frame
* @param: omitFrames - frames to be excluded from sampling
*/
vba::OniToPcd::OniToPcd( std::string outputDirectoryPath, unsigned frameSkipModulus, boost::lockfree::spsc_queue<std::string>* _outputBuffer, const std::set<int>& _omittedFrames )
	: frameSkip( frameSkipModulus )
	, outputDirPath( outputDirectoryPath )
	, outputBuffer( _outputBuffer )
	, redirectOutputFlag( true )
	, omittedFrames(_omittedFrames)
{
	setDebugMode( false );
	init();
}

/// @brief Default destructor that deallocates the dynamically allocated members of the class
vba::OniToPcd::~OniToPcd()
{
	openni::OpenNI::shutdown();
}

///////////////////////////////////////////////////////////////////////////////////////
/// @brief		Function Pointer to message output buffer
///
///	@details	Accepts a function pointer that will be passed all the output from this 
///				function. If no function pointer is ever given, it will default to sending
///				all the output to standard out and standard error. The funciton pointer
///				accepts the following signature `void functionName( std::string )`
///
/// @param[in]	_outputBuffer	Function Pointer to message output buffer
///////////////////////////////////////////////////////////////////////////////////////
void vba::OniToPcd::setOutputBuffer( boost::lockfree::spsc_queue<std::string>* _outputBuffer )
{
	outputBuffer = _outputBuffer;
	redirectOutputFlag = true;
        if( debugMode == true )
            sendOutput( "setOutputBuffer worked" , false );
}

///////////////////////////////////////////////////////////////////////////////////////
/// @brief		Sets the number of frames to skip between reading a frame
///
///	@details	A minimum value of 10 is required, if a value less than 10 is supplied
///				the value 10 will be set.
///
/// @param[in]	framesToSkip	Number of frames to skip
///////////////////////////////////////////////////////////////////////////////////////
void vba::OniToPcd::setFrameSkip( const int framesToSkip )
{
	if( framesToSkip < vba::DEFAULT_FRAME_SKIP )
	{
		this->frameSkip = vba::DEFAULT_FRAME_SKIP;
		return;
	}

	this->frameSkip = framesToSkip;
}

void vba::OniToPcd::setOmittedFrames( const std::set<int>& of ) 
{
	this->omittedFrames = of;  
}

/*Public facing function that sets debug mode to true for testing purposes.
*
*/
void vba::OniToPcd::setDebugMode( bool debugBool )
{
        this->debugMode = debugBool;
}

/*Public facing function that checks if a value meets the minimum frame sampling rate
*
* @param: The positive integer frame sampling rate to check
*
* @return: returns true if the value provided meets or is greater than the minimum value
*			returns false if the value provided is less than the minimum value
*/
bool vba::OniToPcd::minimumSamplingRate (int sampleRate) 
{
	return sampleRate >= vba::DEFAULT_FRAME_SKIP;
}

///////////////////////////////////////////////////////////////////////////////////////
/// @brief	Reads an oni file and exports data as excel docs and point clouds
///
/// @param[in]	inputFile	Absolute path to directory with the input oni file
/// @return		0 if the operation was successful, -1 otherwise
///////////////////////////////////////////////////////////////////////////////////////
int vba::OniToPcd::outputOniData( const std::string inputFile )
{
	// for (std::set<int>::iterator it=this->omittedFrames.begin(); it != this->omittedFrames.end(); ++it)
	// {
	// 	std::cout << '\n' << *it << '\n';
	// }
    
	// Open the .oni file
	openni::Device device;
	openni::Status rc = device.open( inputFile.c_str() );
	if( rc != openni::STATUS_OK )
	{
		if( debugMode == true )
		{
			sendOutput( "Couldn't open device" , false );
		}
		else
		{
			sendOutput( "Couldn't open device\n" + std::string(openni::OpenNI::getExtendedError()) + '\n', true );
		}
		return -1;
	}
	else
	{
		if( debugMode == true)
			sendOutput( "Device opened" , false );
	}
	
	// Device Check
	if( !device.isValid() )
	{
		sendOutput( "The device is not valid.\n", true );
		return -1;
	}
	
	// Verify the device is a file
	if( !device.isFile() )
	{
		sendOutput( "The device is not a file.\n", true );
		return -1;
	}
	
	// Create the Depth Video Stream
	openni::VideoStream depthStream;
	rc = depthStream.create( device, openni::SENSOR_DEPTH );
	if( rc != openni::STATUS_OK )
	{
		sendOutput( "Couldn't create depth stream\n" + std::string(openni::OpenNI::getExtendedError()) + '\n', true );
		return -1;
	}
	
	// Create the Color Video Stream
	openni::VideoStream colorStream;
	rc = colorStream.create( device, openni::SENSOR_COLOR );
	if( rc != openni::STATUS_OK )
	{
		sendOutput( "Couldn't create depth stream\n" + std::string(openni::OpenNI::getExtendedError()) + '\n', true );
		return -1;
	}

	// Set playback controls
	openni::PlaybackControl* pbc = device.getPlaybackControl();
	pbc->setSpeed( -1 );
	pbc->setRepeatEnabled( false );

	// Delete point cloud output directory if it exists, we don't want old frame data
	std::string pointCloudOutputDirectory = vba::filesystemhelper::getOutputFileName( outputDirPath, inputFile, "" );
	if( !vba::filesystemhelper::deleteDirectory(pointCloudOutputDirectory) )
		return -1;

	// Create point cloud output directory
	if( !vba::filesystemhelper::createDirectory(pointCloudOutputDirectory) )
		return -1;

	// Start reading the frame data
	const unsigned totalFrames = pbc->getNumberOfFrames( depthStream );
	std::stringstream framesToProcessSS;
	framesToProcessSS << "Frames to read " << totalFrames / frameSkip << "...\n";
	sendOutput( framesToProcessSS.str(), false );
	
	// Start the Depth Video Stream
	rc = depthStream.start();
	if( rc != openni::STATUS_OK )
	{
		sendOutput( "Couldn't start the depth stream\n" + std::string(openni::OpenNI::getExtendedError()) + '\n', true );
		return -1;
	}
	
	// Start the Color Video Stream
	rc = colorStream.start();
	if( rc != openni::STATUS_OK )
	{
		sendOutput( "Couldn't start the color stream\n" + std::string(openni::OpenNI::getExtendedError()) + '\n', true );
		return -1;
	}
	
	// Open csv output file for writing
	std::ofstream out;
	std::string csvOutputFile = vba::filesystemhelper::getOutputFileName( outputDirPath, inputFile, ".csv" );
	out.open( csvOutputFile.c_str() );
	
	// Read all frames
	openni::VideoFrameRef depthStreamFrame, colorStreamFrame;
	while( true )
	{
		// Read a depth frame
		rc = depthStream.readFrame( &depthStreamFrame );
		if( rc != openni::STATUS_OK )
		{
			sendOutput( "Depth frame read failed!\n" + std::string(openni::OpenNI::getExtendedError()) + '\n', true );
			continue;
		}
		
		// Read a color frame
		rc = colorStream.readFrame( &colorStreamFrame );
		if( rc != openni::STATUS_OK )
		{
			sendOutput( "Color frame read failed!\n" + std::string(openni::OpenNI::getExtendedError()) + '\n', true );
			continue;
		}

		// Verify depth frame data is valid
		if( !depthStreamFrame.isValid() )
		{
			sendOutput( "Error reading depth stream frame\n", true );
			break;
		}
		
		// Verify color frame data is valid
		if( !colorStreamFrame.isValid() )
		{
			sendOutput( "Error reading color stream frame\n", true );
			break;
		}

		// Get the frame index number
		const long frameIndex = depthStreamFrame.getFrameIndex();

		// Skip unneeded frames
		if( *(this->omittedFrames.find(frameIndex)) == *(this->omittedFrames.end()) )
		{
			if(frameIndex % frameSkip == 0 ) {
	                        std::cout<<"Frame Number: " << frameIndex << '\n';
				// Get the video stream field of view
				const float fov_x = depthStream.getHorizontalFieldOfView();
				const float fov_y = depthStream.getVerticalFieldOfView();
				
				// Output needed frame data
				outputFrameToCsv( out, depthStreamFrame );
				outputFrameToPcd<pcl::PointXYZRGBA>( pointCloudOutputDirectory + '/', &device, depthStreamFrame, colorStreamFrame, fov_x, fov_y );
			}
		}
		else
		{
			//std::cout << "Skipping frame " << frameIndex << "\n";
		}

		// Break if reading the last frame
		if( totalFrames == frameIndex )
		{
			sendOutput( "Last frame has been read...\n", false );
			break;
		}
	}

	// Cleanup
	out.clear();
	out.close();
	depthStream.stop();
	depthStream.destroy();
	colorStream.stop();
	colorStream.destroy();
	device.close();
	
	return 0;
}

/// @brief	Performs class setup actions
void vba::OniToPcd::init()
{
	// Check frame skip
	setFrameSkip( frameSkip );
	
	// Create Output directory
	if( !vba::filesystemhelper::createDirectory(outputDirPath) )
		exit( 101 );

	// Initialize openni
	openni::Status rc = openni::OpenNI::initialize();
	if( rc != openni::STATUS_OK )
	{
		sendOutput( "Initialize OpenNI2 failed\n" + std::string(openni::OpenNI::getExtendedError()) + '\n', true );
		exit( 102 );
	}
}

///////////////////////////////////////////////////////////////////////////////////////
/// @brief	Sends output messages to the output buffer
///
/// @param[in]	output	Contains information to be displayed to the user
/// @param[in]	error	True if the message is an error message, False otherwise
///////////////////////////////////////////////////////////////////////////////////////
void vba::OniToPcd::sendOutput( const std::string& output, const bool error )
{
	if( redirectOutputFlag == true )
	{
		if( !outputBuffer->push(output) )
			std::cout << "[" << output << "] did not make it too buffer\n";
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

///////////////////////////////////////////////////////////////////////////////////////
/// @brief	Exports a depth frame to comma separated value (.csv) file
///
/// @param[in]	outFileStream	File data stream for csv output
/// @param[in]	frameReference	A depth frame to output
///////////////////////////////////////////////////////////////////////////////////////
void vba::OniToPcd::outputFrameToCsv( std::ofstream& outFileStream, const openni::VideoFrameRef frameReference )
{
	OniDepthPixel* pDepth = (OniDepthPixel*)frameReference.getData();
	const int frameHeight = frameReference.getHeight();
	const int frameWidth = frameReference.getWidth();

	// Output the frame header
	std::stringstream frameHeaderSS;
	frameHeaderSS << "Processing " << frameWidth << 'x' << frameHeight << " frame number " << frameReference.getFrameIndex() << "...\n";
	sendOutput( frameHeaderSS.str(), false );
	outFileStream << "FrameNumber=" << frameReference.getFrameIndex() << ",FrameWidth=" << frameWidth << ",FrameHeight=" << frameHeight << ",\n";

	// All heights of the frame
	for( int y = 0; y < frameHeight; ++y )
	{
		// All widths of the frame
		for( int x = 0; x < frameWidth; ++x, ++pDepth )
		{
			outFileStream << *pDepth << ",";
		}
		outFileStream << "\n";
	}
	outFileStream << ",\n";
}

///////////////////////////////////////////////////////////////////////////////////////
/// @brief	@a Template @a Function Exports a depth and RGB frame to point cloud data (.pcd) files
///
/// @param[in]	outputFrameDirectory	Directory to output the .pcd files
/// @param[in]	device					Pointer to the device with the loaded oni file
/// @param[in]	depthFrameReference		Depth frame to output
/// @param[in]	colorFrameReference		Color frame to output
///////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
vba::OniToPcd::outputFrameToPcd( const std::string outputFrameDirectory, const openni::Device* device, const openni::VideoFrameRef depthFrameReference, const openni::VideoFrameRef colorFrameReference, const float fieldOfView_X, const float fieldOfView_Y )
{
	// Set cloud meta data
	const int depthWidth = depthFrameReference.getWidth();
	const int depthHeight = depthFrameReference.getHeight();
	const int colorWidth = colorFrameReference.getWidth();
	const int colorHeight = colorFrameReference.getHeight();
	boost::shared_ptr<pcl::PointCloud<PointT> > cloud( new pcl::PointCloud<PointT> );
	cloud->header.seq = depthFrameReference.getFrameIndex();
	cloud->header.frame_id = depthFrameReference.getFrameIndex();
	cloud->header.stamp = depthFrameReference.getTimestamp();
	cloud->height = std::max( depthHeight, colorHeight );
	cloud->width = std::max( depthWidth, colorWidth );
	cloud->is_dense = false;
	cloud->points.resize( cloud->height * cloud->width );

	// Calculate the center of height and width
	const float centerX = ((float)depthWidth - 1.f) / 2.f;
	const float centerY = ((float)depthHeight - 1.f) / 2.f;

	// Use field of view to calculate focal length
	const float focalLength_x = ((float)depthWidth / 2) / tan( (float)fieldOfView_X / 2 );
	const float focalLength_y = ((float)depthHeight / 2) / tan( (float)fieldOfView_Y / 2 );
	
	// Get inverse focal length for calculations below
	const float inverseFocalLength_x = 1.0f / focalLength_x;
	const float inverseFocalLength_y = 1.0f / focalLength_y;
	
	// Get the depth frame data
	const uint16_t* depth_map = (const uint16_t*)depthFrameReference.getData();
	if( depthFrameReference.getWidth() != depthWidth || depthFrameReference.getHeight() != depthHeight )
	{
		// Resize the depth image if nessacery
		std::vector<uint16_t> pointsBuffer;
		pointsBuffer.resize( depthWidth * depthHeight );
		depth_map = pointsBuffer.data();
		fillBufferDepth( depthWidth, depthHeight, (unsigned short*)depth_map, depthFrameReference.getWidth(), depthFrameReference.getHeight(), depthFrameReference );
	}
	
	// Get the color frame data
	const uint8_t* rgb_buffer = (const uint8_t*)colorFrameReference.getData();
	if( colorFrameReference.getWidth() != colorWidth || colorFrameReference.getHeight() != colorHeight )
	{
		// Resize the color image if nessacery
		std::vector<uint8_t> pointsBuffer;
		pointsBuffer.resize(colorWidth * colorHeight * 3);
		rgb_buffer = pointsBuffer.data();
		fillBufferRGB( colorWidth, colorHeight, (unsigned char*)rgb_buffer, colorFrameReference.getWidth(), colorFrameReference.getHeight(), colorFrameReference, colorWidth * 3 );
	}
	
	// Set bad point value
	const float bad_point = std::numeric_limits<float>::quiet_NaN();

	// Set xyz to Nan and rgb to 0 (black)  
	if( colorWidth != depthWidth )
	{
		PointT pt;
		pt.x = pt.y = pt.z = bad_point;
		pt.b = pt.g = pt.r = 0;
		pt.a = 255; // point has no color info -> alpha = max => transparent 
		cloud->points.assign( cloud->points.size(), pt );
	}
	
	// Fill in the XYZ values
	unsigned step = cloud->width / depthWidth;
	unsigned skip = cloud->width * step - cloud->width;
	int value_idx = 0;
	int point_idx = 0;
	for( int v = 0; v < depthHeight; ++v, point_idx += skip )
	{
		for( int u = 0; u < depthWidth; ++u, ++value_idx, point_idx += step )
		{
			PointT& pt = cloud->points[point_idx];
			
			// Check for invalid measurements
			OniDepthPixel pixel = depth_map[value_idx];
			if( pixel != 0 )
			{
				pt.z = depth_map[value_idx] * 0.001f;  // millimeters to meters
				pt.x = (static_cast<float> (u) - centerX) * pt.z * inverseFocalLength_x;
				pt.y = (static_cast<float> (v) - centerY) * pt.z * inverseFocalLength_y;
			}
			else
			{
				pt.x = pt.y = pt.z = bad_point;
			}
		}
	}

	// Fill in the RGB values
	step = cloud->width / colorWidth;
	skip = cloud->width * step - cloud->width;
	value_idx = 0;
	point_idx = 0;
	RGBValue color;
	color.Alpha = 0;
	for( int yIdx = 0; yIdx < colorHeight; ++yIdx, point_idx += skip )
	{
		for( int xIdx = 0; xIdx < colorWidth; ++xIdx, point_idx += step, value_idx += 3 )
		{
			PointT& pt = cloud->points[point_idx];

			color.Red   = rgb_buffer[value_idx];
			color.Green = rgb_buffer[value_idx + 1];
			color.Blue  = rgb_buffer[value_idx + 2];

			pt.rgba = color.long_value;
		}
	}
	
	// Set the point cloud orientation
	cloud->sensor_origin_.setZero();
	cloud->sensor_orientation_.setIdentity();

	// Output the point cloud to a pcd frame file
	std::stringstream outputFrameFile;
	outputFrameFile << outputFrameDirectory << "frame_" << std::setw( 10 ) << std::setfill( '0' ) << depthFrameReference.getFrameIndex() << ".pcd";
	pcl::PCDWriter w;
	w.writeBinaryCompressed( outputFrameFile.str(), *cloud );
	return ( cloud );
}

///////////////////////////////////////////////////////////////////////////////////////
/// @brief	Fills a RGB24 image buffer with downsampling
///
/// @param[in]	newWidth			Width of the new buffer image
/// @param[in]	newHeight			Height of the new buffer image
/// @param[out]	rgb_buffer			Buffer to fill with the new image
/// @param[in]	oldWidth			Width of the input image
/// @param[in]	oldHeight			Height of the input image
/// @param[in]	colorFrameReference	Input image to copy
/// @param[in]	rgb_line_step		Number of lines to step when downsampling
///////////////////////////////////////////////////////////////////////////////////////
void vba::OniToPcd::fillBufferRGB( const unsigned newWidth, const unsigned newHeight, unsigned char* rgb_buffer, const unsigned oldWidth, const unsigned oldHeight, const openni::VideoFrameRef colorFrameReference, unsigned rgb_line_step )
{
	if( newWidth > oldWidth || newHeight > oldHeight )
	{
		std::stringstream notSupportedMessage;
		notSupportedMessage << "Up-sampling not supported. Request was " << oldWidth << 'x' << oldHeight << " -> " << newWidth << 'x' << newHeight << ".\n";
		sendOutput( notSupportedMessage.str(), false );
		exit( 13 );
	}

	if( newWidth == oldWidth && newHeight == oldHeight )
	{
		const unsigned line_size = newWidth * 3;
		if( rgb_line_step == 0 || rgb_line_step == line_size )
		{
			memcpy( rgb_buffer, colorFrameReference.getData(), colorFrameReference.getDataSize() );
		}
		else // line by line
		{
			unsigned char* rgb_line = rgb_buffer;
			const unsigned char* src_line = static_cast<const unsigned char*>(colorFrameReference.getData());
			for( unsigned yIdx = 0; yIdx < newHeight; ++yIdx, rgb_line += rgb_line_step, src_line += line_size )
			{
				memcpy( rgb_line, src_line, line_size );
			}
		}
	}
	else if( oldWidth % newWidth == 0 && oldHeight % newHeight == 0 ) // downsamplig
	{
		unsigned src_step = oldWidth / newWidth;
		unsigned src_skip = (oldHeight / newHeight - 1) * oldWidth;

		if( rgb_line_step == 0 )
			rgb_line_step = newWidth * 3;

		unsigned dst_skip = rgb_line_step - newWidth * 3; // skip of padding values in bytes

		openni::RGB888Pixel* dst_line = reinterpret_cast<openni::RGB888Pixel*> (rgb_buffer);
		const openni::RGB888Pixel* src_line = (openni::RGB888Pixel*)colorFrameReference.getData();

		for( unsigned yIdx = 0; yIdx < newHeight; ++yIdx, src_line += src_skip )
		{
			for( unsigned xIdx = 0; xIdx < newWidth; ++xIdx, src_line += src_step, dst_line++ )
			{
				*dst_line = *src_line;
			}

			if( dst_skip != 0 )
			{
				// use bytes to skip rather than XnRGB24Pixel's, since line_step does not need to be multiple of 3
				unsigned char* temp = reinterpret_cast <unsigned char*> (dst_line);
				dst_line = reinterpret_cast <openni::RGB888Pixel*> (temp + dst_skip);
			}
		}
	}
	else
	{
		std::stringstream notSupportedMessage;
		notSupportedMessage << "Down-sampling only possible for integer scale. Request was " << oldWidth << 'x' << oldHeight << " -> " << newWidth << 'x' << newHeight << ".\n";
		sendOutput( notSupportedMessage.str(), false );
		exit( 14 );
	}
}

///////////////////////////////////////////////////////////////////////////////////////
/// @brief	Fills a depth image buffer with downsampling
///
/// @param[in]	newWidth			Width of the new buffer image
/// @param[in]	newHeight			Height of the new buffer image
/// @param[out]	depth_buffer		Buffer to fill with the new image
/// @param[in]	oldWidth			Width of the input image
/// @param[in]	oldHeight			Height of the input image
/// @param[in]	depthFrameReference	Input image to copy
/// @param[in]	line_step			Number of lines to step when downsampling
///////////////////////////////////////////////////////////////////////////////////////
void vba::OniToPcd::fillBufferDepth( const unsigned newWidth, const unsigned newHeight, unsigned short* depth_buffer, const unsigned oldWidth, const unsigned oldHeight, const openni::VideoFrameRef depthFrameReference, unsigned line_step )
{
	if( newWidth > oldWidth || newHeight > oldHeight )
	{
		std::stringstream notSupportedMessage;
		notSupportedMessage << "Up-sampling not supported. Request was " << oldWidth << 'x' << oldHeight << " -> " << newWidth << 'x' << newHeight << ".\n";
		sendOutput( notSupportedMessage.str(), false );
		exit( 15 );
	}

	if( oldWidth % newWidth != 0 || oldHeight % newHeight != 0 )
	{
		std::stringstream notSupportedMessage;
		notSupportedMessage << "Down-sampling only possible for integer scale. Request was " << oldWidth << 'x' << oldHeight << " -> " << newWidth << 'x' << newHeight << ".\n";
		sendOutput( notSupportedMessage.str(), false );
		exit( 16 );
	}

	if( line_step == 0 )
		line_step = newWidth * static_cast<unsigned> (sizeof (unsigned short));

	// special case no sclaing, no padding => memcopy!
	if( newWidth == oldWidth && newHeight == oldHeight && (line_step == newWidth * sizeof (unsigned short)) )
	{
		memcpy( depth_buffer, depthFrameReference.getData(), depthFrameReference.getDataSize() );
		return;
	}

	// padding skip for destination image
	unsigned bufferSkip = line_step - newWidth * static_cast<unsigned> (sizeof (unsigned short));

	// step and padding skip for source image
	unsigned xStep = oldWidth / newWidth;
	unsigned ySkip = (oldHeight / newHeight - 1) * oldWidth;

	// Fill in the depth image data, converting mm to m
	const short bad_point = std::numeric_limits<short>::quiet_NaN ();
	unsigned depthIdx = 0;

	const unsigned short* inputBuffer = static_cast<const unsigned short*> (depthFrameReference.getData());

	for( unsigned yIdx = 0; yIdx < newHeight; ++yIdx, depthIdx += ySkip )
	{
		for( unsigned xIdx = 0; xIdx < newWidth; ++xIdx, depthIdx += xStep, ++depth_buffer )
		{
			/// @todo Different values for these cases
			unsigned short pixel = inputBuffer[depthIdx];
			if( pixel == 0 )
				*depth_buffer = bad_point;
			else
			{
				*depth_buffer = static_cast<unsigned short>( pixel );
			}
		}
		// if we have padding
		if( bufferSkip > 0 )
		{
			char* cBuffer = reinterpret_cast<char*> (depth_buffer);
			depth_buffer = reinterpret_cast<unsigned short*> (cBuffer + bufferSkip);
		}
	}
}

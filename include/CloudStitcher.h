///////////////////////////////////////////////////////////////////////////////////////
/// @file	CloudStitcher.h
/// @brief	Reads in a directory of .pcd files and stitches them together into a single .pcd file.
///////////////////////////////////////////////////////////////////////////////////////



#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <ctime>

#include <boost/lockfree/spsc_queue.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "CloudRegistrationFunctions.h"


#ifndef CLOUDSTITCHER_H_
#define CLOUDSTITCHER_H_


namespace vba
{

/// @brief  a bunch of typedefs for the point cloud data types so that I can save myself several keystrokes later
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;



/// @brief  This enum defines the different number of threads that can be used in the stitching of
///         pcd files.
enum THREAD_COUNT
{
	THREAD_1,
	THREAD_2,
	THREAD_4,
	THREAD_8
};

//////////////////////////////////////////////////////////////////////////////////////
/// @details	This class provides the functionality for stitching together sequential point clouds gathered
///             from a camera. The stitching is implemented parallely to speed up performance. The class will take a
///             directory on the filesystem and stitch together all the found .pcd files. Note: because the component
///             stitches one point cloud after another; it will follow alphabetical order of the filenames in the provided
///             directory.
///////////////////////////////////////////////////////////////////////////////////////
class CloudStitcher
{

	public:


		/*Default constructor for this class, which just initializes the members to default values.
		 *
		 * @param:
		 * @param:
		 * @return:
		 */
        //////////////////////////////////////////////////////////////////////////////////////
        /// @brief	default constructor
        ///////////////////////////////////////////////////////////////////////////////////////
		CloudStitcher();

		//////////////////////////////////////////////////////////////////////////////////////
        /// @brief	default destructor
        ///////////////////////////////////////////////////////////////////////////////////////
		virtual ~CloudStitcher();



        //////////////////////////////////////////////////////////////////////////////////////
        /// @brief	    sets the output pathname for the final stitched .pcd file
        ///
        /// @param[in]	output_path		Path to where output files will be written.
        ///                             Can be an absolute or relative path with the intended filename appended.
        ///
        /// @return     int             0 if successful or -1 otherwise
        ///////////////////////////////////////////////////////////////////////////////////////
		int setOutputPath( const std::string output_path );



		//////////////////////////////////////////////////////////////////////////////////////
        /// @brief	gets the currently set output path
        ///
        /// @return The output path previously set by the user or an empty string otherwise
        ///////////////////////////////////////////////////////////////////////////////////////
		inline
		std::string getOutputPath(){ return output_path; }


		//////////////////////////////////////////////////////////////////////////////////////
        /// @brief	It is possible to redirect the standard output and standard error messages into a queue for further processing
        ///         with this function
        ///
        /// @param[in]	buf		                A pointer to a buffer that the messages will be loaded into
        ///////////////////////////////////////////////////////////////////////////////////////
		void setOutputBuffer( boost::lockfree::spsc_queue<std::string>* buf);


		//methods to toggle configuration settings

		//////////////////////////////////////////////////////////////////////////////////////
        /// @brief	This component automatically divides the stitching process among several threads. This function can restrict
        ///         the process towards only using 1 worker thread.
        ///
        /// @param[in]	choice		            True to enable multiple threads (default) or false otherwise
        ///////////////////////////////////////////////////////////////////////////////////////
		void enableMultithreading( const bool choice );



		//////////////////////////////////////////////////////////////////////////////////////
        /// @brief	Starts the process of stitching .pcd files together.
        ///
        /// @param[in]	directory_path		    The absolute or relative path to the directory containing the target .pcd files
        ///
        /// @pre    The setOutputPath function must have already been successfully called
        /// @return 0 if successful or -1 otherwise
        ///////////////////////////////////////////////////////////////////////////////////////
		int stitchPCDFiles( const std::string directory_path );


		//class setters and getters


		//////////////////////////////////////////////////////////////////////////////////////
        /// @brief	Returns the number of .pcd files that were found in the supplied directory
        ///
        /// @return The number of .pcd files found or 0 otherwise
        ///////////////////////////////////////////////////////////////////////////////////////
		unsigned int getNumberofFilesRead();


		//////////////////////////////////////////////////////////////////////////////////////
        /// @brief	The stitching component can use a voxel grid filter to reduce the density of the clouds
        ///         and reduce the overall runtime of the stitching process
        ///
        /// @param[in]	value           		The filtering value. The higher the number: the more points that are filtered. Value must
        ///                                     be between 0 and 20.
        /// @return 0 if successful or -1 otherwise
        ///////////////////////////////////////////////////////////////////////////////////////
		int setFilterResolution( unsigned int value );


        //////////////////////////////////////////////////////////////////////////////////////
        /// @brief	Retuns the user set filter resolution or the default resolution otherwise. Note: the return value is a float
        ///         instead of an integer, because the supplied integer in setFilterResolution is converted into a usable number.
        ///
        /// @return A float with the actual value used for the voxel grid filtering. It will be the user set number or the default
        ///         number otherwise.
        ///////////////////////////////////////////////////////////////////////////////////////
		inline
		float getFilterResolution() {return this->filter_leaf_size;}

	private:

		std::string pcd_files_directory;
		std::string output_path;
		unsigned int files_finished;

		std::vector<std::string>* pcd_filenames;

		boost::lockfree::spsc_queue<std::string>* output_buffer;
		bool redirect_output_flag;


		//configurations
		bool multithreading_enabled;
		THREAD_COUNT num_threads;
		float filter_leaf_size;


		/*Private facing function that allows the user to select which directory contains target pcd files.
		 * The function then checks the existence of the directory and extracts all files with .pcd file
		 * extension.
		 *
		 * @param: The string containing the absolute path pointing towards the directory containing
		 * 			the target pcd files.
		 * @return: 0 if the operation was successful, -1 if not.
		 */
		int setPCDDirectory( std::string directory_path );




		/*This is a helper function used internally within CloudStitcher. It subdivides all the pcd
		 * files to be worked on among the threads.
		 *
		 * @param: the number of threads that are going to be used
		 * @param: the total number of files to be stitched by the threads
		 * @return: none
		 */
		void setupWorkerThreads( unsigned int thread_count , unsigned int num_files );


		/*This a function used only in this component. It is called whenever we want to create output messages or
		 * error messages. If the user provided an output function we will use that. Otherwise we just print to
		 * standard output or standard error.
		 *
		 * @param: The string containing the message to be outputted.
		 * @param: True if the message is an error, false otherwise.
		 *
		 */
		void sendOutput( std::string output , bool is_error );



		//This is a class that wraps all the operations associated with one thread. Since there can be multiple
		//instances of this class that are all owned by a single CloudStitcher object, we encapsulated the
		//class definition within CloudStitcher.
		class CloudStitchingThread
		{
			public:

				/*Class Constructor
				 *
				 * @param: a copy of the vector containing the pcd files to stich together
				 * @return: none
				 */
				CloudStitchingThread( const std::vector< std::string >& files , boost::lockfree::spsc_queue<std::string>* buf , unsigned int* files_finished , float filter_res );

				/*Default destructor that deallocates the objects allocated dynamically throughout this instances
				 * lifespan
				 *
				 * @param: none
				 * @return: none
				 */
				~CloudStitchingThread();

				/*This method starts up the thread to stitch clouds together. It also calls detectExitThread() to
				 * wait on the new thread. For this reason,  this function will return immediately to the calling
				 * function instead of blocking until the thread is finished.
				 *
				 * @param: none
				 * @return: none
				 */
				void start();

				/*This function can be repeatedly called to check if the worker thread has finished yet.
				 *
				 * @param: none
				 * @return: true if the worker thread is finished, false otherwise
				 */
				bool isFinished();

				/*This function spawns another thread whose only purpose is to wait for the worker thread to
				 * finish. This allows the start function of this class to be non-blocking.
				 *
				 * @param: none
				 * @return: none
				 */
				void detectThreadExit();

				/*This is the function supplied to the worker thread. It contains all the code required to stitch the
				 * target pcd files together. This function will run until all clouds have been stitched and will then
				 * return and delete the worker thread.
				 *
				 * @param:
				 * @param:
				 * @return:
				 */
				void stitchTargetClouds();


				PointCloud getStitchedPointCloud();

				Eigen::Matrix4f getFinalCloudTransform();

			private:

				bool worker_thread_is_finished;
				std::vector< std::string >* file_list;
				unsigned int* files_finished;
				float filter_leaf_size;

				boost::thread worker_thread;
				boost::thread exit_detect_thread;

				boost::lockfree::spsc_queue<std::string>* output_buffer;

				pcl::PointCloud< pcl::PointXYZRGB > finished_cloud;
				Eigen::Matrix4f cloud_transform;
		};

		std::vector<CloudStitcher::CloudStitchingThread* >* worker_threads;


};

} //vba


#endif /* CLOUDSTITCHER_H_ */

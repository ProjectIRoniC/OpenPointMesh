/*
 * CloudStitcher.h
 *
 *  Created on: Oct 12, 2015
 *      Author: matt
 */

//Generic includes
#include <vector>
#include <string>
#include <utility>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <sstream>

//include this file for the pcd registration stuff
#include "PCDRegistration.h"

//include some of the boost library to help with searching filesystem directories
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

//include some of the boost library for multithreading
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/locale.hpp>
#include <boost/lexical_cast.hpp>

//include for output
#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/policies.hpp>
#include <boost/atomic.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <string.h>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#ifndef CLOUDSTITCHER_H_
#define CLOUDSTITCHER_H_

namespace vba
{

	//This enum just defines the different number of threads that can be used in the stitching of
	//pcd files.
	enum THREAD_COUNT
	{
		THREAD_1,
		THREAD_2,
		THREAD_4,
		THREAD_8,
		THREAD_16
	};

	//Just typedeffing the signature of a function pointer we can accept from the user for output redirection
	typedef void (*outputFunction ) (std::string output , bool is_error );


	class CloudStitcher
	{

		public:


			/*Default constructor for this class, which just initializes the members to default values.
			 *
			 * @param:
			 * @param:
			 * @return:
			 */
			CloudStitcher();

			/*Default destructor that deallocates the dynamically allocated members of the class.
			 *
			 * @param:
			 * @param:
			 * @return:
			 */
			virtual ~CloudStitcher();


			/*Public facing function that allows the user to select which directory contains target pcd files.
			 * The function then checks the existence of the directory and extracts all files with .pcd file
			 * extension.
			 *
			 * @param: The string containing the absolute path pointing towards the directory containing
			 * 			the target pcd files.
			 * @return: 0 if the operation was successful, -1 if not.
			 */
			int setPCDDirectory( std::string directory_path );

			/*Public facing function that allows the user to select which directory contains target pcd files.
			 * The function then checks the existence of the directory and extracts all files with .pcd file
			 * extension.
			 *
			 * @param: The string containing the absolute path where the user wants the final stitched cloud
			 *			to be sent to.
			 * @return: 0 if the operation was successful, -1 if not.
			 */
			int setOutputPath( const std::string output_path );
			


			/*Public facing function that accepts a function pointer. This function pointer will be passed all
			 * the output from this function. If no function pointer is ever given, we will just default to sending
			 * all the output to standard out and standard error.
			 *
			 * @param: Function pointer following the signature   void functionName( std::string )
			 *
		
			 */	
		        /*Public facing function that accepts a queue where class output will be redirected. If never set
			 * output will be sent to std::out & std::error
			 * @param: Buf is a pointer to a buffer that will be monitored by driver class			
			 */		
			void setOutputBuffer( boost::lockfree::spsc_queue<std::string>* buf);


			//methods to toggle configuration settings

			/*Public facing function that allows the user to toggle the utilization of concurrency while
			 * stitching together clouds.
			 *
			 * @param: true if concurrency should be utilized, false otherwise. The default value is true.
			 * @return:none
			 */
			void enableMultithreading( const bool choice );

			/*This function allows the user to initiate the cloud stitching operation. This is the function
			 * that will instantiate all threads, supply their input, and monitor the threads to completion.
			 *
			 * @return: 0 if operation was successful, -1 otherwise
			 */
			int stitchPCDFiles( const std::string directory_path );


			//class setters and getters


			/*Just a getter function to return the number of read in files from the directory.
			 *
			 * @return: the number of pcd files that had been read in from the directory supplied to the
			 * 			setPCDDirectory() function call.
			 */
			unsigned int getNumberofFilesRead();

		private:

			std::string pcd_files_directory;
			std::string output_path;
			std::vector<std::string>* pcd_filenames;
			std::vector<std::string>* temp_directories;

			boost::lockfree::spsc_queue<std::string>* output_buffer;
			bool redirect_output_flag;


			//configurations
			bool multithreading_enabled;
			THREAD_COUNT num_threads;

			/*This is a helper function used internally within CloudStitcher. It subdivides all the pcd
			 * files to be worked on among the threads.
			 *
			 * @param: the number of threads that are going to be used
			 * @param: the total number of files to be stitched by the threads
			 * @return: none
			 */
			void setupWorkerThreads( unsigned int thread_count , unsigned int num_files , std::string output_dir );



			/*This is a function that is called in the destructor to delete all those temp files and temp directories we created while
			 * processing the pcd files. This function should also be called after we encounter any error that causes the operations to
			 * stop.
			 *
			 * @param: none
			 * @return: 0 upon success
			 * 			-1 if an error occurred while deleting temp files and directories
			 */
			int cleanupTempDirectories();


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
					CloudStitchingThread( const std::vector< std::string >& files , std::string output_dir , boost::lockfree::spsc_queue<std::string>* output_buffer );

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

				private:

					bool worker_thread_is_finished;
					std::vector< std::string >* file_list;
					std::string output_directory;
					std::string output_filename;
					PCDRegistration* mPCDRegistration;

					boost::thread worker_thread;
					boost::thread exit_detect_thread;
			};

			std::vector<CloudStitcher::CloudStitchingThread* >* worker_threads;

	};

} /* namespace vba */

#endif /* CLOUDSTITCHER_H_ */

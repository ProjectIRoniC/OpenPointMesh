/*
 * CloudStitcher.cpp
 *
 *  Created on: Oct 12, 2015
 *      Author: matt
 */

#include "CloudStitcher.h"

namespace vba
{

	CloudStitcher::CloudStitcher()
	{
		this->pcd_filenames = new std::vector< std::string >();
		this->worker_threads = new std::vector< CloudStitcher::CloudStitchingThread* >();
		this->output_path = "";
		this->temp_directories = new std::vector< std::string >();

		this->num_threads = THREAD_1;
		this->multithreading_enabled = false;
		this->redirect_output_flag = false;

	}

	CloudStitcher::~CloudStitcher()
	{
		delete pcd_filenames;

		for( int i = 0 ; i < worker_threads->size() ; ++i )
		{
			CloudStitcher::CloudStitchingThread* temp = this->worker_threads->at(i);
			delete temp;
		}

		delete temp_directories;

	}

	int CloudStitcher::setPCDDirectory( std::string directory_path )
	{
		std::string file_extension = ".pcd";
		std::string current_path;


		//check to make sure there actually are characters in the function parameter
		if( directory_path.size() == 0 )
		{
			this->sendOutput( "Error: Provided empty string as directory.\n" , false );
			return -1;
		}

		//now lets check if the last character in the directory path is '/'. If there is none
		//we need to add it in for path concatenation later on
		if( directory_path.at( directory_path.size() - 1 ) != '/' )
		{
			directory_path += "/";
		}

		//check to make sure the directory is real
		if( boost::filesystem::exists( directory_path ))
		{
			this->pcd_files_directory = directory_path;

			boost::filesystem::directory_iterator end;

			//iterate over all entries in the directory
			for( boost::filesystem::directory_iterator itr( directory_path ) ; itr != end ; ++itr )
			{
				//check to make sure the entry is not another directory
				if( !boost::filesystem::is_directory( *itr ) )
				{
					boost::filesystem::path p = itr->path();
					current_path = p.string();

					//convert to all lowercase
					std::transform ( current_path.begin(), current_path.end(), current_path.begin(), (int(*)(int))tolower);

					if( current_path.find( file_extension , 0 ) == std::string::npos )
					{
						continue;
					}
					else
					{
						//add the filename to the vector
						current_path = p.string();
						this->pcd_filenames->push_back( current_path );
					}
				}
			}
		}

		else
		{
			//return if the directory path cannot be found
			std::stringstream output;
			output << "Error: Given directory: " << directory_path << "does not exist.\n";
			this->sendOutput( output.str() , true );
			this->cleanupTempDirectories();
			return -1;
		}

		//make sure that the filenames are sorted in correct order
		std::sort( this->pcd_filenames->begin() , this->pcd_filenames->end() );

		//now we will set the number of threads to be used based on how many files were found
		const unsigned int num_files = this->getNumberofFilesRead();

		if( num_files <= 10 && this->multithreading_enabled == true )
		{
			this->num_threads = THREAD_1;
		}
		else if( num_files <= 20 && this->multithreading_enabled == true )
		{
			this->num_threads = THREAD_2;
		}
		else if( num_files <= 40 && this->multithreading_enabled == true )
		{
			this->num_threads = THREAD_4;
		}
		else if( num_files <= 80 && this->multithreading_enabled == true )
		{
			this->num_threads = THREAD_8;
		}
		else if( num_files <= 160 && this->multithreading_enabled == true )
		{
			this->num_threads = THREAD_16;
		}
		else if( num_files > 160 && this->multithreading_enabled == true )
		{
			this->num_threads = THREAD_16;
		}
		else
		{
			this->num_threads = THREAD_1;
		}

		return 0;
	}


	int CloudStitcher::setOutputPath( const std::string output_path )
	{
		//make a boost path out of the provided path
		boost::filesystem::path fullname( output_path );

		//split up the path into the file's parent directory path and the filename
		boost::filesystem::path parent_path( fullname.parent_path() );
		boost::filesystem::path basename( fullname.filename() );

		//check to make sure the parent directory actually exists and exit otherwise
		if( !boost::filesystem::is_directory( parent_path ) )
		{
			std::stringstream output;
			output << "Error: specified output directory does not exist: " << parent_path.string() << "\n";
			this->sendOutput( output.str() , true );
			return -1;
		}

		//check to make sure the user gave us an actual filename to use
		std::string filename = basename.string();
		if( filename.size() < 2 )
		{
			this->sendOutput( "Error: no filename was specified.\n" , true );
			return -1;
		}

		//if everything goes well, we will set the class member to the provided output path
		//for use later
		this->output_path = output_path;
		return 0;
	}

	void CloudStitcher::setOutputBuffer( boost::lockfree::spsc_queue<std::string>* buf) 
	{
		this->output_buffer = buf;
		this->redirect_output_flag = true;
	}

	int CloudStitcher::stitchPCDFiles( const std::string directory_path )
	{
		//do some basic error handling first and make sure the user gave us an output path to work with
		if( this->output_path == "" )
		{
			this->sendOutput( "Error: No output path has been specified yet.\n" , true );
			return -1;
		}

		//lets open up the directory and see how many files are in there
		int result = this->setPCDDirectory( directory_path );
		if( result != 0 )
		{
			return -1;
		}

		unsigned int file_count = this->getNumberofFilesRead();

		//Since this function will be setup recursively we have to start with a base case to
		//know when to stop and start rewinding the stack. If we only read in one file, then
		//we know that all pcd files have been combined and we are finished. This would also
		//apply if the user only supplied us with one pcd file.
		if( file_count == 1 )
		{
			//TODO we need to catch errors thrown by rename() like trying to put the file in a dir without the right access permissions

			//send our single created file to the desired output location
			try
			{
				boost::filesystem::rename( this->pcd_filenames->at(0) , this->output_path );
			}
			catch( boost::filesystem::filesystem_error const &e )
			{
				std::stringstream output( "Error: Problem moving final pcd file to output directory. Boost filesystem threw error: " );
				output << e.what() << "\n";
				this->sendOutput( output.str() , true );
				this->cleanupTempDirectories();
				return -1;
			}

			std::stringstream output;
            output<< "Successfully outputted stitched pcd file to: " << this->output_path << "\n";
			this->sendOutput( output.str() , false );

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
	        pcl::io::loadPCDFile( this->output_path , *cloud );
			std::string ply_path = this->output_path;
			size_t position = ply_path.find( ".pcd" );
			ply_path[ position ] = '.';
			ply_path[ position + 1 ] = 'p';
			ply_path[ position + 2 ] = 'l';
			ply_path[ position + 3 ] = 'y';
			pcl::io::savePLYFileASCII( ply_path , *cloud );

			output.str("");
			output << "Successfully outputed ply file to: " << ply_path << "\n";
			this->sendOutput( output.str() , false );
			
			return 0;
		}

		//we will create a new temporary directory to contain the output of each recursive step.
		//The first temp dir will be located in the original pcd directory. The second will be placed
		//in the first temp dir and so on.
		unsigned int temp_dir_count = this->temp_directories->size();

		//if this is the first recursive step we will name the temp dir "temp_dir1"
		if( temp_dir_count == 0 )
		{
			std::string temp_dir_name = this->pcd_files_directory;
			temp_dir_name += "temp_dir1/";
			this->temp_directories->push_back( temp_dir_name );
			boost::filesystem::path temp_dir( temp_dir_name );

			try
			{
				boost::filesystem::create_directory( temp_dir );
			}
			catch( boost::filesystem::filesystem_error const &e )
			{
				std::stringstream output;
				output << "Error: Problem creating temporary directory. Boost error: " << e.what() << "\n";
				this->sendOutput( output.str() , true );
				this->cleanupTempDirectories();
				return -1;
			}
		}

		//if this is not the first recursive step then we will name the new temp dir based on how many
		//times we have recursed
		else
		{
			std::stringstream new_temp_dir_name;
			new_temp_dir_name << this->pcd_files_directory;
			new_temp_dir_name << "temp_dir";
			new_temp_dir_name << ( temp_dir_count + 1 );
			new_temp_dir_name << "/";

			boost::filesystem::path temp_dir( new_temp_dir_name.str() );

			try
			{
				boost::filesystem::create_directory( temp_dir );
			}
			catch( boost::filesystem::filesystem_error const &e )
			{
				std::stringstream output;
				output << "Error: Problem creating temporary directory. Boost error: " << e.what() << "\n";
				this->sendOutput( output.str() , true );
				this->cleanupTempDirectories();
				return -1;
			}

			//we have to keep track of the paths to all these temp dirs for cleanup later
			this->temp_directories->push_back( new_temp_dir_name.str() );

		}



		switch( this->num_threads )
		{
		case THREAD_1:
			this->setupWorkerThreads( 1 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;

		case THREAD_2:
			this->setupWorkerThreads( 2 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;

		case THREAD_4:
			this->setupWorkerThreads( 4 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;

		case THREAD_8:
			this->setupWorkerThreads( 8 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;

		case THREAD_16:
			this->setupWorkerThreads( 16 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;

		default:
			this->setupWorkerThreads( 1 , this->getNumberofFilesRead() , this->temp_directories->back() );
			break;
		}

		//just printing some info to the user
		std::stringstream output;
        output << "Read in " << this->pcd_filenames->size() << " files.\n";
		this->sendOutput( output.str() , false );

		output.str("");
        output << "Spinning up " << this->worker_threads->size() << " threads.\n";
		this->sendOutput( output.str() , false );


		//spin up all the allocated threads
		for( unsigned int i = 0 ; i < this->worker_threads->size() ; ++i )
		{
			CloudStitchingThread* temp = this->worker_threads->at( i );
			temp->start();
		}


		//wait for all the threads to finish up there work and delete each thread from the container as
		//it finishes
		while( this->worker_threads->size() > 0 )
		{
			for( std::vector< CloudStitchingThread* >::iterator itr = this->worker_threads->begin() ; itr != this->worker_threads->end() ; ++itr )
			{
				CloudStitchingThread* temp = *itr;
				if( temp->isFinished() )
				{
					worker_threads->erase( itr );
					delete temp;
					break;
				}
			}
		}

		//make sure we clear out the pcd filenames held in the vector so they don't get mixed up with our next operation
		this->pcd_filenames->clear();

		//This is where we hit the recursive part. We now call this function again to start combining the newly stitched pcd files
		//contained in the newly created temporary directory
		int return_code = this->stitchPCDFiles( this->temp_directories->back() );

		//If we have reached this section code, then we have hit the base case and are starting to unwind the stack. All we have to
		//do is delete all of those temporary directories we created.
		this->cleanupTempDirectories();


		return return_code;
	}

	void CloudStitcher::enableMultithreading( const bool choice )
	{
		this->multithreading_enabled = choice;
	}

	unsigned int CloudStitcher::getNumberofFilesRead()
	{
		if( !this->pcd_filenames->empty() )
		{
			return this->pcd_filenames->size();
		}

		return 0;
	}

	void CloudStitcher::setupWorkerThreads( unsigned int thread_count , unsigned int num_files , std::string output_dir )
	{
		//offset is the number of pcd files that should be allocated to each thread
		unsigned int offset = (int)std::ceil( num_files / thread_count );

		//current offset will be a changing variable representing where in the filename array we are looking at
		std::vector< std::string >::iterator current_offset = this->pcd_filenames->begin();

		//if we are using one thread, then just copy all of the filenames into the single thread
		if( thread_count == 1 )
		{
			std::vector< std::string > param_vec( current_offset , this->pcd_filenames->end() );
			this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec , output_dir , this->output_buffer ));
		}

        //otherwise we are going to divide up the filename array as evenly as possible among all the threads
		else
		{
			for( int i = 0 ; i < thread_count - 1 ; ++i )
			{
				std::vector< std::string >::iterator last = ( current_offset + offset );
				std::vector< std::string > param_vec( current_offset , last );
				this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec , output_dir , this->output_buffer ));
				current_offset = last;
			}

			//to prevent a seg-fault we just copy whatever is left of the filename array into the last thread
			std::vector< std::string > param_vec( current_offset , this->pcd_filenames->end() );
			this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec , output_dir , this->output_buffer ));

		}
	}


	int CloudStitcher::cleanupTempDirectories()
	{
		bool result = true;

		//if there are no temporary directories in the array then we don't need to delete anything
		if( this->temp_directories->size() == 0 )
		{
			return 0;
		}

		//use boost to remove each temporary dir from the filesystem and then delete the entry from temp_directories array
		std::vector< std::string >::reverse_iterator itr;
		for( itr = this->temp_directories->rbegin() ; itr != this->temp_directories->rend() ; ++itr )
		{
			try
			{
				boost::filesystem::remove_all( *itr );
				this->temp_directories->erase( --(itr.base()) );
			}
			catch( boost::filesystem::filesystem_error const &e )
			{
				this->sendOutput( "Error: Failed to delete a temporary directory\n" , true );
				result = false;
			}
		}

		if( result == true )
			return 0;
		else
			return -1;
	}




	void CloudStitcher::sendOutput( std::string output , bool is_error )
	{
		if( this->redirect_output_flag == true )
		{
			if(!this->output_buffer->push(output)) {
				std::cout << "[" << output << "] did not make it too buffer\n";
			}
		}

		else
		{
			if( is_error == true )
			{
				std::cerr << output;
			}
			else
			{
				std::cout << output;
			}
		}
	}





	CloudStitcher::CloudStitchingThread::CloudStitchingThread( const std::vector< std::string >& files , std::string output_dir , boost::lockfree::spsc_queue<std::string>* _output_buffer )
	{
		//we will make a copy of the list of target filenames for this instance of the class
		this->file_list = new std::vector< std::string >( files );
		this->worker_thread_is_finished = false;

		//make sure we save a copy of the path to the output directory
		this->output_directory = output_dir;

		//create the absolute path to the final pcd file that is a creation of all the stitching
		this->output_filename = output_dir;
		boost::filesystem::path filename( this->file_list->at(0) );
		this->output_filename += filename.filename().string();

		this->mPCDRegistration = new PCDRegistration( files , this->output_filename );
		this->mPCDRegistration->setOutputBuffer( _output_buffer );
	}

	CloudStitcher::CloudStitchingThread::~CloudStitchingThread()
	{
		delete file_list;
		delete mPCDRegistration;
	}

	void CloudStitcher::CloudStitchingThread::start()
	{
		//spin off the function that performs the stitching on all the target clouds
		this->worker_thread = boost::thread( &CloudStitcher::CloudStitchingThread::stitchTargetClouds , this );

		//spin off another thread that periodically checks if the worker thread has finished yet.
		this->exit_detect_thread = boost::thread( &CloudStitcher::CloudStitchingThread::detectThreadExit , this );
	}

	bool CloudStitcher::CloudStitchingThread::isFinished()
	{
		if( this->worker_thread_is_finished == true )
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void CloudStitcher::CloudStitchingThread::detectThreadExit()
	{
		//Since this function sits inside its own thread, it just spins while waiting for the worker thread to rejoin
		worker_thread.join();

		//when the worker thread has finished, this boolean is set to true, so we know its done
		this->worker_thread_is_finished = true;
	}

	void CloudStitcher::CloudStitchingThread::stitchTargetClouds()
	{
		this->mPCDRegistration->start();
	}

} /* namespace vba */

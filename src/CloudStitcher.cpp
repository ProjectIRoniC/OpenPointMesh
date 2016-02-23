#include "CloudStitcher.h"

namespace vba
{
	CloudStitcher::CloudStitcher()
	{
		this->pcd_filenames = new std::vector< std::string >();
		this->worker_threads = new std::vector< CloudStitcher::CloudStitchingThread* >();
		this->output_path = "";
		this->files_finished = 0;

		this->num_threads = THREAD_1;
		this->multithreading_enabled = true;
		this->redirect_output_flag = false;
		this->filter_leaf_size = 0.1;

	}

	CloudStitcher::~CloudStitcher()
	{
		delete pcd_filenames;

		for( unsigned int i = 0 ; i < worker_threads->size() ; ++i )
		{
			CloudStitcher::CloudStitchingThread* temp = this->worker_threads->at(i);
			delete temp;
		}

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
		else if( num_files > 80 && this->multithreading_enabled == true )
		{
			this->num_threads = THREAD_8;
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

		//if there is only one file in the given directory, then stitching is not really necessary.
		if( file_count == 1 )
		{
			this->sendOutput( "Error: Only 1 .pcd file was found in the directory. Stitching is not necessary.\n" , true );
			return -1;
		}



		//get all our threads setup with work they need to do
		switch( this->num_threads )
		{
		case THREAD_1:
			this->setupWorkerThreads( 1 , this->getNumberofFilesRead() );
			break;

		case THREAD_2:
			this->setupWorkerThreads( 2 , this->getNumberofFilesRead() );
			break;

		case THREAD_4:
			this->setupWorkerThreads( 4 , this->getNumberofFilesRead() );
			break;

		case THREAD_8:
			this->setupWorkerThreads( 8 , this->getNumberofFilesRead() );
			break;

		default:
			this->setupWorkerThreads( 1 , this->getNumberofFilesRead() );
			break;
		}


		//just printing some info to the user
		std::stringstream output;
		output << "Read in " << this->pcd_filenames->size() << " files.\n";
		this->sendOutput( output.str() , false );

		output.str("");
		output << "Spinning up " << this->worker_threads->size() << " threads.\n";
		this->sendOutput( output.str() , false );
        const clock_t begin_time = std::clock();

		//spin up all the allocated threads
		for( unsigned int i = 0 ; i < this->worker_threads->size() ; ++i )
		{
			CloudStitchingThread* temp = this->worker_threads->at( i );
			temp->start();
		}


		unsigned int active_thread_count = this->worker_threads->size();

		//We spin in this while loop until all threads have reported they have finished work.
		while( active_thread_count > 0 )
		{
			active_thread_count = this->worker_threads->size();

			for( std::vector< CloudStitchingThread* >::iterator itr = this->worker_threads->begin() ; itr != this->worker_threads->end() ; ++itr )
			{
				CloudStitchingThread* temp = *itr;
				if( temp->isFinished() )
				{
					active_thread_count--;
				}
			}

		}

		this->sendOutput( "-------All threads finished--------\n" , false );


		PointCloud global_cloud;
		PointCloud current_cloud;
		Eigen::Matrix4f current_transform = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
		CloudStitchingThread* current_thread;

		current_thread = this->worker_threads->at( 0 );
		current_cloud = current_thread->getStitchedPointCloud();
		current_transform = current_thread->getFinalCloudTransform();

		global_cloud += current_cloud;
		global_transform *= current_transform;

		for( unsigned int i = 1; i < this->worker_threads->size() ; i++ )
		{
			current_thread = this->worker_threads->at( i );

			current_cloud = current_thread->getStitchedPointCloud();
			pcl::transformPointCloud( current_cloud , current_cloud , global_transform );
			global_cloud += current_cloud;
			current_transform = current_thread->getFinalCloudTransform();
			global_transform *= current_transform;

		}

		PointCloud::Ptr final_cloud( new PointCloud( global_cloud ));
		vba::voxelGridFilter( final_cloud , final_cloud , this->filter_leaf_size );
		vba::statisticalOutlierFilter( final_cloud , final_cloud , 1.0 , 20 );

		if( pcl::io::savePCDFile( this->output_path , *final_cloud , true ) == -1 )
		{
			this->sendOutput( "Error: Could not save final stitched cloud to specified output path.\n" , true );
			return -1;
		}

		float elapsed_time = float( std::clock() - begin_time ) / CLOCKS_PER_SEC;
		output.str("");
		output << "Finished in " << elapsed_time << " seconds.\n";
		this->sendOutput( output.str() , false );

		return 0;
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


	int CloudStitcher::setFilterResolution( unsigned int value )
	{
		//make sure value is between 0 and 20
		if( value < 0 || value > 20 )
		{
			this->sendOutput( "Error: filter intensity value must be between 0 and 20.\n" , true );
			return -1;
		}

		//We don't want to divide by zero below, so we check if the user entered zero.
		if( value == 0 )
		{
			this->filter_leaf_size = 0.0;
		}

		else
		{
			//the actual value passed to the filter can be between 0.0 and 2.0
			this->filter_leaf_size = float( 0.01 * value );
		}

		std::cout << "filter size set to: " << this->filter_leaf_size << "\n";

		return 0;
	}




	void CloudStitcher::setupWorkerThreads( unsigned int thread_count , unsigned int num_files )
	{
		//offset is the number of pcd files that should be allocated to each thread
		unsigned int offset = (int)std::ceil( num_files / thread_count ) + 1;

		//current offset will be a changing variable representing where in the filename array we are looking at
		std::vector< std::string >::iterator current_offset = this->pcd_filenames->begin();

		//if we are using one thread, then just copy all of the filenames into the single thread
		if( thread_count == 1 )
		{
			std::vector< std::string > param_vec( current_offset , this->pcd_filenames->end() );
			this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec , this->output_buffer , &this->files_finished , this->filter_leaf_size ));
		}

		//otherwise we are going to divide up the filename array as evenly as possible among all the threads
		else
		{
			for( unsigned int i = 0 ; i < thread_count - 1 ; ++i )
			{
				std::vector< std::string >::iterator last = ( current_offset + offset );
				std::vector< std::string > param_vec( current_offset , last );
				this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec , this->output_buffer , &this->files_finished , this->filter_leaf_size ));
				current_offset = last - 1;
			}

			//to prevent a seg-fault we just copy whatever is left of the filename array into the last thread
			std::vector< std::string > param_vec( current_offset , this->pcd_filenames->end() );
			this->worker_threads->push_back( new CloudStitcher::CloudStitchingThread( param_vec , this->output_buffer , &this->files_finished , this->filter_leaf_size ));

		}
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


	CloudStitcher::CloudStitchingThread::CloudStitchingThread( const std::vector< std::string >& files , boost::lockfree::spsc_queue<std::string>* buf , unsigned int* files_finished , float filter_res )
	{
		//we will make a copy of the list of target filenames for this instance of the class
		this->file_list = new std::vector< std::string >( files );
		this->worker_thread_is_finished = false;
		this->files_finished = files_finished;
		this->filter_leaf_size = filter_res;

		this->output_buffer = buf;

		this->cloud_transform = Eigen::Matrix4f::Identity();
	}

	CloudStitcher::CloudStitchingThread::~CloudStitchingThread()
	{
		delete file_list;
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
        std::stringstream output_stream;

        PointCloud::Ptr source( new PointCloud() );
        PointCloud::Ptr target( new PointCloud() );
        PointCloud::Ptr global_cloud( new PointCloud() );

        Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity();


        for( unsigned int i = 1 ; i < file_list->size() ; i++ )
        {
            if( vba::openPCDFile( file_list->at( i - 1 ) , source ) == -1 )
                continue;

            if( vba::openPCDFile( file_list->at( i ) , target ) == -1 )
                continue;

            //for the algorithms to work correctly, we need all the NaN's to be removed from the clouds
            std::vector< int > indices;
            pcl::removeNaNFromPointCloud( *source , *source , indices );
            pcl::removeNaNFromPointCloud( *target , *target , indices );

            //original is 0.05
            vba::voxelGridFilter( source , source , this->filter_leaf_size );
            vba::voxelGridFilter( target , target , this->filter_leaf_size );
            vba::statisticalOutlierFilter( source , source , 1.0 , 5 );
            vba::statisticalOutlierFilter( target , target , 1.0 , 5 );

            //we input the two clouds to compare and the resulting transformation that lines them up is returned in pairTransform
            vba::pairAlign( source, target, pairTransform );

            //update the global transform with the pairTransform returned from aligning the two clouds
            GlobalTransform = GlobalTransform * pairTransform;

            //transform the target cloud using the accumulated transformations and the one we just got from pairAlign
            pcl::transformPointCloud (*target, *target, GlobalTransform);

            //add cloud to the total model. This will append the target's points to the whole cloud we are building up
            *global_cloud += *target;

             //send some output about the progress of stitching so far
             //possible race condition here
            output_stream.str( "" );
            output_stream << "Registered Cloud " << ++*this->files_finished << "\n";
            output_buffer->push( output_stream.str() );

        }

        this->finished_cloud = *global_cloud;
        this->cloud_transform = GlobalTransform;
	}





	PointCloud CloudStitcher::CloudStitchingThread::getStitchedPointCloud()
	{
		return this->finished_cloud;
	}

	Eigen::Matrix4f CloudStitcher::CloudStitchingThread::getFinalCloudTransform()
	{
		return this->cloud_transform;
	}




} //namespace vba
